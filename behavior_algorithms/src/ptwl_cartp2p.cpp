// Cartesian Point to Point move to command the attractor movement in a Pose Target, Wrench Limit
// Matthew Haberbusch, Surag Balajepalli, and Rahul Pokharna 

// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <iostream>
#include <cmath>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <behavior_algorithms/status_service.h>
#include <irb120_accomodation_control/freeze_service.h>
using namespace std;

// Declare variables
geometry_msgs::Pose current_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::Wrench ft_in_robot_frame;

geometry_msgs::Vector3 tool_vector_z;
geometry_msgs::Vector3 task_vector_z;

std_msgs::Int8 freeze_mode_status;
bool freeze_mode; // maybe don't define it as having a starting value? 
bool freeze_updated = false;

// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
}

void ft_callback(const geometry_msgs::Wrench& ft_values) {
    // These are not values from the sensor. They are f/t values transformed into robot base frame.
    ft_in_robot_frame = ft_values;
}

void tool_vector_callback(const geometry_msgs::Vector3& tool_vector_msg_z) {
    tool_vector_z = tool_vector_msg_z;
}
void task_vector_callback(const geometry_msgs::Vector3& task_vector_msg_z) {
    task_vector_z = task_vector_msg_z;
}

void freeze_status_callback(const std_msgs::Int8& freeze_status_msg) {
    freeze_updated = true;
    freeze_mode_status = freeze_status_msg;
    // cout<<freeze_mode_status<<endl;
    if(freeze_mode_status.data == 1) freeze_mode = true;
    else freeze_mode = false;
}


// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"simple_move_until_touch");
    ros::NodeHandle nh;

    // ROS: Define subscribers and publishers used
    //! Can just change transformed FT wrench to be normal ft wrench!!!!!!(only created in acc controller, we should read FT in tool frame, not robot base)
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback); // subscribe to the topic publishing the cartesian state of the end effector
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);                       // subscribe to the force/torque sensor data
    ros::Subscriber tool_vector_sub_z = nh.subscribe("tool_vector_z",1,tool_vector_callback);                  // subscribe to the value of the tool vector in the z, published from the accomodation controller
    ros::Subscriber task_vector_sub_z = nh.subscribe("task_vector_z",1,task_vector_callback);                  // subscribe to the value of the task vector in the z, published from the accomodation controller
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1); // publish the pose of the virtual attractor for the accomodation controller 

    // ROS: Services used in conjunction with buffer.cpp to have delayed program status sent to operator
    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    ros::ServiceClient client_start = nh.serviceClient<behavior_algorithms::status_service>("start_service");
    
    // Define services and subscribers for the freeze mode status

    // subscriber for freeze mode status (used in loop cond)
    ros::Subscriber freeze_mode_sub = nh.subscribe("freeze_mode_topic",1,freeze_status_callback);
    
    // service client for setting and unsetting freeze mode at start of program
    ros::ServiceClient freeze_client = nh.serviceClient<irb120_accomodation_control::freeze_service>("freeze_service");


    // ROS: Service status variable for use with buffer.cpp
    behavior_algorithms::status_service srv;
    srv.request.name = "CartP2PTWL";

    //ROS: Service for toggling freeze mode
    irb120_accomodation_control::freeze_service freeze_srv;

    /*
    How to tune params:
    PULL_DISTANCE: By increasing this, the virtual force is greater, and decreasing decreases it
    KEEP_CONTACT_DISTANCE: If the effort threshold is crossed, the virtual attractor is placed this distance below the current pose to keep contact if pushing, and above if pulling to keep a constant force
    FORCE_THRESHOLD: The limit at which the program will stop if the force in the direction which we are moving is crossed
    NONDIRECTIONAL_FORCE_THRESHOLD: The limit at which the program will stop if the force in the direction which we are not moving is crossed
    TORQUE_THRESHOLD: The limit at which the program will stop if the torque threshold is crossed
    RUN_TIME: How long the program will run before timing out and ending
    
    Params not needed to be tuned:
    DT: Loop rate, how fast each iteration of the loop is (most likely not needed to be changed)
    TARGET_DISTANCE: will change with user input, how far and in what direction the end effector moves

    PULL_DISTANCE and FORCE_THRESHOLD: Default values defined here, but are modified below depending on GUI setting (Line 98)
    MAX_VEL: The maximum velocity for translation (Used for cartP2P) in m/s
    MAX_ANG: The maximum angular velocity for rotation (Used for cartP2P) in rad/s
    */
    double PULL_DISTANCE = 0.012, KEEP_CONTACT_DISTANCE = 0.0075, DT = 0.01, FORCE_THRESHOLD = 12, TARGET_DISTANCE = 0.05; 
    double NONDIRECTIONAL_FORCE_THRESHOLD = 20, KEEP_CUTTING_DISTANCE = 0, MAX_VEL = 0.01, MAX_ANG = 0.2; 
    double x = 0, y = 0, z = 0;
    double psi = 0, theta = 0, phi = 0; 
    double TORQUE_THRESHOLD = 2;
    double RUN_TIME = 15;

    // Parameter for if in the cutting state, easier checking later
    bool cutting = false;

    // Parameter if we are in the task frame or not
    bool task = false;
    

    // Variable for which set of parameters to use
    string param_set = "Peg";

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs

    // spin, while we don't have data
    while(!freeze_updated){
        ros::spinOnce();
        naptime.sleep();
    }
    
    //? make sure this works, sometimes it has issues if alread frozen, will spin once to check the data
    // Check here after a spin, and unfreeze if it is frozen, 
    while(freeze_mode_status.data == 1 && freeze_mode){
        ros::spinOnce();
        naptime.sleep();

        if(freeze_client.call(freeze_srv)){
            // success
            cout<<"Called freeze mode service succesfully"<<endl;
            freeze_updated = true;
        }
        else{
            // failed to call service
            ROS_ERROR("Failed to call freeze service");
            freeze_updated = false;
        }
        ros::spinOnce();
        naptime.sleep();

    }
    
    // spin, while we don't have data
    while(!freeze_updated){
        ros::spinOnce();
        naptime.sleep();
    }


    cout<<freeze_mode_status<<endl;
    cout<<freeze_srv.response<<endl;

    // should now be unfrozen

    // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.
    
    // ROS: Get parameter passed in 
    nh.param("/CartP2PTWL/trans_x", TARGET_DISTANCE, 0.0); //! Convert to target pos and orient
    nh.param<std::string>("/CartP2PTWL/param_set", param_set, "Tool");

    // clear parameter from server 
    nh.deleteParam("/CartP2PTWL/target_distance"); 
    nh.deleteParam("/CartP2PTWL/param_set");

    // Here, the values for PULL_DISTANCE and FORCE_THRESHOLD are changed according to what setting the GUI is on for the appropriate task
    if(!strcmp(param_set.c_str(), "Peg")){
        // set the new values here
        PULL_DISTANCE = 0.012;
        FORCE_THRESHOLD = 12;
        NONDIRECTIONAL_FORCE_THRESHOLD = 20;
        TORQUE_THRESHOLD = 2;
        KEEP_CONTACT_DISTANCE = 0.0075;
        KEEP_CUTTING_DISTANCE = 0;
        RUN_TIME = 15;
        
        cutting = false;
        task = false;
        ROS_INFO("Params set for PEG");
    }
    else if (!strcmp(param_set.c_str(), "Bottle_Cap")){
        // set the other values here
        PULL_DISTANCE = 0.015;
        FORCE_THRESHOLD = 15;
        NONDIRECTIONAL_FORCE_THRESHOLD = 20;
        TORQUE_THRESHOLD = 2;
        KEEP_CONTACT_DISTANCE = 0.0075;
        KEEP_CUTTING_DISTANCE = 0;
        RUN_TIME = 15;

        cutting = false;
        task = false;
        ROS_INFO("Params set for BOTTLE_CAP");
    }
    else if (!strcmp(param_set.c_str(), "Tool")){
        // set the other values here
        PULL_DISTANCE = 0.015;
        FORCE_THRESHOLD = 15;
        NONDIRECTIONAL_FORCE_THRESHOLD = 20;
        TORQUE_THRESHOLD = 2;
        KEEP_CONTACT_DISTANCE = 0;
        KEEP_CUTTING_DISTANCE = 0;
        RUN_TIME = 15;

        cutting = false;
        task = false;
        ROS_INFO("Params set for TOOL");
    }
    else if (!strcmp(param_set.c_str(), "Cutting")){
        // set the other values here
        PULL_DISTANCE = 0.005;
        FORCE_THRESHOLD = 4;
        NONDIRECTIONAL_FORCE_THRESHOLD = 7;
        TORQUE_THRESHOLD = 2;
        KEEP_CONTACT_DISTANCE = 0;
        KEEP_CUTTING_DISTANCE = 0.00075; // was 0.001
        RUN_TIME = 30;

        cutting = true;
        task = true;
        ROS_INFO("Params set for CUTTING");
    }
    else if (!strcmp(param_set.c_str(), "Task")){
        // set the other values here
        PULL_DISTANCE = 0.006;
        FORCE_THRESHOLD = 4;
        NONDIRECTIONAL_FORCE_THRESHOLD = 7;
        TORQUE_THRESHOLD = 2;
        KEEP_CONTACT_DISTANCE = 0;
        KEEP_CUTTING_DISTANCE = 0; 
        RUN_TIME = 30;

        cutting = false;
        task = true;
        ROS_INFO("Params set for TASK");
    }
    //TODO ADD PARAM FOR TOOL EXCHANGE


    ROS_INFO("Output from parameter for target_distance; %f", TARGET_DISTANCE); 

    // With labeled parameter, now call service to send message that program will start
    std::ostringstream request_status; 
    request_status << "target_distance " << TARGET_DISTANCE << "m";

    srv.request.status = request_status.str();

    // ROS: Call the client start service, used in buffer.cpp for operator output
    if(client_start.call(srv)){
        // success
        cout<<"Called service_start with name succesfully"<<endl;
    }
    else{
        // failed to call service
        // ROS_ERROR("Failed to call service service_start");
        naptime.sleep();
    }

    // Set as unknown in case program somehow progresses past loop without any of the 3 conditions
    srv.request.status = "Unkown";

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.position.x == 0 || tool_vector_z.x == 0 || task_vector_z.x == 0) ros::spinOnce();

    //! We want to use the bumpless start, not just the tool pose. We want the initial attractor pose, or do we just interpolate the way nasa is doing it
    geometry_msgs::Vector3 start_position;
    start_position.x = current_pose.position.x;
    start_position.y = current_pose.position.y;
    start_position.z = current_pose.position.z;

    Eigen::Quaterniond start_pose_quat;
    start_pose_quat.x() = current_pose.orientation.x;
    start_pose_quat.y() = current_pose.orientation.y;
    start_pose_quat.z() = current_pose.orientation.z;
    start_pose_quat.w() = current_pose.orientation.w;


    //TODO Task is set to false, need to update to use task frame with cartp2p
    task = false;

    // Update to accept input of goal pose here, and then add the deltas 
    geometry_msgs::Vector3 delta_trans_vec;
    if(task){
        delta_trans_vec = task_vector_z;
    }
    else{
        //! Get the rotation portion included 
        delta_trans_vec.x = x;
        delta_trans_vec.y = y;
        delta_trans_vec.z = z;
    }

    //TODO Calculate the end pose here based on the input, take the relative change and add it in the tool frame,
    //TODO if we use interactive markers in rviz, use that to get ending pos

    //TODO Make sure that the values for the inputs are vectors for translation in the correct frame
    //? i.e. multiply x trans by x tool frame vec, and so on to get the translation in the right frame
    // take the vector, multiply it, and then add all 3 together for a new delta trans vec, and then that is the end
    // 
    //! Populate and update end position and rotation stuff, either callback to the Interactive Markers
    // Change this value to 
    geometry_msgs::Vector3 ending_position;
    ending_position.x = start_position.x + delta_trans_vec.x; // * TARGET_DISTANCE;
    ending_position.y = start_position.y + delta_trans_vec.y; // * TARGET_DISTANCE;
    ending_position.z = start_position.z + delta_trans_vec.z; // * TARGET_DISTANCE;

    //TODO get the orientation input
    Eigen::Quaterniond end_pose_quat;
    end_pose_quat.x() = current_pose.orientation.x;
    end_pose_quat.y() = current_pose.orientation.y;
    end_pose_quat.z() = current_pose.orientation.z;
    end_pose_quat.w() = current_pose.orientation.w;

    // Take the delta input from the gui to rotate.
    // MATH: Define the rotation matrix to the destination
    //! Angles are x: psi, y: theta, z: phi
    Eigen::Matrix3d TO_DESTINATION_ROTATION_MATRIX;
    TO_DESTINATION_ROTATION_MATRIX(0,0) = cos(theta)*cos(phi);
    TO_DESTINATION_ROTATION_MATRIX(0,1) = (sin(psi)*sin(theta)*sin(phi)) - (cos(psi)*sin(phi));
    TO_DESTINATION_ROTATION_MATRIX(0,2) = (cos(psi)*sin(theta)*sin(phi)) - (sin(psi)*cos(phi)); 
    TO_DESTINATION_ROTATION_MATRIX(1,0) = cos(theta)*sin(phi);
    TO_DESTINATION_ROTATION_MATRIX(1,1) = (sin(psi)*sin(theta)*sin(phi)) + (cos(psi)*cos(phi));
    TO_DESTINATION_ROTATION_MATRIX(1,2) = (cos(psi)*sin(theta)*sin(phi)) - (sin(psi)*cos(phi)); 
    TO_DESTINATION_ROTATION_MATRIX(2,0) = -sin(theta);
    TO_DESTINATION_ROTATION_MATRIX(2,1) = sin(psi) * cos(theta);
    TO_DESTINATION_ROTATION_MATRIX(2,2) = cos(psi) * cos(theta);
    // TODO interpolate the pose from the other code, find out how many steps we want > max vel angular/linear

    // Angular would just be deltaTheta/dt <= some value
    // Linear would be the deltaPos (sqrt(x^2+y^2+z^2))/dt 
    // Vector of the difference between the end pose and current pose, used to calculate if we reached target



    Eigen::Affine3d a_flange_des;

    // ROTATION
    Eigen::Vector3d k_rot_axis;
    Eigen::Vector3d dp_vec, O_interp, O_start, O_end;

    Eigen::Matrix3d R_start, R_end, R_change, R_change_interp, R_interp;
    R_start = start_pose_quat.normalized().toRotationMatrix(); // current tool pose, convert to rotation
    R_end = end_pose_quat.normalized().toRotationMatrix(); // end pose, convert to rotation
    //R_end = R_change*R_start
    R_change = TO_DESTINATION_ROTATION_MATRIX;
    Eigen::AngleAxisd angleAxis(R_change); //convert rotation matrix to angle/axis

    //interpolate with angle theta_interp about k_rot_axis, which is updated in main method, dtheta is the increment value
    double angle_axis_theta, theta_interp, dtheta;
    angle_axis_theta = angleAxis.angle();
    k_rot_axis = angleAxis.axis();

    //? unused values
    // Used when we calculate how many steps we want to use
    // geometry_msgs::Vector3 delta_translation; 
    // geometry_msgs::Vector3 rot_axis; //! Get value from k_rot_axis, or keep as is and do rotation with the Eigen library
    // double theta;

    // take theta, split it into the smaller sections, nsteps and dtheta and then use the max rotational velocity to get nsteps, 
    // then take the max steps from trans and then recalcuate the deltas


    // TRANSLATION

    // CUrrent translation code
    geometry_msgs::Vector3 vector_to_goal;
    vector_to_goal.x = ending_position.x - current_pose.position.x;
    vector_to_goal.y = ending_position.y - current_pose.position.y;
    vector_to_goal.z = ending_position.z - current_pose.position.z;

    // Calculate the distance we want to travel, then find the number of steps we need to do to be below the max velocity (param)
    double dist = sqrt(pow(vector_to_goal.x,2) + pow(vector_to_goal.y,2) + pow(vector_to_goal.z,2));
    
    // Check how many steps we want to have (largest number of steps so we are slowest, to be safe)
    // Here we do the math to check how many steps we wish to use, and what the value of dtheta will be, how many loops we will be doing
    int n_steps, ntrans, nrot;
    
    //TODO Take max velocity, and calculate n_steps based on max vel, can go larger, but never smaller, same with the rotation
    ntrans = (dist / MAX_VEL) / DT;
    nrot = (angle_axis_theta / MAX_ANG) / DT;

    // Get the larger of the two, now recalculate the deltas
    n_steps = ntrans < nrot ? nrot : ntrans;

    // redo this line
    // dp_vec = (O_end - O_start) / n_steps;

    delta_trans_vec.x = vector_to_goal.x / n_steps;
    delta_trans_vec.y = vector_to_goal.y / n_steps;
    delta_trans_vec.z = vector_to_goal.z / n_steps;

    //cout<<"O_start = "<<O_start.transpose()<<endl;
    //cout<<"O_end = "<<O_end.transpose()<<endl;
    //cout<<"dp_vec = "<<dp_vec.transpose()<<endl;

    
    

    
    //TODO Update goal reached exit condition
    // Dot product, uf the value is above 0, we hit the target movement
    double dot_product = vector_to_goal.x * delta_trans_vec.x + vector_to_goal.y * delta_trans_vec.y + vector_to_goal.z * delta_trans_vec.z;
    
    // If it is past the plane perpendicular to the direction of movement at the goal position, then we have reached the target 
    bool target_reached;

    if(TARGET_DISTANCE < 0){
        target_reached = dot_product >= 0;
    }
    else {
        target_reached = dot_product <= 0;
    }

    // Debug output
    cout<<"Tool Vector Z"<<endl<<tool_vector_z<<endl;
    cout<<"Difference Vector"<<endl<<vector_to_goal<<endl;
    cout<<"Dot Product"<<endl<<dot_product<<endl<<endl;

    // Output for whether we are frozen or not
    cout<<"Freeze status: "<<freeze_mode<<endl<<"Freeze message: "<<freeze_mode_status<<endl<<endl;

    // ROS_INFO(freeze_mode_status);

    // Loop variable to check effort limit condition
    bool effort_limit_crossed = false;
    effort_limit_crossed = ((abs(ft_in_robot_frame.torque.x) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.y) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.z) > TORQUE_THRESHOLD) ||
                                 (abs(ft_in_robot_frame.force.x) > FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.y) > NONDIRECTIONAL_FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.z) > NONDIRECTIONAL_FORCE_THRESHOLD));


    // Used in the loop to determine the run time and time out
    double total_number_of_loops = RUN_TIME / DT;
    double loops_so_far = 0;

    // Begin loop
    // Assuming we're always going in the x direction.
    /*
    Loop End Conditions:
    1. The operation has timed out (ran the max alloted time)
    2. One of the effort thresholds has been crossed
    3. The target orientation has been reached
    4. There was an external call to freeze the system, exiting this command
    */

    // Add condition for if it is in freeze mode (should be unfrozen before this step above, at definitions of services and publishers)
    // TODO add cojndition of when the command is done interpolating, update the goal reached
    while( (loops_so_far <= total_number_of_loops) && !effort_limit_crossed && !target_reached && !freeze_mode) {
        // ROS: for communication between programs
        ros::spinOnce();

        // Keep virtual attractor at a distance, to pull the end effector
        virtual_attractor.pose = current_pose;
 
        // Move in direction of the task frame when in cutting mode, else move in tool frame
        //TODO update this with new inputs to program
        if(TARGET_DISTANCE > 0){

            virtual_attractor.pose.position.x = current_pose.position.x + delta_trans_vec.x;
            virtual_attractor.pose.position.y = current_pose.position.y + delta_trans_vec.y;
            virtual_attractor.pose.position.z = current_pose.position.z + delta_trans_vec.z;
        }
        // Move in direction of the task frame when in cutting mode, else move in tool frame
        else {
            // May no longer need or want this
            virtual_attractor.pose.position.x = current_pose.position.x - delta_trans_vec.x;
            virtual_attractor.pose.position.y = current_pose.position.y - delta_trans_vec.y;
            virtual_attractor.pose.position.z = current_pose.position.z - delta_trans_vec.z;
        }

        vector_to_goal.x = ending_position.x - current_pose.position.x;
        vector_to_goal.y = ending_position.y - current_pose.position.y;
        vector_to_goal.z = ending_position.z - current_pose.position.z;

        // Dot product, uf the value is above 0, we hit the target movement
        double dot_product = vector_to_goal.x * delta_trans_vec.x + vector_to_goal.y * delta_trans_vec.y + vector_to_goal.z * delta_trans_vec.z;
        
        // If it is past the plane perpendicular to the direction of movement at the goal position, then we have reached the target 
        if(TARGET_DISTANCE < 0){
            target_reached = dot_product >= 0;
        }
        else {
            target_reached = dot_product <= 0;
        }

        // Update the values for the loop condition
        effort_limit_crossed = ((abs(ft_in_robot_frame.torque.x) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.y) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.z) > TORQUE_THRESHOLD) ||
                                 (abs(ft_in_robot_frame.force.x) > FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.y) > NONDIRECTIONAL_FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.z) > NONDIRECTIONAL_FORCE_THRESHOLD));

        loops_so_far = loops_so_far + 1;

        // Debug output
        cout<<"Difference Vector"<<endl<<vector_to_goal<<endl;
        cout<<"Dot Product"<<endl<<dot_product<<endl<<endl;

        // ROS: for communication between programs
        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();

        // Print current position
        // cout<<"Current position: "<<endl<<abs(current_pose.position.x)<<endl;
    }

    // TODO update the end conditions
    // If we've crossed the effort limts, check which is crossed for the status output
    if(effort_limit_crossed) {

        // Print message
        if(abs(ft_in_robot_frame.torque.x) > TORQUE_THRESHOLD){
            cout<<"X Torque threshold crossed"<<endl;
            srv.request.status = "X Torque threshold crossed";
        }
        else if(abs(ft_in_robot_frame.torque.y) > TORQUE_THRESHOLD){
            cout<<"Y Torque threshold crossed"<<endl;
            srv.request.status = "Y Torque threshold crossed";
        }
        else if(abs(ft_in_robot_frame.torque.z) > TORQUE_THRESHOLD){
            cout<<"Z Torque threshold crossed"<<endl;
            srv.request.status = "Z Torque threshold crossed";
        }
        else if(abs(ft_in_robot_frame.force.x) > FORCE_THRESHOLD){
            cout<<"X Force threshold crossed"<<endl;
            srv.request.status = "X Force threshold crossed";
        }
        else if(abs(ft_in_robot_frame.force.y) > NONDIRECTIONAL_FORCE_THRESHOLD){
            cout<<"Y Force threshold crossed"<<endl;
            srv.request.status = "Y Force threshold crossed";
        }
        else if(abs(ft_in_robot_frame.force.z) > NONDIRECTIONAL_FORCE_THRESHOLD){
            cout<<"Z Force threshold crossed"<<endl;
            srv.request.status = "Z Force threshold crossed";
        }
        else{
            cout<<"Effort threshold crossed"<<endl;
            srv.request.status = "Effort threshold crossed";
        }

        // ROS: for communication between programs
        ros::spinOnce();

        // Keep the virtual attractor slightly below the surface, or above if pulling back
        virtual_attractor.pose = current_pose;
        if(TARGET_DISTANCE > 0){
            virtual_attractor.pose.position.x = current_pose.position.x + tool_vector_z.x * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y + tool_vector_z.y * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z + tool_vector_z.z * KEEP_CONTACT_DISTANCE;
        }
        // Pull up in the direction of the tool
        else {
            virtual_attractor.pose.position.x = current_pose.position.x - tool_vector_z.x * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y - tool_vector_z.y * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z - tool_vector_z.z * KEEP_CONTACT_DISTANCE;
        }
        // Wait for some time, then go to a freeze

    }

    // Convert to an else? or rearrange the time out cond

    // If we've reached target position
    if(target_reached) { // (abs(current_pose.position.x) >= target_position && (TARGET_DISTANCE > 0) ) || (abs(current_pose.position.x) < target_position && (TARGET_DISTANCE <= 0) ) 
        // Print message
        cout<<"Target position reached"<<endl;
        srv.request.status = "target position reached";
        // ROS: for communication between programs
        ros::spinOnce();

        // Put the virtual attractor at the end effector
        virtual_attractor.pose = current_pose;
        // go to freeze mode
    }

    //If we've timed out
    if (loops_so_far > total_number_of_loops){
        cout<<"Timed out"<<endl;
        srv.request.status = "Timed out";
        // ROS: for communication between programs
        ros::spinOnce();

        // Put the virtual attractor at the end effector
        virtual_attractor.pose = current_pose;
        // go to freeze mode
    }

    // If the freeze service was called (like a user interrupt)
    // if already in freeze mode (user kill command) send output of why program exited
    if(freeze_mode){
        cout<<"Freeze mode called during loop, ending program."<<endl;
        srv.request.status = "User interrupt";
        // ROS: for communication between programs
        ros::spinOnce();

        // Put the virtual attractor at the end effector
        // virtual_attractor.pose = current_pose; // may be useless here, but setting it anyways just in case

        // go to freeze mode
    }


    // // debug out
    // cout<<"Tool Vec Z"<<endl;
    // cout<<tool_vector_z<<endl;
    // cout<<"Task Vec Z"<<endl;
    // cout<<task_vector_z<<endl<<endl;

    // ROS: for communication between programs
    virtual_attractor_publisher.publish(virtual_attractor);
    naptime.sleep();

    // If we have crossed the effort limit, we want the program to wait for 30 seconds before going to freeze mode, to allow the system to settle
    // if(effort_limit_crossed){
    //     ros::Duration(30).sleep();
    // }
    // if(target_reached){
    //     ros::Duration(5).sleep();
    // }
    
    // Go to freeze mode (if not already in it)

    // ROS: Call service to send reason for program end to buffer.cpp
    if(client.call(srv)){
        // success
        cout<<"Called service_exit with name succesfully"<<endl;
    }
    else{
        // failed to call service
        // ROS_ERROR("Failed to call service status_service");
        naptime.sleep();
    }

    // If we are not in the freeze mode, put the system into freeze mode before we exit
    if(!freeze_mode){
        freeze_client.call(freeze_srv);
    }

    // End of program
}
