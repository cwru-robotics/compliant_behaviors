// Cartesian Point to Point move to command the attractor movement in a Pose Target, Wrench Limit
// Rahul Pokharna 

// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <iostream>
#include <cmath>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <behavior_algorithms/status_service.h>
#include <irb120_accomodation_control/freeze_service.h>
#include <irb120_accomodation_control/set_frame.h>
#include <irb120_accomodation_control/set_current_frame.h>
#include <irb120_accomodation_control/matrix_msg.h>
using namespace std;

// Declare variables
geometry_msgs::Pose current_pose;
geometry_msgs::Pose ft_frame;
geometry_msgs::Pose interaction_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::Wrench ft_in_robot_frame; //TODO use for bumpless

geometry_msgs::PoseStamped task_frame;
geometry_msgs::PoseStamped stowage_frame;
geometry_msgs::Wrench ft_in_sensor_frame; 
// Eigen::VectorXd ft_in_sensor_frame = Eigen::VectorXd::Zero(6);

geometry_msgs::Vector3 tool_vector_x;
geometry_msgs::Vector3 tool_vector_y;
geometry_msgs::Vector3 tool_vector_z;
geometry_msgs::Vector3 task_vector_x;
geometry_msgs::Vector3 task_vector_y;
geometry_msgs::Vector3 task_vector_z;
geometry_msgs::Vector3 stowage_vector_x;
geometry_msgs::Vector3 stowage_vector_y;
geometry_msgs::Vector3 stowage_vector_z;
geometry_msgs::Vector3 ft_vector_x;
geometry_msgs::Vector3 ft_vector_y;
geometry_msgs::Vector3 ft_vector_z;

std_msgs::Int8 freeze_mode_status;
bool freeze_mode; // maybe don't define it as having a starting value? 
bool freeze_updated = false;

// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
}

//TODO For use for bumpless start
void ft_callback(const geometry_msgs::Wrench& ft_values) {
    // These are not values from the sensor. They are f/t values transformed into robot base frame.
    ft_in_robot_frame = ft_values;
}

// For use for exit wrench conditions
void ft_sensor_callback(const geometry_msgs::WrenchStamped &ft_values){
    ft_in_sensor_frame = ft_values.wrench;

    /* To add this for smoother values? 
    wrench_body_coords_(0) = std::round(ft_values.wrench.force.x * 10) / 10;
	wrench_body_coords_(1) = std::round(ft_values.wrench.force.y * 10) / 10;
	wrench_body_coords_(2) = std::round(ft_values.wrench.force.z * 10) / 10;
	wrench_body_coords_(3) = std::round(ft_values.wrench.torque.x * 10) / 10;
	wrench_body_coords_(4) = std::round(ft_values.wrench.torque.y * 10) / 10;
	wrench_body_coords_(5) = std::round(ft_values.wrench.torque.z * 10) / 10;
    */
}

void tool_vector_x_callback(const geometry_msgs::Vector3& tool_vector_msg_x) {
    tool_vector_x = tool_vector_msg_x;
}

void tool_vector_y_callback(const geometry_msgs::Vector3& tool_vector_msg_y) {
    tool_vector_y = tool_vector_msg_y;
}

void tool_vector_z_callback(const geometry_msgs::Vector3& tool_vector_msg_z) {
    tool_vector_z = tool_vector_msg_z;
}

void task_vector_x_callback(const geometry_msgs::Vector3& task_vector_msg_x) {
    task_vector_x = task_vector_msg_x;
}

void task_vector_y_callback(const geometry_msgs::Vector3& task_vector_msg_y) {
    task_vector_y = task_vector_msg_y;
}

void task_vector_z_callback(const geometry_msgs::Vector3& task_vector_msg_z) {
    task_vector_z = task_vector_msg_z;
}

void task_frame_callback(const geometry_msgs::PoseStamped& task_frame_from_sub) {
    task_frame = task_frame_from_sub;
}

void stowage_vector_x_callback(const geometry_msgs::Vector3& stowage_vector_msg_x) {
    stowage_vector_x = stowage_vector_msg_x;
}

void stowage_vector_y_callback(const geometry_msgs::Vector3& stowage_vector_msg_y) {
    stowage_vector_y = stowage_vector_msg_y;
}

void stowage_vector_z_callback(const geometry_msgs::Vector3& stowage_vector_msg_z) {
    stowage_vector_z = stowage_vector_msg_z;
}

void stowage_frame_callback(const geometry_msgs::PoseStamped& stowage_frame_from_sub) {
    stowage_frame = stowage_frame_from_sub;
}

void ft_vector_x_callback(const geometry_msgs::Vector3& ft_vector_msg_x) {
    ft_vector_x = ft_vector_msg_x;
}

void ft_vector_y_callback(const geometry_msgs::Vector3& ft_vector_msg_y) {
    ft_vector_y = ft_vector_msg_y;
}

void ft_vector_z_callback(const geometry_msgs::Vector3& ft_vector_msg_z) {
    ft_vector_z = ft_vector_msg_z;
}

void ft_frame_callback(const geometry_msgs::PoseStamped& ft_frame_from_sub) {
    ft_frame = ft_frame_from_sub.pose;
}

void interaction_frame_callback(const geometry_msgs::PoseStamped& interaction_frame_from_sub) {
    interaction_pose = interaction_frame_from_sub.pose;
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
    ros::init(argc,argv,"CartP2PTWL");
    ros::NodeHandle nh;

    // ROS: Define subscribers and publishers used
    //! Can just change transformed FT wrench to be normal ft wrench!!!!!!(only created in acc controller, we should read FT in tool frame, not robot base)

    //! We want both the TF FT, and the Tool FT
    //TODO Subscribe to current bumpless start, and just use that as the start pose? or set to be FT frame if bumpless is false
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback); // subscribe to the topic publishing the cartesian state of the end effector
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);                       // subscribe to the force/torque sensor data
    ros::Subscriber ft_sensor_sub = nh.subscribe("robotiq_ft_wrench", 1, ft_sensor_callback);                  // ROS: Subscribe to the force-torque sensor wrench
    ros::Subscriber tool_vector_sub_x = nh.subscribe("tool_vector_x", 1, tool_vector_x_callback);                // subscribe to the value of the tool vector in the z, published from the accomodation controller
    ros::Subscriber tool_vector_sub_y = nh.subscribe("tool_vector_y", 1, tool_vector_y_callback);                // subscribe to the value of the tool vector in the z, published from the accomodation controller
    ros::Subscriber tool_vector_sub_z = nh.subscribe("tool_vector_z", 1, tool_vector_z_callback);                // subscribe to the value of the tool vector in the z, published from the accomodation controller
    ros::Subscriber task_vector_sub_x = nh.subscribe("task_vector_x",1,task_vector_x_callback);                  // subscribe to the value of the task vector in the z, published from the accomodation controller
    ros::Subscriber task_vector_sub_y = nh.subscribe("task_vector_y",1,task_vector_y_callback);                  // subscribe to the value of the task vector in the z, published from the accomodation controller
    ros::Subscriber task_vector_sub_z = nh.subscribe("task_vector_z",1,task_vector_z_callback);                  // subscribe to the value of the task vector in the z, published from the accomodation controller
    ros::Subscriber task_frame_sub = nh.subscribe("task_frame",1,task_frame_callback);                         // subscribe to the task frame published by the accomodation controller
    ros::Subscriber stowage_vector_sub_x = nh.subscribe("stowage_vector_x",1,stowage_vector_x_callback);                  // subscribe to the value of the stowage vector in the z, published from the accomodation controller
    ros::Subscriber stowage_vector_sub_y = nh.subscribe("stowage_vector_y",1,stowage_vector_y_callback);                  // subscribe to the value of the stowage vector in the z, published from the accomodation controller
    ros::Subscriber stowage_vector_sub_z = nh.subscribe("stowage_vector_z",1,stowage_vector_z_callback);                  // subscribe to the value of the stowage vector in the z, published from the accomodation controller
    ros::Subscriber stowage_frame_sub = nh.subscribe("stowage_frame",1,stowage_frame_callback);                         // subscribe to the stowage frame published by the accomodation controller
    ros::Subscriber ft_vector_sub_x = nh.subscribe("ft_vector_x",1,ft_vector_x_callback);                  // subscribe to the value of the ft vector in the z, published from the accomodation controller
    ros::Subscriber ft_vector_sub_y = nh.subscribe("ft_vector_y",1,ft_vector_y_callback);                  // subscribe to the value of the ft vector in the z, published from the accomodation controller
    ros::Subscriber ft_vector_sub_z = nh.subscribe("ft_vector_z",1,ft_vector_z_callback);                  // subscribe to the value of the ft vector in the z, published from the accomodation controller
    ros::Subscriber ft_frame_sub = nh.subscribe("ft_frame",1,ft_frame_callback);                         // subscribe to the ft frame published by the accomodation controller
    ros::Subscriber interaction_frame_sub = nh.subscribe("interaction_port_frame",1,interaction_frame_callback);                         // subscribe to the ft frame published by the accomodation controller
    
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1); // publish the pose of the virtual attractor for the accomodation controller 

    // ROS: Services used in conjunction with buffer.cpp to have delayed program status sent to operator
    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    ros::ServiceClient client_start = nh.serviceClient<behavior_algorithms::status_service>("start_service");
    ros::ServiceClient client_set_frame = nh.serviceClient<irb120_accomodation_control::set_current_frame>("set_frame_service");
    
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

    irb120_accomodation_control::set_current_frame set_frame_srv;
    
    ROS_INFO("after setup");
    /*
    How to tune params:
    PULL_DISTANCE: By increasing this, the virtual force is greater, and decreasing decreases it
    KEEP_CONTACT_DISTANCE: If the effort threshold is crossed, the virtual attractor is placed this distance below the current pose to keep contact if pushing, and above if pulling to keep a constant force
    FORCE_THRESHOLD: The limit at which the program will stop if the force in the direction which we are moving is crossed
    NONDIRECTIONAL_FORCE_THRESHOLD: The limit at which the program will stop if the force in the direction which we are not moving is crossed
    TORQUE_THRESHOLD: The limit at which the program will stop if the torque threshold is crossed
    SETTLE_TIME: How long the program will run before timing out and ending after completing interpolation
    
    Params not needed to be tuned:
    DT: Loop rate, how fast each iteration of the loop is (most likely not needed to be changed)
    TARGET_DISTANCE: will change with user input, how far and in what direction the end effector moves

    PULL_DISTANCE and FORCE_THRESHOLD: Default values defined here, but are modified below depending on GUI setting (Line 98)
    MAX_VEL: The maximum velocity for translation (Used for cartP2P) in m/s
    MAX_ANG: The maximum angular velocity for rotation (Used for cartP2P) in rad/s
    */
    double DT = 0.01, FORCE_THRESHOLD = 12, ROTATION_ERROR_THRESHOLD = 0.03; 
    double KEEP_CUTTING_DISTANCE = 0, MAX_VEL = 0.007, MAX_ANG = 0.1, TRANSLATION_ERROR_THRESHOLD = 0.002; // slowed it down and made the check more lenient
    //! These names may want to change
    double x = 0, y = 0, z = 0;
    double psi = 0, theta = 0, phi = 0; 
    double TORQUE_THRESHOLD = 2;
    double SETTLE_TIME = 5;

    // Parameter for if in the cutting state, easier checking later
    bool cutting = false;

    // Parameter if we are in the task frame or not
    bool task = false;

    // Parameter if we are in the stowage frame or not
    bool stowage = false;
    
    // Parameter if we are bumpless or not
    bool bumpless = false;

    // Parameter for if we are using the FT or the Tool Pose
    bool interaction_port_is_ft = true;

    // Variable for which set of parameters to use
    string param_set = "Peg";

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs


    // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.
    ROS_INFO("Before Reading params");
    // ROS: Get parameter passed in 
    nh.param("/CartP2PTWL/trans_x", x, 0.0);
    nh.param("/CartP2PTWL/trans_y", y, 0.0);
    nh.param("/CartP2PTWL/trans_z", z, 0.0);
    nh.param("/CartP2PTWL/rot_x", psi, 0.0);
    nh.param("/CartP2PTWL/rot_y", theta, 0.0);
    nh.param("/CartP2PTWL/rot_z", phi, 0.0);
    nh.param("/CartP2PTWL/bumpless", bumpless, false); //TODO fix and make sure this works

    cout<<"Bumpless status: "<<bumpless<<endl; //! add a user interrupt/pause here to read and check this
    
    nh.param<std::string>("/CartP2PTWL/param_set", param_set, "Tool");

    // clear parameter from server 
    nh.deleteParam("/CartP2PTWL/trans_x"); 
    nh.deleteParam("/CartP2PTWL/trans_y"); 
    nh.deleteParam("/CartP2PTWL/trans_z"); 
    nh.deleteParam("/CartP2PTWL/rot_x"); 
    nh.deleteParam("/CartP2PTWL/rot_y"); 
    nh.deleteParam("/CartP2PTWL/rot_z"); 
    nh.deleteParam("/CartP2PTWL/bumpless"); 

    nh.deleteParam("/CartP2PTWL/param_set");

    ROS_INFO("after reading and deleting params");
    // Here, the values for PULL_DISTANCE and FORCE_THRESHOLD are changed according to what setting the GUI is on for the appropriate task
    if(!strcmp(param_set.c_str(), "Peg")){
        // set the new values here
        FORCE_THRESHOLD = 30;
        TORQUE_THRESHOLD = 4;
        KEEP_CUTTING_DISTANCE = 0;
        SETTLE_TIME = 5;
        
        stowage = false;
        cutting = false;
        task = false;
        ROS_INFO("Params set for PEG");
    }
    else if (!strcmp(param_set.c_str(), "Bottle_Cap")){
        // set the other values here
        FORCE_THRESHOLD = 15;
        TORQUE_THRESHOLD = 2;
        KEEP_CUTTING_DISTANCE = 0;
        SETTLE_TIME = 5;

        stowage = false;
        cutting = false;
        task = false;
        ROS_INFO("Params set for BOTTLE_CAP");
    }
    else if (!strcmp(param_set.c_str(), "Tool")){
        // set the other values here
        FORCE_THRESHOLD = 25; // 25
        TORQUE_THRESHOLD = 4;
        KEEP_CUTTING_DISTANCE = 0;
        SETTLE_TIME = 5;

        stowage = false;
        cutting = false;
        task = false;
        ROS_INFO("Params set for TOOL");
    }
    else if (!strcmp(param_set.c_str(), "Cutting")){
        // set the other values here
        FORCE_THRESHOLD = 4;
        TORQUE_THRESHOLD = 2;
        KEEP_CUTTING_DISTANCE = 0.001; // was 0.001
        SETTLE_TIME = 5;

        stowage = true; // stowage true, task false?
        cutting = true;
        task = false;
        ROS_INFO("Params set for CUTTING");
    }
    else if (!strcmp(param_set.c_str(), "Task")){
        // set the other values here
        FORCE_THRESHOLD = 30;
        TORQUE_THRESHOLD = 4;
        KEEP_CUTTING_DISTANCE = 0; 
        SETTLE_TIME = 5;

        stowage = false;
        cutting = false;
        task = true;
        ROS_INFO("Params set for TASK");
    }
    else if (!strcmp(param_set.c_str(), "Stowage")){
        // set the other values here
        FORCE_THRESHOLD = 25;
        TORQUE_THRESHOLD = 2.5;
        KEEP_CUTTING_DISTANCE = 0; 
        SETTLE_TIME = 5;

        stowage = true;
        cutting = false;
        task = false;
        ROS_INFO("Params set for STOWAGE");
    }
    else if (!strcmp(param_set.c_str(), "Deep_Drive")){
        // set the other values here
        FORCE_THRESHOLD = 30;
        TORQUE_THRESHOLD = 2; // lower this so we turn slow and stop 
        KEEP_CUTTING_DISTANCE = 0; 
        SETTLE_TIME = 5;
        MAX_ANG = 0.02; // this is reduced to slowly turn onto the grooves


        stowage = false;
        cutting = false;
        task = true;
        ROS_INFO("Params set for STOWAGE");
    }

    //TODO ADD PARAM FOR TOOL EXCHANGE
    ROS_INFO("after storing params");

    //TODO call service set_frame_service
    ROS_INFO_STREAM(param_set.c_str());
    set_frame_srv.request.task_name = param_set.c_str();
    // cout<<set_frame_srv<<endl;
    // Define our new kmatrix to store value from the set frame service, used for the bumpless start
    irb120_accomodation_control::matrix_msg k_mat;
    ROS_INFO("Before calling set frame service");
    if(client_set_frame.call(set_frame_srv)){
        k_mat = set_frame_srv.response.K_mat;
        cout<<set_frame_srv.response.status<<endl<<"Current frame: "<<set_frame_srv.response.updated_frame<<endl;
        ROS_INFO_STREAM(set_frame_srv.response.updated_frame);
    }
    ROS_INFO("After calling set frame service");

    // Update our values (may want to spin more than once?)
    for(int i = 0; i < 10; i++){
        naptime.sleep();
        ros::spinOnce();
    }
    

    // With labeled parameter, now call service to send message that program will start
    //TODO update this message
    std::ostringstream request_status; 
    // request_status << "target_distance " << TARGET_DISTANCE << "m";

    srv.request.status = request_status.str();

    // ROS: Call the client start service, used in buffer.cpp for operator output
    // if(client_start.call(srv)){
    //     // success
    //     cout<<"Called buffer service_start with name succesfully"<<endl;
    // }
    // else{
    //     // failed to call service
    //     // ROS_ERROR("Failed to call service service_start");
    //     naptime.sleep();
    // }

    // Set as unknown in case program somehow progresses past loop without any of the 3 conditions
    srv.request.status = "Unkown";

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.position.x == 0 || tool_vector_z.x == 0 || task_vector_z.x == 0 || interaction_pose.position.x == 0) ros::spinOnce();

    //! We want to use the bumpless start, not just the ft pose, we can either subscribe to the bumpless started pose of the virtual attractor (when we toggle we can get the pose of the attractor? or sub to it)

    //TODO change all to be in the FT frame
    geometry_msgs::Vector3 start_position;
    start_position.x = interaction_pose.position.x;
    start_position.y = interaction_pose.position.y;
    start_position.z = interaction_pose.position.z;
    //TODO Subscribe to the TFed wrench and utilize it for the initial attractor position

    Eigen::Quaterniond start_pose_quat;
    start_pose_quat.x() = interaction_pose.orientation.x;
    start_pose_quat.y() = interaction_pose.orientation.y;
    start_pose_quat.z() = interaction_pose.orientation.z;
    start_pose_quat.w() = interaction_pose.orientation.w;

    Eigen::Quaterniond interaction_pose_quat;
    interaction_pose_quat.x() = interaction_pose.orientation.x;
    interaction_pose_quat.y() = interaction_pose.orientation.y;
    interaction_pose_quat.z() = interaction_pose.orientation.z;
    interaction_pose_quat.w() = interaction_pose.orientation.w;
    
    // Convert the geometry quaternion to a quaternion
    Eigen::Quaterniond task_frame_quat;
    task_frame_quat.x() = task_frame.pose.orientation.x;
    task_frame_quat.y() = task_frame.pose.orientation.y;
    task_frame_quat.z() = task_frame.pose.orientation.z;
    task_frame_quat.w() = task_frame.pose.orientation.w;

    // Convert the geometry quaternion to a quaternion
    Eigen::Quaterniond stowage_frame_quat;
    stowage_frame_quat.x() = stowage_frame.pose.orientation.x;
    stowage_frame_quat.y() = stowage_frame.pose.orientation.y;
    stowage_frame_quat.z() = stowage_frame.pose.orientation.z;
    stowage_frame_quat.w() = stowage_frame.pose.orientation.w;

    //TODO Task is set to false, need to update to use task frame with cartp2p, other values are also removed here
    // task = false;
    // KEEP_CUTTING_DISTANCE = 0;

    // Update to accept input of goal pose here, and then add the deltas 
    geometry_msgs::Vector3 delta_trans_vec;
    // In the current frame, we translate x, y, z directly 
    delta_trans_vec.x = x;
    delta_trans_vec.y = y;
    delta_trans_vec.z = z;
    cout<<"Input Delta Vector"<<endl<<delta_trans_vec<<endl;

    //TODO Calculate the end pose here based on the input, take the relative change and add it in the tool frame,
    //TODO if we use interactive markers in rviz, use that to get ending pos

    //TODO Make sure that the values for the inputs are vectors for translation in the correct frame
    //? i.e. multiply x trans by x tool frame vec, and so on to get the translation in the right frame
    // take the vector, multiply it, and then add all 3 together for a new delta trans vec, and then that is the end
    // 
    //! Populate and update end position and rotation stuff, either callback to the Interactive Markers
    // Change this value to 
    geometry_msgs::Vector3 ending_position; 
    ending_position.x = start_position.x + delta_trans_vec.x;
    ending_position.y = start_position.y + delta_trans_vec.y;
    ending_position.z = start_position.z + delta_trans_vec.z;

    
    // Take the delta input from the gui to rotate.
    // MATH: Define the rotation matrix to the destination
    //! Angles are x: psi, y: theta, z: phi
    Eigen::Matrix3d TO_DESTINATION_ROTATION_MATRIX; 
    TO_DESTINATION_ROTATION_MATRIX(0,0) = cos(theta) * cos(phi);
    TO_DESTINATION_ROTATION_MATRIX(0,1) = (sin(psi) * sin(theta) * cos(phi)) - (cos(psi) * sin(phi));
    TO_DESTINATION_ROTATION_MATRIX(0,2) = (cos(psi) * sin(theta) * cos(phi)) + (sin(psi) * sin(phi)); 
    TO_DESTINATION_ROTATION_MATRIX(1,0) = cos(theta) * sin(phi);
    TO_DESTINATION_ROTATION_MATRIX(1,1) = (sin(psi) * sin(theta) * sin(phi)) + (cos(psi) * cos(phi));
    TO_DESTINATION_ROTATION_MATRIX(1,2) = (cos(psi) * sin(theta) * sin(phi)) - (sin(psi) * cos(phi)); 
    TO_DESTINATION_ROTATION_MATRIX(2,0) = -sin(theta);
    TO_DESTINATION_ROTATION_MATRIX(2,1) = sin(psi) * cos(theta);
    TO_DESTINATION_ROTATION_MATRIX(2,2) = cos(psi) * cos(theta);

    ROS_INFO_STREAM(TO_DESTINATION_ROTATION_MATRIX);
    // ROTATION
    Eigen::Vector3d k_rot_axis;

    Eigen::Matrix3d R_start, R_end, R_change, R_change_interp, R_interp;
    R_start = start_pose_quat.normalized().toRotationMatrix(); // current tool pose, convert to rotation

    //! Below is used for rotations in the task frame
    // Task frame will not update during a skill (if using the GUI to interact, if using other commands outside of gui, other issues may arise)
    Eigen::Matrix3d task_frame_rot = task_frame_quat.normalized().toRotationMatrix();
    // MATH: Get rotation matrix from the task frame to the current pose
    Eigen::Matrix3d tool_in_task = task_frame_rot.inverse() * R_start;
    Eigen::Matrix3d goal_pose_wrt_task; // maybe not needed

    //! Below is used for rotations in the stowage frame
    // stowage frame will not update during a skill (if using the GUI to interact, if using other commands outside of gui, other issues may arise)
    Eigen::Matrix3d stowage_frame_rot = stowage_frame_quat.normalized().toRotationMatrix();
    // MATH: Get rotation matrix from the stowage frame to the current pose
    Eigen::Matrix3d tool_in_stowage = stowage_frame_rot.inverse() * R_start;
    Eigen::Matrix3d goal_pose_wrt_stowage; // maybe not needed

    // Initial R_interp is the starting rotation, each delta added:
    R_interp = R_start;

   

    // Calculate the ending rotation matrix based on the current orientation times the desired rotation
    R_end = R_start * TO_DESTINATION_ROTATION_MATRIX;

    //? Below line is used if we have the end pose given to us, here we do not and thus must calculate it above
    // R_end = end_pose_quat.normalized().toRotationMatrix(); // end pose, convert to rotation 


    Eigen::Quaterniond end_pose_quat(R_end);
    //R_end = R_change*R_start //! This is reversed compared to above
    R_change = TO_DESTINATION_ROTATION_MATRIX; 
    Eigen::AngleAxisd angleAxis(R_change); //convert rotation matrix to angle/axis

    //interpolate with angle theta_interp about k_rot_axis, which is updated in main method, dtheta is the increment value
    double angle_axis_theta, theta_interp, dtheta;
    angle_axis_theta = angleAxis.angle();
    k_rot_axis = angleAxis.axis();


    // take theta, split it into the smaller sections, nsteps and dtheta and then use the max rotational velocity to get nsteps, 
    // then take the max steps from trans and then recalcuate the deltas


    // TRANSLATION

    // Current translation code
    geometry_msgs::Vector3 vector_to_goal;
    vector_to_goal.x = ending_position.x - interaction_pose.position.x;
    vector_to_goal.y = ending_position.y - interaction_pose.position.y;
    vector_to_goal.z = ending_position.z - interaction_pose.position.z;

    // Calculate the distance we want to travel, then find the number of steps we need to do to be below the max velocity (param)
    double dist = sqrt(pow(vector_to_goal.x,2) + pow(vector_to_goal.y,2) + pow(vector_to_goal.z,2));
    
    // Check how many steps we want to have (largest number of steps so we are slowest, to be safe)
    // Here we do the math to check how many steps we wish to use, and what the value of dtheta will be, how many loops we will be doing
    int n_steps, ntrans, nrot;
    
    // Calculate the max number of steps for translation and rotation, take the max of those steps, set that as the runtime steps
    ntrans = (dist / MAX_VEL) / DT;
    nrot = (angle_axis_theta / MAX_ANG) / DT;

    // Get the larger of the two, now recalculate the deltas
    n_steps = ntrans < nrot ? nrot : ntrans;

    // Length of the run
    //TODO send this information to be printed in some service? 
    double run_length = n_steps * DT;
    cout<<"Runtime Length: "<<run_length<<endl;
    // ROS_INFO("Runtime Length: %f", run_length); // might want to do a cout
    cout<<"Number of Loops: "<<n_steps<<endl;
    // ROS_INFO("Number of Loops: %i", n_steps); // might want to do a cout

    // Scale the translation steps into the smaller sections to be added in each time step
    delta_trans_vec.x = vector_to_goal.x / n_steps;
    delta_trans_vec.y = vector_to_goal.y / n_steps;
    delta_trans_vec.z = vector_to_goal.z / n_steps;

    // The delta rotation for each step of the interpolation
    dtheta = angle_axis_theta / n_steps;
    theta_interp = 0.0;
    R_change_interp = Eigen::AngleAxisd(theta_interp, k_rot_axis);

    //TODO Update goal reached exit condition to include orientation
    // Dot product, uf the value is above 0, we hit the target movement
    double goal_dist = sqrt(pow(vector_to_goal.x,2) + pow(vector_to_goal.y,2) + pow(vector_to_goal.z,2));
    // If it is past the plane perpendicular to the direction of movement at the goal position, then we have reached the position
    // If the rotation is within some amount, then we have reached the rotation, and if both are true, then we have reached the target 
    bool target_reached, pos_reached, rot_reached;

    pos_reached = goal_dist <= TRANSLATION_ERROR_THRESHOLD;
    rot_reached = interaction_pose_quat.isApprox(end_pose_quat, ROTATION_ERROR_THRESHOLD);

    target_reached = pos_reached && rot_reached;

    // Debug output
    cout<<"Difference Vector"<<endl<<vector_to_goal<<endl;
    cout<<"Distance to Goal"<<endl<<goal_dist<<endl<<endl;
    // cout<<"Current Orientation "<<endl<<current_pose_quat<<endl;
    // cout<<"Goal Orientation "<<endl<<end_pose_quat<<endl<<endl;

    // Output for whether we are frozen or not
    cout<<"Freeze status: "<<freeze_mode<<endl<<"Freeze message: "<<freeze_mode_status<<endl<<endl;

    int temple = 0;
    // ROS_INFO("Before loop");
    // cout << "Before loop (enter any number to continue): ";
    // cin >> temple;
    // ROS_INFO(freeze_mode_status);

    // Loop variable to check effort limit condition
    bool effort_limit_crossed = false;
    //! change to be ft in current frame? 
    effort_limit_crossed = ((abs(ft_in_sensor_frame.torque.x) > TORQUE_THRESHOLD) || (abs(ft_in_sensor_frame.torque.y) > TORQUE_THRESHOLD) || (abs(ft_in_sensor_frame.torque.z) > TORQUE_THRESHOLD) ||
                                 (abs(ft_in_sensor_frame.force.x) > FORCE_THRESHOLD) || (abs(ft_in_sensor_frame.force.y) > FORCE_THRESHOLD) || (abs(ft_in_sensor_frame.force.z) > FORCE_THRESHOLD));


    //TODO Bumpless start here, after making sure we have not crossed the effort limit already, confirm how we want to do this
    virtual_attractor.pose = interaction_pose; //! CHANGE TO BE FT
    virtual_attractor.header.frame_id = "current_frame";

    //! In in cutting, pull down to keep contact
    if(cutting){ //TODO fix this
        cout<<"Adding initial pull down distance"<<endl;
        virtual_attractor.pose.position.z = virtual_attractor.pose.position.z + KEEP_CUTTING_DISTANCE;
    } //!FIX THIS


    // spin, while we don't have data
    while(!freeze_updated){
        ros::spinOnce();
        naptime.sleep();
    }
    
    cout<<freeze_mode_status<<endl;
    cout<<freeze_srv.response<<endl;

    
    // Check here after a spin, and unfreeze if it is frozen //! uncomment after checking new cartp2ptwl with current_frame based math
    while(freeze_mode_status.data == 1 && freeze_mode){
        
        if(freeze_updated && freeze_client.call(freeze_srv)){
            // success
            cout<<"Called freeze mode service succesfully"<<endl;
        }
        else{
            // failed to call service
            ROS_ERROR("Failed to call freeze service");
        }
        ros::spinOnce();
        naptime.sleep();
        freeze_updated = false;

    }
    
    // spin, while we don't have data
    while(!freeze_updated){
        ros::spinOnce();
        naptime.sleep();
    }


    cout<<freeze_mode_status<<endl;
    cout<<freeze_srv.response<<endl;

    // should now be unfrozen

    // Used in the loop to determine the run time and time out additional length to allow it to settle in the main function
    double total_number_of_loops = n_steps + (SETTLE_TIME / DT);
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
    while( (loops_so_far < total_number_of_loops) && !effort_limit_crossed && !target_reached && !freeze_mode) {
        // ROS: for communication between programs
        ros::spinOnce();
        cout<<"Loops so far: "<<loops_so_far<<endl<<"n_steps: "<<n_steps<<endl<<"total_number_of_loops: "<<total_number_of_loops<<endl;



        // only if we are below the number of interpolation steps do we want to adjust the attractor, otherwise we will just check the goal checks
        if(loops_so_far < n_steps){
            // Add translation interpolation (maybe want to instead add the advancement to the starting pose, rather than the end)
            virtual_attractor.pose.position.x = virtual_attractor.pose.position.x + delta_trans_vec.x;
            virtual_attractor.pose.position.y = virtual_attractor.pose.position.y + delta_trans_vec.y;
            virtual_attractor.pose.position.z = virtual_attractor.pose.position.z + delta_trans_vec.z;

            

            // Advance rotation interpolation
            theta_interp = (loops_so_far + 1) * dtheta; //! This is wrong, needs to be transformed properly (maybe be based off of start, or change the initial TF)
            // R_change_interp = Eigen::AngleAxisd(theta_interp, k_rot_axis);
            
            // R_interp = R_change_interp * R_start; // maybe swap this

            // R_interp = R_start * R_change_interp; // Swapped to match initial end check, rotation in tool frame not base
            
            // New method, add to previous interpolated rotation
            R_change_interp = Eigen::AngleAxisd(dtheta, k_rot_axis);
            R_interp = R_interp * R_change_interp; // where R_interp is the previous rotation matrix in base frame, maybe change dtheta to theta interp, r_interp to r_start

            // Update the rotation of the virtual attractor
            Eigen::Quaterniond new_virtual_attractor_quat(R_interp);
            virtual_attractor.pose.orientation.x = new_virtual_attractor_quat.x();
            virtual_attractor.pose.orientation.y = new_virtual_attractor_quat.y();
            virtual_attractor.pose.orientation.z = new_virtual_attractor_quat.z();
            virtual_attractor.pose.orientation.w = new_virtual_attractor_quat.w();
        }
        else if (loops_so_far >= n_steps){
            cout<<"Interpolation complete"<<endl;
            ROS_INFO_STREAM("Interpolation complete");
        }
        
        // Update values for goal checks 
        interaction_pose_quat.x() = interaction_pose.orientation.x;
        interaction_pose_quat.y() = interaction_pose.orientation.y;
        interaction_pose_quat.z() = interaction_pose.orientation.z;
        interaction_pose_quat.w() = interaction_pose.orientation.w;

        // Update vector to goal for target reached check
        vector_to_goal.x = ending_position.x - interaction_pose.position.x;
        vector_to_goal.y = ending_position.y - interaction_pose.position.y;
        vector_to_goal.z = ending_position.z - interaction_pose.position.z;

        // Check rotation

        // Dot product, if the value is above 0, we hit the target movement
        goal_dist = sqrt(pow(vector_to_goal.x,2) + pow(vector_to_goal.y,2) + pow(vector_to_goal.z,2));

        //TODO Update to match earlier check
        // If it is past the plane perpendicular to the direction of movement at the goal position, then we have reached the target 
        // pos_reached = dot_product <= 0; //! Check the signs here
        pos_reached = goal_dist <= TRANSLATION_ERROR_THRESHOLD;
        rot_reached = interaction_pose_quat.isApprox(end_pose_quat, ROTATION_ERROR_THRESHOLD);

        target_reached = pos_reached && rot_reached;

        // Update the values for the loop condition
        //TODO update these to be in sensor frame? 
        effort_limit_crossed = ((abs(ft_in_sensor_frame.torque.x) > TORQUE_THRESHOLD) || (abs(ft_in_sensor_frame.torque.y) > TORQUE_THRESHOLD) || (abs(ft_in_sensor_frame.torque.z) > TORQUE_THRESHOLD) ||
                                 (abs(ft_in_sensor_frame.force.x) > FORCE_THRESHOLD) || (abs(ft_in_sensor_frame.force.y) > FORCE_THRESHOLD) || (abs(ft_in_sensor_frame.force.z) > FORCE_THRESHOLD));

        loops_so_far = loops_so_far + 1;

        // Debug output
        cout<<"Difference Vector"<<endl<<vector_to_goal<<endl;
        cout<<"Distance to Goal"<<endl<<goal_dist<<endl<<endl;

        // ROS: for communication between programs
        virtual_attractor.header.stamp = ros::Time::now();
        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();

        // Print current position
        // cout<<"Current position: "<<endl<<abs(interaction_pose.position.x)<<endl;
    }

    // TODO update the end conditions
    // If we've crossed the effort limts, check which is crossed for the status output
    //TODO change to be the task or stowage frame (or tool)
    if(effort_limit_crossed) {

        // Print message
        if (abs(ft_in_sensor_frame.torque.x) > TORQUE_THRESHOLD)
        {
            cout<<"X Torque threshold crossed"<<endl;
            srv.request.status = "X Torque threshold crossed";
        }
        else if (abs(ft_in_sensor_frame.torque.y) > TORQUE_THRESHOLD)
        {
            cout<<"Y Torque threshold crossed"<<endl;
            srv.request.status = "Y Torque threshold crossed";
        }
        else if (abs(ft_in_sensor_frame.torque.z) > TORQUE_THRESHOLD)
        {
            cout<<"Z Torque threshold crossed"<<endl;
            srv.request.status = "Z Torque threshold crossed";
        }
        else if (abs(ft_in_sensor_frame.force.x) > FORCE_THRESHOLD)
        {
            cout<<"X Force threshold crossed"<<endl;
            srv.request.status = "X Force threshold crossed";
        }
        else if (abs(ft_in_sensor_frame.force.y) > FORCE_THRESHOLD)
        {
            cout<<"Y Force threshold crossed"<<endl;
            srv.request.status = "Y Force threshold crossed";
        }
        else if (abs(ft_in_sensor_frame.force.z) > FORCE_THRESHOLD)
        {
            cout<<"Z Force threshold crossed"<<endl;
            srv.request.status = "Z Force threshold crossed";
        }
        else{
            cout<<"Effort threshold crossed"<<endl;
            srv.request.status = "Effort threshold crossed";
        }

        // ROS: for communication between programs
        ros::spinOnce();

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
        // virtual_attractor.pose = current_pose;
        // go to freeze mode
    }

    //If we've timed out
    //TODO add a goal check
    if (loops_so_far > n_steps){
        srv.request.status = "Timed out";
        // ROS: for communication between programs
        ros::spinOnce();

        // we want to sleep here in compliance for some time
        cout<<"Interpolation completed"<<endl;

    }

    // If the freeze service was called (like a user interrupt)
    // if already in freeze mode (user kill command) send output of why program exited
    if(freeze_mode){
        cout<<"Freeze mode called during loop, ending program."<<endl;
        srv.request.status = "User interrupt";
        // ROS: for communication between programs
        ros::spinOnce();

    }


    // ROS: for communication between programs
    ros::spinOnce();
    virtual_attractor_publisher.publish(virtual_attractor);
    naptime.sleep();

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
