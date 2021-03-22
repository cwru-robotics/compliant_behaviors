// Force Moment Accommodation (FMA)
// Matthew Haberbusch and Surag Balajepalli 
// Last updated 6/19/19
// 
// All ROS-specific code labeled with "ROS:" comments

//TODO List
//* Call set frame
//* store the k matrices
//* have access to the current frame wrench
//* update wrench, then calculate bumpless on this modified wrench

// ROS: include libraries
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <behavior_algorithms/status_service.h>
#include <irb120_accomodation_control/freeze_service.h> // used to toggle the freeze service
#include <irb120_accomodation_control/set_current_frame.h> // set current frame service message to send task name and receive the frame name and k matrix
#include <irb120_accomodation_control/matrix_msg.h> // store k matrix from service call, convert to eigen
using namespace std;

// Declare variables
geometry_msgs::Pose current_pose;
geometry_msgs::Pose current_ft_pose;
geometry_msgs::Pose interaction_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::Wrench ft_in_current_frame;

std_msgs::Int8 freeze_mode_status;
bool freeze_mode; // maybe don't define it as having a starting value? 
bool freeze_updated = false;

// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
}
// ROS: callback functions for how we receive data
void ft_frame_callback(const geometry_msgs::PoseStamped& ft_frame) {
    current_ft_pose = ft_frame.pose;
}
void interaction_frame_callback(const geometry_msgs::PoseStamped& interaction_frame_from_sub) {
    interaction_pose = interaction_frame_from_sub.pose;
}
void ft_callback(const geometry_msgs::Wrench& ft_values) {
    ft_in_current_frame = ft_values;
}
void freeze_status_callback(const std_msgs::Int8& freeze_status_msg) {
    freeze_updated = true;
    freeze_mode_status = freeze_status_msg;
    // cout<<freeze_mode_status<<endl;
    if(freeze_mode_status.data == 1) freeze_mode = true;
    else freeze_mode = false;
}

// MATH: Function converts a vector of euler angles into a rotation matrix
Eigen::Matrix3d rotation_matrix_from_vector_of_angles(Eigen::Vector3d angle_vector) {
	double angle = angle_vector.norm(); // Calculate the length of the vector (the angle)
	Eigen::Vector3d axis = angle_vector / angle; // Divide out this length to get us the unit length vector for the axis of rotation
	Eigen::Matrix3d rotation_matrix_from_vector_of_angles;
	Eigen::AngleAxisd angle_axis(angle, axis);
	rotation_matrix_from_vector_of_angles = angle_axis.toRotationMatrix();
	return rotation_matrix_from_vector_of_angles;
}


// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"force_moment_accommodation");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback); // Needs to be changed to be interaction port, for now subscribe to both FT and EE
    ros::Subscriber ft_frame_sub = nh.subscribe("ft_frame",1,ft_frame_callback); 
    ros::Subscriber interaction_frame_sub = nh.subscribe("interaction_port_frame",1,interaction_frame_callback);  
    ros::Subscriber ft_subscriber = nh.subscribe("current_frame_ft_wrench",1,ft_callback);
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);

    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    ros::ServiceClient client_start = nh.serviceClient<behavior_algorithms::status_service>("start_service");
    ros::ServiceClient client_set_frame = nh.serviceClient<irb120_accomodation_control::set_current_frame>("set_frame_service");

    // subscriber for freeze mode status (used in loop cond)
    ros::Subscriber freeze_mode_sub = nh.subscribe("freeze_mode_topic",1,freeze_status_callback);
    
    // service client for setting and unsetting freeze mode at start of program
    ros::ServiceClient freeze_client = nh.serviceClient<irb120_accomodation_control::freeze_service>("freeze_service");

    behavior_algorithms::status_service srv;
    srv.request.name = "FMA";

    // Declare constants
    double DT = 0.01, RUN_TIME = 1;

    // Values required for the selection matrix, 0 means alleviate load, 1 means maintain
    double trans_x,trans_y,trans_z,rot_x,rot_y,rot_z;
    string param_set = "Task";

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs
    ros::spinOnce();
    naptime.sleep();

    // Calculate the run time
    // cout << "Enter a run time in seconds: ";
    // cin >> RUN_TIME;

    // Replace the runtime with a value from parameters
    // ros::NodeHandle n("~"); // refer to the global path to the local parameters

    // get parameter from server, passed by command line (if nothing passed in, results in default)
    // find topic of the param pushed
    nh.param("/force_moment_accommodation/run_time", RUN_TIME, 3.0); 
    nh.param("/force_moment_accommodation/trans_x", trans_x, 0.0);
    nh.param("/force_moment_accommodation/trans_y", trans_y, 0.0);
    nh.param("/force_moment_accommodation/trans_z", trans_z, 0.0);
    nh.param("/force_moment_accommodation/rot_x", rot_x, 0.0);
    nh.param("/force_moment_accommodation/rot_y", rot_y, 0.0);
    nh.param("/force_moment_accommodation/rot_z", rot_z, 0.0);
    //! Figure out params for the selection matrix
    //! param needed for task name for setting frame
    
    nh.param<std::string>("/force_moment_accommodation/param_set", param_set, "Task");

    // clear parameter from server 
    nh.deleteParam("/force_moment_accommodation/trans_x"); 
    nh.deleteParam("/force_moment_accommodation/trans_y"); 
    nh.deleteParam("/force_moment_accommodation/trans_z"); 
    nh.deleteParam("/force_moment_accommodation/rot_x"); 
    nh.deleteParam("/force_moment_accommodation/rot_y"); 
    nh.deleteParam("/force_moment_accommodation/rot_z"); 
    nh.deleteParam("/force_moment_accommodation/param_set"); 

    
    // clear parameter from server 
    nh.deleteParam("/force_moment_accommodation/run_time"); 
    
    ROS_INFO("Output RWE in interaction from parameter for runtime; %f", RUN_TIME);

    // With labeled parameter, now call service to send message that program will start
    std::ostringstream request_status; 
    request_status << "runtime " << RUN_TIME << " seconds";

    srv.request.status = request_status.str();
    // ROS_WARN("Request Status: %s", request_status.str().c_str());

    // if(client_start.call(srv)){
    //     // success
    //     cout<<"Called service_start with name succesfully"<<endl;
    // }
    // else{
    //     // failed to call service
    //     ROS_ERROR("Failed to call service service_start");
    // }

    // Set as unknown in case program somehow progresses past loop without any of the 3 conditions

    //! Define our selection matrix based on input
    Eigen::VectorXd selection_mat(6); // use .asDiagonal on vector to get it as a matrix, stored as a vector
    selection_mat<<trans_x,trans_y,trans_z,rot_x,rot_y,rot_z; // 1 means keep wrench, 0 means relieve axis, values obtained from input

    srv.request.status = "Unkown";

    double total_number_of_loops = RUN_TIME / DT;
    double loops_so_far = 0;

    

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.position.x == 0) ros::spinOnce();

    //! call the set frame service
    //TODO call service set_frame_service
    irb120_accomodation_control::set_current_frame set_frame_srv;
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

    // Define our K matrix
    Eigen::Vector3d k_trans, k_rot;
    k_trans(0) = k_mat.trans_mat.x;
    k_trans(1) = k_mat.trans_mat.y;
    k_trans(2) = k_mat.trans_mat.z;
    k_rot(0) = k_mat.rot_mat.x;
    k_rot(1) = k_mat.rot_mat.y;
    k_rot(2) = k_mat.rot_mat.z;

    //? Do we want a check for whether the current frame was set properly, how do we want the RWE to place the attractor? 
    // Current functionality to be just place it in whatever frame is active, might add a check for if frame didnt set (check response.status)

    // Update our values (may want to spin more than once?)
    for(int i = 0; i < 10; i++){
        naptime.sleep();
        ros::spinOnce();
    }


    //! Calculate the pose of the attractor based on selection matrix (input) k matrix (from service call) and wrench (subscribed)
    //!  we want to make sure that this works
    //TODO Code for attractor and tool pose in current_frame the bumpless attr pose calculated here

    // Move wrench into an eigen for better math
    Eigen::VectorXd wrench_with_respect_to_current(6);
    wrench_with_respect_to_current(0) = ft_in_current_frame.force.x;
    wrench_with_respect_to_current(1) = ft_in_current_frame.force.y;
    wrench_with_respect_to_current(2) = ft_in_current_frame.force.z;
    wrench_with_respect_to_current(3) = ft_in_current_frame.torque.x;
    wrench_with_respect_to_current(4) = ft_in_current_frame.torque.y;
    wrench_with_respect_to_current(5) = ft_in_current_frame.torque.z;

    //! use selection matrix to modify wrench: DOUBLE CHECK 
    wrench_with_respect_to_current = selection_mat.asDiagonal() * wrench_with_respect_to_current;

    // Define the virtual position based on the wrench on the FT sensor in the current frame, with the interaction port defined in the current frame (convert from )
    Eigen::Vector3d bumpless_virtual_attractor_position = -(k_trans.asDiagonal().inverse() * wrench_with_respect_to_current.head(3)); 
    // Then add this offset onto the current pose of the interaction port
    bumpless_virtual_attractor_position(0) += interaction_pose.position.x;
    bumpless_virtual_attractor_position(1) += interaction_pose.position.y;
    bumpless_virtual_attractor_position(2) += interaction_pose.position.z;


    // Calculate the orientation of the attractor based on the felt wrench, and then add on the orientation of the ft sensor
    // We get the offset of the attractor required, and then we can conver it to a rotation matrix, and then add that rotation on to the current orientation of the IP in the current frame
    Eigen::Vector3d bumpless_virtual_attractor_angles = -(k_rot.asDiagonal().inverse() * wrench_with_respect_to_current.tail(3));
    
    /// Define rotation matrix of the interaction port 
    Eigen::Matrix3d interaction_rot = Eigen::Quaterniond(interaction_pose.orientation.w,interaction_pose.orientation.x,interaction_pose.orientation.y,interaction_pose.orientation.z).toRotationMatrix();
    // Convert bumpless attr to rot matrix (define a func here, vec of angles to AA, then AA to rot)
    Eigen::Matrix3d virtual_attractor_rotation_matrix = interaction_rot * rotation_matrix_from_vector_of_angles(bumpless_virtual_attractor_angles); //! not done yet
    // Take this rot mat, and then post multiply it to the current ft rotation matrix in the appropriate frame (IP, maybe define a new Affine for it?)
    
    //! change to be interaction port
    // Put the virtual attractor at the end effector
    // virtual_attractor.pose = current_pose; // If we are using the tooltip
    virtual_attractor.pose.position.x = bumpless_virtual_attractor_position(0);
    virtual_attractor.pose.position.y = bumpless_virtual_attractor_position(1);
    virtual_attractor.pose.position.z = bumpless_virtual_attractor_position(2);
    Eigen::Quaterniond virtual_quat(virtual_attractor_rotation_matrix);
    virtual_attractor.pose.orientation.w = virtual_quat.w();
    virtual_attractor.pose.orientation.x = virtual_quat.x();
    virtual_attractor.pose.orientation.y = virtual_quat.y();
    virtual_attractor.pose.orientation.z = virtual_quat.z();

    // virtual_attractor.pose = interaction_pose; // if we are using the interaction port 
    virtual_attractor.header.frame_id = "current_frame";


   //! we want to unfreeze here before the loop
   // spin, while we don't have data
    while(!freeze_updated){
        ros::spinOnce();
        naptime.sleep();
    }
    irb120_accomodation_control::freeze_service freeze_srv;
    
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
    
    // Begin loop
    while(loops_so_far <= total_number_of_loops) { //! add freeze mode exit cond?
        // ROS: For communication
        ros::spinOnce();

        virtual_attractor.header.stamp = ros::Time::now();
        // Add to time counter
        loops_so_far = loops_so_far + 1;
         
        // ROS: For communication between programs
        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();
    }
    srv.request.status = "Completed run of specified length";

    // if(client.call(srv)){
    //     // success
    //     cout<<"Called service with name succesfully"<<endl;
    // }
    // else{
    //     // failed to call service
    //     ROS_ERROR("Failed to call service status_service");
    // }

    // If we are not in the freeze mode, put the system into freeze mode before we exit
    if(!freeze_mode){
        freeze_client.call(freeze_srv);
    }
    
    // End of program
    cout<<"Done"<<endl;

}