// Force Moment Accommodation (FMA)
// Rahul Pokharna and Matthew Haberbusch and Surag Balajepalli 
// Last updated 6/19/19
// 
// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64.h>
#include <behavior_algorithms/status_service.h>
using namespace std;

// Declare variables
geometry_msgs::Pose current_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::PoseStamped init_attractor;
geometry_msgs::Wrench ft_in_robot_frame;

// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
}
void ft_callback(const geometry_msgs::Wrench& ft_values) {
    // These are not values from the sensor. They are f/t values transformed into robot base frame.
    ft_in_robot_frame = ft_values;
} 
void virtual_attractor_callback(const geometry_msgs::PoseStamped &cartesian_pose_attr)
{
    init_attractor = cartesian_pose_attr;
}

// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"restore_wrench_eq");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback);
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);
    ros::Subscriber attr_subscriber = nh.subscribe("tfd_virt_attr", 1, virtual_attractor_callback);
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);

    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    ros::ServiceClient client_start = nh.serviceClient<behavior_algorithms::status_service>("start_service");
    behavior_algorithms::status_service srv;
    srv.request.name = "RWE";

    // Declare constants
    double DT = 0.01, RUN_TIME = 1;

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs
    ros::spinOnce();
    naptime.sleep();


    // Which axes will be preserved, which will be ignored (true means relieve loads in that axis, false means keep that axis of the attractor in place)
    bool trans_x, trans_y, trans_z, rot_x, rot_y, rot_z;

    // Replace the runtime with a value from parameters
    // ros::NodeHandle n("~"); // refer to the global path to the local parameters

    // get parameter from server, passed by command line (if nothing passed in, results in default)
    // find topic of the param pushed
    nh.param("/restore_wrench_eq/run_time", RUN_TIME, 3.0);

    nh.param("/restore_wrench_eq/trans_x", trans_x, true);
    nh.param("/restore_wrench_eq/trans_y", trans_y, true);
    nh.param("/restore_wrench_eq/trans_z", trans_z, true);
    nh.param("/restore_wrench_eq/rot_x", rot_x, true);
    nh.param("/restore_wrench_eq/rot_y", rot_y, true);
    nh.param("/restore_wrench_eq/rot_z", rot_z, true);

    // clear parameter from server
    nh.deleteParam("/restore_wrench_eq/trans_x");
    nh.deleteParam("/restore_wrench_eq/trans_y");
    nh.deleteParam("/restore_wrench_eq/trans_z");
    nh.deleteParam("/restore_wrench_eq/rot_x");
    nh.deleteParam("/restore_wrench_eq/rot_y");
    nh.deleteParam("/restore_wrench_eq/rot_z");

    // clear parameter from server
    nh.deleteParam("/restore_wrench_eq/run_time");

    ROS_INFO("Output from parameter for runtime; %f", RUN_TIME);

    // With labeled parameter, now call service to send message that program will start
    std::ostringstream request_status; 
    request_status << "runtime " << RUN_TIME << " seconds";

    srv.request.status = request_status.str();
    // ROS_WARN("Request Status: %s", request_status.str().c_str());

    if(client_start.call(srv)){
        // success
        cout<<"Called service_start with name succesfully"<<endl;
    }
    else{
        // failed to call service
        ROS_ERROR("Failed to call service service_start");
    }

    // Set as unknown in case program somehow progresses past loop without any of the 3 conditions
    srv.request.status = "Unkown";

    double total_number_of_loops = RUN_TIME / DT;
    double loops_so_far = 0;

    // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.position.x == 0 && init_attractor.pose.position.x == 0) ros::spinOnce();
    
    // Set the virtual attractor pose to the initial attractor pose, set once and published until the end of the command
    if(trans_x && trans_y && trans_z && rot_x && rot_y && rot_z){
        virtual_attractor.pose = current_pose;
    }
    else if(!trans_x && !trans_y && !trans_z && !rot_x && !rot_y && !rot_z){
        virtual_attractor.pose = init_attractor.pose;
    }
    else{

        virtual_attractor.pose = init_attractor.pose;

        // Check each axis parameter, if the flag is true, then reset that axis
        //! This is also wrong, we need tool frame axes, and change it to be in the correct frame
        if(trans_x){
            virtual_attractor.pose.position.x = current_pose.position.x;
        }
        if(trans_y){
            virtual_attractor.pose.position.y= current_pose.position.y;
        }
        if(trans_z){
            virtual_attractor.pose.position.z= current_pose.position.z;
        }
        //! How do set the rotation to maintain load in only specified rotational axes?  current is incorrect

        // Break down the rotation into rotations around the 3 axes, and then choose which ones to rotate
        // Convert the rotations for each axis into rotation matrices for each axis from the quaternion
        // Then select which rotations we want, whether that is the rotation of the attractor or the current pose
        // Multiply them all together, confirm with newman
        
        /*
        if(rot_x){
            virtual_attractor.pose.orientation.x = current_pose.orientation.x;
        }
        if(rot_y){
            virtual_attractor.pose.orientation.x = current_pose.orientation.y;
        }
        if(rot_z){
            virtual_attractor.pose.orientation.x = current_pose.orientation.z;
        }
        */
    }

    // Begin loop
    //TODO add an exit with Phold
    while(loops_so_far <= total_number_of_loops) {
        // ROS: For communication
        ros::spinOnce();

        // Old method, update attractor to current pose
        // virtual_attractor.pose = current_pose;

        // Add to time counter
        loops_so_far = loops_so_far + 1;
        virtual_attractor.header.stamp = ros::Time::now();

        // ROS: For communication between programs
        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();
    }
    srv.request.status = "Completed run of specified length";

    if(client.call(srv)){
        // success
        cout<<"Called service with name succesfully"<<endl;
    }
    else{
        // failed to call service
        ROS_ERROR("Failed to call service status_service");
    }
    
    // End of program
    cout<<"Done"<<endl;

}