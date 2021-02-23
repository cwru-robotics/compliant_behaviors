// Force Moment Accommodation (FMA)
// Matthew Haberbusch and Surag Balajepalli 
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
geometry_msgs::Pose current_ft_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::Wrench ft_in_robot_frame;

// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
}
// ROS: callback functions for how we receive data
void ft_frame_callback(const geometry_msgs::PoseStamped& ft_frame) {
    current_ft_pose = ft_frame.pose;
}
void ft_callback(const geometry_msgs::Wrench& ft_values) {
    // These are not values from the sensor. They are f/t values transformed into robot base frame.
    ft_in_robot_frame = ft_values;
}

// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"force_moment_accommodation");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback); // Needs to be changed to be interaction port, for now subscribe to both FT and EE
    ros::Subscriber ft_frame_sub = nh.subscribe("ft_frame",1,ft_frame_callback); 
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);

    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    ros::ServiceClient client_start = nh.serviceClient<behavior_algorithms::status_service>("start_service");
    behavior_algorithms::status_service srv;
    srv.request.name = "FMA";

    // Declare constants
    double DT = 0.01, RUN_TIME = 1;

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

    
    // clear parameter from server 
    nh.deleteParam("/force_moment_accommodation/run_time"); 
    
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
    while(current_pose.position.x == 0) ros::spinOnce();
    
    // Begin loop
    while(loops_so_far <= total_number_of_loops) {
        // ROS: For communication
        ros::spinOnce();

        //! change to be interaction port
        // Put the virtual attractor at the end effector
        // virtual_attractor.pose = current_pose; // If we are using the tooltip
        virtual_attractor.pose = current_ft_pose; // if we are using the Ft 

        // Add to time counter
        loops_so_far = loops_so_far + 1;
         
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