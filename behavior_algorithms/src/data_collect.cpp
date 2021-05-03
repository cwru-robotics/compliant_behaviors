// Data Collection
// Rahul Pokharna
// Last updated 6/19/19
// 
// This node is made to collect wrench metrics and runtime information. It can be run for individual skills or for a total task to gather an estimated length and information on
// Min and max wrench, averages, for each aaxis and total force and torque. 
// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cmath>
// #include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <behavior_algorithms/status_service.h>
using namespace std;

// Declare variables
geometry_msgs::Pose current_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::PoseStamped init_attractor;
geometry_msgs::Wrench ft_in_current_frame;

// Globally accessible so that it can be updated at each callback
double force_x_end, force_y_end, force_z_end, force_end, torque_x_end, torque_y_end, torque_z_end, torque_end;


// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
}
void ft_callback(const geometry_msgs::Wrench& ft_values) {
    // These are not values from the sensor. They are f/t values transformed into robot base frame.
    ft_in_current_frame = ft_values;

    force_x_end = ft_in_current_frame.force.x;
    force_y_end = ft_in_current_frame.force.y;
    force_z_end = ft_in_current_frame.force.z;
    torque_x_end = ft_in_current_frame.torque.x;
    torque_y_end = ft_in_current_frame.torque.y;
    torque_z_end = ft_in_current_frame.torque.z;
    
    force_end = sqrt(pow(ft_in_current_frame.force.x,2) + pow(ft_in_current_frame.force.y,2) + pow(ft_in_current_frame.force.z,2));
    torque_end = sqrt(pow(ft_in_current_frame.torque.x,2) + pow(ft_in_current_frame.torque.y,2) + pow(ft_in_current_frame.torque.z,2));
} 
void virtual_attractor_callback(const geometry_msgs::PoseStamped &cartesian_pose_attr)
{
    init_attractor = cartesian_pose_attr;
}

// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"data_collect");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback);
    ros::Subscriber ft_subscriber = nh.subscribe("current_frame_ft_wrench",1,ft_callback);
    ros::Subscriber attr_subscriber = nh.subscribe("tfd_virt_attr", 1, virtual_attractor_callback);
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);


    // Declare constants
    double DT = 0.01;

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    //? Do we want timestamps for these peaks? can be useful for comparison to graphs/bags

    // Define vars for data and metrics Max/Min of each axis force torque
    double force_x_min, force_x_max, force_y_min, force_y_max, force_z_min, force_z_max;
    double torque_x_min, torque_x_max, torque_y_min, torque_y_max, torque_z_min, torque_z_max;
    double force_min, force_max, torque_min, torque_max;

    // values for average
    double force_avg,force_x_avg, force_y_avg, force_z_avg, torque_avg, torque_x_avg, torque_y_avg, torque_z_avg;

    // values for starting and ending wrench
    double force_x_start, force_y_start, force_z_start, torque_x_start, torque_y_start, torque_z_start;

    //! Change data type to be ros::Time? maybe time starts at 0, how to display number fully
    // Define timestamps for each event
    double force_x_min_time, force_x_max_time, force_y_min_time, force_y_max_time, force_z_min_time, force_z_max_time;
    double torque_x_min_time, torque_x_max_time, torque_y_min_time, torque_y_max_time, torque_z_min_time, torque_z_max_time;
    double force_min_time, force_max_time, torque_min_time, torque_max_time;

    // ROS: for communication between programs
    ros::spinOnce();
    naptime.sleep();

    ROS_INFO("waiting for data");


    // ROS: Wait until we have wrench data. Our default position is 0.
    while(current_pose.position.x == 0 || ft_in_current_frame.force.x == 0) {
        ros::spinOnce();
        naptime.sleep();
    }
    ROS_INFO("Collecting data");

    force_x_start = ft_in_current_frame.force.x;
    force_y_start = ft_in_current_frame.force.y;
    force_z_start = ft_in_current_frame.force.z;
    torque_x_start = ft_in_current_frame.torque.x;
    torque_y_start = ft_in_current_frame.torque.y;
    torque_z_start = ft_in_current_frame.torque.z;

    double force_start = sqrt(pow(ft_in_current_frame.force.x,2) + pow(ft_in_current_frame.force.y,2) + pow(ft_in_current_frame.force.z,2));
    double torque_start = sqrt(pow(ft_in_current_frame.torque.x,2) + pow(ft_in_current_frame.torque.y,2) + pow(ft_in_current_frame.torque.z,2));
    
    // used for calcuiilating averages
    int counter = 0;

    // Begin loop
    //TODO add an exit with Phold
    while(ros::ok()) {
        // ROS: For communication
        ros::spinOnce();
        
        // Gather and update data here
        //! FORCE
        if(ft_in_current_frame.force.x > force_x_max){
            // update new value
            force_x_max = ft_in_current_frame.force.x;
            // timestamp
            force_x_max_time = ros::Time::now().toSec();
        }
        else if(ft_in_current_frame.force.x < force_x_min){
            // update new value
            force_x_min = ft_in_current_frame.force.x;
            // timestamp
            force_x_min_time = ros::Time::now().toSec();
        }
        if(ft_in_current_frame.force.y > force_y_max){
            // update new value
            force_y_max = ft_in_current_frame.force.y;
            // timestamp
            force_y_max_time = ros::Time::now().toSec();
        }
        else if(ft_in_current_frame.force.y < force_y_min){
            // update new value
            force_y_min = ft_in_current_frame.force.y;
            // timestamp
            force_y_min_time = ros::Time::now().toSec();
        }
        if(ft_in_current_frame.force.z > force_z_max){
            // update new value
            force_z_max = ft_in_current_frame.force.z;
            // timestamp
            force_z_max_time = ros::Time::now().toSec();
        }
        else if(ft_in_current_frame.force.z < force_z_min){
            // update new value
            force_z_min = ft_in_current_frame.force.z;
            // timestamp
            force_z_min_time = ros::Time::now().toSec();
        }
        //! TORQUE
        if(ft_in_current_frame.torque.x > torque_x_max){
            // update new value
            torque_x_max = ft_in_current_frame.torque.x;
            // timestamp
            torque_x_max_time = ros::Time::now().toSec();
        }
        else if(ft_in_current_frame.torque.x < torque_x_min){
            // update new value
            torque_x_min = ft_in_current_frame.torque.x;
            // timestamp
            torque_x_min_time = ros::Time::now().toSec();
        }
        if(ft_in_current_frame.torque.y > torque_y_max){
            // update new value
            torque_y_max = ft_in_current_frame.torque.y;
            // timestamp
            torque_y_max_time = ros::Time::now().toSec();
        }
        else if(ft_in_current_frame.torque.y < torque_y_min){
            // update new value
            torque_y_min = ft_in_current_frame.torque.y;
            // timestamp
            torque_y_min_time = ros::Time::now().toSec();
        }
        if(ft_in_current_frame.torque.z > torque_z_max){
            // update new value
            torque_z_max = ft_in_current_frame.torque.z;
            // timestamp
            torque_z_max_time = ros::Time::now().toSec();
        }
        else if(ft_in_current_frame.torque.z < torque_z_min){
            // update new value
            torque_z_min = ft_in_current_frame.torque.z;
            // timestamp
            torque_z_min_time = ros::Time::now().toSec();
        }

        // Calculate the magnitude of the force and the torque to compare it to the values we already have

        // calculate magnitude
        double force_magnitude = sqrt(pow(ft_in_current_frame.force.x,2) + pow(ft_in_current_frame.force.y,2) + pow(ft_in_current_frame.force.z,2));
        double torque_magnitude = sqrt(pow(ft_in_current_frame.torque.x,2) + pow(ft_in_current_frame.torque.y,2) + pow(ft_in_current_frame.torque.z,2));

        // compare and update values
        if(force_magnitude > force_max){
            // update new value
            force_max = force_magnitude;
            // timestamp
            force_max_time = ros::Time::now().toSec();
        }
        else if(force_magnitude < force_min){
            // update new value
            force_min = force_magnitude;
            // timestamp
            force_min_time = ros::Time::now().toSec();
        }
        if(torque_magnitude > torque_max){
            // update new value
            torque_max = torque_magnitude;
            // timestamp
            torque_max_time = ros::Time::now().toSec();
        }
        else if(torque_magnitude < torque_min){
            // update new value
            torque_min = torque_magnitude;
            // timestamp
            torque_min_time = ros::Time::now().toSec();
        }

        //! Calculate averages

        force_avg = (force_avg * counter + force_magnitude) / (counter + 1);
        force_x_avg = (force_x_avg * counter + ft_in_current_frame.force.x) / (counter + 1);
        force_y_avg = (force_y_avg * counter + ft_in_current_frame.force.y) / (counter + 1);
        force_z_avg = (force_z_avg * counter + ft_in_current_frame.force.z) / (counter + 1);

        torque_avg = (torque_avg * counter + torque_magnitude) / (counter + 1);
        torque_x_avg = (torque_x_avg * counter + ft_in_current_frame.torque.x) / (counter + 1);
        torque_y_avg = (torque_y_avg * counter + ft_in_current_frame.torque.y) / (counter + 1);
        torque_z_avg = (torque_z_avg * counter + ft_in_current_frame.torque.z) / (counter + 1);

        counter++;

        // Print all info out
        // ROS_INFO("---------------------------------------");
        // ROS_INFO("Average force: %f\n", force_avg);
        // ROS_INFO("Maximum force: %f \nMax at time: %f\n", force_max, force_max_time);
        // ROS_INFO("Minimum force: %f \nMin at time: %f\n", force_min, force_min_time);
        // ROS_INFO("Average Torque: %f\n", torque_avg);
        // ROS_INFO("Maximum torque: %f \nMax at time: %f\n", torque_max, torque_max_time);
        // ROS_INFO("Minimum torque: %f \nMin at time: %f\n", torque_min, torque_min_time);
        // ROS_INFO("----X----");
        // ROS_INFO("Average X force: %f\n", force_x_avg);
        // ROS_INFO("Maximum X force: %f \nMax at time: %f\n", force_x_max, force_x_max_time);
        // ROS_INFO("Minimum X force: %f \nMin at time: %f\n", force_x_min, force_x_min_time);
        // ROS_INFO("Average X Torque: %f\n", torque_x_avg);
        // ROS_INFO("Maximum X torque: %f \nMax at time: %f\n", torque_x_max, torque_x_max_time);
        // ROS_INFO("Minimum X torque: %f \nMin at time: %f\n", torque_x_min, torque_x_min_time);
        // ROS_INFO("----Y----");
        // ROS_INFO("Average Y force: %f\n", force_y_avg);
        // ROS_INFO("Maximum Y force: %f \nMax at time: %f\n", force_y_max, force_y_max_time);
        // ROS_INFO("Minimum Y force: %f \nMin at time: %f\n", force_y_min, force_y_min_time);
        // ROS_INFO("Average Y Torque: %f\n", torque_y_avg);
        // ROS_INFO("Maximum Y torque: %f \nMax at time: %f\n", torque_y_max, torque_y_max_time);
        // ROS_INFO("Minimum Y torque: %f \nMin at time: %f\n", torque_y_min, torque_y_min_time);
        // ROS_INFO("----Z----");
        // ROS_INFO("Average Z force: %f\n", force_z_avg);
        // ROS_INFO("Maximum Z force: %f \nMax at time: %f\n", force_z_max, force_z_max_time);
        // ROS_INFO("Minimum Z force: %f \nMin at time: %f\n", force_z_min, force_z_min_time);
        // ROS_INFO("Average Z Torque: %f\n", torque_z_avg);
        // ROS_INFO("Maximum Z torque: %f \nMax at time: %f\n", torque_z_max, torque_z_max_time);
        // ROS_INFO("Minimum Z torque: %f \nMin at time: %f\n", torque_z_min, torque_z_min_time);


        // // Print estimated time
        // ROS_INFO("Estimated time: %f",(counter * DT));

        naptime.sleep();
    }

    // Print all info out on kill of node   
    cout<<"---------------------------------------"<<endl;
    cout<<"Estimated time: "<<(counter * DT)<<endl;
    cout<<"Average magnitude of force: "<<force_avg<<endl;
    cout<<"Maximum magnitude of force: "<<force_max<<"\tMax at time: "<<force_max_time<<endl;
    cout<<"Minimum magnitude of force: "<<force_min<<"\tMin at time: "<<force_min_time<<endl;
    cout<<"Average magnitude of torque: "<<torque_avg<<endl;
    cout<<"Maximum magnitude of torque: "<<torque_max<<"\tMax at time: "<<torque_max_time<<endl;
    cout<<"Minimum magnitude of torque: "<<torque_min<<"\tMin at time: "<<torque_min_time<<endl;
    cout<<"----X----"<<endl;
    cout<<"Average X force: "<<force_x_avg<<endl;
    cout<<"Maximum X force: "<<force_x_max<<"\tMax at time: "<<force_x_max_time<<endl;
    cout<<"Minimum X force: "<<force_x_min<<"\tMin at time: "<<force_x_min_time<<endl;
    cout<<"Average X torque: "<<torque_x_avg<<endl;
    cout<<"Maximum X torque: "<<torque_x_max<<"\tMax at time: "<<torque_x_max_time<<endl;
    cout<<"Minimum X torque: "<<torque_x_min<<"\tMin at time: "<<torque_x_min_time<<endl;
    cout<<"----Y----"<<endl;
    cout<<"Average Y force: "<<force_y_avg<<endl;
    cout<<"Maximum Y force: "<<force_y_max<<"\tMax at time: "<<force_y_max_time<<endl;
    cout<<"Minimum Y force: "<<force_y_min<<"\tMin at time: "<<force_y_min_time<<endl;
    cout<<"Average Y torque: "<<torque_y_avg<<endl;
    cout<<"Maximum Y torque: "<<torque_y_max<<"\tMax at time: "<<torque_y_max_time<<endl;
    cout<<"Minimum Y torque: "<<torque_y_min<<"\tMin at time: "<<torque_y_min_time<<endl;
    cout<<"----Z----"<<endl;
    cout<<"Average Z force: "<<force_z_avg<<endl;
    cout<<"Maximum Z force: "<<force_z_max<<"\tMax at time: "<<force_z_max_time<<endl;
    cout<<"Minimum Z force: "<<force_z_min<<"\tMin at time: "<<force_z_min_time<<endl;
    cout<<"Average Z torque: "<<torque_z_avg<<endl;
    cout<<"Maximum Z torque: "<<torque_z_max<<"\tMax at time: "<<torque_z_max_time<<endl;
    cout<<"Minimum Z torque: "<<torque_z_min<<"\tMin at time: "<<torque_z_min_time<<endl;
    cout<<"---Starting Wrench---"<<endl;
    cout<<"Starting X Force: "<<force_x_start<<endl;
    cout<<"Starting Y Force: "<<force_y_start<<endl;
    cout<<"Starting Z Force: "<<force_z_start<<endl;
    cout<<"Starting X Torque: "<<torque_x_start<<endl;
    cout<<"Starting Y Torque: "<<torque_y_start<<endl;
    cout<<"Starting Z Torque: "<<torque_z_start<<endl;
    cout<<"---Ending Wrench---"<<endl;
    cout<<"Starting X Force: "<<force_x_end<<endl;
    cout<<"Starting Y Force: "<<force_y_end<<endl;
    cout<<"Starting Z Force: "<<force_z_end<<endl;
    cout<<"Starting X Torque: "<<torque_x_end<<endl;
    cout<<"Starting Y Torque: "<<torque_y_end<<endl;
    cout<<"Starting Z Torque: "<<torque_z_end<<endl;
    cout<<"---Starting and Ending Magnitudes---"<<endl;
    cout<<"Starting force: "<<force_start<<endl;
    cout<<"Starting torque: "<<torque_start<<endl;
    cout<<"Ending force: "<<force_end<<endl;
    cout<<"Ending torque: "<<torque_end<<endl;
    cout<<"Change in Force: "<<(force_end - force_start)<<endl;
    cout<<"Change in Torque: "<<(torque_end - torque_start)<<endl;
    

    //TODO add in starting and ending wrench, magnitude or whatever


    // Print estimated time
    
    // End of program
    cout<<"Done"<<endl;

}