// Torsional Wiggle
// Matthew Haberbusch, Surag Balajepalli, and Rahul Pokharna 
// Last updated 10/15/19
// 
// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64.h>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <behavior_algorithms/status_service.h>
#include <std_msgs/Int8.h>
#include <irb120_accomodation_control/freeze_service.h> // used to toggle the freeze service
using namespace std;

std_msgs::Int8 freeze_mode_status;
bool freeze_mode; // maybe don't define it as having a starting value? 
bool freeze_updated = false;

// Declare variables
geometry_msgs::Pose current_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::Wrench ft_in_current_frame;
// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
}
void ft_callback(const geometry_msgs::Wrench& ft_values) {
    // These are not values from the sensor. They are f/t values transformed into robot base frame.
    ft_in_current_frame = ft_values;
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
    ros::init(argc,argv,"torsional_wiggle_push");
    ros::NodeHandle nh;

    // ROS: Define subscribers and publishers for the program
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("interaction_port_frame",1, cartesian_state_callback); // subscribe to the topic publishing the cartesian state of the end effector
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);                       // subscribe to the force/torque sensor data
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1); // publish the pose of the virtual attractor for the accomodation controller 

    // ROS: Services used in conjunction with buffer.cpp to have delayed program status sent to operator
    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    ros::ServiceClient client_start = nh.serviceClient<behavior_algorithms::status_service>("start_service");

    // subscriber for freeze mode status (used in loop cond)
    ros::Subscriber freeze_mode_sub = nh.subscribe("freeze_mode_topic",1,freeze_status_callback);
    
    // service client for setting and unsetting freeze mode at start of program
    ros::ServiceClient freeze_client = nh.serviceClient<irb120_accomodation_control::freeze_service>("freeze_service");
    
    // ROS: Service status variable for use with buffer.cpp
    behavior_algorithms::status_service srv;
    srv.request.name = "Torsional wiggle push";

    // Declare constants
    /*
    How to tune params:
    MOVE_DISTANCE: The distance between the end effector and the virtual attractor in the axis of push but not the rotation, keep positive as it is a push, increase for greater force, decrease for lesser force
    FORCE_THRESHOLD: The limit at which the program will stop if the force threshold is crossed
    TORQUE_THRESHOLD: The limit at which the program will stop if the torque threshold is crossed
    WIGGLE_RADIUS: Radius of the wiggle, increase for larger virtual circle that the end effector follows
    WIGGLE_RATE: The rate at which the cycles for each wiggle is completed, increase the number to have more wiggles per second
    ROTATE_ANGLE: Increase this to increase the amount of push (up until pi/2 rads) [range of (0, pi/2) ]

    Params not needed to be tuned:
    DT: Loop rate, how fast each iteration of the loop is (most likely not needed to be changed)
    WIGGLE_TIME: How long the program will run before timing out and ending, defined by user input, but has a default value of 5

    */
    double MOVE_DISTANCE = 0.012, DT = 0.01; 
    double WIGGLE_RADIUS = 0.01, WIGGLE_RATE = 1, WIGGLE_TIME = 5;
    double TORQUE_THRESHOLD = 2, FORCE_THRESHOLD = 20; 
    double ROTATE_ANGLE = 0.5;

    // Parameters used in loop
    double current_loop = 0;
    double current_loop_of_state = 0;
    double current_state = 1;
    double STATES = 2;

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs
    ros::spinOnce();
    naptime.sleep();

    // The end effector pose (current_pose) and force torque data (ft_in_current_frame) are global variables.

    // ROS: get parameter from server, passed by command line (if nothing passed in, results in default, which is the final number)
    nh.param("/torsional_wiggle_push/wiggle_time", WIGGLE_TIME, 5.0); 

    // clear parameter from server 
    nh.deleteParam("/torsional_wiggle_push/wiggle_time"); 
    
    ROS_INFO("Output from parameter for runtime; %f", WIGGLE_TIME);

    // With labeled parameter, now call service to send message that program will start
    std::ostringstream request_status; 
    request_status << "runtime " << WIGGLE_TIME << " seconds";

    srv.request.status = request_status.str();

    // ROS: Call the client start service, used in buffer.cpp for operator output
    // if(client_start.call(srv)){
    //     // success
    //     cout<<"Called service_start with name succesfully"<<endl;
    // }
    // else{
    //     // failed to call service
    //     ROS_ERROR("Failed to call service service_start");
    // }

    // Set as unknown in case program somehow progresses past loop without any of the 3 conditions
    srv.request.status = "Unkown";

    // Calculate the total loops needed and loops between states
    double MAX_LOOPS = WIGGLE_TIME / DT;

    // Replaced with constant for set frequency
    double LOOPS_BETWEEN_STATES = 1 / (DT * WIGGLE_RATE * STATES); //MAX_LOOPS / (WIGGLE_RATE * STATES); 
    cout<<"MAX_LOOPS"<<endl<<MAX_LOOPS<<endl;
    cout<<"LOOPS_BETWEEN_STATES"<<endl<<LOOPS_BETWEEN_STATES<<endl;

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.position.x == 0) ros::spinOnce();

    // MATH: Define current pose quaternion
    Eigen::Quaterniond current_pose_quat;

    // Get starting position and turn it into a quaternion
    virtual_attractor.pose = current_pose;
    current_pose_quat.x() = current_pose.orientation.x;
    current_pose_quat.y() = current_pose.orientation.y;
    current_pose_quat.z() = current_pose.orientation.z;
    current_pose_quat.w() = current_pose.orientation.w;
    
    Eigen::Matrix3d current_pose_rot = current_pose_quat.normalized().toRotationMatrix();

    // MATH: Define rotation matrix for moving WRT TOOL FRAME
    Eigen::Matrix3d ROT_MAT_MOVE_RIGHT;
    ROT_MAT_MOVE_RIGHT(0,0) = cos(ROTATE_ANGLE);
    ROT_MAT_MOVE_RIGHT(0,1) = -sin(ROTATE_ANGLE);
    ROT_MAT_MOVE_RIGHT(0,2) = 0;
    ROT_MAT_MOVE_RIGHT(1,0) = sin(ROTATE_ANGLE);
    ROT_MAT_MOVE_RIGHT(1,1) = cos(ROTATE_ANGLE);
    ROT_MAT_MOVE_RIGHT(1,2) = 0;
    ROT_MAT_MOVE_RIGHT(2,0) = 0;
    ROT_MAT_MOVE_RIGHT(2,1) = 0;
    ROT_MAT_MOVE_RIGHT(2,2) = 1;

    Eigen::Matrix3d ROT_MAT_MOVE_LEFT;
    ROT_MAT_MOVE_LEFT(0,0) = cos(-ROTATE_ANGLE);
    ROT_MAT_MOVE_LEFT(0,1) = -sin(-ROTATE_ANGLE);
    ROT_MAT_MOVE_LEFT(0,2) = 0;
    ROT_MAT_MOVE_LEFT(1,0) = sin(-ROTATE_ANGLE);
    ROT_MAT_MOVE_LEFT(1,1) = cos(-ROTATE_ANGLE);
    ROT_MAT_MOVE_LEFT(1,2) = 0;
    ROT_MAT_MOVE_LEFT(2,0) = 0;
    ROT_MAT_MOVE_LEFT(2,1) = 0;
    ROT_MAT_MOVE_LEFT(2,2) = 1;

    //! we want to unfreeze here before the loop
   // spin, while we don't have data
    while(!freeze_updated){
        ros::spinOnce();
        naptime.sleep();
    }
    irb120_accomodation_control::freeze_service freeze_srv;
    
    cout<<freeze_mode_status<<endl;
    cout<<freeze_srv.response<<endl;

    
    // Check here after a spin, and unfreeze if it is frozen //! uncomment after checking with current_frame based math
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

    // Keep virtual attractor at a distance, to push the end effector
    virtual_attractor.pose = current_pose;
    Eigen::Matrix3d new_virtual_attractor_rot;

    // Loop variable to check effort limit condition
    bool effort_limit_crossed = false;
    effort_limit_crossed = ((abs(ft_in_current_frame.torque.x) > TORQUE_THRESHOLD) || (abs(ft_in_current_frame.torque.y) > TORQUE_THRESHOLD) || (abs(ft_in_current_frame.torque.z) > TORQUE_THRESHOLD) ||
                                 (abs(ft_in_current_frame.force.x) > FORCE_THRESHOLD) || (abs(ft_in_current_frame.force.y) > FORCE_THRESHOLD) || (abs(ft_in_current_frame.force.z) > FORCE_THRESHOLD));

    // Start loop
    /*
    Loop End Conditions:
    1. The operation has timed out (ran the max alloted time)
    2. One of the effort thresholds has been crossed
    */
    while(current_loop <= MAX_LOOPS && !effort_limit_crossed && !freeze_mode){
        ros::spinOnce();
        
        // IN HERE, INSTEAD OF CHECKING STATES, CHECK IF THE LIMIT WAS CROSSED, IF IT WAS THEN REVERSE THE STATE, KEEP DOING THAT UNTIL TIME RUNS OUT AND REMOVE THE CONDITION FROM ABOVE 
        // If we spent enough time in one state, move to the next
        if(current_loop_of_state >= LOOPS_BETWEEN_STATES){
            if(current_state < STATES){
                current_state = current_state + 1;
                current_loop_of_state = 0;
            }
            else{
                current_state = 1;
                current_loop_of_state = 0;
            }
        }

        // Calculate the current position
        virtual_attractor.pose = current_pose;
        current_pose_quat.x() = current_pose.orientation.x;
        current_pose_quat.y() = current_pose.orientation.y;
        current_pose_quat.z() = current_pose.orientation.z;
        current_pose_quat.w() = current_pose.orientation.w;

        // Keep the robot pressing down
        virtual_attractor.pose.position.x = current_pose.position.x;
        virtual_attractor.pose.position.y = current_pose.position.y;
        virtual_attractor.pose.position.z = current_pose.position.z + MOVE_DISTANCE;

        // Convert current pose quaternion to Euler Angles
        current_pose_rot = current_pose_quat.normalized().toRotationMatrix();

        // Rotate right
        if (current_state == 1){

            // Do rotation
            new_virtual_attractor_rot = current_pose_rot * ROT_MAT_MOVE_RIGHT;
        }
        // Rotate left
        else if (current_state == 2){
            

            // Do rotation
            new_virtual_attractor_rot = current_pose_rot * ROT_MAT_MOVE_LEFT;

        }
        // Convert new orientation to quaternion
        Eigen::Quaterniond new_virtual_attractor_quat(new_virtual_attractor_rot);
        virtual_attractor.pose.orientation.x = new_virtual_attractor_quat.x();
        virtual_attractor.pose.orientation.y = new_virtual_attractor_quat.y();
        virtual_attractor.pose.orientation.z = new_virtual_attractor_quat.z();
        virtual_attractor.pose.orientation.w = new_virtual_attractor_quat.w();

        // Check if the effort limit is crossed
        effort_limit_crossed = ((abs(ft_in_current_frame.torque.x) > TORQUE_THRESHOLD) || (abs(ft_in_current_frame.torque.y) > TORQUE_THRESHOLD) || (abs(ft_in_current_frame.torque.z) > TORQUE_THRESHOLD) ||
                                 (abs(ft_in_current_frame.force.x) > FORCE_THRESHOLD) || (abs(ft_in_current_frame.force.y) > FORCE_THRESHOLD) || (abs(ft_in_current_frame.force.z) > FORCE_THRESHOLD));

        // Increment counters
        current_loop = current_loop + 1;
        current_loop_of_state = current_loop_of_state + 1;
        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();
    }

    // Set final attractor pose to current ee position
    // virtual_attractor.pose = current_pose;
    virtual_attractor_publisher.publish(virtual_attractor);
    naptime.sleep();

    srv.request.status = "Completed run of specified length";

    // If we've crossed the effort limts, check which is crossed for the status output
    if(effort_limit_crossed){
        // Print message
        if(abs(ft_in_current_frame.torque.x) > TORQUE_THRESHOLD){
            cout<<"X Torque threshold crossed"<<endl;
            srv.request.status = "X Torque threshold crossed";
        }
        else if(abs(ft_in_current_frame.torque.y) > TORQUE_THRESHOLD){
            cout<<"Y Torque threshold crossed"<<endl;
            srv.request.status = "Y Torque threshold crossed";
        }
        else if(abs(ft_in_current_frame.torque.z) > TORQUE_THRESHOLD){
            cout<<"Z Torque threshold crossed"<<endl;
            srv.request.status = "Z Torque threshold crossed";
        }
        else if(abs(ft_in_current_frame.force.x) > FORCE_THRESHOLD){
            cout<<"X Force threshold crossed"<<endl;
            srv.request.status = "X Force threshold crossed";
        }
        else if(abs(ft_in_current_frame.force.y) > FORCE_THRESHOLD){
            cout<<"Y Force threshold crossed"<<endl;
            srv.request.status = "Y Force threshold crossed";
        }
        else if(abs(ft_in_current_frame.force.z) > FORCE_THRESHOLD){
            cout<<"Z Force threshold crossed"<<endl;
            srv.request.status = "Z Force threshold crossed";
        }
        else{
            cout<<"Effort threshold crossed"<<endl;
            srv.request.status = "Effort threshold crossed";
        }
    }

    // If we are not in the freeze mode, put the system into freeze mode before we exit
    if(!freeze_mode){
        srv.request.status = "Completed run of specified length";
        freeze_client.call(freeze_srv);
        cout<<"Completed run of "<<WIGGLE_TIME<<" seconds"<<endl;
    }else{
        cout<<"Freeze mode called during loop"<<endl;
    }

    
    // ROS: Call service to send reason for program end to buffer.cpp
    // if(client.call(srv)){
    //     // success
    //     cout<<"Called service with name succesfully"<<endl;
    // }
    // else{
    //     // failed to call service
    //     ROS_ERROR("Failed to call service status_service");
    // }

    cout<<"End"<<endl;

    // End of program
}