
// This file will send interpolated joint state commands, and should not be used in contact, only in free space
// Rahul Pokharna, Quan Nguyen

// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <irb120_accomodation_control/freeze_service.h>
#include <cmath>
#include <ros/ros.h>
using namespace std;

// Define Variables we will use in this program
Eigen::VectorXd joint_states_goal_ = Eigen::VectorXd::Zero(6); // command input value
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6); // input value
sensor_msgs::JointState desired_joint_state; // our commanded value/output

// boolean used for user interrupt
bool freeze = false;

// bool jnt_cmd = true, cart_cmd = false; // which method of commands we will use
bool jnt_states_update = false; // while we have not gotten data, wait until we do
vector<double> zero_vec{0,0,0,0,0,0};

// Math: convert a vector from radian to degree
Eigen::VectorXd rad2deg_vect(Eigen::VectorXd input_vect) {
	// vector<double> output_vect;
	Eigen::VectorXd output_vect = input_vect * 180.0 / M_PI;
	// for(int i = 0; i < input_vect.size(); i++) {
	// 	output_vect.push_back(input_vect[i] * 180.0 / M_PI);
	// }

	return output_vect;
}

// Math: convert a vector from degree to radian
Eigen::VectorXd deg2rad_vect(Eigen::VectorXd input_vect) {
    Eigen::VectorXd output_vect = input_vect * M_PI / 180.0;
	// for(int i = 0; i < input_vect.size(); i++) {
	// 	output_vect.push_back(input_vect[i] * M_PI / 180.0);
	// }
	return output_vect;
}


// ROS: Callback function receives the joint state from "abb120_joint_state" topic IN RADIANS
void jointStateCallback(const sensor_msgs::JointState& joint_state) {
	jnt_states_update = true;
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state.position[i] ;
    // can convert joint states to degrees here (for readability)
}

// ROS: Used as a user interrupt for joint comands
bool freezeServiceCallback(irb120_accomodation_control::freeze_serviceRequest &request, irb120_accomodation_control::freeze_serviceResponse &response) {
	// Toggle freezemode/interrupt interpolation
	freeze = true;
	return true;
}

// ROS: main fnc to compute the transition
int main(int argc, char** argv) {
	ros::init(argc, argv, "simple_joint_commander");
	ros::NodeHandle nh;
	ros::Subscriber joint_states_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback); // ROS: Subscribe to our robot's joint states
	ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1); // TODO CHANGE FOR ROBOT AGNOSTIC
	// ros::Subscriber cartesian_command_sub = nh.subscribe<geometry_msgs::Pose>("abb_cartesian_command",1,&CartesianCmdCb);
	// ros::Subscriber cartesian_twist_sub = nh.subscribe<geometry_msgs::Twist>("abb_twist_command",1,&TwistCmdCb);

    // ROS: Service to toggle freeze mode
	ros::ServiceServer freeze_service = nh.advertiseService("joint_freeze_service",freezeServiceCallback);
    
    // ROS: Define our joint state
    desired_joint_state.position.resize(6);
	desired_joint_state.velocity.resize(6);
	
    // Define our delta poses
    Eigen::VectorXd desired_joint_velocity = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd des_jnt_pos = Eigen::VectorXd::Zero(6);

	// Define constants
	double dt_ = 0.01;
	double MAX_JOINT_VELOCITY_NORM = 0.05; //? may want to be smaller, was at 10
    double GOAL_REACHED_ERR = 0.001; // Within a 3 degree total joint error
    

    // Define our goal pose in degrees:
    joint_states_goal_(0) = 0;
    joint_states_goal_(1) = 0;
    joint_states_goal_(2) = 0;
    joint_states_goal_(3) = 0;
    joint_states_goal_(4) = 0;
    joint_states_goal_(5) = 0;
    
    // get params for end pose (in degree) from cmd input with default values
    nh.param("/simple_joint_commander/joint_1", joint_states_goal_(0), 0.0);
    nh.param("/simple_joint_commander/joint_2", joint_states_goal_(1), -48.0);
    nh.param("/simple_joint_commander/joint_3", joint_states_goal_(2), 16.0);
    nh.param("/simple_joint_commander/joint_4", joint_states_goal_(3), 0.0);
    nh.param("/simple_joint_commander/joint_5", joint_states_goal_(4), 30.0);
    nh.param("/simple_joint_commander/joint_6", joint_states_goal_(5), 0.0);


    // clear parameters from server
    nh.deleteParam("/simple_joint_commander/joint_1");
    nh.deleteParam("/simple_joint_commander/joint_2");
    nh.deleteParam("/simple_joint_commander/joint_3");
    nh.deleteParam("/simple_joint_commander/joint_4");
    nh.deleteParam("/simple_joint_commander/joint_5");
    nh.deleteParam("/simple_joint_commander/joint_6");
    
    
    //! Hardcode a value for the goal
    // joint_states_goal_ << 0,-48,16,0,30,0;
    // joint_states_goal_ << 0,-57,25,0,30,0;
    // joint_states_goal_ << 0,-57,25,0,-60,0;
    // joint_states_goal_ << 0,-37,5,0,-60,0;


    // ROS_INFO_STREAM(joint_states_goal_);
    /*
    // Command line user input
    for(int i = 0; i < 6; i++){
        double var;
        cout <<endl<< "Input values for joint "<<(i+1) <<": "<<endl;
        cin >> joint_states_goal_(i);
    }
    */

    /*
    0,-53,24,0,-3,0; // Current EGM pose
    0,-37,5,0,-60,0; // New tool exchange to quick connect valve (starting here)
    0,-48,16,0,30,0; // Tool stowage with quick connect
    0,-48,24,0,22,0; // Pencil and knife starting position
    0,-40,8,0,30,0; // Pencil and knife starting position OLD 
    0,-35,8,0,25,0; // Validation move until touch
    0,-25,8,0,15,0; // Validation rotate until torque
    0,-20,6,0,12,0; // Validation cap removal
    0,-28,7,0,20,0; // changed [5] from 19 to 21 7/22/19 // Everything else starting position
    */

    
    // Wait until we have data before proceeding with code
    while(!jnt_states_update) ros::spinOnce();

    // now that we have data, we want to start interpolation and then continue to the end pose
    // Calculate interpolation requirements

    // Max number of steps we need (recalculated after normalizing the joint velocity)
    int n_steps = 0;
    // The current number of loops we are at
    int loops_so_far = 0;

    // Convert the goal to radians
    joint_states_goal_ = deg2rad_vect(joint_states_goal_);

    // Eigen::VectorXd diff = joint_states_goal_ - joint_states_;
    // Get the vector from start to end, and treat as velocity in 1 second of joint movement
    Eigen::VectorXd joint_states_diff = joint_states_goal_ - joint_states_;
    
    // Calculate the number of loops required
    // ensure we do not go past a max velocity value for the velocity
    if(joint_states_diff.norm() > MAX_JOINT_VELOCITY_NORM){
        n_steps = std::round( (joint_states_diff.norm() / MAX_JOINT_VELOCITY_NORM) / dt_);
        desired_joint_velocity = (joint_states_diff / joint_states_diff.norm()) * MAX_JOINT_VELOCITY_NORM;
    }
    else{
        desired_joint_velocity = joint_states_diff;
        n_steps = std::round(joint_states_diff.norm()  / dt_);
    }

    // Increase length of steps to allow for more iterations to hit the goal check
    n_steps *= 2;

    // debug output
    // ROS_INFO_STREAM("Diff in deg");
    // ROS_INFO_STREAM(rad2deg_vect(joint_states_diff));
    // ROS_INFO_STREAM("Diff");
    // ROS_INFO_STREAM(joint_states_diff);
    // ROS_INFO_STREAM("Diff Norm");
    // ROS_INFO_STREAM(joint_states_diff.norm());
    // ROS_INFO_STREAM("Vel in rad");
    // ROS_INFO_STREAM(desired_joint_velocity);
    // ROS_INFO_STREAM("Vel in rad norm");
    // ROS_INFO_STREAM(desired_joint_velocity.norm());
    // ROS_INFO_STREAM("N_steps");
    // ROS_INFO_STREAM(n_steps);

    // Define rate at which we loop through main program, in Hz
	ros::Rate naptime(1/dt_);

    // Initialize our desired pose
    des_jnt_pos = joint_states_;
    
    // make sure we are not frozen, if there is a service call to toggle the freeze then we will stop interpolation
    freeze = false;

    ros::spinOnce();
    naptime.sleep();
    // update to only run until we interpolated to the final pose
    //? do we want to add an updated velocity, and check, with the exit condition of within some norm off?
    while(ros::ok() && joint_states_diff.norm() > GOAL_REACHED_ERR && loops_so_far < n_steps && !freeze){
        
        // Calculate our distance to goal again (exit condition)
        joint_states_diff = joint_states_goal_ - joint_states_;
        
        
        // Reduce our speed to below threshold
        if(joint_states_diff.norm() > MAX_JOINT_VELOCITY_NORM){
            // n_steps = std::round( (joint_states_diff.norm() / MAX_JOINT_VELOCITY_NORM) / dt_);
            desired_joint_velocity = (joint_states_diff / joint_states_diff.norm()) * MAX_JOINT_VELOCITY_NORM;
        }
        else{
            // n_steps = std::round(desired_joint_velocity.norm()  / dt_);
            desired_joint_velocity = joint_states_diff;
        }

        // ROS_INFO_STREAM("Diff in rad");
        // ROS_INFO_STREAM(joint_states_diff);
        // ROS_INFO_STREAM("Vel in rad");
        // ROS_INFO_STREAM(desired_joint_velocity);

        loops_so_far++;

        // We want to add our incrememnts, and check end condition
        des_jnt_pos = des_jnt_pos + (desired_joint_velocity * dt_); // sus
        // des_jnt_pos = joint_states_ + (desired_joint_velocity * dt_); // may want to do this

        // Put velocity and position commands into Jointstate message and round
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; 
		for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(desired_joint_velocity(i) * 1000) /1000;

		// Publish desired jointstate
		joint_states_pub.publish(desired_joint_state);
        // ROS_INFO_STREAM(desired_joint_state);
		// last_desired_joint_states_ = desired_joint_state;
        naptime.sleep();
		ros::spinOnce();
    }

    if(freeze){
        ROS_INFO_STREAM("User interrupt called.");
    }

    // Final command is 0 velocity, and current pose, we publiush then sleep and exit
    jnt_states_update = false;
    while(!jnt_states_update) ros::spinOnce();

    des_jnt_pos = joint_states_;
    desired_joint_velocity = Eigen::VectorXd::Zero(6);
    // Put velocity and position commands into Jointstate message and round
    for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; 
    for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(desired_joint_velocity(i) * 1000) /1000;

    joint_states_pub.publish(desired_joint_state);
    // ROS_INFO_STREAM(desired_joint_state);
    // last_desired_joint_states_ = desired_joint_state;
    naptime.sleep();
    ros::spinOnce();

    // End of program
}