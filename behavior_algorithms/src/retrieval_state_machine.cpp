// Rahul Pokharna and Quan Nguyen April 2021

// State machine for tool stowage
// treat the origin of the current frame as the fully seated pose, goal is to approach it
// use a mix of wrench and position for determining the status of the system

//! This is to be used when we are within an error of 6mm translation and each of the "legs/keys" of the tool should be oriented above the respective lock sockets
//! The Z axis of the tool should be approximately parallel to that of the stowage bay and heading down

// States being used:
// d. Soft E-stop: if wrench_lim,
// 0. Initialization: check initial pose
// 1. Move down into retrieving pose (the gripper has to be inside the tool's hole, bottomed out)
// 2. Grab tool: toggle compliance, open gripper, RWE
// 3. Rotate to unlock bay
// 4. Move upward 6cm (tool completely out of lock)
// 5. Rotate back/ Homing.

//TODO
/**
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
// #include <string.h>
#include <string>
#include <geometry_msgs/Wrench.h>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <cmath>
#include <irb120_accomodation_control/freeze_service.h>

//#include <iostream.h>
using namespace std;

geometry_msgs::PoseStamped curr_pose;
geometry_msgs::PoseStamped virt_attr;
geometry_msgs::Wrench ft_current_frame;

Eigen::Quaterniond curr_orientation;
Eigen::Affine3d curr_affine;

ros::ServiceClient freeze_client;
irb120_accomodation_control::freeze_service freeze_srv;

// freeze service information
std_msgs::Int8 freeze_mode_status;
bool freeze_mode; 
bool freeze_updated = false;

void freeze_status_callback(const std_msgs::Int8& freeze_status_msg) {
    freeze_updated = true;
    freeze_mode_status = freeze_status_msg;
    // cout<<"FREEZE STATUS: "<<freeze_mode_status<<endl;
    if(freeze_mode_status.data == 1) freeze_mode = true;
    else freeze_mode = false;
}

// Helper function to calculate
double ang_between_vecs(Eigen::Vector3d a, Eigen::Vector3d b)
{
    return acos(a.dot(b) / a.norm() * b.norm());
}

// Function to determine whether or not the tool is oriented properly
bool orient_check(Eigen::Matrix3d target_orient, double tol_x = 0.1, double tol_y = 0.1, double tol_z = 0.1)
{
    // tol_x, tol_y, tol_z in rad
    Eigen::Vector3d curr_x = curr_affine.linear().col(0);
    Eigen::Vector3d curr_y = curr_affine.linear().col(1);
    Eigen::Vector3d curr_z = curr_affine.linear().col(2);

    Eigen::Vector3d frame_x = target_orient.col(0);
    Eigen::Vector3d frame_y = target_orient.col(1);
    Eigen::Vector3d frame_z = target_orient.col(2);

    // target_orient.isApprox(curr_affine.linear(), tol)

    double err_x = abs(ang_between_vecs(curr_x, frame_x));
    double err_y = abs(ang_between_vecs(curr_y, frame_y));
    double err_z = abs(ang_between_vecs(curr_z, frame_z));

    return (err_x <= tol_x) && (err_y <= tol_y) && (err_z <= tol_z);

    // alternative is (vec_a - vec_b).norm() < threshold
}


// Maybe not needed
// Require an input for the position to check against, with some tolerances that can be adjusted
bool pos_check(Eigen::Vector3d target_pos, double tol_x = 0.002, double tol_y = 0.002, double tol_z = 0.002)
{
    Eigen::Vector3d curr_position = curr_affine.translation();
    return (abs(curr_position.x() - target_pos.x()) < tol_x) & (abs(curr_position.y() - target_pos.y()) < tol_y) && (abs(curr_position.z() - target_pos.z()) < tol_z);
}

void unfreeze(ros::Rate naptime)
{
    // spin, while we don't have data
    while(!freeze_updated){
        ros::spinOnce();
        naptime.sleep();
        // cout<<"spinning for data"<<endl;
    }
    // Check here after a spin, and unfreeze if it is frozen 
    while(freeze_mode_status.data == 1 && freeze_mode){
        
        if(freeze_updated && freeze_client.call(freeze_srv)){
            // success
            // cout<<"Called freeze mode service succesfully"<<endl;
        }
        else{
            // failed to call service
            // ROS_ERROR("Failed to call freeze service");
        }
        ros::spinOnce();
        naptime.sleep();
        freeze_updated = false;
    }
    
    // spin, while we don't have data
    while(!freeze_updated){
        ros::spinOnce();
        naptime.sleep();
        
        // cout<<"spinning for data"<<endl;
    }   
}

void cart_state_Cb(const geometry_msgs::PoseStamped &cart_pos)
{
    // store curr_pose
    curr_pose = cart_pos;

    // store curr_affine
    curr_orientation.w() = curr_pose.pose.orientation.w;
    curr_orientation.x() = curr_pose.pose.orientation.x;
    curr_orientation.y() = curr_pose.pose.orientation.y;
    curr_orientation.z() = curr_pose.pose.orientation.z;
    curr_affine.linear() = curr_orientation.normalized().toRotationMatrix();
    curr_affine.translation() = Eigen::Vector3d(curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z); //! Make sure this works
}

void ft_Cb(const geometry_msgs::Wrench &ft_vals)
{
    //these are not values from the sensor. They are f/t values transformed into robot base frame
    ft_current_frame = ft_vals;
}

void call_ptwl_fnc(double trans_x, double trans_y, double trans_z, double rot_x, double rot_y, double rot_z)
{
    string cmd = "rosrun behavior_algorithms cartp2ptwl _trans_x:=" + to_string(trans_x) + " _trans_y:=" + to_string(trans_y) + " _trans_z:=" + to_string(trans_z) +
                 " _rot_x:=" + to_string(rot_x) + " _rot_y:=" + to_string(rot_y) + " _rot_z:=" + to_string(rot_z) + " _param_set:=Stowage _bumpless:=0";
    system(cmd.c_str());
}
// integer flags, 0 to alleviate that load, 1 to preserve it in that axis
void call_rwe_fnc(int trans_x=0, int trans_y=0, int trans_z=0, int rot_x=0, int rot_y=0, int rot_z=0)
{
    string cmd = "rosrun behavior_algorithms force_moment_accommodation_interaction_port _trans_x:=" + to_string(trans_x) + " _trans_y:=" + to_string(trans_y) + " _trans_z:=" + to_string(trans_z) +
                 " _rot_x:=" + to_string(rot_x) + " _rot_y:=" + to_string(rot_y) + " _rot_z:=" + to_string(rot_z) + " _param_set:=Stowage _run_time:=5";
    system(cmd.c_str());
}
void call_joint_fnc(double joint_1 = 0, double joint_2 = -48, double joint_3 = 16, double joint_4 = 0, double joint_5 = 30, double joint_6 = 0)
{
    string cmd = "rosrun irb120_accomodation_control simple_joint_commander _joint_1:=0 _joint_2:=-48 _joint_3:=16 _joint_4:=0 _joint_5:=30 _joint_6:=0";
    system(cmd.c_str());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "stowage_state_machine");
    ros::NodeHandle nh;
    ros::Subscriber cart_state_sub = nh.subscribe("interaction_port_frame", 1, cart_state_Cb); //! make sure this is accurate
    ros::Subscriber ft_sub = nh.subscribe("current_frame_ft_wrench", 1, ft_Cb);
    // subscriber for freeze mode status (used in loop cond)
    ros::Subscriber freeze_mode_sub = nh.subscribe("freeze_mode_topic",1,freeze_status_callback);

    freeze_client = nh.serviceClient<irb120_accomodation_control::freeze_service>("freeze_service");


    //? Parameters for each state check
    /*
    Upper_stop_z_height: The height of the upper stop for tool stowage. The tool needs to be at or below this level to make sure we are in contact at the right place
    Lower_stop_z_height: The height of the lower stop for tool stowage. The tool needs to be at or below this level to make sure we are fullow bottomed out
    Approach_z_height: The approximate height of the approach pose
    Rotate angle: The angle that we expect the interaction port to be at when we fully open the stowage bay locks 
    Contact force threshold: The force threshold to determine whether or not we are in proper contact
    Contact torque threshold: The torque threshold for opening the lock
    Force Limit: force limit to use an RWE, or pull back
    Torque Limit: torque limit to use an RWE or pull back
    */
    double upper_stop_z_height = -0.01837, lower_stop_z_height = 0, approach_z_height = -0.0551, ROTATE_ANGLE = -0.136;
    double contact_force_threshold = 10, contact_torque_threshold = .5, force_limit = 25, torque_limit = 2.5;

    int state = 0;
    double dt_ = 0.01;
    int loops_in_current_state = 0;
    int restart_attempts = 0;
    bool state_machine_OK = true;

    // The goal orientation for the Unlock pose, a rotation around the Z axis
    Eigen::Matrix3d unlock_orientation;
    unlock_orientation(0, 0) = cos(ROTATE_ANGLE);
    unlock_orientation(0, 1) = -sin(ROTATE_ANGLE);
    unlock_orientation(0, 2) = 0;
    unlock_orientation(1, 0) = sin(ROTATE_ANGLE);
    unlock_orientation(1, 1) = cos(ROTATE_ANGLE);
    unlock_orientation(1, 2) = 0;
    unlock_orientation(2, 0) = 0;
    unlock_orientation(2, 1) = 0;
    unlock_orientation(2, 2) = 1;

    // We do not plan to run live code here,
    ros::Rate naptime(1 / dt_);

    system("rosservice call /set_frame_service \"task_name: 'Stowage'\" ");
    system("rosservice call /robotiq_ft_sensor_acc \"command_id: 8\"");
    for(int i = 0; i < 10; i++){
        ros::spinOnce();
        naptime.sleep();
    }

    while (ros::ok() && state_machine_OK)
    {
        ros::spinOnce();

        switch (state)
        {

        // Base case, before we start movement
        // check state of the system, we do not want to proceed until we are stationary, stable, and in freeze mode above the stowage bay
        case 0:
            ROS_INFO("State 0: Stationary check");

            if(restart_attempts >= 3){
                ROS_WARN("Too many restart attempts, exiting state machine.");
                state_machine_OK = false;
                state = -1;
                break;
            }
            
            // Send the join command here: 
            call_joint_fnc();

            // first make sure that the current frame is the stowage frame:
            for(int i = 0; i < 10; i++){
                ros::spinOnce();
                naptime.sleep();
            }

            // calc if the current pose is good to proceed (check if it's within translational and rotational constraints)
            // Make sure that these numbers are the ones we want for the approach pose
            if (pos_check(Eigen::Vector3d(0, 0, approach_z_height), 0.006, 0.006, 0.005) && orient_check(Eigen::Matrix3d::Identity(), 0.06, 0.06))
            { 
                state = 1;
                
            }
            else
            {
                // cartesian move to approach pose?

                //! Change to a cartesian move in a known manner, maybe want to add a condition that will not run joint command if we were kicked back here? or just transition to a different pose
                call_joint_fnc(); //TODO update the approach pose joint angles
                restart_attempts++;
            }

            break;

        // Move down to top section of lock, check contact force and also z pose (in current frame, so we can specify it to be the estimated height we expect)
        case 1:
            ROS_INFO("State 1: Move down into contact");
            ROS_INFO("Loop ctr = %d", loops_in_current_state);
            ROS_INFO("Contact threshold: %i", (contact_force_threshold < abs(ft_current_frame.force.z)) ? 1 : 0);
            ROS_INFO("Position Check: %i", (pos_check(Eigen::Vector3d(0, 0, lower_stop_z_height),  0.01, 0.01, 0.006)) ? 1 : 0);
            ROS_INFO("Orientation Check: %i", (orient_check(Eigen::Matrix3d::Identity(), 0.2, 0.2)) ? 1 : 0);

            // based on current frame, known z height of the stowage bay, also orientation check (make sure we are not tilted drastically)
            if (contact_force_threshold < abs(ft_current_frame.force.z) && pos_check(Eigen::Vector3d(0, 0, lower_stop_z_height), 0.01, 0.01, 0.006) && orient_check(Eigen::Matrix3d::Identity(), 0.2, 0.2))
            {
                state = 2;
                loops_in_current_state = 0;

                cout << "Transitioning to state 2." << endl;
                ROS_INFO("Transitioning to state 2.");
                break;
            }

            // If we have been stuck for 5 loops, pull back, restart
            if (loops_in_current_state > 6)
            {
                // can call an rwe in case we are jammed
                call_rwe_fnc();
                // pull back and clear of the bay
                call_ptwl_fnc(0, 0, -0.05, 0, 0, 0);
                state = 0; // make it -1 maybe
                loops_in_current_state = 0;
                cout << "Restarting state machine from state 1" << endl;
                ROS_INFO("Restarting state machine from state 1");
                restart_attempts++;
                break;
            }
            // if we have high forces, still too high, and maybe misaligned, then run RWE
            else if (loops_in_current_state > 2 && (!orient_check(Eigen::Matrix3d::Identity()) || (force_limit < abs(ft_current_frame.force.z))))
            {                                   // may want to update the wrench checks, more options
                call_rwe_fnc(0, 0, 1, 0, 0, 0); // may want to chage z force from 1 or 0
            }
            // Move down normally otherwise
            else
            {
                // // desired pos vec
                // Eigen::Vector3d des_pos = Eigen::Vector3d(0,0,-0.021);
                // Eigen::Vector3d des_cmd = des_pos - curr_affine.translation(); 
                // call_ptwl_fnc(des_pos.x(), des_pos.y(), des_pos.z(), 0, 0, 0);

                // vs standard move down
                call_ptwl_fnc(0,0,0.03, 0, 0, 0);
            }

            loops_in_current_state += 1;

            break;

        // Grab tool: compliance on, open gripper, RWE
        case 2:
            ROS_INFO("State 2: Grabbing the tool");

            ROS_INFO("Turn on compliance");
            unfreeze(naptime);

            ROS_INFO("Open Gripper while in compliance");
            system("rosservice call /grip '[{id: 1, state: false}, {id: 2, state: true}]'");
            
            ROS_INFO("RWEEEEEEEEEEEEEEE");
            call_rwe_fnc();

            state = 3;

            break;

        // rotate unlock
        case 3:
            ROS_INFO("State 3: Unlock the Stowage Bay");
            ROS_INFO("Loop ctr = %d", loops_in_current_state);
            ROS_INFO("Contact threshold: %i", (contact_torque_threshold < abs(ft_current_frame.torque.z)) ? 1 : 0);
            ROS_INFO("Position Check: %i", (pos_check(Eigen::Vector3d(0, 0, lower_stop_z_height),  0.01, 0.01, 0.006)) ? 1 : 0);
            ROS_INFO("Orientation Check: %i", (orient_check(unlock_orientation)) ? 1 : 0);

            if (contact_torque_threshold < abs(ft_current_frame.torque.z) && pos_check(Eigen::Vector3d(0, 0, lower_stop_z_height), 0.015, 0.015, 0.015) && orient_check(unlock_orientation))
            {
                state = 4;
                loops_in_current_state = 0;

                cout << "Transitioning to state 4." << endl;
                ROS_INFO("Transitioning to state 4.");
                break;
            }

            // If we have been stuck for 5 loops, pull back, restart
            if (loops_in_current_state > 5) // we may want an additional check here? maybe change what we do (instead of restart, do we want to terminate)
            { 
                // can call an rwe in case we are jammed
                call_rwe_fnc(0, 0, 0, 0, 0, 0);
                state = -1; // stop, exit the state machine
                loops_in_current_state = 0;
                cout << "Unlock Error!!!! Please check the stowage bay" << endl;
                ROS_INFO("Unlock Error!!!! Please check the stowage bay");
                break;
            }
            // if we have high forces, still too high, and maybe misaligned, then run RWE
            else if ((force_limit < abs(ft_current_frame.force.z)) || (torque_limit < abs(ft_current_frame.torque.z)))
            {
                call_rwe_fnc(0, 0, 0, 0, 0, 0); //relieve ALL loads
            }
            // Rotate normally otherwise
            else
            {
                call_ptwl_fnc(0, 0, 0, 0, 0, -0.2);
            }

            loops_in_current_state += 1;

            break;


        // rotate to lock bay
        case 4:
            ROS_INFO("State 4: Pulling out the tool");
            ROS_INFO("Loop ctr = %d", loops_in_current_state);

            call_rwe_fnc(0, 0, 0, 0, 0, 0); //relieve ALL loads
            
            // pull out
            call_ptwl_fnc(0, 0, -0.06, 0, 0, 0);

            naptime.sleep();
            ros::spinOnce();
            
            cout<<"Current affine: "<<curr_affine.translation()<<endl;

            // Check if we have cleared the stowage bay, then give succes outpt
            if (curr_affine.translation().z() <= approach_z_height + 0.01)
            {
                state = -1;
                loops_in_current_state = 0;
                cout << "Successfully stowed tool in bay. Exiting state machine." << endl;
                ROS_INFO("Successfully stowed tool in bay. Exiting state machine.");
                // return true;
                break;
            }

            // if we have not succeeded, give failure output
            state = -1;
            loops_in_current_state = 0;
            cout << "Exiting state machine from state 4" << endl;
            ROS_INFO("Exiting state machine from state 4");
            // return false; // false
            

            break;

        default:
            // wrap up the program, then exit the loop
            state_machine_OK = false;
            // return false;
        }
        // exit cond when state is -1, or we can exit in default

        // call freeze mode for all, kill all nodes etc

        naptime.sleep();
        ros::spinOnce();
    }
    // finishing touches and terminations
    ros::spinOnce();
    naptime.sleep();

}