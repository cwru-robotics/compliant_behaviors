//Updated 2/1/2021 //! FT 
//Surag Balajepalli, Matthew Haberbusch, and Rahul Pokharna and Quan Nguyen
//subscribes to a 'virtual attractor' pose
//Performs accommodation control in a set frame, all with the inteeraction port of the FT sensor

// TODO 
/*
* Define a TF publisher for /task_frame and /stowage_frame for rviz and others to know the TF between them and map (aka base) (static or dynamic? would this then by a dynamic TF?)
* Define a published interaction frame, with a set frame ID for a transform (confirm with rviz)
* Update the Interaction port (with TF updated in the loop as well) based on the set_frame, all in the current_frame (including the attractor, when in a PHold how do we define if we change frames)
* Define a wrench in the current frame
* Update compliance to be in the task/stowage/set frame (can use interaction point and virt attr as both in same frame_id relative)
*/

// ROS: include libraries
#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <irb120_accomodation_control/freeze_service.h>
#include <irb120_accomodation_control/set_task_frame.h>
#include <irb120_accomodation_control/set_current_frame.h>
#include <irb120_accomodation_control/set_frame.h>
#include <irb120_accomodation_control/matrix_msg.h> 
#include <tf/transform_broadcaster.h> 
// #include <tf/transform_listener.h> 
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h> 
#include <string.h>
#include <stdio.h>
using namespace std;

// Declare freeze mode variables
sensor_msgs::JointState last_desired_joint_state_;
bool freeze_mode = false;
std_msgs::Int8 freeze_mode_status;

// Declare filter variables 
// Uncomment if you decide to use a low pass filter
/*
Eigen::MatrixXd wrench_filter = Eigen::MatrixXd::Zero(10,6);
int filter_counter = 0;
*/

// Declare global variables for subscribers
Eigen::VectorXd frozen_joint_states_ = Eigen::VectorXd::Zero(6); 
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd virtual_attractor_pos(3);
Eigen::Matrix3d virtual_attractor_rotation_matrix;
Eigen::Quaterniond virt_quat;

bool virtual_attractor_established = false; 
bool jnt_state_update = false;

// initialize our K matrix, and our current frame
string current_frame = "Task";
std_msgs::String current_frame_msg;
// May want to define a 6x6, and get submatrices
Eigen::Vector3d k_trans, k_rot;
Eigen::VectorXd b_des_vec = Eigen::VectorXd::Zero(6);
Eigen::MatrixXd k_combined = Eigen::MatrixXd::Zero(6,6);
Eigen::MatrixXd b_des_inv = Eigen::MatrixXd::Zero(6,6);

// Declare variables of tool frame vectors for skills to use. 
Eigen::Affine3d tool_with_respect_to_robot_;
geometry_msgs::Vector3 x_vec_message;
geometry_msgs::Vector3 y_vec_message;
geometry_msgs::Vector3 z_vec_message;

// Declare variables for task frame vectors
Eigen::Affine3d task_frame_with_respect_to_robot_;
Eigen::MatrixXd task_frame_rotation_matrix_;
Eigen::Vector3d x_vec_task_;
Eigen::Vector3d y_vec_task_;
Eigen::Vector3d z_vec_task_;
geometry_msgs::PoseStamped task_frame_pose_stamped;
// Declare variables of task frame vectors for skills to use.
geometry_msgs::Vector3 x_vec_task_message_;
geometry_msgs::Vector3 y_vec_task_message_;
geometry_msgs::Vector3 z_vec_task_message_;


// Declare variables for stowage frame vectors
Eigen::Affine3d stowage_frame_with_respect_to_robot_;
Eigen::MatrixXd stowage_frame_rotation_matrix_;
// Declare variables for the static tool stowage bay
Eigen::Vector3d x_vec_stowage_;
Eigen::Vector3d y_vec_stowage_;
Eigen::Vector3d z_vec_stowage_;
// Frame of the stowage vectors for skills to use.
geometry_msgs::Vector3 x_vec_stowage_message_;
geometry_msgs::Vector3 y_vec_stowage_message_;
geometry_msgs::Vector3 z_vec_stowage_message_;
geometry_msgs::PoseStamped stowage_frame_pose_stamped;


// Declare variables for ft frame vectors
Eigen::MatrixXd ft_frame_rotation_matrix_;
// Declare variables for the ft frame
Eigen::Vector3d x_vec_ft_;
Eigen::Vector3d y_vec_ft_;
Eigen::Vector3d z_vec_ft_;
// Frame of the fr frame vectors for skills to use.
geometry_msgs::Vector3 x_vec_ft_message_;
geometry_msgs::Vector3 y_vec_ft_message_;
geometry_msgs::Vector3 z_vec_ft_message_;
geometry_msgs::PoseStamped ft_frame_pose_stamped;
// Define a transform of the ft in the base frame, to be converted for FT in the current frame
geometry_msgs::TransformStamped ft_frame_transform_stamped; 

// Define an interaction Port to be published to other skills for relative motion of the attractor
geometry_msgs::PoseStamped interaction_port_frame_pose_stamped; 
Eigen::Affine3d interaction_port_frame_with_respect_to_robot_; //unused
Eigen::MatrixXd interaction_port_frame_rotation_matrix_; //unused
// tf::StampedTransform interaction_frame_transform_stamped; 

// Define the transform for the current frame, it is either the stowage frame or the task frame 
//TODO (how to set the attractor or the interaction port and of the attractor start pose)
geometry_msgs::TransformStamped current_frame_transform_stamped; 
Eigen::Affine3d sensor_with_respect_to_robot;
Eigen::Affine3d sensor_with_respect_to_current;


// MATH: Function converts a rotation matrix to a vector of angles (phi_x, phi_y, phi_z)
Eigen::Vector3d decompose_rot_mat(Eigen::Matrix3d rot_mat) {
	Eigen::Vector3d vec_of_angles;
	vec_of_angles(0) = atan2(rot_mat(2,1),rot_mat(2,2));
	vec_of_angles(1) = atan2(-1 * rot_mat(2,0), sqrt(pow(rot_mat(2,1),2) + pow(rot_mat(2,2),2)));
	vec_of_angles(2) = atan2(rot_mat(1,0), rot_mat(0,0)); 
	return vec_of_angles;
}

// MATH: Function takes two rotation matricies and ouputs the angular displacement between them in a vector of angles
Eigen::Vector3d delta_phi_from_rots(Eigen::Matrix3d source_rot, Eigen::Matrix3d dest_rot) {
	// R_reqd = R_rob.inv * R_des
	Eigen::Matrix3d desired_rotation =  dest_rot * source_rot.inverse();
	Eigen::AngleAxisd desired_ang_ax(desired_rotation);
	Eigen::Vector3d delta_phi;
	Eigen::Vector3d axis = desired_ang_ax.axis();
	delta_phi(0) = axis(0)*desired_ang_ax.angle();
	delta_phi(1) = axis(1)*desired_ang_ax.angle();
	delta_phi(2) = axis(2)*desired_ang_ax.angle();
	return delta_phi;
}

// MATH: Function converts a vector of euler angles into a rotation matrix
Eigen::Matrix3d rotation_matrix_from_euler_angles(Eigen::Vector3d euler_angle_vector) {
	Eigen::Matrix3d x_rotation;
	Eigen::Matrix3d y_rotation;
	Eigen::Matrix3d z_rotation;
	Eigen::Matrix3d rotation_matrix_from_euler_angles;

	z_rotation(0,0) = cos(euler_angle_vector(2));
   	z_rotation(0,1) = -sin(euler_angle_vector(2));
   	z_rotation(0,2) = 0;
   	z_rotation(1,0) = sin(euler_angle_vector(2));
   	z_rotation(1,1) = cos(euler_angle_vector(2));
   	z_rotation(1,2) = 0;
   	z_rotation(2,0) = 0;
   	z_rotation(2,1) = 0;
   	z_rotation(2,2) = 1;

   	y_rotation(0,0) = cos(euler_angle_vector(1));
   	y_rotation(0,1) = 0;
   	y_rotation(0,2) = sin(euler_angle_vector(1));
   	y_rotation(1,0) = 0;
   	y_rotation(1,1) = 1;
   	y_rotation(1,2) = 0;
   	y_rotation(2,0) = -sin(euler_angle_vector(1));
   	y_rotation(2,1) = 0;
   	y_rotation(2,2) = cos(euler_angle_vector(1));

   	x_rotation(0,0) = 1;
   	x_rotation(0,1) = 0;
   	x_rotation(0,2) = 0;
   	x_rotation(1,0) = 0;
   	x_rotation(1,1) = cos(euler_angle_vector(0));
   	x_rotation(1,2) = -sin(euler_angle_vector(0));
   	x_rotation(2,0) = 0;
   	x_rotation(2,1) = sin(euler_angle_vector(0));
   	x_rotation(2,2) = cos(euler_angle_vector(0));

   	rotation_matrix_from_euler_angles = z_rotation * y_rotation * x_rotation;
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

// ROS: Callback function receives the force-torque wrench
void forceTorqueSensorCallback(const geometry_msgs::WrenchStamped& ft_sensor) {
	// Subscribed to "robotiq_ft_wrench"
	// Round our force values
	wrench_body_coords_(0) = std::round(ft_sensor.wrench.force.x * 10) / 10;
	wrench_body_coords_(1) = std::round(ft_sensor.wrench.force.y * 10) / 10;
	wrench_body_coords_(2) = std::round(ft_sensor.wrench.force.z * 10) / 10;
	wrench_body_coords_(3) = std::round(ft_sensor.wrench.torque.x * 10) / 10;
	wrench_body_coords_(4) = std::round(ft_sensor.wrench.torque.y * 10) / 10;
	wrench_body_coords_(5) = std::round(ft_sensor.wrench.torque.z * 10) / 10;

	// Optional low pass filter starts here (it's a moving average)
	// This was removed, but kept in case anyone would like to use this, we chose not to
	/* 
	if (filter_counter > 9) { //if last 10 readings are recorded
		//low pass filter
		//Block of size (p,q), starting at (i,j); so block(i,j,p,q)
		wrench_filter.block(1,0,9,6) = wrench_filter.block(0,0,9,6); //move everything down
		wrench_filter.block(0,0,1,6) = wrench_body_coords_.transpose(); // top row is the new coordinates
		wrench_body_coords_ = wrench_filter.colwise().sum(); // sums up the columns into an array
		wrench_body_coords_ /= 10; // now we have an average
	}
	else {
		filter_counter++;
		wrench_filter.block(1,0,9,6) = wrench_filter.block(0,0,9,6);
		wrench_filter.block(0,0,1,6) = wrench_body_coords_.transpose(); 
		// and then wrench_body_coords_ is just exactly what the sensor reads
	}
	*/
	// low pass filter ends here

}

// ROS: Callback function receives the joint state from "abb120_joint_state" topic
void jointStateCallback(const sensor_msgs::JointState& joint_state) {
	jnt_state_update = true;
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state.position[i] ;
}

// ROS: Callback funtion receives the virtual attractor pose from "virtual_attractor" topic
void virtualAttractorCallback(const geometry_msgs::PoseStamped& des_pose) {
	virtual_attractor_established = true;
	virtual_attractor_pos(0) = des_pose.pose.position.x;
	virtual_attractor_pos(1) = des_pose.pose.position.y;
	virtual_attractor_pos(2) = des_pose.pose.position.z;

	virt_quat.w() = des_pose.pose.orientation.w;
	virt_quat.x() = des_pose.pose.orientation.x;
	virt_quat.y() = des_pose.pose.orientation.y;
	virt_quat.z() = des_pose.pose.orientation.z;
			
	//seperately filling out the 3x3 rotation matrix
	virtual_attractor_rotation_matrix = virt_quat.normalized().toRotationMatrix();
}

// ROS: Callback funtion called with freeze_service is called; toggles freezemode and sets frozen joint states
bool freezeServiceCallback(irb120_accomodation_control::freeze_serviceRequest &request, irb120_accomodation_control::freeze_serviceResponse &response) {
	// Toggle freezemode
	freeze_mode = !freeze_mode;
	if (freeze_mode) freeze_mode_status.data = 1;
	else freeze_mode_status.data = 0;

	// Set frozen joint states to last desired joing state
	frozen_joint_states_(0) = last_desired_joint_state_.position[0];
	frozen_joint_states_(1) = last_desired_joint_state_.position[1];
	frozen_joint_states_(2) = last_desired_joint_state_.position[2];
	frozen_joint_states_(3) = last_desired_joint_state_.position[3];
	frozen_joint_states_(4) = last_desired_joint_state_.position[4];
	frozen_joint_states_(5) = last_desired_joint_state_.position[5];

	// Print status info to display
	ROS_WARN("Freeze mode has been toggled");
	if(freeze_mode) {
		ROS_INFO("Currently in latch mode");
		response.status = "Latch mode";
		
	}
	else {
		ROS_INFO("Currently in normal mode");
		response.status = "Communication mode";
	}

	// maybe output the status change into the buffer, whenever the freeze mode is toggled

	return true;
}

//TODO Update frame callback, will take a set_frame.srv and then read string to set current kmat as the appropriate preset, return those values and the string name of the current frame

// Click a button and set the task frame to be the current tool frame
bool setTaskFrameCallback(irb120_accomodation_control::set_task_frameRequest &request, irb120_accomodation_control::set_task_frameResponse &response) {
	// Find tool's current x, y, and vector for use in skills 
	task_frame_with_respect_to_robot_ = sensor_with_respect_to_robot;
	// = tool_with_respect_to_robot_; // If tool tip is the interaction port

	// Just get the data directly from the tool frame, the service does not send or receive data, just used as a trigger mechanism
	task_frame_rotation_matrix_ = task_frame_with_respect_to_robot_.linear();
	x_vec_task_ = task_frame_rotation_matrix_.col(0);
	y_vec_task_ = task_frame_rotation_matrix_.col(1);
	z_vec_task_ = task_frame_rotation_matrix_.col(2);

	// Update the ros message to be published (for rviz) of the frame
	task_frame_pose_stamped.pose.position.x = task_frame_with_respect_to_robot_.translation()(0);
	task_frame_pose_stamped.pose.position.y = task_frame_with_respect_to_robot_.translation()(1);
	task_frame_pose_stamped.pose.position.z = task_frame_with_respect_to_robot_.translation()(2);
	Eigen::Quaterniond task_frame_orientation_quaternion(task_frame_with_respect_to_robot_.linear());
	task_frame_pose_stamped.pose.orientation.x = task_frame_orientation_quaternion.x();
	task_frame_pose_stamped.pose.orientation.y = task_frame_orientation_quaternion.y();
	task_frame_pose_stamped.pose.orientation.z = task_frame_orientation_quaternion.z();
	task_frame_pose_stamped.pose.orientation.w = task_frame_orientation_quaternion.w();

	// Update the current frame transform
	current_frame_transform_stamped.transform.translation.x = task_frame_pose_stamped.pose.position.x;
	current_frame_transform_stamped.transform.translation.y = task_frame_pose_stamped.pose.position.y;
	current_frame_transform_stamped.transform.translation.z = task_frame_pose_stamped.pose.position.z;
	current_frame_transform_stamped.transform.rotation.x = task_frame_pose_stamped.pose.orientation.x;
	current_frame_transform_stamped.transform.rotation.y = task_frame_pose_stamped.pose.orientation.y;
	current_frame_transform_stamped.transform.rotation.z = task_frame_pose_stamped.pose.orientation.z;
	current_frame_transform_stamped.transform.rotation.w = task_frame_pose_stamped.pose.orientation.w;

	

	response.status = "task frame set"; 
	return true; 
}

// Click a button and set the task frame to be the current tool frame
bool setCurrentFrameServiceCallback(irb120_accomodation_control::set_current_frameRequest &request, irb120_accomodation_control::set_current_frameResponse &response) {
	string task_name = request.task_name;
	bool success = true;
	// do an if else for set tasks with predefined K matrices here, then update the response.updated_frame in the if else, and the last else will be return false, status = no known task sent
	if (!strcmp(task_name.c_str(), "Peg")){
		current_frame = "Task";
		k_trans << 1000,1000,1000;
		k_rot << 40,40,40;
		k_combined.topLeftCorner(3,3) = k_trans.asDiagonal();
		k_combined.bottomRightCorner(3,3) = k_rot.asDiagonal();
		b_des_vec << 4000,4000,4000,100,100,100;
		b_des_inv = b_des_vec.asDiagonal().inverse();
	}
	else if (!strcmp(task_name.c_str(), "Bottle_Cap")){
		current_frame = "Task";
		k_trans << 1500,1500,1500;
		k_rot << 40,40,40;
		k_combined.topLeftCorner(3,3) = k_trans.asDiagonal();
		k_combined.bottomRightCorner(3,3) = k_rot.asDiagonal();
		b_des_vec << 4000,4000,4000,200,200,200;
		b_des_inv = b_des_vec.asDiagonal().inverse();
	}
	else if (!strcmp(task_name.c_str(), "Tool")){
		current_frame = "Task";
		k_trans << 100,100,100;
		k_rot << 10,10,10;
		k_combined.topLeftCorner(3,3) = k_trans.asDiagonal();
		k_combined.bottomRightCorner(3,3) = k_rot.asDiagonal();
		b_des_vec << 4000,4000,4000,200,200,200;
		b_des_inv = b_des_vec.asDiagonal().inverse();
	}
	else if (!strcmp(task_name.c_str(), "Cutting")){
		current_frame = "Stowage";
		k_trans << 1000,1000,1000;
		k_rot << 80,80,80;
		k_combined.topLeftCorner(3,3) = k_trans.asDiagonal();
		k_combined.bottomRightCorner(3,3) = k_rot.asDiagonal();
		b_des_vec << 4000,4000,4000,200,200,200;
		b_des_inv = b_des_vec.asDiagonal().inverse();
	}
	else if (!strcmp(task_name.c_str(), "Task")){
		current_frame = "Task";
		k_trans << 1500,1500,1500;
		k_rot << 40,40,40;
		k_combined.topLeftCorner(3,3) = k_trans.asDiagonal();
		k_combined.bottomRightCorner(3,3) = k_rot.asDiagonal();
		b_des_vec << 4000,4000,4000,200,200,200;
		b_des_inv = b_des_vec.asDiagonal().inverse();
	}
	else if (!strcmp(task_name.c_str(), "Stowage")){
		current_frame = "Stowage";
		k_trans << 1000,1000,1000;
		k_rot << 40,40,40;
		k_combined.topLeftCorner(3,3) = k_trans.asDiagonal();
		k_combined.bottomRightCorner(3,3) = k_rot.asDiagonal();
		b_des_vec << 4000,4000,4000,200,200,200;
		b_des_inv = b_des_vec.asDiagonal().inverse();
	}
	else if (!strcmp(task_name.c_str(), "Quick_Disconnect")){ // is this implemented ? 
		current_frame = "Task";
		k_trans << 1500,1500,1500;
		k_rot << 40,40,40;
		k_combined.topLeftCorner(3,3) = k_trans.asDiagonal();
		k_combined.bottomRightCorner(3,3) = k_rot.asDiagonal();
		b_des_vec << 4000,4000,4000,200,200,200;
		b_des_inv = b_des_vec.asDiagonal().inverse();
	}
	else if (!strcmp(task_name.c_str(), "Deep_Drive")){
		current_frame = "Task";
		k_trans << 1500,1500,1000; // softer in Z, but not too soft
		k_rot << 40,40,30; // softer in Z so that it will rotate until it locks in place, but not too soft that it doesnt rotate. We want to pull down and rotate until we are aligned. Maybe reduce wrench limits? 
		k_combined.topLeftCorner(3,3) = k_trans.asDiagonal();
		k_combined.bottomRightCorner(3,3) = k_rot.asDiagonal();
		b_des_vec << 4000,4000,4000,200,200,200;
		b_des_inv = b_des_vec.asDiagonal().inverse();
	}
	else {
		cout<< "Failed"<<endl<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
		response.status = "Failed to find a known preset.";
		success = false;
	}
	// Based on current_frame, set the header of the interaction port to be in the correct frame, and change the TF flag
	if(!strcmp(current_frame.c_str(), "Task")){
		// interaction_port_frame_pose_stamped.header.frame_id = "/current_frame"; //! Make this into a TF tree thing
		// Update the current frame transform
		current_frame_transform_stamped.transform.translation.x = task_frame_pose_stamped.pose.position.x;
		current_frame_transform_stamped.transform.translation.y = task_frame_pose_stamped.pose.position.y;
		current_frame_transform_stamped.transform.translation.z = task_frame_pose_stamped.pose.position.z;
		current_frame_transform_stamped.transform.rotation.x = task_frame_pose_stamped.pose.orientation.x;
		current_frame_transform_stamped.transform.rotation.y = task_frame_pose_stamped.pose.orientation.y;
		current_frame_transform_stamped.transform.rotation.z = task_frame_pose_stamped.pose.orientation.z;
		current_frame_transform_stamped.transform.rotation.w = task_frame_pose_stamped.pose.orientation.w;

		// Update what interaction port is here as well (fixed a bug that the first frame to recieve a command causes a major displacement of the attractor)
		sensor_with_respect_to_current = task_frame_with_respect_to_robot_.inverse() * sensor_with_respect_to_robot;
	}
	else{
		// interaction_port_frame_pose_stamped.header.frame_id = "/current_frame";
		// Update the current frame transform
		current_frame_transform_stamped.transform.translation.x = stowage_frame_pose_stamped.pose.position.x;
		current_frame_transform_stamped.transform.translation.y = stowage_frame_pose_stamped.pose.position.y;
		current_frame_transform_stamped.transform.translation.z = stowage_frame_pose_stamped.pose.position.z;
		current_frame_transform_stamped.transform.rotation.x = stowage_frame_pose_stamped.pose.orientation.x;
		current_frame_transform_stamped.transform.rotation.y = stowage_frame_pose_stamped.pose.orientation.y;
		current_frame_transform_stamped.transform.rotation.z = stowage_frame_pose_stamped.pose.orientation.z;
		current_frame_transform_stamped.transform.rotation.w = stowage_frame_pose_stamped.pose.orientation.w;

		// Update what interaction port is here as well (fixed a bug that the first frame to recieve a command causes a major displacement of the att4ractor)
		sensor_with_respect_to_current = stowage_frame_with_respect_to_robot_.inverse() * sensor_with_respect_to_robot;
	}
	// send Updated information back
	response.updated_frame = current_frame;
	// update the value of the frame message
	current_frame_msg.data = current_frame;
	geometry_msgs::Vector3 trans_vec, rot_vec;
	trans_vec.x = k_trans.x();
	trans_vec.y = k_trans.y();
	trans_vec.z = k_trans.z();
	rot_vec.x = k_rot.x();
	rot_vec.y = k_rot.y();
	rot_vec.z = k_rot.z();
	response.K_mat.trans_mat = trans_vec;
	response.K_mat.rot_mat = rot_vec;
	
	if(success){
		cout<< "Success"<<endl<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
		response.status = "new frame set"; // move into each one above, 
	}
	return true; 
}


// Main rogram
int main(int argc, char **argv) {
	// ROS: for communication
	ros::init(argc, argv, "acc_controller");
	ros::NodeHandle nh;

	// ROS: Subscribers receive data from these topics
	ros::Subscriber virtual_attractor_sub = nh.subscribe("Virt_attr_pose",1,virtualAttractorCallback); // ROS: Subscribe to the virtual attractor pose
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, forceTorqueSensorCallback); // ROS: Subscribe to the force-torque sensor wrench
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback); // ROS: Subscribe to our robot's joint states

	// ROS: Publishers send data to these topics
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1); // TODO CHANGE FOR ROBOT AGNOSTIC
	ros::Publisher cart_log_pub = nh.advertise<geometry_msgs::PoseStamped>("cartesian_logger",1); // ROS: Publish the cartesian coordinates of the end effector in the robot coordinate frame
	ros::Publisher robot_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_frame", 1);  // ROS: Publish a zero pose stamped to visualize the robot frame
	ros::Publisher task_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("task_frame", 1);	  // ROS: Publish the task frame
	ros::Publisher ft_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("ft_frame", 1);	  // ROS: Publish the ft frame 
	ros::Publisher stowage_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("stowage_frame", 1); // ROS: Publish the stowage frame
	ros::Publisher interaction_port_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("interaction_port_frame", 1); // ROS: Publish the interaction port frame
	ros::Publisher ft_pub = nh.advertise<geometry_msgs::Wrench>("transformed_ft_wrench",1); // ROS: Publish the force-torque wrench in the robot coordinate frame
	ros::Publisher current_frame_ft_pub = nh.advertise<geometry_msgs::Wrench>("current_frame_ft_wrench",1); // ROS: Publish the force-torque wrench in the current compliance coordinate frame
	ros::Publisher virtual_attractor_after_tf = nh.advertise<geometry_msgs::PoseStamped>("tfd_virt_attr",1); // ROS: Publish the virtual attractor pose in the robot coordinate frame
	ros::Publisher bumpless_virtual_attractor_after_tf = nh.advertise<geometry_msgs::PoseStamped>("bumpless_tfd_virt_attr",1); // ROS: Publish the bumpless virtual attractor pose calculated from the force-torque wrench
	ros::Publisher freeze_mode_pub = nh.advertise<std_msgs::Int8>("freeze_mode_topic",1); // ROS: Publish whether freeze mode is on or off
	ros::Publisher x_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_x",1); // ROS: Publish the tool coordinate frame's x vector in the robot coordinate frame
	ros::Publisher y_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_y",1); // ROS: Publish the tool coordinate frame's y vector in the robot coordinate frame
	ros::Publisher z_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_z",1); // ROS: Publish the tool coordinate frame's z vector in the robot coordinate frame
	ros::Publisher x_vec_task_pub = nh.advertise<geometry_msgs::Vector3>("task_vector_x",1); // ROS: Publish the task coordinate frame's x vector in the robot coordinate frame
	ros::Publisher y_vec_task_pub = nh.advertise<geometry_msgs::Vector3>("task_vector_y",1); // ROS: Publish the task coordinate frame's y vector in the robot coordinate frame
	ros::Publisher z_vec_task_pub = nh.advertise<geometry_msgs::Vector3>("task_vector_z",1); // ROS: Publish the task coordinate frame's z vector in the robot coordinate frame
	ros::Publisher x_vec_stowage_pub = nh.advertise<geometry_msgs::Vector3>("stowage_vector_x",1); // ROS: Publish the stowage coordinate frame's x vector in the robot coordinate frame
	ros::Publisher y_vec_stowage_pub = nh.advertise<geometry_msgs::Vector3>("stowage_vector_y",1); // ROS: Publish the stowage coordinate frame's y vector in the robot coordinate frame
	ros::Publisher z_vec_stowage_pub = nh.advertise<geometry_msgs::Vector3>("stowage_vector_z",1); // ROS: Publish the stowage coordinate frame's z vector in the robot coordinate frame
	ros::Publisher x_vec_ft_pub = nh.advertise<geometry_msgs::Vector3>("ft_vector_x",1); // ROS: Publish the ft coordinate frame's x vector in the robot coordinate frame
	ros::Publisher y_vec_ft_pub = nh.advertise<geometry_msgs::Vector3>("ft_vector_y",1); // ROS: Publish the ft coordinate frame's y vector in the robot coordinate frame
	ros::Publisher z_vec_ft_pub = nh.advertise<geometry_msgs::Vector3>("ft_vector_z",1); // ROS: Publish the ft coordinate frame's z vector in the robot coordinate frame

	ros::Publisher current_frame_pub = nh.advertise<std_msgs::String>("current_frame",1); // ROS: Publish the name of the current frame 

	// ROS: Service to set the task frame to the current EE pose
	ros::ServiceServer task_frame_service = nh.advertiseService("task_frame_service",setTaskFrameCallback);

	// ROS: Service to toggle freeze mode
	ros::ServiceServer freeze_service = nh.advertiseService("freeze_service",freezeServiceCallback);

	// ROS: Service to update the current frame of compliance
	ros::ServiceServer set_frame_service = nh.advertiseService("set_frame_service",setCurrentFrameServiceCallback);

	// ROS: Define our current frame transform broadcaster (used for compliance and movement in subframes, transform back and forth between poses)
	tf::TransformBroadcaster br;

	// tf::TransformListener listener; 

	// Set the freeze mode status to off. This does not toggle freeze mode. It's only for display purposes.
	freeze_mode_status.data = 1; // or here make it frozen, and set the mode to be 1 by default
	
	// MATH: Instantiate an object of the custom forward kinematics solver class for our ABB IRB 120 robot
	
	Irb120_fwd_solver irb120_fwd_solver;
	cout << "After irb120 fkik init"<<endl; 

	// Declare matricies and vectors
	Eigen::VectorXd current_end_effector_pose(6);
	Eigen::VectorXd sensor_pose(6);
	Eigen::VectorXd wrench_with_respect_to_robot(6);
	Eigen::VectorXd wrench_with_respect_to_current(6);
	Eigen::VectorXd virtual_force(6);
	Eigen::VectorXd desired_joint_velocity = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd desired_cartesian_acceleration(6);
	Eigen::VectorXd desired_twist = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd desired_twist_with_gain = Eigen::VectorXd::Zero(6);

	cout << "Defined vectors"<<endl; 

	// Define constants
	double dt_ = 0.01;
	double MAX_JOINT_VELOCITY_NORM = 10;
	
	// Define damping and gains
	double B_virtual_translational = 4000; 
	double B_virtual_rotational = 200; // was 100
	double K_virtual_translational = 1500; // was 1000
	double K_virtual_angular = 40; // was 40

	cout << "After definitions of old k and b mats "<<endl; 
	//TODO Define the k_virt 
	k_trans << 1500,1500,1500;
	k_rot << 40,40,40;
	k_combined.topLeftCorner(3,3) = k_trans.asDiagonal();
	k_combined.bottomRightCorner(3,3) = k_rot.asDiagonal();
	cout << "After definitions of k mat, before b mat"<<endl; 
	b_des_vec << 4000,4000,4000,200,200,200;
	b_des_inv = b_des_vec.asDiagonal().inverse();
	cout << "After definitions of b mat"<<endl; 
	current_frame = "Task";
	current_frame_msg.data = current_frame;
	cout<<"After defining new k and b mats"<<endl;

	// Declare joint state, pose stamped, pose, and wrench variables
	sensor_msgs::JointState desired_joint_state;
	geometry_msgs::PoseStamped cartesian_log, virtual_attractor_log, bumpless_virtual_attractor_log;
	geometry_msgs::Pose virtual_attractor;
	geometry_msgs::Wrench transformed_wrench;
	geometry_msgs::Wrench current_frame_wrench;

	// Declare a zero PoseStamped for robot frame visualization
	geometry_msgs::PoseStamped robot_frame_pose_stamped;
	robot_frame_pose_stamped.header.frame_id = "map";

	// ROS: Format messages for ROS communications
	cartesian_log.header.frame_id = "map";
	virtual_attractor_log.header.frame_id = "current_frame";
	bumpless_virtual_attractor_log.header.frame_id = "map";
	desired_joint_state.position.resize(6);
	desired_joint_state.velocity.resize(6);
	last_desired_joint_state_.position.resize(6);
	last_desired_joint_state_.velocity.resize(6);
	

	// TODO CHANGE FOR ROBOT AGNOSTIC
	// Declare our sensor's static transform 
	Eigen::Affine3d sensor_with_respect_to_flange;
	Eigen::Matrix3d sensor_rotation;
	Eigen::Vector3d sensor_origin;
	sensor_origin<<0,0,0.08;
	sensor_rotation<<0,-1,0,
				1,0,0,
				0,0,1;
	sensor_with_respect_to_flange.linear() = sensor_rotation;
	sensor_with_respect_to_flange.translation() = sensor_origin;

	// Declare our tool's static transform
	Eigen::Affine3d tool_with_repsect_to_sensor;
	Eigen::Matrix3d tool_with_repsect_to_sensor_rotation = Eigen::Matrix3d::Identity();
	Eigen::Vector3d tool_with_repsect_to_sensor_translation;
	tool_with_repsect_to_sensor_translation<<0,0,0.21; // 0,0,0.1 is with cyllinder, 0.24 -> 0.23 -> 0.22 with pencil, 
	tool_with_repsect_to_sensor.linear() = tool_with_repsect_to_sensor_rotation;
	tool_with_repsect_to_sensor.translation() = tool_with_repsect_to_sensor_translation;
	
	// ROS: Wait until there are some valid values of joint states from the robot controller
	// ros::spinOnce() allows subscribers to look at their topics
	while(!jnt_state_update) ros::spinOnce();

	// Initialize current end effector position and use values recieved from hand controller as offsets to that value

	Eigen::Affine3d flange_with_respect_to_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
	sensor_with_respect_to_robot = flange_with_respect_to_robot * sensor_with_respect_to_flange;
	tool_with_respect_to_robot_ = sensor_with_respect_to_robot * tool_with_repsect_to_sensor;

	// Define initial task frame as the ft frame wrt base, can be updated via service call
	task_frame_with_respect_to_robot_ = sensor_with_respect_to_robot; //! change to tool_with_respect_to_robot_ if IP is tool tip
	sensor_with_respect_to_current = task_frame_with_respect_to_robot_.inverse() * sensor_with_respect_to_robot; // check if this math is correct

	// Define default/starting transform for the tool stowage frame, set from values obtained with a working task frame
	stowage_frame_with_respect_to_robot_.translation() = Eigen::Vector3d(0.305318775107,0.00417424672753,0.702955076039);
	Eigen::Quaterniond rot(0.494044627121,0.484363114784,0.513944396825,0.507122703516);
	stowage_frame_with_respect_to_robot_.linear() = rot.toRotationMatrix();


	// Define 
	Eigen::VectorXd initial_end_effector_pose = Eigen::VectorXd::Zero(6); 
	initial_end_effector_pose.head(3) = tool_with_respect_to_robot_.translation();
	initial_end_effector_pose.tail(3) = decompose_rot_mat(tool_with_respect_to_robot_.linear());

	// Find the tool's x, y, and z vectors with respect to the robot; needed for certain skills
	Eigen::MatrixXd tool_R = tool_with_respect_to_robot_.linear();
	Eigen::Vector3d x_vec = tool_R.col(0);
	Eigen::Vector3d y_vec = tool_R.col(1);
	Eigen::Vector3d z_vec = tool_R.col(2);
	x_vec_message.x = x_vec(0);
	x_vec_message.y = x_vec(1);
	x_vec_message.z = x_vec(2);
	y_vec_message.x = y_vec(0);
	y_vec_message.y = y_vec(1);
	y_vec_message.z = y_vec(2);
	z_vec_message.x = z_vec(0);
	z_vec_message.y = z_vec(1);
	z_vec_message.z = z_vec(2);

	// Find the task's x, y, and z vectors with respect to the robot; needed for certain skills
	task_frame_rotation_matrix_ = task_frame_with_respect_to_robot_.linear();
	x_vec_task_ = task_frame_rotation_matrix_.col(0);
	y_vec_task_ = task_frame_rotation_matrix_.col(1);
	z_vec_task_ = task_frame_rotation_matrix_.col(2);
	x_vec_task_message_.x = x_vec_task_(0);
	x_vec_task_message_.y = x_vec_task_(1);
	x_vec_task_message_.z = x_vec_task_(2);
	y_vec_task_message_.x = y_vec_task_(0);
	y_vec_task_message_.y = y_vec_task_(1);
	y_vec_task_message_.z = y_vec_task_(2);
	z_vec_task_message_.x = z_vec_task_(0);
	z_vec_task_message_.y = z_vec_task_(1);
	z_vec_task_message_.z = z_vec_task_(2);

	// Define task frame as a pose stamped
	task_frame_pose_stamped.pose.position.x = task_frame_with_respect_to_robot_.translation()(0);
	task_frame_pose_stamped.pose.position.y = task_frame_with_respect_to_robot_.translation()(1);
	task_frame_pose_stamped.pose.position.z = task_frame_with_respect_to_robot_.translation()(2);
	Eigen::Quaterniond task_frame_orientation_quaternion(task_frame_with_respect_to_robot_.linear());
	task_frame_pose_stamped.pose.orientation.x = task_frame_orientation_quaternion.x();
	task_frame_pose_stamped.pose.orientation.y = task_frame_orientation_quaternion.y();
	task_frame_pose_stamped.pose.orientation.z = task_frame_orientation_quaternion.z();
	task_frame_pose_stamped.pose.orientation.w = task_frame_orientation_quaternion.w();
	task_frame_pose_stamped.header.frame_id = "map";

	// Find the tool stowage bay's x, y, and z vectors with respect to the robot; needed for certain skills
	stowage_frame_rotation_matrix_ = stowage_frame_with_respect_to_robot_.linear();
	x_vec_stowage_ = stowage_frame_rotation_matrix_.col(0);
	y_vec_stowage_ = stowage_frame_rotation_matrix_.col(1);
	z_vec_stowage_ = stowage_frame_rotation_matrix_.col(2);
	x_vec_stowage_message_.x = x_vec_stowage_(0);
	x_vec_stowage_message_.y = x_vec_stowage_(1);
	x_vec_stowage_message_.z = x_vec_stowage_(2);
	y_vec_stowage_message_.x = y_vec_stowage_(0);
	y_vec_stowage_message_.y = y_vec_stowage_(1);
	y_vec_stowage_message_.z = y_vec_stowage_(2);
	z_vec_stowage_message_.x = z_vec_stowage_(0);
	z_vec_stowage_message_.y = z_vec_stowage_(1);
	z_vec_stowage_message_.z = z_vec_stowage_(2); 

	// Define stowage frame as a pose stamped
	stowage_frame_pose_stamped.pose.position.x = stowage_frame_with_respect_to_robot_.translation()(0);
	stowage_frame_pose_stamped.pose.position.y = stowage_frame_with_respect_to_robot_.translation()(1);
	stowage_frame_pose_stamped.pose.position.z = stowage_frame_with_respect_to_robot_.translation()(2);
	Eigen::Quaterniond stowage_frame_orientation_quaternion(stowage_frame_with_respect_to_robot_.linear());
	stowage_frame_pose_stamped.pose.orientation.x = stowage_frame_orientation_quaternion.x();
	stowage_frame_pose_stamped.pose.orientation.y = stowage_frame_orientation_quaternion.y();
	stowage_frame_pose_stamped.pose.orientation.z = stowage_frame_orientation_quaternion.z();
	stowage_frame_pose_stamped.pose.orientation.w = stowage_frame_orientation_quaternion.w();
	stowage_frame_pose_stamped.header.frame_id = "map";

	// Find the ft's x, y, and z vectors with respect to the robot; needed for certain skills
	ft_frame_rotation_matrix_ = sensor_with_respect_to_robot.linear();
	x_vec_ft_ = ft_frame_rotation_matrix_.col(0);
	y_vec_ft_ = ft_frame_rotation_matrix_.col(1);
	z_vec_ft_ = ft_frame_rotation_matrix_.col(2);
	x_vec_ft_message_.x = x_vec_ft_(0);
	x_vec_ft_message_.y = x_vec_ft_(1);
	x_vec_ft_message_.z = x_vec_ft_(2);
	y_vec_ft_message_.x = y_vec_ft_(0);
	y_vec_ft_message_.y = y_vec_ft_(1);
	y_vec_ft_message_.z = y_vec_ft_(2);
	z_vec_ft_message_.x = z_vec_ft_(0);
	z_vec_ft_message_.y = z_vec_ft_(1);
	z_vec_ft_message_.z = z_vec_ft_(2);

	// Define ft frame as a pose stamped
	ft_frame_pose_stamped.pose.position.x = sensor_with_respect_to_robot.translation()(0);
	ft_frame_pose_stamped.pose.position.y = sensor_with_respect_to_robot.translation()(1);
	ft_frame_pose_stamped.pose.position.z = sensor_with_respect_to_robot.translation()(2);
	Eigen::Quaterniond ft_frame_orientation_quaternion(sensor_with_respect_to_robot.linear());
	ft_frame_pose_stamped.pose.orientation.x = ft_frame_orientation_quaternion.x();
	ft_frame_pose_stamped.pose.orientation.y = ft_frame_orientation_quaternion.y();
	ft_frame_pose_stamped.pose.orientation.z = ft_frame_orientation_quaternion.z();
	ft_frame_pose_stamped.pose.orientation.w = ft_frame_orientation_quaternion.w();
	ft_frame_pose_stamped.header.frame_id = "map";

	// Define our initial transform for the ft frame we are complying and moving in
	ft_frame_transform_stamped.transform.translation.x = ft_frame_pose_stamped.pose.position.x;
	ft_frame_transform_stamped.transform.translation.y = ft_frame_pose_stamped.pose.position.y;
	ft_frame_transform_stamped.transform.translation.z = ft_frame_pose_stamped.pose.position.z;
	ft_frame_transform_stamped.transform.rotation.x = ft_frame_pose_stamped.pose.orientation.x;
	ft_frame_transform_stamped.transform.rotation.y = ft_frame_pose_stamped.pose.orientation.y;
	ft_frame_transform_stamped.transform.rotation.z = ft_frame_pose_stamped.pose.orientation.z;
	ft_frame_transform_stamped.transform.rotation.w = ft_frame_pose_stamped.pose.orientation.w;
	ft_frame_transform_stamped.header.frame_id = "map";
	ft_frame_transform_stamped.child_frame_id = "ft_frame";
	

	// Define our initial transform for the current frame we are complying and moving in
	current_frame_transform_stamped.transform.translation.x = task_frame_pose_stamped.pose.position.x;
	current_frame_transform_stamped.transform.translation.y = task_frame_pose_stamped.pose.position.y;
	current_frame_transform_stamped.transform.translation.z = task_frame_pose_stamped.pose.position.z;
	current_frame_transform_stamped.transform.rotation.x = task_frame_pose_stamped.pose.orientation.x;
	current_frame_transform_stamped.transform.rotation.y = task_frame_pose_stamped.pose.orientation.y;
	current_frame_transform_stamped.transform.rotation.z = task_frame_pose_stamped.pose.orientation.z;
	current_frame_transform_stamped.transform.rotation.w = task_frame_pose_stamped.pose.orientation.w;
	current_frame_transform_stamped.header.frame_id = "map";
	current_frame_transform_stamped.child_frame_id = "current_frame";

	//! Calculate the ft pose in the current frame (tf listener or tf_echo??) to store into interaction port

	// Define interaction port frame as a pose stamped, initially there is no difference between the IP and the current frame/task frame at initialization
	interaction_port_frame_pose_stamped.pose.position.x = sensor_with_respect_to_current.translation()(0);
	interaction_port_frame_pose_stamped.pose.position.y = sensor_with_respect_to_current.translation()(1);
	interaction_port_frame_pose_stamped.pose.position.z = sensor_with_respect_to_current.translation()(2);
	Eigen::Quaterniond interaction_port_frame_orientation_quaternion(sensor_with_respect_to_current.linear());
	interaction_port_frame_pose_stamped.pose.orientation.x = interaction_port_frame_orientation_quaternion.x();
	interaction_port_frame_pose_stamped.pose.orientation.y = interaction_port_frame_orientation_quaternion.y();
	interaction_port_frame_pose_stamped.pose.orientation.z = interaction_port_frame_orientation_quaternion.z();
	interaction_port_frame_pose_stamped.pose.orientation.w = interaction_port_frame_orientation_quaternion.w();
	interaction_port_frame_pose_stamped.header.frame_id = "current_frame"; //! Change this based on what the frame currently is defined as, maybe also define a second tf broadcaster

	// Declare the bumpless virtual attractor's pose
	Eigen::VectorXd bumpless_virtual_attractor_position(3);
	Eigen::Vector3d bumpless_virtual_attractor_angles;

	// Define the freeze/latch_mode variables
	bool first_loop_after_freeze = true;
	freeze_mode = true;  // here set freeze mode to be true, always start in freeze mode

	// store the current joint states as the frozen joints before entering the loop
	for(int i = 0; i < 6; i++) frozen_joint_states_(i) = joint_states_(i);

	// Define rate at which we loop through main program
	ros::Rate naptime(1/dt_);
	
	// Main loop
	while(ros::ok()) {
		// ROS: Subscribers get info
		ros::spinOnce();

		// Initialize jacobians 
		// TODO CHANGE FOR ROBOT AGNOSTIC
		// THIS IS NOT ROBOT AGNOSTIC, but if this section is replaced with the appropriate jacobian calculation for a new robot, then the program will run properly. 
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
		flange_with_respect_to_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
		sensor_with_respect_to_robot = flange_with_respect_to_robot * sensor_with_respect_to_flange;
		tool_with_respect_to_robot_ = sensor_with_respect_to_robot * tool_with_repsect_to_sensor;
		Eigen::FullPivLU<Eigen::MatrixXd> lu_jac(jacobian);
		if(!lu_jac.isInvertible()) continue; // Jump to the next iteration in the loop if inverse is not defined
		Eigen::MatrixXd jacobian_inv = lu_jac.inverse(); // TODO what to do when matrix is non invertible?   
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();

		// Find current end effector pose
		current_end_effector_pose.head(3) = tool_with_respect_to_robot_.translation();
		current_end_effector_pose.tail(3) = decompose_rot_mat(tool_with_respect_to_robot_.linear()); 
		Eigen::Quaterniond flange_quat(tool_with_respect_to_robot_.linear());

		// Find tool's current x, y, and z vector for use in skills 
		tool_R = tool_with_respect_to_robot_.linear();
		Eigen::Vector3d x_vec = tool_R.col(0);
		Eigen::Vector3d y_vec = tool_R.col(1);
		Eigen::Vector3d z_vec = tool_R.col(2);
		x_vec_message.x = x_vec(0);
		x_vec_message.y = x_vec(1);
		x_vec_message.z = x_vec(2);
		y_vec_message.x = y_vec(0);
		y_vec_message.y = y_vec(1);
		y_vec_message.z = y_vec(2);
		z_vec_message.x = z_vec(0);
		z_vec_message.y = z_vec(1);
		z_vec_message.z = z_vec(2);

		//! Update the interaction port values, from tf of ft in current frame

		// Update wrench with respect to robot
		wrench_with_respect_to_robot.head(3) = sensor_with_respect_to_robot.linear() * (wrench_body_coords_.head(3));
		wrench_with_respect_to_robot.tail(3) = sensor_with_respect_to_robot.linear() * (wrench_body_coords_.tail(3));

		// WRench with respect to the current frame
		wrench_with_respect_to_current.head(3) = sensor_with_respect_to_current.linear() * (wrench_body_coords_.head(3));
		wrench_with_respect_to_current.tail(3) = sensor_with_respect_to_current.linear() * (wrench_body_coords_.tail(3));

		// Update wrench in the current frame: (we need orientation of the FT in the current frame, interaction port)

		//! Change this to be in the set frame, code is commented out
		// Compute virtual attractor forces
		// If the controller has just started, use the bumpless virtual attractor
		if (!freeze_mode && first_loop_after_freeze) { //TODO Change to be based off of FT sensor
			
			//TODO Code for attractor and tool pose in current_frame the bumpless attr pose calculated here
			// Define the virtual position based on the wrench on the FT sensor in the current frame, with the interaction port defined in the current frame (convert from )
			bumpless_virtual_attractor_position = -(k_trans.asDiagonal().inverse() * wrench_with_respect_to_current.head(3)); 
			// Then add this offset onto the current pose of the interaction port
			bumpless_virtual_attractor_position(0) += interaction_port_frame_pose_stamped.pose.position.x;
			bumpless_virtual_attractor_position(1) += interaction_port_frame_pose_stamped.pose.position.y;
			bumpless_virtual_attractor_position(2) += interaction_port_frame_pose_stamped.pose.position.z;

			// Update virtual attractor position to bumpless virtual attractor position
			virtual_attractor_pos(0) = bumpless_virtual_attractor_position(0);
			virtual_attractor_pos(1) = bumpless_virtual_attractor_position(1);
			virtual_attractor_pos(2) = bumpless_virtual_attractor_position(2);


			// Calculate the orientation of the attractor based on the felt wrench, and then add on the orientation of the ft sensor
			// We get the offset of the attractor required, and then we can conver it to a rotation matrix, and then add that rotation on to the current orientation of the IP in the current frame
			bumpless_virtual_attractor_angles = -(k_rot.asDiagonal().inverse() * wrench_with_respect_to_current.tail(3));
			
			// Convert bumpless attr to rot matrix (define a func here, vec of angles to AA, then AA to rot)
			virtual_attractor_rotation_matrix = sensor_with_respect_to_current.linear() * rotation_matrix_from_vector_of_angles(bumpless_virtual_attractor_angles); //! not done yet
			// Take this rot mat, and then post multiply it to the current ft rotation matrix in the appropriate frame (IP, maybe define a new Affine for it?)

			// Another implementation is to combine the small angle representations of the IP and this calculated delta and then convert from vector of angles (AA) to Rotation matrix
			// calculate as angle axis to rot, no euler
			// Get the angle axis representation of the ft, multiply the axis by the angle


			//! !!!!!!!!!!!!!!!!!
			//! OLD CONTROL LAW
			//! !!!!!!!!!!!!!!!!!
			// // Find bumpless virtual attractor position: the negative force devided by the translational spring constant, plus the end effector position
			// bumpless_virtual_attractor_position = -wrench_with_respect_to_robot.head(3) / K_virtual_translational + sensor_with_respect_to_robot.translation();
			// // Update virtual attractor position to bumpless virtual attractor position
			// virtual_attractor_pos(0) = bumpless_virtual_attractor_position(0);
			// virtual_attractor_pos(1) = bumpless_virtual_attractor_position(1);
			// virtual_attractor_pos(2) = bumpless_virtual_attractor_position(2);
			// // Find bumpless virtual attractor angles: (the negative torque divided by the anglular spring constant) plus (the angle of the tool with respect to the robot)
			// bumpless_virtual_attractor_angles(0) =  -wrench_with_respect_to_robot(3) / K_virtual_angular + decompose_rot_mat(sensor_with_respect_to_robot.linear())(0);
			// bumpless_virtual_attractor_angles(1) =  -wrench_with_respect_to_robot(4) / K_virtual_angular + decompose_rot_mat(sensor_with_respect_to_robot.linear())(1);
			// bumpless_virtual_attractor_angles(2) =  -wrench_with_respect_to_robot(5) / K_virtual_angular + decompose_rot_mat(sensor_with_respect_to_robot.linear())(2);
			// // Update the virtual attractor rotation matrix
			// virtual_attractor_rotation_matrix = rotation_matrix_from_euler_angles(bumpless_virtual_attractor_angles);

			// //! Define our virtual wrench, we just swap out the K_virts with the matrices as diagonals (or the box), and use the variables that are defined in the current frame (virt attr and IP)
			// // Calculate the virtual forces
			// virtual_force.head(3) = K_virtual_translational * (virtual_attractor_pos - sensor_with_respect_to_robot.translation());
			// virtual_force.tail(3) = K_virtual_angular * (delta_phi_from_rots(sensor_with_respect_to_robot.linear(), virtual_attractor_rotation_matrix));

			// Output 
			cout<<"Bumpless attractor used"<<endl;
			cout<<"wrench torques"<<endl<<wrench_with_respect_to_robot.tail(3)<<endl;
			cout<<"decomposed angles of the tool frame"<<endl<<decompose_rot_mat(sensor_with_respect_to_robot.linear())<<endl;
			cout<<"bumpless virtual attractor pose: "<<endl;
			cout<<bumpless_virtual_attractor_position<<endl;
		}

		// If the controller has not just started, use the normal virtual attractor
		else {
			// If we have established a virtual attractor
			if(virtual_attractor_established) {
				//! Define our virtual wrench, we just swap out the K_virts with the matrices as diagonals (or the box), and use the variables that are defined in the current frame (virt attr and IP)
				// Calculate the virtual forces
				// virtual_force.head(3) = K_virtual_translational * (virtual_attractor_pos - sensor_with_respect_to_robot.translation());
				// virtual_force.tail(3) = K_virtual_angular * (delta_phi_from_rots(sensor_with_respect_to_robot.linear(), virtual_attractor_rotation_matrix));
				// Output
				cout<<"virtual attractor pose"<<endl;
				cout<<virtual_attractor_pos<<endl;
				cout<<decompose_rot_mat(virtual_attractor_rotation_matrix)<<endl;
			}
			// If we haven't established a virtual attractor
			else{
				// Set virtual force to 0
				virtual_force<<0,0,0,0,0,0;
				cout<<"No virtual attractor command received"<<endl;	
			}
		}

		cout<<"virtual force: "<<endl<<virtual_force<<endl;
		cout<<"current frame: "<<current_frame<<endl;

		//! new changes, calculate the virtual wrench out here
		if(virtual_attractor_established && !freeze_mode){
			// calculate the virtual force 
			virtual_force.head(3) = K_virtual_translational * (virtual_attractor_pos - sensor_with_respect_to_current.translation());
			virtual_force.tail(3) = K_virtual_angular * (delta_phi_from_rots(sensor_with_respect_to_current.linear(), virtual_attractor_rotation_matrix));
		}else{
			// in a position hold
			virtual_force<<0,0,0,0,0,0;
		}

		// CONTROL LAW BEGIN
		//! Remove cartesian acceleration, just do:
		// Combine the virtual and measured wrench in current frame
		Eigen::VectorXd combined_wrench = virtual_force + wrench_with_respect_to_current;
		// Calculate our desired twist in current frame through accommodation control
		Eigen::VectorXd desired_twist_in_current_frame = b_des_inv * combined_wrench;
		// Transform the twist into the base frame for the jacobian
		desired_twist.head(3) = sensor_with_respect_to_robot.linear() * desired_twist_in_current_frame.head(3);
		desired_twist.tail(3) = sensor_with_respect_to_robot.linear() * desired_twist_in_current_frame.tail(3);

		// Check if we are frozen or not
		if(freeze_mode){
			desired_twist<<0,0,0,0,0,0;
			// Output
			cout<<"freeze mode on"<<endl;
		}
		cout<<"Desired twist: "<<endl<<desired_twist<<endl;
		cout<<"Combined Wrench: "<<endl<<combined_wrench<<endl;

		// Calculate the desired twists with the two damping gains
		// desired_twist_with_gain.head(3) = -B_virtual_translational * desired_twist.head(3);
		// desired_twist_with_gain.tail(3) = -B_virtual_rotational * desired_twist.tail(3);
		// // Calculate the desired cartesian accelleration
		// desired_cartesian_acceleration = inertia_matrix_inverted*(desired_twist_with_gain + wrench_with_respect_to_robot + virtual_force);
		// CONTROL LAW END

		// // Calculate the desired twist by integrating the acceleration
		// if(!freeze_mode) desired_twist += desired_cartesian_acceleration*dt_;
		// // Zero the twist if freeze/latched mode is on
		// else {
		// 	desired_twist<<0,0,0,0,0,0;
		// 	// Output
		// 	cout<<"freeze mode on"<<endl;
		// }

		// Calculate the desired joint velocities from the desired twist
		// TODO CHANGE FOR ROBOT AGNOSTIC
		// This is where it is not robot agnostic, if you want to change the robot, change the jacobian accordingly
		if(!freeze_mode) desired_joint_velocity = jacobian_inv*desired_twist;
		else desired_joint_velocity<<0,0,0,0,0,0;
									
		// Ensure that desired joint vel is within set safe limits
		if(desired_joint_velocity.norm() > MAX_JOINT_VELOCITY_NORM) desired_joint_velocity = (desired_joint_velocity / desired_joint_velocity.norm()) * MAX_JOINT_VELOCITY_NORM;
		
		// Calculate joint positions from velocities using Euler one step integration
		Eigen::MatrixXd des_jnt_pos = Eigen::VectorXd::Zero(6);
		if(!freeze_mode) des_jnt_pos = joint_states_ + (desired_joint_velocity * dt_);
		else des_jnt_pos = frozen_joint_states_;

		// Put velocity and pososition commands into Jointstate message and round
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; 
		for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(desired_joint_velocity(i) * 1000) /1000;

		// Publish desired jointstate
		//! Use freeze mode to not publish this if we are frozen 
		if(!freeze_mode){
			arm_publisher.publish(desired_joint_state);
			cout<<"Commanding joint pose"<<endl;
		}
			
		last_desired_joint_state_ = desired_joint_state;

		// Debug Output
		sensor_pose.head(3) = sensor_with_respect_to_current.translation();
		sensor_pose.tail(3) = decompose_rot_mat(sensor_with_respect_to_current.linear()); 
		cout<<"Current FT Pose in Current:"<<endl<<sensor_pose<<endl;

		// Publish cartesian coordinates of robot end effector
		cartesian_log.pose.position.x = current_end_effector_pose(0);
		cartesian_log.pose.position.y = current_end_effector_pose(1);
		cartesian_log.pose.position.z = current_end_effector_pose(2);
		cartesian_log.pose.orientation.w = flange_quat.w();
		cartesian_log.pose.orientation.x = flange_quat.x();
		cartesian_log.pose.orientation.y = flange_quat.y();
		cartesian_log.pose.orientation.z = flange_quat.z();
		cartesian_log.header.stamp = ros::Time::now();
		cart_log_pub.publish(cartesian_log);

		// Publish cartesian coordinates of FT sensor
		ft_frame_pose_stamped.pose.position.x = sensor_with_respect_to_robot.translation()(0);
		ft_frame_pose_stamped.pose.position.y = sensor_with_respect_to_robot.translation()(1);
		ft_frame_pose_stamped.pose.position.z = sensor_with_respect_to_robot.translation()(2);
		Eigen::Quaterniond ft_frame_orientation_quaternion(sensor_with_respect_to_robot.linear());
		ft_frame_pose_stamped.pose.orientation.x = ft_frame_orientation_quaternion.x();
		ft_frame_pose_stamped.pose.orientation.y = ft_frame_orientation_quaternion.y();
		ft_frame_pose_stamped.pose.orientation.z = ft_frame_orientation_quaternion.z();
		ft_frame_pose_stamped.pose.orientation.w = ft_frame_orientation_quaternion.w();
		ft_frame_pub.publish(ft_frame_pose_stamped);

		// Update the transform of the ft frame, to be used to calculate the interaction port pose
		ft_frame_transform_stamped.transform.translation.x = ft_frame_pose_stamped.pose.position.x;
		ft_frame_transform_stamped.transform.translation.y = ft_frame_pose_stamped.pose.position.y;
		ft_frame_transform_stamped.transform.translation.z = ft_frame_pose_stamped.pose.position.z;
		ft_frame_transform_stamped.transform.rotation.x = ft_frame_pose_stamped.pose.orientation.x;
		ft_frame_transform_stamped.transform.rotation.y = ft_frame_pose_stamped.pose.orientation.y;
		ft_frame_transform_stamped.transform.rotation.z = ft_frame_pose_stamped.pose.orientation.z;
		ft_frame_transform_stamped.transform.rotation.w = ft_frame_pose_stamped.pose.orientation.w;
		ft_frame_transform_stamped.header.stamp = ros::Time::now();
		// cout<<"Before ft tf broadcast"<<endl;
		br.sendTransform(ft_frame_transform_stamped);
		// cout<<"After ft tf broadcast"<<endl;

		// Publish virtual attractor coordinates
		virtual_attractor_log.pose.position.x = virtual_attractor_pos(0);
		virtual_attractor_log.pose.position.y = virtual_attractor_pos(1);
		virtual_attractor_log.pose.position.z = virtual_attractor_pos(2);
		virtual_attractor_log.pose.orientation.w = virt_quat.w();
		virtual_attractor_log.pose.orientation.x = virt_quat.x();
		virtual_attractor_log.pose.orientation.y = virt_quat.y();
		virtual_attractor_log.pose.orientation.z = virt_quat.z();
		virtual_attractor_log.header.stamp = ros::Time::now();
		virtual_attractor_after_tf.publish(virtual_attractor_log);

		// Publish zero posed stamp for robot frame visualization in rviz
		robot_frame_pose_stamped.header.stamp = ros::Time::now();
		robot_frame_pub.publish(robot_frame_pose_stamped);

		// Publish the task frame for visualization in rviz
		task_frame_pose_stamped.header.stamp = ros::Time::now();
		task_frame_pub.publish(task_frame_pose_stamped);

		// Publish the tool stowage bay frame for visualization in rviz
		stowage_frame_pose_stamped.header.stamp = ros::Time::now();
		stowage_frame_pub.publish(stowage_frame_pose_stamped);

		// Publish the tool stowage bay frame for visualization in rviz
		//! Update TF here, we will have to manually calculate it in the current frame
		if(!strcmp(current_frame.c_str(),"Task")){
			sensor_with_respect_to_current = task_frame_with_respect_to_robot_.inverse() * sensor_with_respect_to_robot;
		}
		else{
			sensor_with_respect_to_current = stowage_frame_with_respect_to_robot_.inverse() * sensor_with_respect_to_robot;
		}
		interaction_port_frame_pose_stamped.pose.position.x = sensor_with_respect_to_current.translation()(0);
		interaction_port_frame_pose_stamped.pose.position.y = sensor_with_respect_to_current.translation()(1);
		interaction_port_frame_pose_stamped.pose.position.z = sensor_with_respect_to_current.translation()(2);
		Eigen::Quaterniond interaction_port_frame_orientation_quaternion(sensor_with_respect_to_current.linear());
		interaction_port_frame_pose_stamped.pose.orientation.x = interaction_port_frame_orientation_quaternion.x();
		interaction_port_frame_pose_stamped.pose.orientation.y = interaction_port_frame_orientation_quaternion.y();
		interaction_port_frame_pose_stamped.pose.orientation.z = interaction_port_frame_orientation_quaternion.z();
		interaction_port_frame_pose_stamped.pose.orientation.w = interaction_port_frame_orientation_quaternion.w();
		interaction_port_frame_pose_stamped.header.stamp = ros::Time::now();
		interaction_port_frame_pub.publish(interaction_port_frame_pose_stamped);

		//! TEST AND MAKE SURE THIS WORKS FOR INTERACTION PORT
		// Broadcast the current frame transform
		current_frame_transform_stamped.header.stamp = ros::Time::now();
		br.sendTransform(current_frame_transform_stamped);

		// Publish force-torque values transformed into robot frame
		transformed_wrench.force.x = wrench_with_respect_to_robot(0);
		transformed_wrench.force.y = wrench_with_respect_to_robot(1);
		transformed_wrench.force.z = wrench_with_respect_to_robot(2);
		transformed_wrench.torque.x = wrench_with_respect_to_robot(3);
		transformed_wrench.torque.y = wrench_with_respect_to_robot(4);
		transformed_wrench.torque.z = wrench_with_respect_to_robot(5);
		ft_pub.publish(transformed_wrench);

		// Publish the wrench transformed in the current frame
		current_frame_wrench.force.x = wrench_with_respect_to_current(0);
		current_frame_wrench.force.y = wrench_with_respect_to_current(1);
		current_frame_wrench.force.z = wrench_with_respect_to_current(2);
		current_frame_wrench.torque.x = wrench_with_respect_to_current(3);
		current_frame_wrench.torque.y = wrench_with_respect_to_current(4);
		current_frame_wrench.torque.z = wrench_with_respect_to_current(5);
		current_frame_ft_pub.publish(current_frame_wrench);

		// Update FT vec frame
		ft_frame_rotation_matrix_ = sensor_with_respect_to_robot.linear();
		x_vec_ft_ = ft_frame_rotation_matrix_.col(0);
		y_vec_ft_ = ft_frame_rotation_matrix_.col(1);
		z_vec_ft_ = ft_frame_rotation_matrix_.col(2);
		x_vec_ft_message_.x = x_vec_ft_(0);
		x_vec_ft_message_.y = x_vec_ft_(1);
		x_vec_ft_message_.z = x_vec_ft_(2);
		y_vec_ft_message_.x = y_vec_ft_(0);
		y_vec_ft_message_.y = y_vec_ft_(1);
		y_vec_ft_message_.z = y_vec_ft_(2);
		z_vec_ft_message_.x = z_vec_ft_(0);
		z_vec_ft_message_.y = z_vec_ft_(1);
		z_vec_ft_message_.z = z_vec_ft_(2);

		// Publish tool frame vectors
		x_vec_pub.publish(x_vec_message);
		y_vec_pub.publish(y_vec_message);
		z_vec_pub.publish(z_vec_message);

		// Publish task frame vectors
		x_vec_task_pub.publish(x_vec_task_message_);
		y_vec_task_pub.publish(y_vec_task_message_);
		z_vec_task_pub.publish(z_vec_task_message_);

		// Publish stowage frame vectors
		x_vec_stowage_pub.publish(x_vec_stowage_message_);
		y_vec_stowage_pub.publish(y_vec_stowage_message_);
		z_vec_stowage_pub.publish(z_vec_stowage_message_);

		// Publish ft frame vectors
		x_vec_ft_pub.publish(x_vec_ft_message_);
		y_vec_ft_pub.publish(y_vec_ft_message_);
		z_vec_ft_pub.publish(z_vec_ft_message_);


		// Pulish freeze mode status
		freeze_mode_pub.publish(freeze_mode_status);

		// Publish current frame
		current_frame_pub.publish(current_frame_msg);

		// Set freeze/latched mode checkers
		if (freeze_mode) first_loop_after_freeze = true;
		else first_loop_after_freeze = false;

		// Ensure update rate is consistent 
		naptime.sleep();
		ros::spinOnce();
	}
}