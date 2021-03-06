//This node reads egm feedback messages and publishes them onto ROS topic -
//Also, to keep the robot controller 'interested' in the node, it sends the recieved joint values back to the controller
//The above mess can be remedied by increasing \CommTimeout in the RAPID program. TODO!


#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <abb_libegm/egm_controller_interface.h>
#include <sys/socket.h>
#include <errno.h>
#include <cmath>
#include <math.h>

int g_seq_no = 0;
vector<double> g_desired_joint_angles;
bool jnt_cmd = true, cart_cmd = false;
geometry_msgs::Pose g_des_pose;
geometry_msgs::Twist g_des_twist;
vector<double> home_vec{0,-28,7,0,5,0};
vector<double> zero_vec{0,0,0,0,0,0};
int dbg;
sensor_msgs::JointState g_joint_state;

uint32_t get_tick() {
	struct timespec now;
  	if (clock_gettime(CLOCK_MONOTONIC, &now))
    	return 0;
	return now.tv_sec * 1000 + now.tv_nsec / 1000;
}

vector<double> rad2deg_vect(vector<double> input_vect) {
	vector<double> output_vect;
	for(int i = 0; i < input_vect.size(); i++) {
		output_vect.push_back(input_vect[i] * 180.0 / M_PI);
	}
	return output_vect;
}


void JointPosCallBack(const sensor_msgs::JointState::ConstPtr& joint_state_message) {

	if(joint_state_message->position.size() == 6) g_joint_state.position = rad2deg_vect(joint_state_message->position);
	if(joint_state_message->velocity.size() == 6) g_joint_state.velocity = rad2deg_vect(joint_state_message->velocity);
	jnt_cmd = true;
}





void CartesianCmdCb(const geometry_msgs::Pose::ConstPtr& cart_cmd_pose) {
	g_des_pose = *cart_cmd_pose;
	cart_cmd = true;

}

void TwistCmdCb(const geometry_msgs::Twist::ConstPtr& twist_cmd) {
	g_des_twist = *twist_cmd;
}

void ShowRobotMessage(abb::egm::EgmRobot *rob_msg) {
	if(rob_msg->has_header()); 
	//write this function
	//check for header, seq_no, timestamp, and mtype
	//if yes, print out joint angs and cart pose
	//extend it to publish 
}

void CreateEgmSensorMessage(abb::egm::EgmSensor* sensor, sensor_msgs::JointState joint_state) {
	abb::egm::EgmHeader *header = new abb::egm::EgmHeader();
	header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
	header->set_seqno(g_seq_no++);
	header->set_tm(get_tick());
	sensor->set_allocated_header(header);
	abb::egm::EgmPlanned *planned = new abb::egm::EgmPlanned();
	abb::egm::EgmJoints *joints = new abb::egm::EgmJoints();
	
	abb::egm::EgmJoints *joint_vel = new abb::egm::EgmJoints();
	abb::egm::EgmSpeedRef *speedRef = new abb::egm::EgmSpeedRef();

	for(int i = 0; i < 6; i++) {
		joints->add_joints(joint_state.position[i]);
	}
	planned->set_allocated_joints(joints);
	sensor->set_allocated_planned(planned);

	for(int i = 0; i < 6; i++) {
		joint_vel->add_joints(joint_state.velocity[i]);
	}
	
	speedRef->set_allocated_joints(joint_vel);
	sensor->set_allocated_speedref(speedRef);

}

void CreateEgmSensorMessage(abb::egm::EgmSensor* sensor, geometry_msgs::Pose pose, geometry_msgs::Twist twist) {
	abb::egm::EgmHeader *header = new abb::egm::EgmHeader();
	header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
	header->set_seqno(g_seq_no++);
	header->set_tm(get_tick());
	sensor->set_allocated_header(header);
	abb::egm::EgmPlanned *planned = new abb::egm::EgmPlanned();
	abb::egm::EgmCartesianSpeed *cs = new abb::egm::EgmCartesianSpeed();
	abb::egm::EgmSpeedRef *speedRef = new abb::egm::EgmSpeedRef();
	abb::egm::EgmPose *pos = new abb::egm::EgmPose();
	abb::egm::EgmCartesian *cart = new abb::egm::EgmCartesian();
	abb::egm::EgmEuler *eu = new abb::egm::EgmEuler();
	
	eu->set_x(pose.orientation.x);
	eu->set_y(pose.orientation.y);
	eu->set_z(pose.orientation.z);
	
	pos->set_allocated_euler(eu);
	
	cart->set_x(pose.position.x);
	cart->set_y(pose.position.y);
	cart->set_z(pose.position.z);
	
	pos->set_allocated_pos(cart);
	
	planned->set_allocated_cartesian(pos);
	
	cs->add_value(twist.linear.x);
	cs->add_value(twist.linear.y);
	cs->add_value(twist.linear.z);
	cs->add_value(twist.angular.x);
	cs->add_value(twist.angular.y);
	cs->add_value(twist.angular.z);
	
	speedRef->set_allocated_cartesians(cs);
	sensor->set_allocated_planned(planned);
	sensor->set_allocated_speedref(speedRef);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "custom_egm_test");
	ros::NodeHandle nh;
	ros::Publisher send_cmd = nh.advertise<std_msgs::Float64>("jnt4_cmd",1);
	ros::Publisher fdbk = nh.advertise<std_msgs::Float64>("jnt4_fdbk",1);
	ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_state",1);
	ros::Publisher joint_command_log = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1);
	sensor_msgs::JointState jnt_cmd_log;
	double val = 0.001;
	double dt_ = 0.01;
	int flips = 0;
	double dawn_of_time = -1;
	sensor_msgs::JointState jointstate;
	jointstate.position.resize(6);
	char *robot_ip = "192.168.125.1";
	char *serv_ip = "192.168.125.3";
	//char *msg = "R";
	std::string send_string;
	unsigned char buf[1024];
	int recvlen;
	int fd;
	int enable = 1;
	struct sockaddr_in serv_addr;
	struct sockaddr_in client_addr;
	struct timeval timeout;
	timeout.tv_sec = 2;
	timeout.tv_usec = 0;
	const unsigned short port_number = 6510;
	const unsigned short port_number_robot = 64007;
	memset(&serv_addr, 0, sizeof(serv_addr));
	
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port_number);
	
	client_addr.sin_family = AF_INET;
	client_addr.sin_addr.s_addr = inet_addr("192.168.125.1");
	//client_addr.sin_port = htons(port_number_robot);
	

	socklen_t addrlen = sizeof(struct sockaddr_in);
	
	abb::egm::EgmRobot *robot_message = new abb::egm::EgmRobot();
	abb::egm::EgmSensor *sensor_message = new abb::egm::EgmSensor();
	
	

	if((fd = socket(AF_INET,SOCK_DGRAM,0)) < 0) {
		ROS_WARN("Cannot create socket, go home");
	}

	if(bind(fd,(struct sockaddr *) &serv_addr, addrlen) < 0) {
		ROS_WARN("Cannot bind, go home\n");
		perror("error is : ");
	}
	
	if(setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout,sizeof(timeout)) < 0) ROS_INFO("setsockopt failed");
	
	g_desired_joint_angles = home_vec;
	g_joint_state.position = home_vec;
	g_joint_state.velocity = zero_vec;
	ros::Rate naptime(1/dt_);
	
	double start_time = ros::Time::now().toSec();
	ROS_INFO("waiting");
	double t = 0;
	double omega = 10;
	double fake_epoch_s;
	while(dawn_of_time  == -1) {
		recvlen = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&client_addr, &addrlen);
		if(recvlen > 0) {
			ros::spinOnce();
			if(robot_message->ParseFromArray(buf,recvlen)) { 
				if(robot_message->has_feedback()) {
					dawn_of_time = robot_message->header().tm();
					ROS_INFO_STREAM("Dawn of time set"<<dawn_of_time);
					fake_epoch_s = ros::Time::now().toSec();
	
				}
			}
		}
	}



	while(ros::ok()) {
	
		double sine_start_time = ros::Time::now().toSec();
		recvlen = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&client_addr, &addrlen);
		if(recvlen > 0) {
			ros::spinOnce();
			
			if(robot_message->ParseFromArray(buf,recvlen)) { 
				
				if(robot_message->has_feedback()) {
				
					for(int i = 0; i < 6; i++) jointstate.position[i] = robot_message->feedback().joints().joints(i);
					jointstate.position = rad2deg_vect(jointstate.position);
					
					g_seq_no = robot_message->header().seqno();
					double ticks = 	robot_message->header().tm() - dawn_of_time;
					double curr_sec = fake_epoch_s + ticks/1000;
					//jointstate.header.stamp = ros::Time(curr_sec);
					//jointstate.header.stamp.sec = (int) ticks/1000;
					//jointstate.header.stamp.nsec = (ticks - jointstate.header.stamp.sec*1000)/1000;			
					jointstate.header.stamp = ros::Time::now();
					joint_state_publisher.publish(jointstate);
					val = 1 * sin(omega*(t));
					t+=dt_;
					home_vec[4] = val;
					
								

								
					abb::egm::EgmHeader *header = new abb::egm::EgmHeader();
					header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
					header->set_seqno(g_seq_no++);
					//header->set_tm(get_tick());
					sensor_message->set_allocated_header(header);
					abb::egm::EgmPlanned *planned = new abb::egm::EgmPlanned();
					abb::egm::EgmJoints *joints = new abb::egm::EgmJoints();
								
					abb::egm::EgmJoints *joint_vel = new abb::egm::EgmJoints();
					abb::egm::EgmSpeedRef *speedRef = new abb::egm::EgmSpeedRef();
												
					for(int i = 0; i < 6; i++) joints->add_joints(zero_vec[i]);
					//joints->set_joints(4,val);
					planned->set_allocated_joints(joints);
					sensor_message->set_allocated_planned(planned);
					sensor_message->SerializeToString(&send_string);
					ros::spinOnce();
					
					jnt_cmd_log.header.stamp = ros::Time::now(); 
					jnt_cmd_log.position = home_vec;

					if(sendto(fd, send_string.c_str(), send_string.length(),0,(struct sockaddr*)&client_addr,sizeof(client_addr)) < 0 ) {
						ROS_WARN("Could not send");
						}
					joint_command_log.publish(jnt_cmd_log);
					naptime.sleep();
				}
			}						
		}
	
	}
	double end_time = ros::Time::now().toSec();
	cout<<"Took time: "<<end_time-start_time<<"With dt_ set as :"<<dt_<<endl;
	close(fd);
	return 0;
}
	