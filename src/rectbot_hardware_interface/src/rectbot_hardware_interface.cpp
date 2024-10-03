#include <rectbot_hardware_interface.h>

float v_r,v_l;
void callback(const geometry_msgs::Twist& cmd_vel ) {
  	float v=cmd_vel.linear.x;
  	float w=cmd_vel.angular.z;

	v_r=((2*v)+(w*0.135))/2;
 	v_l=((2*v)-(w*0.135))/2;
  
}

rectbot::rectbot(ros::NodeHandle& nh) : nh_(nh) {
	init();
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
	loop_hz_=10;
	ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    	pubR = nh_.advertise<std_msgs::Float64>("rectbot/RightWheelJoint_VelocityController/command",10);
	pubL = nh_.advertise<std_msgs::Float64>("rectbot/LeftWheelJoint_VelocityController/command",10);
	sub = nh_.subscribe("cmd_vel", 100, callback);
	my_control_loop_ = nh_.createTimer(update_freq, &rectbot::update, this);
}
rectbot::~rectbot() {
}
void rectbot::init() {
	hardware_interface::JointStateHandle JointStateHandleRight("right_wing_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
	joint_state_interface_.registerHandle(JointStateHandleRight);
	hardware_interface::JointHandle JointPositionHandleRight(JointStateHandleRight, &joint_position_command_[0]);
	position_joint_interface_.registerHandle(JointPositionHandleRight);
	joint_limits_interface::getJointLimits("right_wing_joint", nh_, limits);
	joint_limits_interface::PositionJointSaturationHandle JointLimitsHandleRight(JointPositionHandleRight, limits);
	positionJointSaturationInterface.registerHandle(JointLimitsHandleRight);

	hardware_interface::JointStateHandle JointStateHandleLeft("left_wing_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
	joint_state_interface_.registerHandle(JointStateHandleLeft);
	hardware_interface::JointHandle JointPositionHandleLeft(JointStateHandleLeft, &joint_position_command_[1]);
	position_joint_interface_.registerHandle(JointPositionHandleLeft);
	joint_limits_interface::getJointLimits("left_wing_joint", nh_, limits);
	joint_limits_interface::PositionJointSaturationHandle JointLimitsHandleLeft(JointPositionHandleLeft, limits);
	positionJointSaturationInterface.registerHandle(JointLimitsHandleLeft);
	
	hardware_interface::JointStateHandle JointStateHandleRightWheel("right_wheel_joint", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
	joint_state_interface_.registerHandle(JointStateHandleRightWheel);
	hardware_interface::JointHandle JointVelocityHandleRightWheel(JointStateHandleRightWheel, &joint_velocity_command_[0]);
	velocity_joint_interface_.registerHandle(JointVelocityHandleRightWheel);
	joint_limits_interface::getJointLimits("right_wheel_joint", nh_, limits);
	joint_limits_interface::VelocityJointSaturationHandle JointLimitsHandleRightWheel(JointVelocityHandleRightWheel, limits);
	velocityJointSaturationInterface.registerHandle(JointLimitsHandleRightWheel);

	hardware_interface::JointStateHandle JointStateHandleLeftWheel("left_wheel_joint", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
	joint_state_interface_.registerHandle(JointStateHandleLeftWheel);
	hardware_interface::JointHandle JointVelocityHandleLeftWheel(JointStateHandleLeftWheel, &joint_velocity_command_[1]);
	velocity_joint_interface_.registerHandle(JointVelocityHandleLeftWheel);
	joint_limits_interface::getJointLimits("left_wheel_joint", nh_, limits);
	joint_limits_interface::VelocityJointSaturationHandle JointLimitsHandleLeftWheel(JointVelocityHandleLeftWheel, limits);
	velocityJointSaturationInterface.registerHandle(JointLimitsHandleLeftWheel);

	registerInterface(&joint_state_interface_);
	registerInterface(&position_joint_interface_);
	registerInterface(&positionJointSaturationInterface);
	registerInterface(&velocity_joint_interface_);
	registerInterface(&velocityJointSaturationInterface);
}
void rectbot::update(const ros::TimerEvent& e) {
	elapsed_time_ = ros::Duration(e.current_real - e.last_real);
	read();
	controller_manager_->update(ros::Time::now(), elapsed_time_);
	write(elapsed_time_);
}
void rectbot::read(){
        joint_position_[0]=joint_position_command_[0];
        joint_position_[1]=joint_position_command_[1];
	joint_velocity_[2]=joint_velocity_command_[0];
	joint_velocity_[3]=joint_velocity_command_[1];
}
void rectbot::write(ros::Duration elapsed_time) {
	positionJointSaturationInterface.enforceLimits(elapsed_time);
	velocityJointSaturationInterface.enforceLimits(elapsed_time);
	RW_pub.data=v_r;
	pubR.publish(RW_pub);
	LW_pub.data=v_l;
	pubL.publish(LW_pub);
	joint_position_[2] += joint_velocity_[2]*elapsed_time.toSec();
	joint_position_[3] += joint_velocity_[3]*elapsed_time.toSec();
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "rectbot_hardware_inerface");
	ros::NodeHandle nh;
	ros::MultiThreadedSpinner spinner(4); 
	rectbot rectbot(nh);
	spinner.spin();
	return 0;
}
