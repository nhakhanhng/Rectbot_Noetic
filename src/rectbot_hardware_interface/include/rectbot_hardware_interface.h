#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

class rectbot : public hardware_interface::RobotHW 
{
	public:
		rectbot(ros::NodeHandle& nh);
		~rectbot();
		void init();
		void update(const ros::TimerEvent& e);
		void read();
		void write(ros::Duration elapsed_time);
		ros::Publisher pubR;
		ros::Publisher pubL;
		ros::Subscriber sub;
		std_msgs::Float64 RW_pub;
		std_msgs::Float64 LW_pub;
	protected:
		hardware_interface::JointStateInterface joint_state_interface_;
		hardware_interface::PositionJointInterface position_joint_interface_;
		hardware_interface::VelocityJointInterface velocity_joint_interface_;
		joint_limits_interface::JointLimits limits;
		joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
		joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
		double joint_position_[2];
		double joint_velocity_[2];
		double joint_effort_[2];
		double joint_position_command_[2];
		double joint_velocity_command_[2];
		ros::NodeHandle nh_;
		ros::Timer my_control_loop_;
		ros::Duration elapsed_time_;
		double loop_hz_;
		boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};
