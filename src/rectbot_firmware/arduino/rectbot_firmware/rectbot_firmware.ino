#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include "MPU9250.h"
#include <Servo.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h> 
#include <tf/tf.h> 
#include <tf/transform_broadcaster.h> 

#define address 0x80
#define LOOPTIME 100 

#define kp1 9.50428
#define ki1 1.03402
#define kd1 0.05
#define qpps1 3187

#define kp2 9.50428
#define ki2 1.32439
#define kd2 0.05
#define qpps2 3000

Servo leftSW;
Servo rightSW;
SoftwareSerial serial(11,12);  
RoboClaw roboclaw(&serial,10000);
int32_t enc1, enc2, speed1, speed2;
double enc1new =0, enc2new =0, enc1old =0, enc2old =0;
double x=0,y=0,phi=0,vx=0,vy=0,vth=0, vl=0,vr=0;
double dx=0,dy=0,dphi=0;
unsigned long quakhu=0;
char base_footprint[] = "base_footprint"; 
char odom[] = "odom";
int r;
int l;
float rw=0;
float lw=0.6;
int lw_pos=34.38;
int rw_pos=0;

ros::NodeHandle  nh;
MPU9250 mpu;
//geometry_msgs::Twist twist_msg;
geometry_msgs::TransformStamped t; 
geometry_msgs::Quaternion odom_quat;
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom_msg;
sensor_msgs::Imu imu_msg;
ros::Publisher odom_pub("odom", &odom_msg);
ros::Publisher imu_pub("imu", &imu_msg);


void twistCb(const geometry_msgs::Twist& cmd_vel ) {
  float v=cmd_vel.linear.x;
  float w=cmd_vel.angular.z;

  float v_r=((2*v)+(w*0.135))/2;
  float v_l=((2*v)-(w*0.135))/2;
  
  r=(v_r/(3.14*0.065))*1760;
  l=(v_l/(3.14*0.065))*1760;
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", twistCb);

void leftwingCb(const std_msgs::Float64& left_wing){
  lw=left_wing.data;
  lw=abs(0.6-lw);
  lw_pos=lw*57.3;
}
ros::Subscriber<std_msgs::Float64> left_wing("rectbot/LeftWingJoint_PositionController/command", leftwingCb);

void rightwingCb(const std_msgs::Float64& right_wing){
  rw=right_wing.data;
  rw_pos=rw*57.3;
}
ros::Subscriber<std_msgs::Float64> right_wing("rectbot/RightWingJoint_PositionController/command", rightwingCb);

void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(cmd_vel);
  nh.subscribe(left_wing);
  nh.subscribe(right_wing);
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
  roboclaw.begin(38400);
  roboclaw.SetM1VelocityPID(address,kp1,ki1,kd1,qpps1);
  roboclaw.SetM2VelocityPID(address,kp2,ki2,kd2,qpps2);
  Wire.begin();
  mpu.setup();
}

void loop() {
  nh.spinOnce(); 
  enc1= roboclaw.ReadEncM1(address);
  enc2= roboclaw.ReadEncM2(address);
  enc1new= enc1 - enc1old;
  enc2new= enc2 - enc2old;
  double D_l= (0.065*3.14*(enc1new/1760));
  double D_r= (0.065*3.14*(enc2new/1760));
  double v= (D_l + D_r)/2;
  double w= (D_r - D_l)/0.135;
//  if (D_l >= 0){
//    vl=roboclaw.ReadSpeedM1(address);
//  }else{
//    vl=-roboclaw.ReadSpeedM1(address);
//  }
//  if(D_r >=0){
//    vr=roboclaw.ReadSpeedM2(address);
//  }else{
//    vr=-roboclaw.ReadSpeedM2(address);
//  }
//  vx=((0.065*3.14*(vl/1760)) + (0.065*3.14*(vr/1760)))/2;
//  vy=0;
//  vth=((0.065*3.14*(vr/1760)) - (0.065*3.14*(vl/1760)))/0.135;

  dphi =w;
  if (enc1new==enc2new){
    dx   =v*cos(w);
    dy   =v*sin(w);

    x=x + dx;
    y=y + dy;
  }else{
    dx=(v/w)*(sin(dphi+phi)-sin(phi));
    dy=(v/w)*(cos(dphi+phi)-cos(phi));
       
    x=x + dx;
    y=y - dy;
  }
  phi=phi + dphi;

  if(phi>=6.28)
  {
    phi=phi-6.28;
  }
  if(phi<=-6.28)
  {
    phi=phi+6.28;
  }
  enc1old = enc1;
  enc2old = enc2;
  roboclaw.SpeedM1(address,l);
  roboclaw.SpeedM2(address,r);
  PubOdom();
  PubIMU();
  wingservo();
  //delay(10);
}

void wingservo(){
  leftSW.attach(10);
  rightSW.attach(9);
  leftSW.write(lw_pos);
  rightSW.write(rw_pos);
  delay(100);
  leftSW.detach();
  rightSW.detach();
}

void PubOdom(){
  t.header.stamp = nh.now(); 
  t.header.frame_id = odom; 
  t.child_frame_id = "base_footprint"; 
  t.transform.translation.x = x; 
  t.transform.translation.y = y; 
  t.transform.translation.z = 0; 
  odom_quat= tf::createQuaternionFromYaw(phi);

  t.transform.rotation = odom_quat;
  broadcaster.sendTransform(t);
  nh.spinOnce();
    
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation =  odom_quat;
  
//  odom_msg.twist.twist.linear.x=vx;
//  odom_msg.twist.twist.linear.y=vy;
//  odom_msg.twist.twist.angular.z=vth;
  odom_pub.publish(&odom_msg);
  nh.spinOnce();
}

void PubIMU(){
  mpu.update();
  imu_msg.orientation.x = mpu.getQuaternion(1);
  imu_msg.orientation.y = mpu.getQuaternion(2);
  imu_msg.orientation.z = mpu.getQuaternion(3);
  imu_msg.orientation.w = mpu.getQuaternion(0);
  imu_msg.angular_velocity.x = mpu.getGyro(0);
  imu_msg.angular_velocity.y = mpu.getGyro(1);
  imu_msg.angular_velocity.z = mpu.getGyro(2);
  imu_msg.linear_acceleration.x = mpu.getAcc(0);
  imu_msg.linear_acceleration.y = mpu.getAcc(1);
  imu_msg.linear_acceleration.z = mpu.getAcc(2);

  imu_msg.header.seq++;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";
  
  imu_pub.publish(&imu_msg);
  nh.spinOnce();  
}
