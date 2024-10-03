#!/usr/bin/env python
import roslib
# import roslib.packages
# roslib.load_manifest('rep_cmd_vel')
from roslib import scriptutil
import sys, select, termios, tty
import os
import rospy
from math import *
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point, PoseStamped, PointStamped, Vector3, Pose, PoseArray 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from time import time
import tf2_ros
import tf2_geometry_msgs
import tf
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Char, ColorRGBA

import string
import math
from threading import Lock
from visualization_msgs.msg import Marker
human_mutex = Lock()
fusion_mutex = Lock()
class map_inf:
	size_x = 0
	size_y =  0
	origin_x = 0
	scale = 0
	origin_y = 0
class main():
	def __init__(self):
		rospy.init_node("compute_closest_obstcl")
		self.init_variable()
		rospy.Subscriber("scan", LaserScan, self.laser_callback)
#		rospy.Subscriber("/odom_laser",Odometry,self.Odom_callback)
#		rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
		self.closestP=rospy.Publisher("/closest_point", PoseArray, queue_size=1)
#		self.pub_marker = rospy.Publisher("maker_s", Marker, queue_size = 100)
		self.pub_marker_d_tr = rospy.Publisher("maker_d_tr_end", Marker, queue_size = 100)
		self.pub_marker_d_tl = rospy.Publisher("maker_d_tl_end", Marker, queue_size = 100)
		self.pub_marker_d_br = rospy.Publisher("maker_d_br_end", Marker, queue_size = 100)
		self.pub_marker_d_bl = rospy.Publisher("maker_d_bl_end", Marker, queue_size = 100)
 		self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)

		rospy.Timer(rospy.Duration(0.05), self.publish_closest_obstacle)   
		while not rospy.is_shutdown():
			rospy.sleep(0.01)
#			print("Shutting down")
		
	def init_variable(self):
        
            
##########################################################################
		self.marker=Marker()
		self.marker.header.frame_id = "base_link"
		self.marker.type = Marker.ARROW
		self.marker.scale.x = 0.2
		self.marker.scale.y = 0.02
		self.marker.scale.z = 0.02
		self.marker.color.a = 1.0
		self.marker.color.r = 1.0
		self.marker.color.b = 1.0
		
		
		self.marker_d=Marker()
		self.marker_d.header.frame_id = "base_link"
		self.marker_d.type = Marker.ARROW
		self.marker_d.scale.x = 0.2
		self.marker_d.scale.y = 0.02
		self.marker_d.scale.z = 0.02
		self.marker_d.color.a = 1.0
		self.marker_d.color.g = 1.0

		

		self.line_color = ColorRGBA()       # a nice color for my line (royalblue)
		self.line_color.r = 0.254902
		self.line_color.g = 0.411765
		self.line_color.b = 0.882353
		self.line_color.a = 1.0
		self.start_point = Point()        #start point
#self.start_point.x = 0.2
#self.start_point.y = 0.0
#self.start_point.z = 0.2
   		self.end_point = Point()        #end point
#self.end_point.x = 0.7
#self.end_point.y = 0
#self.end_point.z = 0.2

		self.marker_vshape = Marker()
		self.marker_vshape.header.frame_id = "base_link"
		self.marker_vshape.type = self.marker_vshape.LINE_STRIP
		self.marker_vshape.action = self.marker_vshape.ADD

    # marker scale
		self.marker_vshape.scale.x = 0.03
		self.marker_vshape.scale.y = 0.03
		self.marker_vshape.scale.z = 0.03

    # marker color
		self.marker_vshape.color.a = 1.0
		self.marker_vshape.color.r = 1.0
		self.marker_vshape.color.g = 1.0
		self.marker_vshape.color.b = 0.0

    # marker orientaiton
		self.marker_vshape.pose.orientation.x = 0.0
		self.marker_vshape.pose.orientation.y = 0.0
		self.marker_vshape.pose.orientation.z = 0.0
		self.marker_vshape.pose.orientation.w = 1.0

    # marker position
		self.marker_vshape.pose.position.x = 0.0
		self.marker_vshape.pose.position.y = 0.0
		self.marker_vshape.pose.position.z = 0.0

		
		
		self.laser=LaserScan()
		self.map=OccupancyGrid()
		self.pose_robot=PoseArray()
		p=Pose()
		self.pose_robot.poses.append(Pose(p.position, p.orientation))
		self.pose_robot.poses.append(Pose(p.position, p.orientation))
		self.pose_robot.poses.append(Pose(p.position, p.orientation))
		self.pose_robot.poses.append(Pose(p.position, p.orientation))
		self.pose_robot.poses.append(Pose(p.position, p.orientation))

# PARAMETER SET
		self.location=[0, 0, 0]
#		self.tf_buffer=tf2_ros.Buffer(rospy.Duration(1200.0))
#		self.tf_listener=tf2_ros.TransformListener(self.tf_buffer)
#		self.listener =tf.TransformListener()

#		self.get_transform()
#	def get_transform(self):
#		try:
#			self.transform = self.tf_buffer.lookup_transform("R_000/base_link", "R_000/base_laser_link", rospy.Time(0), rospy.Duration(1.0))
#		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#			rospy.logerror("Error getting transform")
#			print "Error"
			
#	def Odom_callback(self, data):
#		human_mutex.acquire()
## 		(trans,rot) = self.listener.lookupTransform('/map', 'R_000/base_link', rospy.Time(0))
#		try:
#			self.transform_ = self.tf_buffer.lookup_transform("R_000/base_link", "R_000/base_laser_link", rospy.Time(0), rospy.Duration(1.0))
#		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#			rospy.logerror("Error getting transform")
#			print "Error"
#		#self.location[0]=data.pose.pose.position.x
#		#self.location[1]= data.pose.pose.position.y
#		#self.location[2]=data.pose.pose.orientation.z
#		self.location[0]=self.transform_.transform.translation.x
#		self.location[1]= self.transform_.transform.translation.y
#		self.location[2]=self.transform_.transform.rotation.z
#		human_mutex.release()
#	def map_callback (self, msg):
#	   self.map=msg
	def laser_callback (self, msg):
		human_mutex.acquire()
		self.laser=msg
#		self.get_transform()
#		try:
#			self.transform_ = self.tf_buffer.lookup_transform("R_000/base_link", "R_000/base_laser_link", rospy.Time(0), rospy.Duration(1.0))
#		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#			rospy.logerr("Error getting transform")
#			print "Error"
		#self.location[0]=data.pose.pose.position.x
		#self.location[1]= data.pose.pose.position.y
		#self.location[2]=data.pose.pose.orientation.z
#		self.location[0]=self.transform_.transform.translation.x
#		self.location[1]= self.transform_.transform.translation.y
#		self.location[2]=self.transform_.transform.rotation.z

		self.location[0]=0
		self.location[1]= 0
		self.location[2]=0
		human_mutex.release()
	def publish_closest_obstacle(self,event):
############publish the static obstacle
		fusion_mutex.acquire()
##robot in map frame reference
		pt_x = self.location[0]
		pt_y = self.location[1]
		pt_th= self.location[2]
## variables to keep the distance
		dist_sq = 1000000
		min_dist = 1000000
		d_x=0
		d_y=0
		angle=10000

############publish the dynamic obstacle
		laser = self.laser.ranges
		X=[]
		Y=[]
		Z=[]
		A=[]
		for ii in range(len(self.laser.ranges)):
			angle=self.laser.angle_min-self.location[2]*(self.laser.angle_min/(2*0.7)) + ii*self.laser.angle_increment
			if self.laser.ranges[ii]*cos(angle)>0 and self.laser.ranges[ii] < 2:
				
				X.append(self.laser.ranges[ii]*cos(angle))
				Y.append(self.laser.ranges[ii]*sin(angle))
				Z.append(self.laser.ranges[ii])
				A.append(angle)



###################11111111111111111111nearest obstacle on top right coner of robot locaiton
		shortest_laser = 10000
		point=Point()
#loop over laser data point to find the point statisfy condition nereast on the right coner of robot
		for ii in range(len(X)):
			if Z[ii] < shortest_laser:
				angle=A[ii]
				point_temp=Point()
				point_temp.x=X[ii]
				point_temp.y=Y[ii]	
				if self.location[0]+point_temp.x>self.location[0] and self.location[1]+point_temp.y<self.location[1]:				
					shortest_laser=Z[ii]
					point.x=point_temp.x
					point.y=point_temp.y
#find the end of line from the nearest seed of top right 

# 		dis=0.001
# 		shortest_laser1 = 10000
# 		point1=Point()
# 		count=1
# 		while (shortest_laser1 <0.15 or shortest_laser1 ==10000) and count <len(X):
# #			print('radaybefore',shortest_laser1,point)
# 			shortest_laser1 = 10000
# 			count=count+1
# 			for ii in range(len(X)):
# 				if not math.isinf(Z[ii]):
# 					angle=A[ii]				
# 					point_temp1=Point()
# 					point_temp1.x=X[ii]
# 					point_temp1.y=Y[ii]
# 					dis=sqrt((point_temp1.x-point.x)**2+(point_temp1.y-point.y)**2)
# #					print('radayafter',dis,point_temp.y,point.y,point_temp1.y<point.y)				
# 					if dis < shortest_laser1 and dis!=0.0 and point_temp1.y<point.y:
# 						shortest_laser1=dis
# 						if shortest_laser1 <0.15:
# 							point1.x=point_temp1.x
# 							point1.y=point_temp1.y
# #							print ('dis',dis,'point1',point1.y,'point',point.y,'point_temp',point_temp1.y)
# #			print('raday',shortest_laser1,point1.y,point.y)
# 			if shortest_laser1 <0.15:
# 				point=point1
# #				print('radayafter',shortest_laser1,point,point1)
# 		self.start_point=Point()
# 		self.start_point.x=round(point.x,2)
# 		self.start_point.y=round(point.y,2)

#########################################################################11111111111111111111 PUBLISH THE OBSTACLE TOP RIGHT
#		print("countttttttttttttttttt",count,pose_trans_dyn_obstacle_temp.pose.position.x)
	 	pose=PoseStamped()
		pose.header=self.laser.header
# 		point.z= 0
		pose.pose.position=point
		pose_trans_dyn_obstacle=PoseStamped()

		pose_trans_dyn_obstacle.pose.position.x=point.x
		pose_trans_dyn_obstacle.pose.position.y=point.y
		min_dy_dis=math.sqrt(point.x**2+point.y**2)
		pose_trans_dyn_obstacle.pose.position.z=min_dy_dis
#		pose_trans_dyn_obstacle= tf2_geometry_msgs.do_transform_pose(pose, self.transform)
		pose_trans_dyn_obstacle.pose.orientation.z=math.atan2(pose_trans_dyn_obstacle.pose.position.y, pose_trans_dyn_obstacle.pose.position.x)-pt_th

		
		self.pose_robot.header=self.laser.header
		self.pose_robot.poses[1].position=pose_trans_dyn_obstacle.pose.position
		self.pose_robot.poses[1].position.z=min_dy_dis

		self.pose_robot.poses[1].orientation=pose_trans_dyn_obstacle.pose.orientation
		self.closestP.publish(self.pose_robot)
		
		
## 		draw dynamic obstacle postion 		

		self.marker_d.pose.position.x = self.location[0]+pose_trans_dyn_obstacle.pose.position.x
		self.marker_d.pose.position.y = self.location[1]+pose_trans_dyn_obstacle.pose.position.y
#		min_dy_dis=math.sqrt(pose_trans_dyn_obstacle.pose.position.x**2+pose_trans_dyn_obstacle.pose.position.y**2)
		
		self.marker_d.pose.position.z = 0
		self.marker_d.pose.orientation.x = 0
		self.marker_d.pose.orientation.y = 0
		self.marker_d.pose.orientation.z = pose_trans_dyn_obstacle.pose.orientation.z
		self.marker_d.pose.orientation.w = 1
		
		self.pub_marker_d_tr.publish(self.marker_d)
#		self.pub_marker.publish(self.marker)
###################2222222222222222nearest obstacle on top left coner of robot locaiton
		X=[]
		Y=[]
		Z=[]
		A=[]
		for ii in range(len(self.laser.ranges)):
			if self.laser.ranges[ii] < 3:
				angle=self.laser.angle_min-self.location[2]*(self.laser.angle_min/(2*0.7)) + ii*self.laser.angle_increment
				X.append(self.laser.ranges[ii]*cos(angle))
				Y.append(self.laser.ranges[ii]*sin(angle))
				Z.append(self.laser.ranges[ii])
				A.append(angle)

#		print(len(X),len(Y),len(Z),len(A))

		shortest_laser = 10000
		point=Point()

		for ii in range(len(X)):
			if Z[ii] < shortest_laser:
				angle=A[ii]
				point_temp=Point()
				point_temp.x=X[ii]
				point_temp.y=Y[ii]
				if self.location[0]+point_temp.x>self.location[0] and self.location[1]+point_temp.y>self.location[1]:				
					shortest_laser=Z[ii]
					point.x=point_temp.x
					point.y=point_temp.y


# 		dis=0.001
# 		shortest_laser1 = 10000
# 		point1=Point()
# 		count=1
# 		while (shortest_laser1 <0.15 or shortest_laser1 ==10000) and count <len(X):
# 			shortest_laser1 = 10000
# 			count=count+1
# 			for ii in range(len(X)):
# 				if not math.isinf(X[ii]):
# 					angle=A[ii]				
# 					point_temp1=Point()
# 					point_temp1.x=X[ii]
# 					point_temp1.y=Y[ii]
# 					dis=sqrt((point_temp1.x-point.x)**2+(point_temp1.y-point.y)**2)			
# 					if dis < shortest_laser1 and dis!=0.0 and point_temp1.y>point.y:
# 						shortest_laser1=dis
# 						if shortest_laser1 <0.15:
# 							point1.x=point_temp1.x
# 							point1.y=point_temp1.y
# #							print ('dis',dis,'shortest_laser1',shortest_laser1,'point1',point1,'point',point)
# #			print('raday',shortest_laser1,point1,point)
# 			if shortest_laser1 <0.15:
# 				point=point1
# #				print('radayafter',shortest_laser1,point,point1)
# #publish marker vshape
# 		self.end_point=Point()
# 		self.end_point.x=round(point.x,2)
# 		self.end_point.y=round(point.y,2)


#     # marker line points
# 		leng=sqrt((self.end_point.x-self.start_point.x)**2+(self.end_point.y-self.start_point.y)**2)

#     # first point
# 		if leng>0.5 and leng <0.8:	
# 			print('leng',leng)
# 			print('start point of dock ',self.start_point,2,' end point of dock', (self.end_point) )
# 			self.marker_vshape.points = []
# 			self.marker_vshape.points.append(self.start_point)

#     # second point

# 			self.marker_vshape.points.append(self.end_point)

#     # Publish the Marker
# 		self.marker_pub.publish(self.marker_vshape)

			
		
#########################################################################22222222222222222 PUBLISH THE OBSTACLE TOP LEFT
#		print("countttttttttttttttttt",count,pose_trans_dyn_obstacle_temp.pose.position.x)
	 	pose=PoseStamped()
		pose.header=self.laser.header
# 		point.z= 0
		pose.pose.position=point
		pose_trans_dyn_obstacle=PoseStamped()
#		print ('bbbbbb',point)
		pose_trans_dyn_obstacle.pose.position.x=point.x
		pose_trans_dyn_obstacle.pose.position.y=point.y
		min_dy_dis=math.sqrt(point.x**2+point.y**2)
		pose_trans_dyn_obstacle.pose.position.z=min_dy_dis
#		pose_trans_dyn_obstacle= tf2_geometry_msgs.do_transform_pose(pose, self.transform)
		pose_trans_dyn_obstacle.pose.orientation.z=math.atan2(pose_trans_dyn_obstacle.pose.position.y, pose_trans_dyn_obstacle.pose.position.x)-pt_th

		
		self.pose_robot.header=self.laser.header
		self.pose_robot.poses[2].position=pose_trans_dyn_obstacle.pose.position
		self.pose_robot.poses[2].position.z=min_dy_dis
		self.pose_robot.poses[2].orientation=pose_trans_dyn_obstacle.pose.orientation
		self.closestP.publish(self.pose_robot)
		
		
# 		draw static obstacle postion 		
		self.marker.header.stamp = rospy.Time.now()
		self.marker.pose.position.x = d_x
		self.marker.pose.position.y = d_y
		self.marker.pose.position.z = 0
		self.marker.pose.orientation.x = 0
		self.marker.pose.orientation.y = 0
		self.marker.pose.orientation.z = angle
		self.marker.pose.orientation.w = 1
# 		draw dynamic obstacle postion 
		self.marker_d.pose.position.x = self.location[0]+pose_trans_dyn_obstacle.pose.position.x
		self.marker_d.pose.position.y = self.location[1]+pose_trans_dyn_obstacle.pose.position.y
#		min_dy_dis=math.sqrt(pose_trans_dyn_obstacle.pose.position.x**2+pose_trans_dyn_obstacle.pose.position.y**2)
		
		self.marker_d.pose.position.z = 0
		self.marker_d.pose.orientation.x = 0
		self.marker_d.pose.orientation.y = 0
		self.marker_d.pose.orientation.z = pose_trans_dyn_obstacle.pose.orientation.z
		self.marker_d.pose.orientation.w = 1
		
		self.pub_marker_d_tl.publish(self.marker_d)
# 		self.pub_marker.publish(self.marker)

###################3333333333333333nearest obstacle on bottom right coner of robot locaiton


		shortest_laser = 10000
		point=Point()
		for ii in range(len(laser)):
			if laser[ii] < shortest_laser:
				angle=self.laser.angle_min-self.location[2]*(self.laser.angle_min/(2*0.7)) + ii*self.laser.angle_increment				
				point_temp=Point()
				point_temp.x=laser[ii]*cos(angle)
				point_temp.y=laser[ii]*sin(angle)
				if self.location[0]+point_temp.x<self.location[0] and self.location[1]+point_temp.y<self.location[1]:				
#				if count==0 and self.location[0]+pose_trans_dyn_obstacle_temp.pose.position.x<self.location[0] and self.location[1]+pose_trans_dyn_obstacle_temp.pose.position.y<self.location[1]:
					shortest_laser=laser[ii]
					point.x=point_temp.x
					point.y=point_temp.y


#		dis=0.001
#		shortest_laser1 = 10000
#		point1=Point()
#		while shortest_laser1 <0.05 or shortest_laser1 ==10000:
##			print('radaybefore',shortest_laser1,point,point1)
#			shortest_laser1 = 10000
#			for ii in range(len(self.laser.ranges)):
#				if not math.isinf(self.laser.ranges[ii]):
#					angle=self.laser.angle_min-self.location[2]*(self.laser.angle_min/(2*0.7)) + ii*self.laser.angle_increment				
#					point_temp=Point()
#					point_temp.x=self.laser.ranges[ii]*cos(angle)
#					point_temp.y=self.laser.ranges[ii]*sin(angle)
#					dis=sqrt((point_temp.x-point.x)**2+(point_temp.y-point.y)**2)				
#					if dis < shortest_laser1 and dis!=0.0 and point_temp.y<point.y:
#						shortest_laser1=dis
#						if shortest_laser1 <0.05:
#							point1.x=point_temp.x
#							point1.y=point_temp.y
##							print ('dis',dis,'shortest_laser1',shortest_laser1,'point1',point1,'point',point)
##			print('raday',shortest_laser1,point1,point)
#			if shortest_laser1 <0.05:
#				point=point1
##				print('radayafter',shortest_laser1,point,point1)
#########################################################################333333333333 PUBLISH THE OBSTACLE BOTTOM RIGHT
#		print("countttttttttttttttttt",count,pose_trans_dyn_obstacle_temp.pose.position.x)
	 	pose=PoseStamped()
		pose.header=self.laser.header
# 		point.z= 0
		pose.pose.position=point
		pose_trans_dyn_obstacle=PoseStamped()

		pose_trans_dyn_obstacle.pose.position.x=point.x
		pose_trans_dyn_obstacle.pose.position.y=point.y
		min_dy_dis=math.sqrt(point.x**2+point.y**2)
		pose_trans_dyn_obstacle.pose.position.z=min_dy_dis
#		pose_trans_dyn_obstacle= tf2_geometry_msgs.do_transform_pose(pose, self.transform)
		pose_trans_dyn_obstacle.pose.orientation.z=math.atan2(pose_trans_dyn_obstacle.pose.position.y, pose_trans_dyn_obstacle.pose.position.x)-pt_th

		
		self.pose_robot.header=self.laser.header
		self.pose_robot.poses[3].position=pose_trans_dyn_obstacle.pose.position
		self.pose_robot.poses[3].orientation=pose_trans_dyn_obstacle.pose.orientation
		self.pose_robot.poses[3].position.z=min_dy_dis
		self.closestP.publish(self.pose_robot)
		
		
# 		draw static obstacle postion 		
		self.marker.header.stamp = rospy.Time.now()
		self.marker.pose.position.x = d_x
		self.marker.pose.position.y = d_y
		self.marker.pose.position.z = 0
		self.marker.pose.orientation.x = 0
		self.marker.pose.orientation.y = 0
		self.marker.pose.orientation.z = angle
		self.marker.pose.orientation.w = 1
# 		draw dynamic obstacle postion 
		self.marker_d.pose.position.x = self.location[0]+pose_trans_dyn_obstacle.pose.position.x
		self.marker_d.pose.position.y = self.location[1]+pose_trans_dyn_obstacle.pose.position.y

		
		self.marker_d.pose.position.z = 0
		self.marker_d.pose.orientation.x = 0
		self.marker_d.pose.orientation.y = 0
		self.marker_d.pose.orientation.z = pose_trans_dyn_obstacle.pose.orientation.z
		self.marker_d.pose.orientation.w = 1
		
		self.pub_marker_d_br.publish(self.marker_d)
#		self.pub_marker.publish(self.marker)
###################444444444444nearest obstacle on bottom left coner of robot locaiton


		shortest_laser = 10000
		point=Point()
		for ii in range(len(laser)):
			if laser[ii] < shortest_laser:
				angle=self.laser.angle_min-self.location[2]*(self.laser.angle_min/(2*0.7)) + ii*self.laser.angle_increment				
				point_temp=Point()
				point_temp.x=laser[ii]*cos(angle)
				point_temp.y=laser[ii]*sin(angle)
				if self.location[0]+point_temp.x<self.location[0] and self.location[1]+point_temp.y>self.location[1]:				
#				if count==0 and self.location[0]+pose_trans_dyn_obstacle_temp.pose.position.x<self.location[0] and self.location[1]+pose_trans_dyn_obstacle_temp.pose.position.y<self.location[1]:
					shortest_laser=laser[ii]
					point.x=point_temp.x
					point.y=point_temp.y

#		dis=0.001
#		shortest_laser1 = 10000
#		point1=Point()
#		while shortest_laser1 <0.05 or shortest_laser1 ==10000:
##			print('radaybefore',shortest_laser1,point,point1)
#			shortest_laser1 = 10000
#			for ii in range(len(self.laser.ranges)):
#				if not math.isinf(self.laser.ranges[ii]):
#					angle=self.laser.angle_min-self.location[2]*(self.laser.angle_min/(2*0.7)) + ii*self.laser.angle_increment				
#					point_temp=Point()
#					point_temp.x=self.laser.ranges[ii]*cos(angle)
#					point_temp.y=self.laser.ranges[ii]*sin(angle)
#					dis=sqrt((point_temp.x-point.x)**2+(point_temp.y-point.y)**2)				
#					if dis < shortest_laser1 and dis!=0.0 and point_temp.y>point.y:
#						shortest_laser1=dis
#						if shortest_laser1 <0.05:
#							point1.x=point_temp.x
#							point1.y=point_temp.y
##							print ('dis',dis,'shortest_laser1',shortest_laser1,'point1',point1,'point',point)
##			print('raday',shortest_laser1,point1,point)
#			if shortest_laser1 <0.05:
#				point=point1
##				print('radayafter',shortest_laser1,point,point1)

   

#########################################################################4444444444444444 PUBLISH THE OBSTACLE BOTTOM LEFT
#		print("countttttttttttttttttt",count,pose_trans_dyn_obstacle_temp.pose.position.x)
	 	pose=PoseStamped()
		pose.header=self.laser.header
# 		point.z= 0
		pose.pose.position=point
		pose_trans_dyn_obstacle=PoseStamped()

		pose_trans_dyn_obstacle.pose.position.x=point.x
		pose_trans_dyn_obstacle.pose.position.y=point.y
		min_dy_dis=math.sqrt(point.x**2+point.y**2)
		pose_trans_dyn_obstacle.pose.position.z=min_dy_dis
#		pose_trans_dyn_obstacle= tf2_geometry_msgs.do_transform_pose(pose, self.transform)
		pose_trans_dyn_obstacle.pose.orientation.z=math.atan2(pose_trans_dyn_obstacle.pose.position.y, pose_trans_dyn_obstacle.pose.position.x)-pt_th

		
		self.pose_robot.header=self.laser.header
		self.pose_robot.poses[4].position=pose_trans_dyn_obstacle.pose.position
		self.pose_robot.poses[4].position.z=min_dy_dis
		self.pose_robot.poses[4].orientation=pose_trans_dyn_obstacle.pose.orientation
		self.closestP.publish(self.pose_robot)
		
		
# 		draw static obstacle postion 		
		self.marker.header.stamp = rospy.Time.now()
		self.marker.pose.position.x = d_x
		self.marker.pose.position.y = d_y
		self.marker.pose.position.z = 0
		self.marker.pose.orientation.x = 0
		self.marker.pose.orientation.y = 0
		self.marker.pose.orientation.z = angle
		self.marker.pose.orientation.w = 1
# 		draw dynamic obstacle postion 
		self.marker_d.pose.position.x = self.location[0]+pose_trans_dyn_obstacle.pose.position.x
		self.marker_d.pose.position.y = self.location[1]+pose_trans_dyn_obstacle.pose.position.y

		
		self.marker_d.pose.position.z = 0
		self.marker_d.pose.orientation.x = 0
		self.marker_d.pose.orientation.y = 0
		self.marker_d.pose.orientation.z = pose_trans_dyn_obstacle.pose.orientation.z
		self.marker_d.pose.orientation.w = 1
		
		self.pub_marker_d_bl.publish(self.marker_d)
#		self.pub_marker.publish(self.marker)
		
		
		
#		print ("Robot is at position ", pt_x,  ",",pt_y," ,",pt_th*180/3.14)
#		print"#######################"
#		print("Min.dynamic Obstacle Distance: ", min_dy_dis, " min angle ",pose_trans_dyn_obstacle.pose.orientation.z)
#		print ("Closest DYNAMIC Obstacle is at position ", self.location[0]+pose_trans_dyn_obstacle.pose.position.x, " ,",self.location[1]+pose_trans_dyn_obstacle.pose.position.y, "angle", pose_trans_dyn_obstacle.pose.orientation.z*180/3.14)
#		print ("Shortest DYNAMIC  distance  ", pose_trans_dyn_obstacle.pose.position.x, " ,",pose_trans_dyn_obstacle.pose.position.y)
#		print"#######################"
#		print("Min.Static Obstacle Distance: ", min_dist, " min angle ",angle*180/3.14)
#		print ("Closest STATIC Obstacle is at position ", d_x, " ,",d_y, "angle", angle*180/3.14)
#		print ("Shortest STATIC distance  ", d_x-pt_x, " ,",d_y-pt_y)
		
		fusion_mutex.release()
if __name__ == '__main__':
    try: main() 
    except rospy.ROSInterruptException: pass


