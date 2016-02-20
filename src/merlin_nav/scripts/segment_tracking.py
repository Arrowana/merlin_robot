#!/usr/bin/python

import rospy
import numpy as np
import tf.transformations as tflib

from geometry_msgs.msg import Pose2D,Twist
from nav_msgs.msg import Odometry

class SegmentTracker:
	def __init__(self):
		# Temporary
		self.vel_cmd = 0.5 # Velocity command (m/s)
		# Guidance design parameters
		self.L = 0.5 # Look-ahead distance (m)
		# Control design parameters
		self.Kp = 2 # Proportionnal gain
		
		self.goal_reached = False
		self.gazebo_cmd = Twist()
		
		self.pose = Pose2D() # Current pose of the robot
		
		rospy.init_node('segment_tracking')
		rospy.Subscriber("/merlin/base_controller/odom", Odometry, self.pose_callback)
		rospy.Subscriber("/target_point", Pose2D, self.guidance_callback)
		self.gazebo_pub = rospy.Publisher("/merlin/base_controller/cmd_vel", Twist, queue_size=10)

		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			print self.gazebo_cmd
			self.gazebo_pub.publish(self.gazebo_cmd)
			rate.sleep()

	def norm_angle(self,angle):
		angle=angle % 360
		angle=(angle+360) % 360
		if(angle>180):
			angle-=360
		return angle
	
	def pose_callback(self,odom):
		# Conversion from quaternion to euler representation
		quaternion = (odom.pose.pose.orientation.x, \
					 odom.pose.pose.orientation.y, \
					 odom.pose.pose.orientation.z, \
					 odom.pose.pose.orientation.w)
		orientation = tflib.euler_from_quaternion(quaternion)
		
		self.pose.x = odom.pose.pose.position.x
		self.pose.y = odom.pose.pose.position.y
		self.pose.theta = orientation[2]
		
		#print self.pose
	
	def guidance_callback(self,target):
		print target
		segment_start = self.pose
		dx = target.x - segment_start.x
		dy = target.y - segment_start.y
		print dx,dy
		
		while( np.abs(dx)>0.01 or np.abs(dy)>0.01 ):
			segment_start.x = self.pose.x
			segment_start.y = self.pose.y
			segment_start.theta = self.pose.theta # Unit to be verified
			
			dx = target.x-segment_start.x
			dy = target.y-segment_start.y
			path_angle = np.arctan2(dy,dx)
			
			dirX = [np.cos(path_angle),np.sin(path_angle)]
			dirY = [-np.sin(path_angle),np.cos(path_angle)]
			
			y_e = np.abs(dx*dirY[0]+dy*dirY[1])
			
			# Heading command (rad)
			head_cmd = path_angle+np.arctan2(-y_e,self.L)
			self.controller(head_cmd,self.vel_cmd,False)
			
		head_cmd = self.pose.theta
		self.controller(head_cmd,self.vel_cmd,True)

	def controller(self,head_cmd,vel_cmd,goal_reached):
		if( not goal_reached ):
			e = head_cmd-self.pose.theta
			P_value = self.Kp*e
			self.gazebo_cmd.linear.x = self.vel_cmd
			self.gazebo_cmd.angular.z = self.norm_angle(P_value)
		else:
			self.gazebo_cmd.linear.x = 0
			self.gazebo_cmd.angular.z = 0

if __name__ == '__main__':
	try:
		SegmentTracker()
	except rospy.ROSInterruptException:
		pass
