#!/usr/bin/python
"""
    Node for simulation. Set inital pose in gazebo when received
    Send pose regarding odom update    
"""

import rospy
import numpy
import tf.transformations
import math

from geometry_msgs.msg import Pose2D, Quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

# Transfom matrix
#cos(th) sin(th) x
#-sin(th) cos(th) y
#0         0       1

class TF2D:
    def __init__(self):
        pass

class FakeLocalization(object):
    def __init__(self):
        rospy.init_node('fake_localization2D')
        self.odom_sub = rospy.Subscriber("merlin/base_controller/odom", Odometry, self.on_odom)
        self.inital_pose_sub = rospy.Subscriber("initial_pose", Pose2D, self.on_initial_pose)

        self.pose_pub = rospy.Publisher("pose", Pose2D)

        self.last_odom = Odometry()
        self.odom_f_pose = Pose2D()
        rospy.spin()

    def on_odom(self, odom_msg):
        self.last_odom = odom_msg

        #Sum odometry to the odom frame position in the global frame
        pose = Pose2D()
        pose.x = self.odom_f_pose.x + odom_msg.pose.pose.position.x*math.cos(self.odom_f_pose.theta)-\
            odom_msg.pose.pose.position.y*math.sin(self.odom_f_pose.theta)
        pose.y = self.odom_f_pose.y + odom_msg.pose.pose.position.x*math.sin(self.odom_f_pose.theta)+\
            odom_msg.pose.pose.position.y*math.cos(self.odom_f_pose.theta)
        pose.theta = self.odom_f_pose.theta + self.yaw_from_quaternion(odom_msg.pose.pose.orientation)

        self.pose_pub.publish(pose)

    def ModelState_from_Pose2D(self, pose):
        new_state = ModelState()

        ns = rospy.get_namespace()
        print ns
        #To remove this forced model state name
        ns = "merlin"
        new_state.model_name = ns
        new_state.pose.position.x = pose.x
        new_state.pose.position.y = pose.y

        #Build quaternion
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = tf.transformations.quaternion_from_euler(0,0, pose.theta)
        new_state.pose.orientation = quat
        return new_state

    def yaw_from_quaternion(self, quat):
        r,p, yaw =tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w)) 
        return yaw

    def on_initial_pose(self, pose_msg):
        self.initial_pose = pose_msg

        #Send pose to gazebo
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(self.ModelState_from_Pose2D(pose_msg))

        odom = self.last_odom
        #self.tf = self.tf*delta

        odom_theta = self.yaw_from_quaternion(odom.pose.pose.orientation)

        self.odom_f_pose.theta = self.initial_pose.theta - odom_theta
        print "self.odom_f_pose.theta:", self.odom_f_pose.theta
        self.odom_f_pose.x = self.initial_pose.x - (odom.pose.pose.position.x*math.cos(self.odom_f_pose.theta)-\
            odom.pose.pose.position.y*math.sin(self.odom_f_pose.theta))
        print "self.odom_f_pose.x:", self.odom_f_pose.x
        self.odom_f_pose.y = self.initial_pose.y - (odom.pose.pose.position.x*math.sin(self.odom_f_pose.theta)+\
            odom.pose.pose.position.y*math.cos(self.odom_f_pose.theta))
        print "self.odom_f_pose.y:", self.odom_f_pose.y
        print odom_theta
        print self.initial_pose.theta 

if __name__ == "__main__":
    FakeLocalization()
