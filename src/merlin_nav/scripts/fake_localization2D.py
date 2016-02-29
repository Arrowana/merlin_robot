#!/usr/bin/python
"""
    Node for simulation. Set inital pose in gazebo when received
    Send pose regarding odom update    
"""

import rospy
import numpy
import tf.transformations
import math
import control
import re

from geometry_msgs.msg import Pose2D, Quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState

class FakeLocalization(object):
    def __init__(self):
        rospy.init_node('fake_localization2D')
        self.odom_sub = rospy.Subscriber("base_controller/odom", Odometry, self.on_odom)
        self.inital_pose_sub = rospy.Subscriber("initial_pose", Pose2D, self.on_initial_pose)

        self.pose_pub = rospy.Publisher("pose", Pose2D)

        self.last_odom = Odometry()
        self.odom_f_pose = Pose2D()

        #Wait for gazebo to start
        rospy.sleep(5.0)

        #At startup get model state to get inital_pose
        self.current_state()

        rospy.spin()

    def current_state(self):
        rospy.loginfo("Get model state to set fake localization")
        #Send pose to gazebo
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_state(model_name=rospy.get_namespace()[1:-1], relative_entity_name="world")
        print resp
        
        initial_pose = Pose2D()
        initial_pose.x = resp.pose.position.x
        initial_pose.y = resp.pose.position.y
        initial_pose.theta = self.yaw_from_quaternion(resp.pose.orientation)
        self.on_initial_pose(initial_pose)
        rospy.loginfo("Retreived model_state from gazebo to set global pose")

    def on_odom(self, odom_msg):
        self.last_odom = odom_msg

        #Sum odometry to the odom frame position in the global frame
        pose = Pose2D()
        pose.x = self.odom_f_pose.x + odom_msg.pose.pose.position.x*math.cos(self.odom_f_pose.theta)-\
            odom_msg.pose.pose.position.y*math.sin(self.odom_f_pose.theta)
        pose.y = self.odom_f_pose.y + odom_msg.pose.pose.position.x*math.sin(self.odom_f_pose.theta)+\
            odom_msg.pose.pose.position.y*math.cos(self.odom_f_pose.theta)
        pose.theta = control.cap_angle(self.odom_f_pose.theta + self.yaw_from_quaternion(odom_msg.pose.pose.orientation))

        self.pose_pub.publish(pose)

    def ModelState_from_Pose2D(self, pose):
        new_state = ModelState()

        #Get namespace and remove first and last slash
        ns = rospy.get_namespace()
        new_state.model_name = ns[1:-1]
        new_state.pose.position.x = pose.x
        new_state.pose.position.y = pose.y

        #Build quaternion
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = tf.transformations.quaternion_from_euler(0,0, pose.theta)
        new_state.pose.orientation = quat
        return new_state

    def yaw_from_quaternion(self, quat):
        r,p, yaw = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w)) 
        return yaw

    def on_initial_pose(self, pose_msg):
        initial_pose = pose_msg

        #Send pose to gazebo
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(self.ModelState_from_Pose2D(pose_msg))

        odom = self.last_odom

        odom_theta = self.yaw_from_quaternion(odom.pose.pose.orientation)

        self.odom_f_pose.theta = initial_pose.theta - odom_theta
        self.odom_f_pose.x = initial_pose.x - (odom.pose.pose.position.x*math.cos(self.odom_f_pose.theta)-\
            odom.pose.pose.position.y*math.sin(self.odom_f_pose.theta))
        self.odom_f_pose.y = initial_pose.y - (odom.pose.pose.position.x*math.sin(self.odom_f_pose.theta)+\
            odom.pose.pose.position.y*math.cos(self.odom_f_pose.theta))
        rospy.loginfo("Received inital pose msg, set pose in gazebo simulator")

if __name__ == "__main__":
    FakeLocalization()
