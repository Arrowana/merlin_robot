#!/usr/bin/python
"""
    Nav is performing navigation given a path. When path is finish it reports to the brain
"""
import rospy

from merlin_msgs.msg import Path2D
from geometry_msgs.msg import Pose2D, Twist

from std_srvs.srv import Trigger

import control
import math

p1 = 0.5
p2 = 0.5

V = 0.5

def to_target_frame(pose, target):
    pose_target = Pose2D()
    pose_target.theta = control.cap_angle(pose.theta - target.theta)
    pose_target.x = (pose.x-target.x)*math.cos(target.theta)+(pose.y-target.y)*math.sin(target.theta)
    pose_target.y = -(pose.x-target.x)*math.sin(target.theta)+(pose.y-target.y)*math.cos(target.theta)
    return pose_target

class PathController(object):
    def __init__(self, path):
        self.speed_pub = rospy.Publisher("base_controller/cmd_vel", Twist)
        self.pose = Pose2D()
        self.pose_sub = rospy.Subscriber("pose", Pose2D, self.on_pose)
        rospy.sleep(1.0)
        self.navigate(path.poses)

    def on_pose(self, pose):
        self.pose = pose

    def navigate(self, path):
        generated_path = [self.pose] + path
        previous_waypoint = generated_path.pop(0)
        #Complete path starts from the robot
        for waypoint in generated_path:
            target = Pose2D()
            target = waypoint

            #Target heading is the direction between the current waypoint and the previous waypoint
            target.theta = math.atan2(waypoint.y-previous_waypoint.y,\
                 waypoint.x - previous_waypoint.x)
            rospy.loginfo("current pose:")
            print self.pose
            rospy.loginfo("target:")
            print target

            initial_error = to_target_frame(self.pose, target)

            #Check if we need to perform a spot turn to align the robot
            if abs(initial_error.theta) > math.pi/4:
                spot_turn_mode = True
            else:
                spot_turn_mode = False

            while True:
                pose_target = to_target_frame(self.pose, target)
                print "pose_target:", pose_target
                twist = Twist()

                if spot_turn_mode:
                    if abs(pose_target.theta) < math.pi/15:
                        spot_turn_mode = False
                    else:
                        twist.angular.z = -math.copysign(0.2, pose_target.theta)
                else:
                    #Check waypoint reached condition
                    if pose_target.x > 0:
                        rospy.loginfo("NEXT")
                        previous_waypoint = waypoint
                        break
                    else:
                        w = -(p1*pose_target.y + p2*pose_target.theta) 
                        twist.linear.x = V
                        twist.angular.z = w

                self.speed_pub.publish(twist)
                rospy.sleep(0.1)

        rospy.loginfo("End of path reached")

def path_received(path_msg):
    rospy.loginfo("Path received")
    PathController(path_msg)

    rospy.loginfo("Path completed")

    #Report that control is done
    report_done = rospy.ServiceProxy("ready", Trigger)
    report_done()

if __name__ == '__main__':
    rospy.init_node("navigator")        
    
    rospy.Subscriber("path", Path2D, path_received)

    rospy.spin() 
