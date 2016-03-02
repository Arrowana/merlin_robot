#!/usr/bin/python
"""
    Publish at lower rate robot_name/pose for control and webapp
    Initial pose setting
"""
import rospy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion

from gazebo_msgs.srv import SetModelState

class LocalisationSim:
    def __init__(self):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_states)
        rospy.Subscriber("initial_pose", Pose2D, self.on_initial_pose)

        self._pub = rospy.Publisher("pose", Pose2D)

        self.robot_name = rospy.get_namespace()[1:-1]
        self.numb = 0

    def on_states(self, states):
        if self.robot_name in states.name:
            if self.numb > 9:
                pose3D=states.pose[states.name.index(self.robot_name)]
                pose2D = Pose2D()
                pose2D.x = pose3D.position.x
                pose2D.y = pose3D.position.y

                q = pose3D.orientation
                pose2D.theta = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

                self._pub.publish(pose2D)

                self.numb = 0
            self.numb+=1

    def on_initial_pose(self, pose_msg):
        initial_pose = pose_msg

        #Send pose to gazebo
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(self.ModelState_from_Pose2D(pose_msg))

if __name__ == "__main__":
    rospy.init_node("pose_publisher")
    LocalisationSim()

    rospy.spin()
