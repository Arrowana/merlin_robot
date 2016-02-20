#!/usr/bin/python
import rospy
from std_msgs.msg import String, Bool
import functools

def notify_alive(pub, msg):
    rospy.loginfo("Notify that node is alive") 
    pub.publish(rospy.get_namespace())

def test():
    pass
    
if __name__ == "__main__":
    rospy.init_node("brain")

    alive_pub = rospy.Publisher("/alive", String, queue_size=10)
    rospy.Subscriber("/who_is_alive", Bool, functools.partial(notify_alive, alive_pub))

    rospy.loginfo("Done")

    rospy.spin()
