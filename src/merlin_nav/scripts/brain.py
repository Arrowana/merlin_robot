#!/usr/bin/python
import rospy
from std_msgs.msg import String, Bool
from merlin_msgs.msg import Path2D

from merlin_msgs.srv import GetPath
from std_srvs.srv import Trigger

import functools

def notify_alive(pub, msg):
    rospy.loginfo("Notify that node is alive") 
    pub.publish(rospy.get_namespace())

def on_trigger(req):
    """Nav is available and want a new path"""
    get_next_path = rospy.ServiceProxy("/get_next_path", GetPath)
    resp = get_next_path()
    
    path_pub = rospy.Publisher("path", Path2D)
    rospy.sleep(2.0)
    path_pub.publish(resp.path)

    return {}

if __name__ == "__main__":
    rospy.init_node("brain")

    alive_pub = rospy.Publisher("/alive", String, queue_size=10)
    rospy.Subscriber("/who_is_alive", Bool, functools.partial(notify_alive, alive_pub))
    rospy.Service("ready", Trigger, on_trigger)

    rospy.loginfo("Done")

    rospy.spin()
