#!/usr/bin/python
import rosbag
import rospy
import rospkg

import os

from geometry_msgs.msg import Pose2D
from merlin_msgs.msg import Path2D
from merlin_msgs.srv import GetPaths, SavePath, StartPath, AppendPath, GetPath

PATH_FILE = rospkg.RosPack().get_path('merlin_nav')+'/paths.bag' 

def load_paths():
    loaded_paths = []
    
    if os.path.isfile(PATH_FILE):
        with rosbag.Bag(PATH_FILE) as bag:
            for topic, msg, t in bag.read_messages(['path']):
                print msg
                loaded_paths.append(msg)
    else:
        with rosbag.Bag(PATH_FILE, 'w') as bag:
            rospy.loginfo("Create path bag")

    return loaded_paths

class PathStore:
    def __init__(self):
        rospy.init_node('path_store', log_level=rospy.DEBUG)

        self.paths = load_paths()    
        self.path_queue = []

        #Store services
        rospy.Service('get_paths', GetPaths, self.send_paths)
        rospy.Service('save_path', SavePath, self.save_path)
        rospy.Service('start_path', StartPath, self.start_path)

        #Queue services
        rospy.Service("get_queue", GetPaths, self.send_queue)
        rospy.Service("get_next_path", GetPath, self.send_path_queued)
        rospy.Service("append_path", AppendPath, self.append_path)

        rospy.loginfo('path_store started')

        rospy.spin()

    def send_queue(self, req):
        return {"paths": self.path_queue}

    def send_path_queued(self, req):
        if self.path_queue:
            rospy.loginfo("Send path")
            path = self.path_queue.pop(0) 
        else:
            rospy.loginfo("Returning empty path")
            path = Path2D()

        return {"path": path}

    def append_path(self, req):
        for path in self.paths:
            if path.name == req.path_name:
                self.path_queue.append(path)
                rospy.loginfo("Queue is now")
                print self.path_queue
        return {"success": True}
                    
    def start_path(self, req):
        path_pub = rospy.Publisher(req.robot_name+"/path", Path2D)
        rospy.loginfo("Wait for publisher to be ready")
        rospy.sleep(2)
        print req
        
        matching = [path for path in self.paths if req.path_name in path.name]
        if matching:
            path_pub.publish(matching[0])
            success = True
        else:
            rospy.logerr("This path does not exist")
            success = False
                
        return {'success': success}

    def send_paths(self, req):
        return {'paths': self.paths}

    def save_path(self, req):
        rospy.loginfo("Received a path: "+ req.path.name)
        print req.path
        
        with rosbag.Bag(PATH_FILE, 'a') as bag:
            bag.write('path', req.path)
            self.paths.append(req.path)
            rospy.loginfo("path saved")    

        return []


if __name__ == '__main__':
    PathStore()

