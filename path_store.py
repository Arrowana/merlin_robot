#!/usr/bin/python
import rosbag
import rospy

from geometry_msgs.msg import Pose2D
from merlin_msgs.msg import Path2D
from merlin_msgs.srv import GetPaths, SavePath

PATH_FILE = 'paths.bag' 

def load_paths():
    loaded_paths = []

    with rosbag.Bag(PATH_FILE) as bag:
        for topic, msg, t in bag.read_messages(['path']):
            print msg
            loaded_paths.append(msg)

    return loaded_paths

class PathStore:
    def __init__(self):
        rospy.init_node('path_store', log_level=rospy.DEBUG)

        self.paths = load_paths()    

        rospy.Service('get_paths', GetPaths, self.send_paths)
        rospy.Service('save_path', SavePath, self.save_path)

        rospy.loginfo('path_store started')

        rospy.spin()
                    
    def send_paths(self, req):
        answer = GetPaths()
        answer.paths = self.paths
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

