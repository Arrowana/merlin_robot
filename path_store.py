from geometry_msgs.msg import Pose2D
from merlin_msgs.msg import Path2D
from merlin_msgs.srv import GetPaths, SavePath

import rosbag
import rospy

#Path2D is an array of Pose2D with name and id
def load_paths():
    loaded_paths = []

    with rosbag.Bag('paths.bag') as bag:
        for topic, msg, t in bag.read_messages(['path']):
            print msg

    return loaded_paths
                
def send_paths(req):
    return GetPaths(paths)

def save_path(req):
    print req.path

if __name__ == '__main__':
    rospy.init_node('path_store')

    paths = load_paths()    
    
    rospy.Service('get_paths', GetPaths, send_paths)
    rospy.Service('save_path', SavePath, save_path)
    rospy.spin()
