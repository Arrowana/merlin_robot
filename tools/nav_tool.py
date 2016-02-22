import rospy

from merlin_msgs.srv import GetPaths
from geometry_msgs.msg import Pose2D

import matplotlib.pyplot as plt
import matplotlib.image as img
import argparse
import math

def draw_map(filename, res=1/20.0):
    """Draw map given file and resolution in m/px"""
    map_image = img.imread(filename) 
    dims = map_image.shape

    #Transform dimensions in px to meters
    width=dims[1]*res
    height=dims[0]*res
    rect=(0, width, 0, height)
    imgplot = plt.imshow(map_image, aspect = 'equal', extent = rect)

def draw_path(path):
    plt.plot([wpt.x for wpt in path.poses], [wpt.y for wpt in path.poses])

class NavTools:
    def __init__(self):
        rospy.Subscriber("merlin/pose", Pose2D, self.on_pose) 
        self.initial_pose_pub = rospy.Publisher("merlin/initial_pose", Pose2D)
        self.ax = plt.gca()
        self.Q = self.ax.quiver([0], [0], [1], [0])

    def on_pose(self, pose):
        self.Q.set_offsets([pose.x, pose.y])
        self.Q.set_UVC(math.cos(pose.theta), math.sin(pose.theta))
        plt.draw()

    def on_press(self, event):
        if event.button == 1:
            print event.xdata, event.ydata
            self.initial_pose_pub.publish(x=event.xdata, y=event.ydata, theta=0)
        elif event.button == 3:
            print "Not implemented"

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', help='robot name', default='merlin')
    args=parser.parse_args()

    rospy.init_node('nav_tool')
    get_paths = rospy.ServiceProxy('get_paths', GetPaths)
    resp=get_paths()
    paths = resp.paths

    fig = plt.figure()
    nav_tools = NavTools()

    fig.canvas.mpl_connect('button_press_event', nav_tools.on_press)
    for path in paths:
        draw_path(path)

    draw_map('factory_map.png', 2.6/77.0) #Truck is 2.6m for 77 px
    plt.show()
