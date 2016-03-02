import rospy
from merlin_msgs.srv import GetPaths, SavePath, AppendPath

path_queue = []

def send_queue(req):
    return {"paths": path_queue} 

def send_path(req):
    #Remove the first path in the queue
    path = path_queue.pop(0)
    return {"path": path}

def append_path(req):
    path_queue.append(req.path)
    rospy.loginfo("Queue is now")
    print path_queue

if __name__ == "__main__":
    rospy.Service("get_queue", GetPaths, send_queue)
    #Used by robot
    rospy.Service("get_next_path", GetPath, send_path)
    rospy.Service("append_path", AppendPath, append_path)

    rospy.spin()
