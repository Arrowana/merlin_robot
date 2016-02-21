start merlin with ./start_merlin

TODO:
setup fake localisation http://wiki.ros.org/fake_localization
FIRST NAVIGATION
FIRST FRONT END

DOC:
take a differential gazebo_ros interface https://github.com/ros-simulation/gazebo_ros_pkgs/blob/indigo-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_diff_drive.h

add teleop capabilities in the launch file

TIPS:
For development launch an empty gazebo then kill/relaunch the spawn of the model only (avoid the slow GUI loading). Use the nogz argument in merlin.launch
teleops from keyboard http://wiki.ros.org/teleop_twist_keyboard

roslaunch gazebo_ros empty_world.launch
roslaunch merlin.launch nogz:=true

USEFUL COMMANDS:
roslaunch rosbridge_server rosbridge_websocket.launch

rostopic pub -r 10 merlin/base_contller/cmd_vel geometry_msgs/Twist '[0.5, 0, 0]' '[0,0,0.4]'

//Set the position via inital_pose (safe)
rostopic pub merlin/inital_pose geometry_msgs/Pose2D '{x: 0, y:0, theta: 0}'

//Set the position of the robot in simulation, be careful this will offset odometry
rosservice call gazebo/set_model_state '{model_state: {model_name: "merlin", pose: {position: {x: 2, y: 3}}}}'

path_store.py is a proof of concept node to read/write path to a rosbag
//get existing paths
rosservice call get_paths "{}"

//Save manually a path
rosservice call /save_path "path:
  name: 'first'
  poses:
  - x: 1.0
    y: 1.0
    theta: 0.0
  - x: 2.0
    y: 2.0
    theta: 0.0
  - x: 2.0
    y: 4.0
    theta: 0.0
  - x: 4.0
    y: 4.0
    theta: 0.0"
 
rosservice call /save_path "path:
  name: 'second'
  poses:
  - x: 1.0
    y: 1.0
    theta: 0.0
  - x: 2.0
    y: 1.0
    theta: 0.0
  - x: 2.0
    y: 4.0
    theta: 0.0
  - x: 4.0
    y: 4.0
    theta: 0.0"
