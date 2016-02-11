TODO:
setup fake localisation http://wiki.ros.org/fake_localization

DONE:
write and load robot description in the parameter server

set up the gazebo controller to be able to control the joints

take a differential gazebo_ros interface https://github.com/ros-simulation/gazebo_ros_pkgs/blob/indigo-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_diff_drive.h

add teleop capabilities in the launch file

TIPS:
For development launch an empty gazebo then kill/relaunch the spawn of the model only (avoid the slow GUI loading). Use the nogz argument in merlin.launch
roslaunch gazebo_ros empty_world.launch
roslaunch merlin.launch nogz:=true

teleops from keyboard http://wiki.ros.org/teleop_twist_keyboard
