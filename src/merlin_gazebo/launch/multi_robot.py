import os

for i in range(10):
    os.system("roslaunch merlin_robot.launch robot_name:=yo"+str(i)+" offset:="+str(i)+"&")
