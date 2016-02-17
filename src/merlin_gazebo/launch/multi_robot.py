import os

for i in range(20):
    os.system("roslaunch merlin_robot.launch robot_name:=yo"+str(i)+" offset:="+str(i)+"&")
