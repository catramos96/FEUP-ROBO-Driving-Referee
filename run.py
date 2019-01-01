import os
import time
import sys

# build
print "Building Project ..."
os.system("gnome-terminal -e 'sh -c 'ls; catkin_make''")
print "Build complete."
time.sleep(1)

# lauch master
os.system(
    "gnome-terminal -e 'bash -c \"source devel/setup.bash; roscore; exec bash\"'")
time.sleep(1)

# lauch simulator
os.system("gnome-terminal -e 'bash -c \"source devel/setup.bash; roslaunch conde_world spawn_robot_and_world.launch; exec bash\"'")
time.sleep(1)

# lauch referee
os.system("gnome-terminal -e 'bash -c \"source devel/setup.bash; rosrun conde_referee conde_referee_node; exec bash\"'")
time.sleep(1)

# lauch key teleop
os.system("gnome-terminal -e 'bash -c \"source devel/setup.bash; rosrun conde_key_teleop conde_key_teleop_node.py; exec bash\"'")
time.sleep(1)

# control
os.system("gnome-terminal -e 'bash -c \"source devel/setup.bash; rosrun gazebo_signalling_panel_control gazebo_signalling_panel_control_node; exec bash\"'")
time.sleep(1)

'''
source devel/setup.bash

roslaunch conde_world spawn_robot_and_world

rosrun conde_referee conde_referee_node

#spawn robots
os.system("gnome-terminal -e 'bash -c \"source devel/setup.bash; roslaunch conde_world spawn_robot.launch; exec bash\"'")
time.sleep(1)

#spawn obstacles
#roslaunch conde_world spawn_obstacles_v1.launch
#roslaunch conde_world spawn_obstacles_v2.launch
os.system("gnome-terminal -e 'bash -c \"source devel/setup.bash; roslaunch conde_world spawn_obstacles_v1.launch; exec bash\"'")
time.sleep(1)

#parking challenge
#roslaunch conde_world spawn_parking_obstacles_v1.launch
#roslaunch conde_world spawn_parking_obstacles_v2.launch
os.system("gnome-terminal -e 'bash -c \"source devel/setup.bash; roslaunch conde_world spawn_parking_obstacles_v1.launch; exec bash\"'")
time.sleep(1)

#spawn trafic lights
os.system("gnome-terminal -e 'bash -c \"source devel/setup.bash; roslaunch conde_world spawn_traffic_sign_panels.launch; exec bash\"'")
time.sleep(1)

#control
os.system("gnome-terminal -e 'bash -c \"source devel/setup.bash; rosrun gazebo_signalling_panel_control gazebo_signalling_panel_control_node; exec bash\"'")
time.sleep(1)
'''
