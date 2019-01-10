How to test the code:
in a terminal compile the code using the command
catkin_make

(be sure to source the terminal to the correct setup.bash, either with source devel/setup.bash or to your ROS instalation folder)

To run the program after compiling it:
in 3 terminals run

roslaunch conde_world spawn_robot_and_world
rosrun conde_referee conde_referee_node
rosrun conde_key_teleop conde_key_teleop_node.py

this last command will be used to move the robot.
If the robot you're using has an autonomous module, refrain from using the third command


The developed software was successfully developed and tested within the following specifications:
Ubuntu 16.04 operating system.
ROS Kinetic distribution.
Gazebo 9.0.0

No aditional packages are required to compile the code if the full guide on ROS installation is followed.

Source code included in src folder together with the makefile.
Executables are automatically generated into a bin folder

FEUP - Mestrado Integrado em Engenharia Informática e Computação - Robotics

github: https://github.com/catramos96/FEUP-ROBO-Driving-Referee (it is private, please contact
ec12143@fe.up.pt for access)

Project made by:
Catarina Ramos
Mário Fernandes
Tiago Filipe





