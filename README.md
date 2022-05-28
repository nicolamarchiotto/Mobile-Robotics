# Mobile-Robotics

Master Degree in Computer Engineering for Robotics and Smart Industry - A.Y. 2021/2022

ROS code for the Mobile Robotics course

The colcon_ws is the ros ROS2 workspace, the version used is foxy

Here's reported the .bashrc script to not source every time the ros2-foxy setup.bash file, paste them in the ~/.bashrc file to have automatic source upon terminal startup. The export lines are necessary for teleoperating real TurtleBot

source /opt/ros/foxy/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
#export ROS_DOMAIN_ID=30 #TURTLEBOT3
export TURTLEBOT3_MODEL=burger

