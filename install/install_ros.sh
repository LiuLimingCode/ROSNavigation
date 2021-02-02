#!/bin/bash

# Varibles
rosversion="kinetic"
# Install the ros

if [ `id -u` == 0 ]; then
	echo "Don't running this use root(sudo)."
	exit 0
fi

echo "Start to install the ros, http://wiki.ros.org/$rosversion/Installation/Ubuntu"
echo "Update the software list"
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

echo "Install the ros from apt" 
sudo apt-get install ros-$rosversion-desktop-full -y

echo "Setup the ROS environment variables"
echo -e "if [ -f /opt/ros/$rosversion/setup.bash ]; then\n\tsource /opt/ros/$rosversion/setup.bash\nfi" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init
rosdep update

echo "--Finish"
