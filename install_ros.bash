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
#sudo rosdep init
#rosdep update

echo "Setup the ROS environment variables"
echo -e "if [ -f /opt/ros/kinetic/setup.bash ]; then\n\tsource /opt/ros/kinetic/setup.bash\nfi" >> ~/.bashrc
echo "source ~/racecar/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Install the rosinstall"
sudo apt-get install python-rosinstall -y

echo "Install the ssh"
sudo apt-get install ssh -y

echo "Install the ntpdate"
sudo apt-get install ntpdate -y

echo "Install the chrony"
sudo apt-get install chrony -y

# Install the dependecies for the project 
echo "Start to config for the project"

#echo "Install the python dependecies"
#sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose -y

#echo "Install the eigen3"
#sudo apt install libeigen3-dev -y

#echo "Install the nlopt"
#sudo apt install libnlopt* -y



echo "Install the ROS package for art_racecar"
sudo apt-get install ros-$rosversion-joy -y
sudo apt-get install ros-$rosversion-move-base -y
sudo apt-get install ros-$rosversion-mrpt* -y
sudo apt-get install ros-$rosversion-geographic-msgs -y
sudo apt-get install ros-$rosversion-map-server -y
sudo apt-get install ros-$rosversion-gmapping -y
sudo apt-get install ros-$rosversion-amcl -y
sudo apt-get install ros-$rosversion-rviz-imu-plugin -y
sudo apt-get install ros-$rosversion-dwa-local-planner -y

sudo apt install -y ros-$rosversion-joy
sudo apt install -y ros-$rosversion-joystick-drivers

# vis
sudo apt install -y ros-$rosversion-urdf-tutorial 

# sim
sudo apt install -y ros-$rosversion-gazebo-ros-pkgs
sudo apt install -y ros-$rosversion-gazebo-ros-control
sudo apt install -y ros-$rosversion-ros-control
sudo apt install -y ros-$rosversion-ros-controllers

# adv
sudo apt install -y ros-$rosversion-robot-pose-publisher 

# info
sudo apt install -y ros-$rosversion-jsk-visualization
sudo apt install -y ros-$rosversion-robot-pose-publisher

# camera
sudo apt install -y ros-$rosversion-uvc-camera
sudo apt install -y ros-$rosversion-image-transport
sudo apt install -y ros-$rosversion-image-transport-plugins
sudo apt install -y ros-$rosversion-camera-calibration
sudo apt install -y ros-$rosversion-image-proc
sudo apt install -y ros-$rosversion-opencv-apps

# web
sudo apt install -y ros-$rosversion-roswww
sudo apt install -y ros-$rosversion-rosbridge-suite 
sudo apt install -y ros-$rosversion-web-video-server

# nav
sudo apt install -y ros-$rosversion-robot-localization
sudo apt install -y ros-$rosversion-gmapping
sudo apt install -y ros-$rosversion-amcl
sudo apt install -y ros-$rosversion-map-server
sudo apt install -y ros-$rosversion-move-base
sudo apt install -y ros-$rosversion-navigation

# arm
sudo apt install -y ros-$rosversion-moveit
sudo apt install -y ros-$rosversion-moveit-ros-visualization

# hardware
sudo apt install -y ros-$rosversion-rosserial
sudo apt install -y ros-$rosversion-rosserial-arduino

# other (not ROS)
sudo apt install -y chrony
sudo apt install -y libarmadillo-dev libarmadillo6
sudo apt install -y lpc21isp

echo "--Finish"
