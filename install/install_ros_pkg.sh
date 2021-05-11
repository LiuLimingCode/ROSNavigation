#!/bin/bash

# Varibles
rosversion="melodic"

if [ `id -u` == 0 ]; then
	echo "Don't running this use root(sudo)."
	exit 0
fi

#sudo apt-get update
sudo apt-get upgrade -y

echo "Install the rosinstall"
sudo apt-get install -y python-rosinstall

echo "Install the ssh"
sudo apt-get install -y ssh

echo "Install the ntpdate"
sudo apt-get install -y ntpdate

echo "Install the chrony"
sudo apt-get install -y chrony

echo "Install the python dependecies"
sudo apt-get install -y ipython
sudo apt-get install -y python-numpy
sudo apt-get install -y python3-numpy
sudo apt-get install -y python-scipy
sudo apt-get install -y python3-scipy
sudo apt-get install -y python-matplotlib
sudo apt-get install -y python3-matplotlib
sudo apt-get install -y python-tk
sudo apt-get install -y python3-tk

echo "Install the eigen3"
sudo apt install -y libeigen3-dev

echo "Install the nlopt"
sudo apt install -y libnlopt*


echo "Install the ROS package for art_racecar"
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
sudo apt install -y glibc-doc
sudo apt install -y manpages-posix
sudo apt install -y manpages-posix-dev
sudo apt install -y ros-$rosversion-uvc-camera
sudo apt install -y ros-$rosversion-image-transport
sudo apt install -y ros-$rosversion-image-transport-plugins
sudo apt install -y ros-$rosversion-camera-calibration
sudo apt install -y ros-$rosversion-image-proc
sudo apt install -y ros-$rosversion-opencv-apps
sudo apt install -y ros-$rosversion-ecl-threads
sudo apt install -y ros-$rosversion-libuvc
sudo apt install -y ros-$rosversion-libuvc-*
sudo apt install -y ros-$rosversion-rgbd-launch
sudo apt install -y ros-$rosversion-libuvc
sudo apt install -y ros-$rosversion-libuvc-camera
sudo apt install -y ros-$rosversion-libuvc-ros
sudo apt install -y ros-$rosversion-openni-launch

# nav
sudo apt install -y ros-$rosversion-robot-localization
sudo apt install -y ros-$rosversion-gmapping
sudo apt install -y ros-$rosversion-amcl
sudo apt install -y ros-$rosversion-map-server
sudo apt install -y ros-$rosversion-move-base
sudo apt install -y ros-$rosversion-navigation
sudo apt install -y ros-$rosversion-teb-local-planner
sudo apt install -y ros-$rosversion-ackermann-msgs
sudo apt install -y ros-$rosversion-hector-slam
sudo apt install -y ros-$rosversion-slam-karto
sudo apt install -y ros-$rosversion-move-base
sudo apt install -y ros-$rosversion-mrpt-*
sudo apt install -y ros-$rosversion-geographic-msgs
sudo apt install -y ros-$rosversion-rviz-imu-plugin

# arm
sudo apt install -y ros-$rosversion-moveit
sudo apt install -y ros-$rosversion-moveit-ros-visualization

# hardware
sudo apt install -y ros-$rosversion-rosserial
sudo apt install -y ros-$rosversion-rosserial-arduino
sudo apt install -y ros-$rosversion-serial

# other (not ROS)
sudo apt install -y chrony
sudo apt install -y libarmadillo-dev
sudo apt install -y libarmadillo6
sudo apt install -y lpc21isp

