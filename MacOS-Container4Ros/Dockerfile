FROM	ubuntu:14.04

ENV DEBIAN_FRONTEND noninteractive

RUN		apt-get update
RUN 	apt-get install -y wget
RUN		apt-get install -y git curl nano
RUN 	apt-get install -y firefox
RUN 	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN 	sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
RUN		apt-get update
RUN 	sudo apt-get install -y ros-indigo-desktop-full
RUN 	sudo rosdep init
RUN 	sudo rosdep fix-permissions
RUN 	rosdep update
RUN 	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
ENV		. ~/.bashrc
RUN 	sudo apt-get install -y python-rosinstall
RUN 	sudo apt-get install -y python-setuptools python-dev build-essential
RUN 	sudo easy_install pip
RUN 	sudo pip install --upgrade virtualenv
RUN 	pip install pygame
RUN 	pip install scikit-image
ENV 	DEBIAN_FRONTEND teletype

#FROM	ubuntu:14.04

#ENV DEBIAN_FRONTEND noninteractive

#RUN		apt-get update
#RUN 	apt-get install -y wget
#RUN		apt-get install -y git curl nano
#RUN 	apt-get install -y firefox
#RUN 	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN 	sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
#RUN		apt-get update
#RUN 	sudo apt-get install -y ros-indigo-desktop-full
#RUN 	sudo rosdep init
#RUN 	sudo rosdep fix-permissions
#RUN 	rosdep update
#RUN 	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
#ENV		. ~/.bashrc
#RUN 	sudo apt-get install -y python-rosinstall
#CMD 	["roscore"]
#
#RUN 	sudo apt-get install -y ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator #ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
#ENV 	DEBIAN_FRONTEND teletype
#RUN 	rosdep update
#CMD 	["roslaunch", "turtlebot_gazebo turtlebot_world.launch"]
#CMD /usr/bin/firefox