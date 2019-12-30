
SHELL := /bin/bash

WS_DIR := ${CURDIR}

ros:
	@echo "== Building ROS workspace =="
	cd '${WS_DIR}'/ && source /opt/ros/kinetic/setup.bash && catkin_make -j8 -DCMAKE_BUILD_TYPE=Release && source devel/setup.bash && sudo cp -a libTrafficLightsGUIPlugin.so devel/lib/


