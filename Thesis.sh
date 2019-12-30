
# !/bin/bash
SLEEP_TIME=5
env (){
	source ~/mcity_ws/devel/setup.bash
	roslaunch car_demo bolt.launch

}


trans (){
	#rosrun tf static_transform_publisher -3.95 0.1988317 3.19931767 3.08820603 4.28580279 4.78443878 /velodyne_link /camera_link 10
	rosrun tf static_transform_publisher -3.95 0.19 3.19 3.08 4.30 4.70 /velodyne_link /camera_link 10
}

map(){
rosrun velodyne_height_map heightmap_node _grid_dimensions:=100 _cell_size:=0.25 _full_clouds:=true
}

lidar(){
	cd ~/mcity_ws/src/lidar_camera_calibration/scripts/ &&
	python2 lidar_image.py 
}
overlay(){
	cd ~/mcity_ws/src/lidar_camera_calibration/scripts/ &&
	python2 lidar_overlay_image.py  
}

YOLO_Start(){
	cd ~/workspaces/yolo_ros_ws
	roslaunch darknet_ros darknet_lidar.launch
}

if [ "SIL" = "$1" ]; then
	env
	sleep $((SLEEP_TIME*12))
	trans
	sleep $((SLEEP_TIME))
	map
	sleep $((SLEEP_TIME))
	lidar
	
fi
if [ "env" = "$1" ]; then
	env
fi
if [ "map" = "$1" ]; then
	map
fi
if [ "trans" = "$1" ]; then
	trans
fi
if [ "lidar" = "$1" ]; then
	lidar
fi
if [ "overlay" = "$1" ]; then
	overlay
fi
