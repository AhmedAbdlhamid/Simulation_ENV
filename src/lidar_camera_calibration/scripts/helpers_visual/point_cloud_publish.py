#!/usr/bin/env python
import rospy
import math
import sys
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

if __name__ == '__main__':
    rospy.init_node('visualization')
    pcl_pub = rospy.Publisher("/pointcloud_visual", PointCloud2,queue_size = 1000)
    rospy.sleep(1.)
    cloud_points = [[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]
    print(type(cloud_points))
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    point_cloud_1 = pcl2.create_cloud_xyz32(header, cloud_points)
    pcl_pub.publish(point_cloud_1)
    rospy.spin()