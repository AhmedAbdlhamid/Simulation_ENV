#!/usr/bin/env python
import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg

if __name__ == '__main__':
    rospy.init_node('visualization')
    pointcloud_publisher = rospy.Publisher("/pointcloud_visual", PointCloud,queue_size = 1000)
    rospy.sleep(0.5)
    pointcloud_publish = PointCloud()
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    pointcloud_publish.header = header
    pointcloud_publish.points.append(Point32(1.0, 1.0, 0.0))
    pointcloud_publish.points.append(Point32(2.0, 2.0, 0.0))
    pointcloud_publish.points.append(Point32(3.0, 3.0, 0.0))
    pointcloud_publisher.publish(pointcloud_publish)
    rospy.spin()