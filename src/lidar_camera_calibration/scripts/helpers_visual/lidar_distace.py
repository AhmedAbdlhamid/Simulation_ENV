#!/usr/bin/env python
from roslib import message
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

#listener
def listen():
    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("/velodyne_obstacles", PointCloud2, callback_kinect)
    rospy.spin()

def callback_kinect(data) :
    # pick a height
    height =  int (data.height / 2)
    # pick x coords near front and center
    middle_x = int (data.width / 2)
    # examine point
    middle = read_depth (middle_x, height, data)
    # do stuff with middle


def read_depth(width, height, data) :
    # read function
    if (height >= data.height) or (width >= data.width) :
        return -1
    data_out = pc2.read_points(data, field_names=None, skip_nans=False, uvs=[[width, height]])
    int_data = next(data_out)
    rospy.loginfo("int_data " + str(int_data))
    return int_data


if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
