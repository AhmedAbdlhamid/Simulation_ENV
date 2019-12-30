#!/usr/bin/env python
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

class coordinates:

    def __init__(self):
        self.bbox=rospy.Subscriber('/image_coordinates', Vector3, self.coordinates)
        self.bbox=rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes, self.detect)
        self.u=None
        self.v=None
        self.distance=None

    def detect(self, data):    
        box_data=data.bounding_boxes
        bbox_xmin=(box_data[0].xmin)
        bbox_ymin=(box_data[0].ymin)
        bbox_xmax=(box_data[0].xmax)
        bbox_ymax=(box_data[0].ymax)
        xB=bbox_xmax
        yB=bbox_ymax
        xA=bbox_xmin
        yA=bbox_ymin
        if(xA < self.u < xB) and (yA < self.v < yB):
            print(self.distance)
            print("Class", box_data[0].Class)

    def coordinates(self, data):
        self.u = data.x
        self.v= data.y
        self.distance=data.z

if __name__ == "__main__":
    rospy.init_node("access")
    detector = coordinates()
    rospy.spin()
