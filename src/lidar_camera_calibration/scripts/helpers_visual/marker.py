from geometry_msgs.msg import Vector3 , Point , Pose ,Quaternion , Point32
import rospy
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
rospy.init_node('visual')

def show_text_in_rviz(marker_publisher):
    point=Point()
    point.x=0.5
    point.y=0.5
    point.z=1.45
    marker = Marker(
                type=Marker.POINTS,
                id=0,
                lifetime=rospy.Duration(60),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(5, 5, 5),
                header=Header(frame_id='chassis'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
    marker.points.append(point)
    marker_publisher.publish(marker)


if __name__ == '__main__':
    try:
        marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        rospy.sleep(0.5)                                                             
        show_text_in_rviz(marker_publisher)
        rospy.spin()
    except rospy.ROSInterruptException: pass