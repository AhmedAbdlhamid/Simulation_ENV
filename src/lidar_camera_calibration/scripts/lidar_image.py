#!/usr/bin/env python
import math
import rospy
import sys
import cv2
import cv_bridge
from image_geometry import PinholeCameraModel
import tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import struct
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import open3d

rospy.init_node('lidar_overlay_image')
camera ={}
imageOverlay = {}
cameraInfo = CameraInfo()
cameraModel = PinholeCameraModel()
bridge = cv_bridge.CvBridge()
tf_ = tf.TransformListener()
velodyneData = []

def camera_callback(data):
	global cameraModel, camera
	cameraInfo = data
	print('Receiving Camera Info.. ')
	cameraModel.fromCameraInfo( cameraInfo )

def velodyne_callback(data):
	global velodyneData
	global velodyne_header
	global velodyne_fields
	velodyne_header=data.header
	velodyne_fields=data.fields

	print('In velodyne callback - received point cloud')
	formatString = 'ffff'
	if data.is_bigendian:
		formatString = '>' + formatString
	else:
		formatString = '<' + formatString
	points = []
	for index in range( 0, len( data.data ), 16 ):
		points.append( struct.unpack( formatString, data.data[ index:index + 16 ] ) )
	velodyneData = points

def image_callback(data):
	global velodyneData, bridge, tf_
	print('In input image callback - received rectified image')
	cv_image = {}
	try:
		 cv_image=np.zeros((800,800,3), np.uint8)
	except cv_bridge.CvBridgeError as e:
			print('Failed to convert image', e)
			return 
	(trans, rot) = tf_.lookupTransform( '/velodyne_link', '/camera_link', rospy.Time(0) )
	trans = tuple(trans) + ( 1,  )
	#print(trans)
	#print(rot)
	rotationMatrix = tf.transformations.quaternion_matrix( rot )
	rotationMatrix[ :, 3 ] = trans
	if velodyneData:
		for i in range(0, len(velodyneData) - 1):
			try:
				point = [velodyneData[i][0], velodyneData[i][1], velodyneData[i][2],1]
				if math.sqrt( np.sum( np.array( point[ :3 ] ) ** 2 ) ) > 9.0:
					continue
			except IndexError:
				print("Index Error!!!!!")
				break
			
			rotatedPoint = rotationMatrix.dot( point )
			uv = cameraModel.project3dToPixel( rotatedPoint )
			print((rotatedPoint))
			#data_out=pc2.create_cloud(velodyne_header,velodyne_fields, rotatedPoint)
			#velodyne_filtered_points.publish(data_out)
			#point_cloud = open3d.PointCloud()
			#point_cloud.points = open3d.Vector3dVector(rotatedPoint)
			#open3d.draw_geometries([point_cloud])

			if uv[0] >= 0 and uv[0] <= data.width and uv[1] >= 0 and uv[1] <= data.height:
				cv2.line(cv_image,(int( uv[0] ),int( uv[1] )),(int( uv[0] )+2,int( uv[1] ) +2),(255,0,255),3)

	try:
		imageOverlay.publish(bridge.cv2_to_imgmsg( cv_image, 'bgr8' ) )
	except cv_bridge.CvBridgeError as e:
		#print( 'Failed to convert image', e )
		return

if __name__ == '__main__':
	try:
		camera = rospy.Subscriber(  '/stereo_camera/camera_info' , CameraInfo, camera_callback)
		imageRect = rospy.Subscriber('/stereo_camera/image_rect_color', Image,  image_callback)
		velodynePoint = rospy.Subscriber('/velodyne_obstacles', PointCloud2, velodyne_callback)
		velodyne_filtered_points=rospy.Publisher( 'overlayed_lidar', PointCloud2, queue_size = 1000)
		imageOverlayName = rospy.resolve_name( 'image_overlay')
		imageOverlay = rospy.Publisher( imageOverlayName, Image, queue_size = 1000)
		rospy.spin()
	except rospy.ROSInterruptException: pass