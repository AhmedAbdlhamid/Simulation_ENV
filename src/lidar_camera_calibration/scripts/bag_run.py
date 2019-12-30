#!/usr/bin/env python
import cv2
import cv_bridge
from image_geometry import PinholeCameraModel
import tf
import sensor_msgs
import rospy
import sys
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Vector3 , Point , Pose ,Quaternion
import pcl
import struct
import std_msgs.msg
from std_msgs.msg import Float32
from std_msgs.msg import Header, ColorRGBA
import sensor_msgs.point_cloud2 as pcl2


import math
import numpy as np

class LidarImage:
	def __init__( self ):
		self._imageInput = rospy.Subscriber( '/sensors/camera/image_color', Image, callback = self.imageCallback, queue_size = 1 )
		self._camera = rospy.Subscriber( '/sensors/camera/camera_info', CameraInfo, callback = self.cameraCallback, queue_size = 1 )
		self._velodyne = rospy.Subscriber( '/sensors/velodyne_points', PointCloud2, callback = self.velodyneCallback, queue_size = 20 )
		self._imageOutput = rospy.Publisher( 'image_overlay', Image, queue_size = 10 )
		self._bridge = cv_bridge.CvBridge()
		self._cameraModel = PinholeCameraModel()
		self._tf = tf.TransformListener()
		self.point_list = []
		self.image_coordinates=Vector3()

	def imageCallback( self, data ):
		print( 'Received an image!' )
		cvImage = {}
		try:
			cvImage = self._bridge.imgmsg_to_cv2( data, 'bgr8' )
		except cv_bridge.CvBridgeError as e:
			rospy.logerr( '[lidar_image] Failed to convert image' )
			rospy.logerr( '[lidar_image] ' + e )
			print( e )
			return
		( translation, rotation ) = self._tf.lookupTransform( '/world', '/velodyne', rospy.Time( 0 ) )
		translation = tuple(translation) + ( 1,  )
		Rq = tf.transformations.quaternion_matrix( rotation )
		Rq[ :, 3 ] = translation
		if self._velodyneData:
			for i in range( 0, len( self._velodyneData ) - 1 ):
				try:
					point = self._velodyneData[ i ][ :3 ] + ( 1, )
					if math.sqrt( np.sum( np.array( point[ :3 ] ) ** 2 ) ) > 2.5:
						continue
				except IndexError:
					break
				self.point_list.append([self._velodyneData[i][0], self._velodyneData[i][1], self._velodyneData[i][2]])
				rotatedPoint = Rq.dot( point )
				uv = self._cameraModel.project3dToPixel( rotatedPoint )
				if uv[ 0 ] >= 0 and uv[ 0 ] <= 1920 and uv[ 1 ] >= 0 and uv[ 1 ] <= 1200:
					cv2.circle( cvImage, ( int( uv[ 0 ] ), int( uv[ 1 ] ) ), 2,  (255,0,0))

		try:
			self._imageOutput.publish( self._bridge.cv2_to_imgmsg( cvImage, 'bgr8' ) )
		except cv_bridge.CvBridgeError as e:
			rospy.logerr( '[lidar_image] Failed to convert image to message' )
			rospy.logerr( '[lidar_image] ' + e )
			print( e )
			return
	def cameraCallback( self, data ):
		print( 'Received camera info' )
		self._cameraModel.fromCameraInfo( data )

	def velodyneCallback( self, data ):
		print( 'Received velodyne point cloud' )
		formatString = 'ffff'
		if data.is_bigendian:
			formatString = '>' + formatString
		else:
			formatString = '<' + formatString
		points = []
		for index in range( 0, len( data.data ), 32 ):
			points.append( struct.unpack( formatString, data.data[ index:index + 16 ] ) )
		print( len( points ) )
		self._velodyneData = points

if __name__ == '__main__':
	try:
		rospy.init_node( 'lidar_image' )
		lidarimage = LidarImage()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
