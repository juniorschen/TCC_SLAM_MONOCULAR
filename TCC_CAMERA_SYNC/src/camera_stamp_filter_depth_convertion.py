#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

image_raw_pub = rospy.Publisher('image_raw', Image, queue_size=1)
camera_info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=1)

global i
i = 0
bridge = CvBridge()

def callback(image_raw, camera_info):
        global i
        i += 1
        print('got an sync Image and CameraInfo', i)

	stamp = rospy.Time.now()
        
	img_mono8 = bridge.imgmsg_to_cv2(image_raw, desired_encoding="mono8")
	img_mono16 = np.uint16(img_mono8) * 256

	image_raw = bridge.cv2_to_imgmsg(img_mono16, encoding='mono16')
	image_raw.height = 240
        image_raw.width = 320
        image_raw.encoding = 'mono16'
        image_raw.header.frame_id = 'camera_link'
        image_raw.header.stamp = stamp
	image_raw_pub.publish(image_raw)

	camera_info.header.stamp = stamp
	camera_info.header.frame_id = 'camera_link'
	camera_info.height = 240
	camera_info.width = 320
	camera_info_pub.publish(camera_info)

rospy.init_node('SyncFilterDate')
image_sub = message_filters.Subscriber('/TCC_cam/image_raw', Image)
info_sub = message_filters.Subscriber('/TCC_cam/camera_info', CameraInfo)
ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
ts.registerCallback(callback)
rospy.spin()


