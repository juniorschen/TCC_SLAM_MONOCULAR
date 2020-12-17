#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import TransformBroadcaster
from rospy import Time 

global lastLeftMsg
global lastRightMsg
global lastCenterMsg
lastLeftMsg = 0.0
lastRightMsg = 0.0
lastCenterMsg = 0.0
odometryArray = 0.0

global checkingPosition
checkingPosition = 0
global isFirst
isFirst = True

pubServos = rospy.Publisher('slam_manager_servos', Int32, queue_size=1)
pubWheels = rospy.Publisher('slam_manager_wheels', Int32, queue_size=1)

def angleServos(anglePos):
	pubServos.publish(anglePos)

def checkArea():
	global checkingPosition
	angleServos(checkingPosition)
	checkingPosition = checkingPosition + 1

def moveRobot(pos):
  	pubWheels.publish(pos)

def move_robot_model_x():
	translation = (0.2, 0.0, 0.0)
	rotation = (0.0, 0.0, 0.0, 1.0)
	b = TransformBroadcaster()
	b.sendTransform(translation, rotation, Time.now(), 'base_link', '/map')
	b.sendTransform(translation, rotation, Time.now(), 'camera_link', '/map')
	b.sendTransform(translation, rotation, Time.now(), 'camera_link_normalized', '/map')

def move_robot_model_rotate():
	translation = (0.0, 0.0, 0.0)
	rotation = (0.0, 3.0, 0.0, 1.0)
	b = TransformBroadcaster()
	b.sendTransform(translation, rotation, Time.now(), 'base_link', '/map')
	b.sendTransform(translation, rotation, Time.now(), 'camera_link', '/map')
	b.sendTransform(translation, rotation, Time.now(), 'camera_link_normalized', '/map')

def moveDecision():
	global lastLeftMsg
	global lastRightMsg
	global lastCenterMsg
	if lastCenterMsg > max(lastLeftMsg,lastRightMsg):
		print('Indo para Frente')
		moveRobot(2)
	elif lastLeftMsg > max(lastCenterMsg,lastRightMsg):
		print('Girando para Esquerda')
		moveRobot(4)
		rospy.sleep(1.)
		print('Indo para Frente')
		moveRobot(2)
	elif lastRightMsg > max(lastCenterMsg,lastLeftMsg):
		print('Girando para Direita')
		moveRobot(5)
		rospy.sleep(1.)
		print('Indo para Frente')
		moveRobot(2)
	else:
		print('Indo para Frente Else',lastCenterMsg, lastLeftMsg, lastRightMsg)
		moveRobot(2)
	rospy.sleep(1.)
	pubWheels.publish(1)

def assignMessage(msg):
	global lastLeftMsg
	global lastRightMsg
	global lastCenterMsg
	if checkingPosition == 0:
		lastCenterMsg = msg.ranges[180]
	elif checkingPosition == 1:
		lastRightMsg = msg.ranges[180]
	elif checkingPosition == 2:
		lastLeftMsg = msg.ranges[180]
	print('LastCenterMsg', lastCenterMsg)
	print('LastRightMsg', lastRightMsg)
	print('LastLeftMsg', lastLeftMsg)
	

def slam_manager(msg):
	global isFirst
	if isFirst:
		print('Angulando robo para posicao inicial')
		angleServos(2)
		isFirst = False
	else:
		global checkingPosition
		print('Laser Center', msg.ranges[180])
		assignMessage(msg)
		if checkingPosition != 3:
			checkArea()
		else:
			moveDecision()
			checkingPosition = 0
			angleServos(2)
	print('-----------------------------------------------')
	rospy.sleep(5.)

if __name__== '__main__':
	print('starting slam manager')
	print('-----------------------------------------------')
	rospy.init_node('slam_manager')
	sub = rospy.Subscriber('/scan', LaserScan, slam_manager)
	rospy.spin()
