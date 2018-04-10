#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.
# it requires "chmod +x mypythonscript.py" to be called by ROS

import math
import rospy
import time
import cv2
import roslaunch
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray

def myShut():
	print 'Fin del Programa'

def corr(x):
	return int(x*40)+123

def obj90_callback(ros_data):
	global dist90
	dist90 = ros_data.data

def obj180_callback(ros_data):
	global dist180
	dist180 = ros_data.data
def obj270_callback(ros_data):
	global dist270
	dist270 = ros_data.data
def obj360_callback(ros_data):
	global dist360
	dist360 = ros_data.data
def obj_angle90_callback(ros_data):
	global an90
	an90 = ros_data.data

def obj_angle180_callback(ros_data):
	global an180
	an180 = ros_data.data

def max180_callback(ros_data):
	global max_180
	max_180 = ros_data.data


def yawF_callback(ros_data):
	global yaw
	global yaws
	global yaw_inic
	yaw=ros_data.data
	if yaw_inic<40:
		yaws = yaw
	elif  yaw<0:
		yaws = yaw+360
	else:
		yaws = yaw

def routine(espacio):
	global yaw_inic
	global yaw_rel
	global dist360
	global fin

	
	yaw_inic=yaws
	yaw_rel=yaw_inic-30

	while yaws>=yaw_rel:
		angDir=10
		vel=400
		pubVel.publish(vel)
		pubDir.publish(angDir)

	print('Hola 1')
	angDir=100
	pubDir.publish(angDir)
	while an90>63:
		angDir=100
		vel=300
		pubVel.publish(vel)
		pubDir.publish(angDir)

	angDir=170
	pubDir.publish(angDir)
	print('Hola 2')

	while dist270>.33:
		angDir=170
		vel=120
		pubVel.publish(vel)
		pubDir.publish(angDir)
		if yaws>yaw_inic:
			print('YA1')
			break

	while yaws<(yaw_inic-3):
		angDir=30
		vel=-150
		pubVel.publish(vel)
		pubDir.publish(angDir)
		if dist360<0.25:
			print('YA')
			break
	'''if yaws<yaw_inic-5:
		while yaws<yaw_inic-5:
			angDir=170
			vel=70
			pubVel.publish(vel)
			pubDir.publish(angDir)
			if dist270<0.32:
				print('YA')
				break'''
	promedio=(espacio)/2.0
	print(promedio)
	print(dist270)
	while(promedio-dist270)>0.04:
		angDir=96
		vel=-140
		pubVel.publish(vel)
		pubDir.publish(angDir)



def scan_callback(scan):
	global espacioFlag
    	global pubVel
    	global pubDir
	global dist90
	global dist180
	global an90
	global an180
	global fin
	global vel
	global angDir
	global max_180
	global espacio
	global launch
	global launch_lane
	global objBandera
	#print(espacioFlag)

	if fin==False:
		if espacioFlag==0:
			if an180>=95:
				l1=-dist180*math.cos(an180*3.1416/180.0)
				l2=dist90*math.cos(an90*3.1416/180.0)
				espacio=l1+l2
				if espacio>0.70:
					espacioFlag=1
					os.system("rosnode kill LaneDetection")
					pubVel.publish(-200)
				print(espacio)
		else:	

			if an180<100:
				objBandera=True
			x=dist180*math.sin(an180*3.1416/180.0)
			limite=corr(x)
			if objBandera==True:
				if (max_180>=limite):
					vel=0
					pubVel.publish(vel)
					time.sleep(1)
					fin=True
					routine(espacio)
	else:
		vel=0
		angDir=100
		pubVel.publish(vel)
		#launch.shutdown()
		pubDir.publish(angDir)


       
def main():
    global intCent
    global pubVel
    global pubDir
    global espacioFlag
    global fin
    global vel
    global pub_stopFlag
    global yaw_inic
    global yaw_rel
    global dist90
    global dist180
    global an90
    global an180
    global max_180
    global launch
    global launch_lane
    global objBandera

    y_inic=0
    intCent = 0.0
    vel = 0
    espacioFlag=0
    fin=False
    yaw_inic=360
    objBandera=False

    rospy.init_node('master_laser')
    #set_zero()

    rospy.on_shutdown(myShut)

 

    rospy.Subscriber('/scan_follower/scan', LaserScan, scan_callback, queue_size = 1)
    rospy.Subscriber('/scan_follower/obj_90', Float32, obj90_callback, queue_size = 1)
    rospy.Subscriber('/scan_follower/obj_180', Float32, obj180_callback, queue_size = 1)
    rospy.Subscriber('/scan_follower/obj_270', Float32, obj270_callback, queue_size = 1)
    rospy.Subscriber('/scan_follower/obj_360', Float32, obj360_callback, queue_size = 1)
    rospy.Subscriber('/scan_follower/obj_angle90', Int16, obj_angle90_callback, queue_size = 1)
    rospy.Subscriber('/scan_follower/obj_angle180', Int16, obj_angle180_callback, queue_size = 1)
    rospy.Subscriber('/scan_follower/max180', Int16, max180_callback, queue_size = 1)
    rospy.Subscriber('/scan_follower/yaw2', Float32, yawF_callback, queue_size = 1)

    pubDir = rospy.Publisher('/manual_control/steering', Int16, queue_size=1)
    pubVel = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)


    rospy.spin()

if __name__ == '__main__':
    main()
