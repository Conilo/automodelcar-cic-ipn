#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.
# it requires "chmod +x mypythonscript.py" to be called by ROS

import roslaunch
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray


pub_laser = rospy.Publisher('/scan_follower/scan', LaserScan, queue_size = 1)
pub_obj90 = rospy.Publisher('/scan_follower/obj_90', Float32, queue_size = 1)
pub_obj180 = rospy.Publisher('/scan_follower/obj_180', Float32, queue_size = 1)
pub_obj270 = rospy.Publisher('/scan_follower/obj_270', Float32, queue_size = 1)
pub_obj360 = rospy.Publisher('/scan_follower/obj_360', Float32, queue_size = 1)

pub_objAng90 = rospy.Publisher('/scan_follower/obj_angle90', Int16, queue_size = 1)
pub_objAng180 = rospy.Publisher('/scan_follower/obj_angle180', Int16, queue_size = 1)
pub_yawF = rospy.Publisher('/scan_follower/yaw2', Float32, queue_size = 1)
pub_max180 = rospy.Publisher('/scan_follower/max180', Int16, queue_size = 1)

MAX_DISTANCE = .6
AMP_ANG_min = 0
AMP_ANG_max = 359

def myShut():
	print 'Fin del Programa'


def yaw_callback(yaw):
		
	#Almacenamiento de la informacion del topico del yaw
    	yaw2 = yaw
	pub_yawF.publish(yaw2)
       

def laser_callback(scan):
	
	global intCent
	#Almacenamiento de la informacion del topico del escaner
    	newscan = scan
	#Arreglos de rango e intensidad para cada punto
    	ranges = [float('Inf')]*360
    	intensities = [0.0]*360
    	RoI = range(AMP_ANG_min ,AMP_ANG_max)

    	
    	# To see the RoI

	obj90 = []
	obj180=[]
	obj270=[]
	obj360=[]
	senos=[]

   	for i in RoI:
       		if scan.ranges[i] < MAX_DISTANCE:
			ranges[i] = scan.ranges[i]
          		intensities[i] = scan.intensities[i]
 			if i<=90:
				obj90.append ([scan.ranges[i],i])
			elif i<=180 and i>90:
				obj180.append ([scan.ranges[i],i])
			elif i<=270 and i>180:
				obj270.append ([scan.ranges[i],i])
			else:
				obj360.append ([scan.ranges[i],i])
	if obj90==[]:
			obj90=[[1.0,90]]
	if obj180==[]:
			obj180=[[1.0,90]]
	if obj270==[]:
			obj270=[[1.0,90]]
	if obj360==[]:
			obj360=[[1.0,90]]


	dist1=min(obj90)[0]
	dist2=min(obj180)[0]
	dist3=min(obj270)[0]
	dist4=min(obj360)[0]
	#obj=[dist1,dist2]
	ang1=min(obj90)[1]
	ang2=min(obj180)[1]
	max_180=obj180[-1][1]
	x=dist2*math.sin(ang2*3.1416/180.0)
	print x
	#print max(obj180)[0]
	#print(objCent, len(objCent))	
	pub_obj90.publish(dist1)
	pub_obj180.publish(dist2)
	pub_obj270.publish(dist3)
	pub_obj360.publish(dist4)
	pub_objAng90.publish(ang1)
	pub_objAng180.publish(ang2)
	pub_max180.publish(max_180)
					
    	newscan.ranges = ranges
    	newscan.intensities = intensities
    	pub_laser.publish(newscan)
	obj90 = []
	obj180=[]
	obj270=[]

       
        
def main():
    global intCent
    intCent = 0.0

    rospy.init_node('laser_follower')
    #set_zero()
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.Subscriber("/model_car/yaw", Float32, yaw_callback)
    rospy.on_shutdown(myShut)
    rospy.spin()

if __name__ == '__main__':
    main()
