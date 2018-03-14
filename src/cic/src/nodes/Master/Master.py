#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Int16
from cic.msg import Intersection, \
                    Lane
from master_module import Master, \
                          Task, \
                          LANE_DRIVING
speed_pub = \
    rospy.Publisher(
        '/manual_control/speed',
        Int16, queue_size = 1)

steering_pub = \
    rospy.Publisher(
        '/manual_control/steering',
        Int16, queue_size = 1)

# Global parameters
PWM_STEERING_CENTER = 90
CROSSING_SPEED = -300
VEL_DECREASING_FACTOR = -10
STEERING_CHANGE_FACTOR = -2
MAX_DIST_TO_LINE = 70
MIN_DIST_TO_LINE = 10

# Initiates Master class object
master = Master(PWM_STEERING_CENTER,
                CROSSING_SPEED,
                VEL_DECREASING_FACTOR,
                STEERING_CHANGE_FACTOR,
                MAX_DIST_TO_LINE,
                MIN_DIST_TO_LINE,
                Task(LANE_DRIVING))

def on_new_intersection_msg(msg):

    start_time = cv2.getTickCount()
    # Saves received data
    master.dist_to_line = msg.distance
    master.line_angle = msg.angle

    # Process received data
    master.run()

    # Set speed and steering policies pusblishing
    speed_pub.publish(master.current_speed)
    steering_pub.publish(master.current_steering)

    end_time = cv2.getTickCount()
    total_time = end_time - start_time
    rospy.loginfo("Time elapsed: " + str(total_time))

def on_new_lane_msg(msg):

    start_time = cv2.getTickCount()
    # Saves received data
    master.lane_steering = msg.steering_value
    master.lane_speed = msg.speed_value

    # Process received data
    master.run()

    # Set speed and steering policies pusblishing
    speed_pub.publish(master.current_speed)
    steering_pub.publish(master.current_steering)

    end_time = cv2.getTickCount()
    total_time = \
        (end_time - start_time)/cv2.getTickFrequency()
    rospy.loginfo(" ----- Time elapsed: " + str(total_time))

def main():
    """
    All the node messages will be processed by 
    the master node to set the car's behavior.
    """

    rospy.init_node('Master')
    rospy.loginfo("Master node running...")

    rospy.Subscriber(
        '/crossing_detection', 
        Intersection, 
        on_new_intersection_msg)
    
    rospy.Subscriber(
        '/lane_detection', 
        Lane, 
        on_new_lane_msg)

    rospy.spin()

if __name__ == '__main__':
    main()