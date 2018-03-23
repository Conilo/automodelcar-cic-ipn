#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Int16, \
                         String
from cic.msg import Intersection, \
                    Lane, \
                    Obstacles
from master_module import Master, \
                          Task, \
                          LANE_DRIVING, \
                          speed_saturation
speed_pub = \
    rospy.Publisher(
        '/manual_control/speed',
        Int16, queue_size = 1)

steering_pub = \
    rospy.Publisher(
        '/manual_control/steering',
        Int16, queue_size = 1)

lights_pub = \
    rospy.Publisher(
        '/manual_control/lights',
        String, queue_size= 1)

# Global parameters
PWM_STEERING_CENTER = 90
CROSSING_SPEED = -350
VEL_DECREASING_FACTOR = -10
STEERING_CHANGE_FACTOR = -2
MAX_DIST_TO_LINE = 100
MIN_DIST_TO_LINE = 10

# Initiates Master class object
master = Master(PWM_STEERING_CENTER,
                CROSSING_SPEED,
                VEL_DECREASING_FACTOR,
                STEERING_CHANGE_FACTOR,
                MAX_DIST_TO_LINE,
                MIN_DIST_TO_LINE,
                Task(LANE_DRIVING))

def publish_policies():

    global speed_PWM

    # Velocity saturation
    speed_PWM = speed_saturation( \
        speed_PWM, master.current_speed)

    # Publish policies
    speed_pub.publish(speed_PWM)
    steering_pub.publish(master.current_steering)
    lights_pub.publish(master.lights)

def on_new_obstacle_msg(msg):

    detected_obstacles = msg.detected_obstacles
    rospy.loginfo("Number of detected obstacles: %i" %detected_obstacles)

def on_new_intersection_msg(msg):

    start_time = cv2.getTickCount()

    # Saves received data
    master.dist_to_line = msg.distance
    master.line_angle = msg.angle

    #rospy.loginfo('Distance to line: '+ str(msg.distance))

    # Process received data
    master.run()

    # Pusblish policies
    publish_policies()

    elapsed_time = \
        (cv2.getTickCount() - start_time)/cv2.getTickFrequency()

    rospy.loginfo(" Elapsed time: %5f --------- " % elapsed_time)

def on_new_lane_msg(msg):

    start_time = cv2.getTickCount()

    # Saves received data
    master.lane_steering = msg.steering_value
    master.lane_speed = msg.speed_value

    # Process received data
    master.run()

    # Pusblish policies
    publish_policies()

    
    elapsed_time = \
        (cv2.getTickCount() - start_time)/cv2.getTickFrequency()

    rospy.loginfo(" Elapsed time: %5f --------- " % elapsed_time)

    

def main():
    """
    All the node messages will be processed by 
    the master node to set the car's behavior.
    """

    rospy.init_node('Master')
    rospy.loginfo("Master node running...")

    global speed_PWM
    speed_PWM = -100

    rospy.Subscriber(
        '/crossing_detection', 
        Intersection, 
        on_new_intersection_msg)
    
    rospy.Subscriber(
        '/lane_detection', 
        Lane, 
        on_new_lane_msg)

    rospy.Subscriber(
        '/obstacle_detection', 
        Obstacles, 
        on_new_obstacle_msg)

    rospy.spin()

if __name__ == '__main__':
    main()