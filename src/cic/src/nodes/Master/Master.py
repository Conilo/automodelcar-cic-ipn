#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Int16, \
                         String

from geometry_msgs.msg import Point

from cic.msg import Intersection, \
                    Lane, \
                    Obstacles
from master_module import Master, \
                          Task, \
                          LANE_DRIVING, \
                          speed_saturation, \
                          steering_saturation
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
CROSSING_SPEED = -400
VEL_DECREASING_FACTOR = -15
STEERING_CHANGE_FACTOR = -2
MAX_DIST_TO_LINE = 1
MIN_DIST_TO_LINE = 10
DIST_TO_KEEP = 80.0
MAX_WAIT_TIME = 5
PASSING_ENABLED = False


master = None


def publish_policies():

    global master, speed_PWM, steering_PWM

    # Velocity saturation
    speed_PWM = speed_saturation( \
        speed_PWM, master.current_speed)
    steering_PWM = steering_saturation( \
        steering_PWM, master.current_steering)

    # Publish policies
    speed_pub.publish(speed_PWM)
    steering_pub.publish(master.current_steering)
    lights_pub.publish(master.lights)
    

def on_new_obstacle_msg(msg):

    global master

    # Saves received data
    master.number_obstacles = msg.detected_obstacles
    master.obstacles = msg.obstacle_vertices

def on_new_intersection_msg(msg):

    global master

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

    global master

    start_time = cv2.getTickCount()

    # Saves received data
    master.lane_steering = msg.steering_value
    master.lane_speed = msg.speed_value

    # Process received data
    #master.run()

    # Pusblish policies
    publish_policies()

    
    elapsed_time = \
        (cv2.getTickCount() - start_time)/cv2.getTickFrequency()

    # rospy.loginfo(" Elapsed time: %5f --------- " % elapsed_time)

    

def main():
    """
    All the node messages will be processed by 
    the master node to set the car's behavior.
    """
    global master, speed_PWM, steering_PWM

    rospy.init_node('Master')
    rospy.loginfo("Master node running...")

    # Get parameters from launch
    VEL_DECREASING_FACTOR = rospy.get_param("~vel_dec_factor")
    PASSING_ENABLED = rospy.get_param("~passing_enabled")

    speed_PWM = -200
    steering_PWM = PWM_STEERING_CENTER
    
    master = Master(PWM_STEERING_CENTER,
                CROSSING_SPEED,
                VEL_DECREASING_FACTOR,
                STEERING_CHANGE_FACTOR,
                MAX_DIST_TO_LINE,
                MIN_DIST_TO_LINE,
                DIST_TO_KEEP,
                MAX_WAIT_TIME,
                PASSING_ENABLED,
                Task(LANE_DRIVING))

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