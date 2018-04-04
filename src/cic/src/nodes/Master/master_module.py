import rospy
import time
import math as mt
import numpy as np
import os
from geometry_msgs.msg import Point


# Global constants
STOP_VELOCITY = 0
CURRENT = -1
NO_LINE = -1
REVERSE = -1
NO_OBSTACLE = -1.0

LANE_DRIVING = 0
INTER_APPROACHING = 1
WATTING = 2
FOLLOWING = 3
MOVING_LEFT = 4
PASSING = 5
MOVING_RIGHT = 6

# Task names definition
task_names = {
        LANE_DRIVING:'Lane driving',
        INTER_APPROACHING: 'Intersection approaching',
        WATTING: 'Watting',
        FOLLOWING: 'Following',
        MOVING_LEFT: 'Moving to left lane',
        PASSING: 'Passing obstacle',
        MOVING_RIGHT: 'Returning right lane'
        }

# Misc. functions definition
def following_speed(vel_decreasing_factor,
                    dist_to_keep,
                    dist_to_obstacle):

    distance_diff = \
        dist_to_obstacle - dist_to_keep

    return vel_decreasing_factor * distance_diff

def calculate_speed(vel_decreasing_factor,
                    dist_to_line):
    """
    Calculates the PWM speed value accordingly
    to the distace to the intersection line.
    """
    
    tmp = \
        vel_decreasing_factor * dist_to_line
    
    if tmp > -200:
        return -200
    else:
        return tmp


def calculate_steering(pwm_steering_center,
                       steering_change_factor,
                       line_angle):
    """
    Calculates the PWM steering accordingly
    to the line angle in the intersections.
    """
    
    calculated_steering = \
       int(steering_change_factor * line_angle) + 2
    
    return pwm_steering_center + calculated_steering

def speed_saturation(current_speed, calculated_speed):
    """
    Saturates the PWM speed smoothly.
    """
    if calculated_speed > current_speed:
        current_speed += 15
    elif calculated_speed < current_speed:
        current_speed -= 5

    return current_speed

def steering_saturation(curren_steering, calculated_steering):
    """
    Saturates the PWM speed smoothly.
    """
    if calculate_steering > curren_steering:
        curren_steering += 3
    elif calculate_steering < curren_steering:
        curren_steering -= 3

    return curren_steering


# Classes definition
class Task:
    """
    This class contains the task descriptors.
    """
    
    name = None
    ID = None

    def __init__(self, task_identifier):
        self.name = task_names[task_identifier]
        self.ID = task_identifier


class Master:
    """
    This class contains the master state variables
    that are taken into consideration for the state
    assigning and solving steps.
    """

    pwm_steering_center = None
    crossing_speed = None
    vel_decreasing_factor = None
    steering_change_factor = None
    max_dist_to_line = None
    min_dist_to_line = None
    dist_to_keep = None
    
    lane_speed = None
    lane_steering = None

    dist_to_line = None
    line_angle = None

    number_obstacles = None
    obstacles = None
    max_waiting_time = None
    
    task_pile = None
    count = None
    current_speed = None
    current_steering = None

    lights = None

    def __init__(self,
                 PWM_STEERING_CENTER,
                 CROSSING_SPEED,
                 VEL_DECREASING_FACTOR,
                 STEERING_CHANGE_FACTOR,
                 MAX_DIST_TO_LINE,
                 MIN_DIST_TO_LINE,
                 DIST_TO_KEEP,
                 MAX_WAIT_TIME,
                 task):
        
        self.pwm_steering_center = \
            PWM_STEERING_CENTER
        self.crossing_speed = CROSSING_SPEED
        self.vel_decreasing_factor = \
            VEL_DECREASING_FACTOR
        self.steering_change_factor = \
            STEERING_CHANGE_FACTOR
        self.max_dist_to_line = \
            MAX_DIST_TO_LINE
        self.min_dist_to_line = \
            MIN_DIST_TO_LINE
        self.lane_speed = 0
        self.lane_steering = \
            PWM_STEERING_CENTER
        self.dist_to_line = NO_LINE
        self.line_angle = 0
        self.number_obstacles = 0
        self.dist_to_keep = DIST_TO_KEEP
        self.obstacles = []
        self.max_waiting_time = MAX_WAIT_TIME
        self.task_pile = []
        self.count = 0
        self.current_speed = 0
        self.current_steering = \
            PWM_STEERING_CENTER
        self.add_task(task)

    def add_task(self, task):
        """
        Adds a new task to the pile.
        """
        self.task_pile.append(task)

    def remove_task(self, task_index):
        """
        Remove the last task from the pile.
        """
        if len(self.task_pile) > 1:
            self.task_pile.pop(task_index)
            
    def get_current_task(self):
        """
        Get the indicated task from the pile.
        """
        return self.task_pile[CURRENT]
    
    def task_assigner(self):
        """
        Verifies the current enviroment status and
        checks whether is necesary, or not, to add
        a new task to the pile.
        """
        
        # Get the current task   
        current_task = self.get_current_task()

        # First, checks if a close intersection exists.
        if self.dist_to_line > 0:
        
            # Checks current task to avoid intersection 
            # routine duplication
            if (current_task.ID != INTER_APPROACHING 
                and current_task.ID != WATTING):
            
                # Adds intersection routine
                self.add_task(Task(WATTING))
                self.add_task(Task(INTER_APPROACHING)) 
        
        # If not, check for obstacles
        elif self.dist_to_line == NO_LINE:

            
            # Checks the number of detected obstacles.
            if self.number_obstacles > 0:

                
                # Checks each obstacle info
                for obstacle in self.obstacles:
                    
                    # Obstacle in front detected while driving
                    if (obstacle.x > 330.0) or (obstacle.x < 30.0):

                        if current_task.ID == LANE_DRIVING:

                            # Adds following routine
                            self.add_task(Task(FOLLOWING))
                            break

                        if ((current_task.ID == FOLLOWING) 
                            and (self.count > self.max_waiting_time)):

                            rospy.loginfo(self.max_waiting_time)
                            # Resets count
                            self.count = 0

                            # Adds passing obstacle routine
                            self.add_task(Task(MOVING_RIGHT))
                            self.add_task(Task(PASSING))
                            self.add_task(Task(MOVING_LEFT))
                            break



                
    def task_solver(self):
        """
        Evaluates the current task to set the speed
        and steering policies accordingly. 
        """
        # Get the current task
        current_task = self.get_current_task()
        print('TS: Current task: ' + str(current_task.name))
        
        # Evaluates and tires to sove the current
        # task, setting the speed and steering 
        # policies accordingly.
        
        # Lane driving case
        if current_task.ID == LANE_DRIVING:
            
            # Sets speed and steering policies
            self.current_speed = self.lane_speed
            self.current_steering = self.lane_steering
            self.lights = 'diL'
        
        # Intersection approaching case
        elif current_task.ID == INTER_APPROACHING:
            
            # Checks if terminal conditions are met
            if (self.dist_to_line > 0  and  
                self.dist_to_line <= self.min_dist_to_line):
                
                # Sets speed and steeering policies
                self.current_speed = -200
                self.current_steering = \
                    calculate_steering(self.pwm_steering_center,
                                       self.steering_change_factor,
                                       self.line_angle)
                self.lights = 'fr'

                # Removes current task from pile
                self.remove_task(CURRENT)
                
            # if not, continue with the task policies 
            else:
                
                # Sets speed and steeering policies
                self.current_speed = \
                    calculate_speed(self.vel_decreasing_factor,
                                    self.dist_to_line)
                self.current_steering = \
                    calculate_steering(self.pwm_steering_center,
                                       self.steering_change_factor,
                                       self.line_angle)
                self.lights = 'stop'
        
        # Watting case
        elif current_task.ID == WATTING:

            # Intersection line approaching case
            if (self.dist_to_line > 0):

                if (self.dist_to_line < self.min_dist_to_line):

                    # Set policies
                    self.current_speed = -200
                    self.current_steering = \
                        calculate_steering(self.pwm_steering_center,
                                           self.steering_change_factor,
                                           self.line_angle)
                    self.lights = 'fr'
   
                else:
                    # Set policies
                    self.current_speed = self.crossing_speed
                    self.current_steering = \
                        calculate_steering(self.pwm_steering_center,
                                       self.steering_change_factor,
                                       self.line_angle)
                    self.lights = 'diL'

                    # Removes current task from pile
                    self.remove_task(CURRENT)

            # Intersection end case
            else:

                # Kill LaneDetection node to restart it
                os.system('rosnode kill LaneDetection') 

                # Set policies
                self.current_speed = -200
                self.current_steering = self.lane_steering
                self.lights = 'diL'

                # Removes current task from pile
                self.remove_task(CURRENT)
        
        elif current_task.ID == FOLLOWING:

            # If no obstacles detected
            if self.number_obstacles == 0:

                # Set policies
                self.current_speed = -150
                self.current_steering = self.lane_steering
                self.lights = 'diL'

                # Reset count

                # Removes current task from pile
                self.remove_task(CURRENT)


            # Checks distance to each obstacle
            else:
                
                for obstacle in self.obstacles:
                    
                    # No obstacle in front, finish task
                    if (obstacle.x > 30.0 and obstacle.x < 330.0):
                        
                        # Set policies
                        self.current_speed = -150
                        self.current_steering = self.lane_steering
                        self.lights = 'diL'

                        # Reset count
                        self.count = 0

                        # Removes current task from pile
                        self.remove_task(CURRENT)
                        break

                    # Following case
                    elif (obstacle.y > (self.dist_to_keep + 7)):

                        self.count = 0
                        # Set policies
                        self.current_speed = \
                            following_speed(self.vel_decreasing_factor,
                                            self.dist_to_keep,
                                            obstacle.y)
                        self.current_steering = self.lane_steering
                        self.lights = 'stop'
                        break

                    # Reverse case
                    elif (obstacle.y < (self.dist_to_keep - 7)):

                        self.count = 0
                        # Set policies
                        self.current_speed = \
                            following_speed(self.vel_decreasing_factor,
                                            self.dist_to_keep,
                                            obstacle.y)
                        self.current_steering = self.lane_steering
                        self.lights = 're'
                        break

                    # Waitting case
                    else:

                        # Counting
                        self.count += 1
                        time.sleep(0.5)
                        rospy.loginfo("Waiting, counting... %i" % self.count)

                        # Set policies
                        self.current_speed = 0
                        self.current_steering = self.lane_steering
                        self.lights = 'diL'
                        break
        
        elif current_task.ID == MOVING_LEFT:

            for obstacle in self.obstacles:
                    
                # Ostacle in front, move to left lane
                if (obstacle.x < 27.0) or (obstacle.x > 330.0):
                    
                    # Set policies
                    self.current_speed = -250
                    self.current_steering = 150
                    self.lights = 'le'
                    break
                
                # On left lane
                elif (obstacle.x > 27.0) and (obstacle.x < 120.0):

                    # Kill LaneDetection node to restart it
                    os.system('rosnode kill LaneDetection') 

                    # Set policies
                    self.current_speed = self.lane_speed
                    self.current_steering = self.lane_steering
                    self.lights = 'diL'

                    # Removes current task from pile
                    self.remove_task(CURRENT)
                    break
            
        elif current_task.ID == PASSING:

            for obstacle in self.obstacles:
                    
                # Ostacle passed, finish task
                #if (obstacle.x > 65.0) and (obstacle.x < 180.0):
                if (obstacle.x > 65.0) and (obstacle.x < 110.0):
                    # Set policies
                    self.current_speed = -250
                    self.current_steering = 50
                    self.lights = 'ri'

                    # Removes current task from pile
                    self.remove_task(CURRENT)
                    break
                
                # Drive left lane
                else:

                    # Set policies
                    self.current_speed = self.lane_speed
                    self.current_steering = self.lane_steering
                    self.lights = 'diL'

                    

        elif current_task.ID == MOVING_RIGHT:

            for obstacle in self.obstacles:
                    
                # Ostacle passed, finish task
                #if (((obstacle.x > 115.0) and (obstacle.x < 180.0))
                if (((obstacle.x > 110.0) and (obstacle.x < 180.0))
                    and obstacle.y > 50.0):

                    # Kill LaneDetection node to restart it
                    os.system('rosnode kill LaneDetection') 

                    # Set policies
                    self.current_speed = self.lane_speed
                    self.current_steering = self.lane_steering
                    self.lights = 'diL'

                    # Removes current task from pile
                    self.remove_task(CURRENT)
                    break
                
                # Return right lane
                elif (obstacle.x > 80) and (obstacle.x < 110):

                    # Set policies
                    self.current_speed = -300
                    self.current_steering = 35
                    self.lights = 'ri'




    def run(self):
        """
        Executes the task assigner and solver 
        to process the received data.
        """

        # Task assigner
        self.task_assigner()
        # Task solver
        self.task_solver()
