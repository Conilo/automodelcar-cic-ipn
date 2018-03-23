import rospy
import time
import math as mt
import numpy as np
import os


# Global constants
STOP_VELOCITY = 0
CURRENT = -1
NO_LINE = -1

LANE_DRIVING = 0
INTER_APPROACHING = 1
WATTING = 2
CROSSING = 3

# Task names definition
task_names = {
        LANE_DRIVING:'Lane_driving',
        INTER_APPROACHING: 'Intersection_approaching',
        WATTING: 'Watting',
        CROSSING: 'Crossing'
        }

# Misc. functions definition
def calculate_speed(vel_decreasing_factor,
                    dist_to_line):
    """
    Calculates the PWM speed value accordingly
    to the distace to the intersection line.
    """
    
    return vel_decreasing_factor * dist_to_line

def calculate_steering(pwm_steering_center,
                       steering_change_factor,
                       line_angle):
    """
    Calculates the PWM steering accordingly
    to the line angle in the intersections.
    """
    
    calculated_steering = \
        steering_change_factor * line_angle
    
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
    
    lane_speed = None
    lane_steering = None

    dist_to_line = None
    line_angle = None
    
    task_pile = None
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
        self.task_pile = []
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
                self.current_speed = -100
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
                self.current_steering = self.lane_steering
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
                self.current_speed = -50
                self.current_steering = self.lane_steering
                self.lights = 'diL'

                # Removes current task from pile
                self.remove_task(CURRENT)


    def run(self):
        """
        Executes the task assigner and solver 
        to process the received data.
        """

        # Task assigner
        self.task_assigner()
        # Task solver
        self.task_solver()
