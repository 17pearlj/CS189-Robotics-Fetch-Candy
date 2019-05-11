"""
Helper functions that return Twist objects that are used to direct the robot
"""
import math
from math import radians, degrees
import cool_math as cm
from geometry_msgs.msg import Twist

# constants for movement
LIN_SPEED = 0.1 # m/s
ROT_SPEED = math.radians(15)  # 45 deg/s in radians/s
ROT_SPEED_2 = math.radians(45)  # 45 deg/s in radians/s
ROT_K = 2.5  # Constant for proportional angular velocity control
LIN_K = 0.5  # Constant for proportional linear velocity control

    

class MoveMaker:
    def __init__(self):
        # movement command that will be sent to robot 
        self.move_cmd = Twist()
        self.position = [0,0]
        self.orientation = 0
        self.AR_close = False
        self.handle_AR_step = 0

    #--------Simple Moves ------#
    def back_out(self):
        """
        - Used for backing out from the dispenser after parking
        :param: None
        :return: Twist Object
        """
        self.move_cmd.linear.x = -LIN_SPEED
        self.move_cmd.angular.z = 0
        return self.move_cmd

    def wait(self):
        """
        - Make the robot wait by stopping movement 
        :param: None
        :return: Twist Object
        """
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        return self.move_cmd

    def go_forward(self):
        """
        - Make the robot move forward 
        :param: None
        :return: Twist Object
        """
        self.move_cmd.linear.x = LIN_SPEED
        self.move_cmd.angular.z = 0
        return self.move_cmd

    def go_forward_K(self, dist):
        """
        - Make the robot move forward using constant of proportionality 
        :param: distance left to travel 
        :return: Twist Object
        """
        self.move_cmd.linear.x = LIN_K * dist
        self.move_cmd.angular.z = 0
        return self.move_cmd

    def twist(self, my_vel):
        """
        - Make the robot twist using a pre-computed velocity
        :param: angular velocity 
        :return: Twist Object
        """
        self.move_cmd.angular.z = my_vel
        self.move_cmd.linear.x = 0
        return self.move_cmd
    
    def bumped(self):
        """
        - Robot should back up when bumped 
        :param: None
        :return: Twist Object
        """
        self.move_cmd.linear.x =  -LIN_SPEED
        self.move_cmd.angular.z = 0
        return self.move_cmd
    
    def avoid_obstacle(self, side):
        """
        - Make the robot turn slightly when it sees an obstacle 
        :param: side that the obstacle is on 
        :return: Twist Object
        """
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = side*ROT_SPEED_2
        return self.move_cmd


    # --------- Not So Simple Moves -------------
    def go_to_pos(self, str, my_pos, my_orr):
        """
        - Go to position from current location until CORRECT AR tag seen
        - Calls go_to_AR while correct AR is not in sight
        :param: movement to make, current position, current orientation
        :return: Twist Object
        """
        self.position = my_pos
        self.orientation = my_orr
        if (str == 'forward'):
            self.move_cmd.angular.z = 0
            self.move_cmd.linear.x = LIN_SPEED * 2
        elif (str == 'left'):
            self.move_cmd.angular.z = ROT_SPEED_2
            self.move_cmd.linear.x = 0
        elif (str == 'right'):
            self.move_cmd.angular.z = -ROT_SPEED_2
            self.move_cmd.linear.x = 0
        return self.move_cmd
    
    
   
