"""
return Twist objects that are used to direct the robot
"""
import math
from geometry_msgs.msg import Twist

# constant for speed 
LIN_SPEED = 0.2  # 0.2 m/s
ROT_SPEED = math.radians(45)  # 45 deg/s in radians/s
ROT_K = 5  # Constant for proportional angular velocity control
LIN_K = 0.5  # Constant for proportional linear velocity control

class MoveMaker:
    def __init__(self):
        # movement command that will be sent to robot 
        self.move_cmd = Twist()
    def wander(self):
        self.move_cmd.linear.x = LIN_SPEED
        self.move_cmd.angular.z = 0
        return self.move_cmd
    
    def bumped(self):
        print "bumped"
        self.move_cmd.linear.x =  - LIN_SPEED
        self.move_cmd.angular.z = 0
        return self.move_cmd
    
    def avoid_obstacle(self):
        print "avoid obstacle"
        self.move_cmd.linear.x = -LIN_SPEED
        self.move_cmd.angular.z = 0
        return self.move_cmd

    # --------- ARTags ------------------#
    def choose_AR(self):
        print "choose AR"
        return 0
    
    def go_to_AR(self):
        print "go to AR"
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        return self.move_cmd
    
    def handle_AR(self):
        print "handle AR"
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        return self.move_cmd
