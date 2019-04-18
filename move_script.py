"""
return Twist objects that are used to direct the robot
"""
import math

import cool_math as cm
# uncomment later!
from geometry_msgs.msg import Twist

# constant for speed 
LIN_SPEED = 0.2/2  # 0.2 m/s
ROT_SPEED = math.radians(45)  # 45 deg/s in radians/s
ROT_K = 5  # Constant for proportional angular velocity control
LIN_K = 0.5  # Constant for proportional linear velocity control

    

class MoveMaker:
    def __init__(self):
        # movement command that will be sent to robot 
        self.move_cmd = Twist()
        self.position = [0,0]
        self.orientation = 0
        self.AR_close = False

    def wander(self):
        self.move_cmd.linear.x = LIN_SPEED
        self.move_cmd.angular.z = 0
        return self.move_cmd
    
    def bumped(self):
        self.move_cmd.linear.x =  - LIN_SPEED
        self.move_cmd.angular.z = 0
        return self.move_cmd
    
    def avoid_obstacle(self):
        #self.move_cmd.linear.x = -LIN_SPEED
        self.move_cmd.angular.z = radians(30)
        return self.move_cmd

    # --------- ARTags ------------------#
    def choose_AR(self, my_dict):
        """
        Get the key of the closest ARTag from a dictionary ARTags that have structure (self.AR_q):
        (key, [(pos.x,.y.z)at the time seen, robot's orientation at the time seen, 'unvisited'])
        """
        unvisited = []
        for item in my_dict.items():
            if (item[1][2] == 'unvisited'):
                unvisited.append(item)
        # for checking 
        print unvisited 

        # new list of all the distances in the list of ARTags that have not been visited 
        close = []
        for i in unvisited:
            pos = (i[1][0].x , i[1][0].x)
            curr_dist = cm.dist_btwn(pos, self.position)
            # list of dictionary items, and their distances 
            close.append([i, curr_dist])

        # actually reorder from smallest to largest distance 
        close.sort(reverse = True)
        
        # close is sorted smallest to large -> key into the dictionary
        the_key = close[0][0][0]

        return the_key
    
    def go_to_AR(self, my_dict, my_key, my_orr):
        """
        Go to the AR_tag
        look at x and z 
        """

        # get the orientation of robot at the time ar tag was seen 
        curr_tag = my_dict.get(my_key)
        tag_orr = curr_tag[1]
        tag_pos = (curr_tag[0].x, curr_tag[0].z)

        # get the difference between this orientation and my current orientation 
        angle_diff = cm.angle_compare(tag_orr, my_orr)
        # determine turn angle with proportional control
        prop_angle = abs(angle_diff) * ROT_K
        # choose angle that requires minimal turning 
        turn_angle = cm.sign(angle_diff) * min(prop_angle, ROT_SPEED)
        print 'turn angle:'
        print turn_angle
        # print "sel orr in move-script"
        # print math.degrees(my_orr)
        

        # change turn angle to approach the ARTag
        self.move_cmd.angular.z = turn_angle

        # proportional control to approach the ARTag based on current distance
        curr_dist = cm.dist_btwn(self.position, tag_pos)
        self.move_cmd.linear.x = min(LIN_SPEED, curr_dist * LIN_K)
        if curr_dist < 0.05:
                # Consider destination reached if within 5 cm
                print "WE OUT HERE"
                self.AR_close= True
                self.move_cmd.angular.z = 0

        return self.move_cmd
    
    def handle_AR(self,my_dict, my_key):
        print "handle AR"
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = math.radians(180)

        # set the ARTag that has been visited to indicate this 
        curr_tag = my_dict.get(my_key)
        curr_tag[2] = 'visited'

        return self.move_cmd
