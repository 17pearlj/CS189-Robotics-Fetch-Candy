"""
return Twist objects that are used to direct the robot
"""
import rospy
import math
from math import radians, degrees
import cool_math as cm
# uncomment later!
from geometry_msgs.msg import Twist

# constant for speed 
LIN_SPEED = 0.2/2 # 0.2 m/s
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
        self.handle_AR_turn = True


    def wander(self):
        self.move_cmd.linear.x = LIN_SPEED
        self.move_cmd.angular.z = 0
        return self.move_cmd
    
    def bumped(self):
        self.move_cmd.linear.x =  -LIN_SPEED
        self.move_cmd.angular.z = 0
        return self.move_cmd
    
    def avoid_obstacle(self):
        self.move_cmd.linear.x = -LIN_SPEED *0.75
        self.move_cmd.angular.z = radians(45)
        return self.move_cmd

    # --------- ARTags ------------------#
    def choose_AR(self, my_dict):
        """
        Get the key of the closest ARTag from a dictionary ARTags that have structure (self.AR_q):
        (key, [(pos.x,.y.z)at the time seen, robot's orientation at time seen, 'unvisited'])
        """
        unvisited = []
        for item in my_dict.items():
            if (item[1][2] == 'unvisited'):
                unvisited.append(item)
        

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

        # lets us know when we have reached the AR
        AR_close = False
        obs_off = False

        # get the orientation of robot at the time ar tag was seen 
        curr_tag = my_dict.get(my_key)
        tag_orr = curr_tag[1]
        tag_pos = (curr_tag[0].x, curr_tag[0].y)
        # print("tag_orr %.2f" % tag_orr) # this should stay the same


        # get the difference between this orientation and my current orientation 
        angle_diff = cm.angle_compare(tag_orr, my_orr)
        # determine turn angle with proportional control
        prop_angle = abs(angle_diff) * ROT_K
        # choose angle that requires minimal turning 
        turn_angle = cm.sign(angle_diff) * min(prop_angle, ROT_SPEED)
        # print("angle_diff %.2f" % angle_diff) # this should change 
        # print("turn angle %.2f" % turn_angle)


        # change turn angle to approach the ARTag
        self.move_cmd.angular.z = turn_angle

        # don't want the robot to move while it is orienting to ARTag
        self.move_cmd.linear.x = 0

        if (abs(angle_diff) < 0.02):
            print "robot orientation %.2f and angle to ar_tag %.2f are the same" % (my_orr, tag_orr)
            self.move_cmd.angular.z = 0

            # proportional control to approach the ARTag based on current distance
            curr_dist = cm.dist((curr_tag[0].x, curr_tag[0].y, curr_tag[0].z))
            self.move_cmd.linear.x = min(LIN_SPEED, curr_dist * LIN_K)
            print("current distance from ar_tag %.2f" % curr_dist)

            # turn off obstacles when robot is close enough 
            if (curr_dist < 1):
                obs_off = True

                if (curr_dist < 0.4):
                        # Consider destination reached if within 5 cm
                        self.move_cmd.linear.x = 0
                        self.move_cmd.angular.z = 0
                        obs_off = True
                        AR_close = True


        return self.move_cmd, AR_close, obs_off
    
    def handle_AR(self,my_dict, my_key, step):
        print "step %d" % step
        curr_tag = my_dict.get(my_key)
        tag_orr = curr_tag[1]
        
        if (step == 1):
            self.move_cmd.linear.x = 0
            if (tag_orr > -.25):
                self.move_cmd.angular.z = math.radians(-90)
            elif (tag_orr < .25):
                self.move_cmd.angular.z = math.radians(90)
            else:
                self.handle_AR_turn = False
                self.move_cmd.angular.z = 0
        if (step == 2):
            self.move_cmd.linear.x = .07
        if (step == 3):
            if(not(self.handle_AR_turn)):
                self.move_cmd.linear.x = 0
            else:
                self.move_cmd.linear.x = -LIN_SPEED*2 
            rospy.sleep(5)
            self.move_cmd.angular.z = 0 
        if (step == 4):
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 180        
        # set the ARTag that has been visited to indicate this 
            curr_tag = my_dict.get(my_key)
            curr_tag[2] = 'visited'

        return self.move_cmd
