"""
Slow integration of bumps
"""

import random
import math
from math import radians, degrees
import cv2
import numpy as np
import sys

#imports for rospy
import rospy
from geometry_msgs.msg import Twist
import tf
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent, Sound
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers

#imports for other functions
import map_script
import move_script
import cool_math as cm 

from itertools import groupby

# valid ids for AR Tags
VALID_IDS = range(18)
Home = 1

#states in 1st



# states in park(); ie state2
SEARCHING = 0
ZERO_X = 1
TURN_ALPHA = 2
MOVE_ALPHA = 3
MOVE_PERF  = 4
SLEEPING = 5
BACK_OUT = 6
DONE_PARKING = 7
SEARCHING_2 = -1

class Main2:
    def __init__(self):
        # information about the robot's current position and orientation relative to start
        self.position = [0, 0]
        self.orientation = radians(0) # CCW +, radians 
        
        # mapping object will come from imported module 
        self.mapper = map_script.MapMaker()
        self.mapper.position = self.position
        self.mapper.orientation = self.orientation
        # move commands come from imported module 
        self.mover = move_script.MoveMaker()
        self.markers = {}


        # states: wait, go_to_pos, go_to_AR, handle_AR
        self.state = 'wait'
        self.prev_state = 'wait'
        # used in park() to decrease confusion 
        self.state2 = None 

        # depth image for getting obstacles
        self.depth_image = []

        # ---- ARTag stuff ----
        # key of AR_TAG we are seeking
        self.AR_curr = -1
        # dictionary for ar ids and coordinates
        self.AR_ids = {
            1: (0, 17), 
            11: (15, 18),
            2: (4, 24),
            3: (45, 24),
            4: (31, 10), 
            51: (35, 18), #fake location to get around table
            5: (23, 10),
            61: (35, 18), #fake location to get around table
            6: (23, 8),
            7: (9, 5)
        } 

        # vector orientation of ARTag relative to robot 
        # (usually an obtuse angle)
        self.ar_orientation = 0 # radians 
        # length of 'arm' between robot and ARTag 
        # when 0, robot is pointing at ARTag directly
        self.ar_x = 0 # m
        # distance between robot and ARTag 
        self.ar_z = 0 # m

        self.close = False # if there's an obstacle and we are really close to the ar_tag, it's probably another robot
        self.close_VERY = False # if we are extremely close to the ar_tag, we are just going to park or get bumped
        self.AR_seen = False
            
        # # ---- rospy stuff ----
        # Initialize the node
        rospy.init_node('Main2', anonymous=False)

        # Tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)

        # Create a publisher which can "talk" to TurtleBot wheels and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi',Twist, queue_size=10)

        # Subscribe to robot_pose_ekf for odometry/position information
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.process_ekf)

        # Set up the odometry reset publisher (publishing Empty messages here will reset odom)
        reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=1)
        # Reset odometry (these messages take about a second to get through)
        self.state_change_time = rospy.Time.now()
        timer = rospy.Time.now()
        while rospy.Time.now() - timer < rospy.Duration(1) or self.position is None:
            reset_odom.publish(Empty())

        

        # Use a CvBridge to convert ROS image type to CV Image (Mat)
        self.bridge = CvBridge()
        # Subscribe to depth topic
        rospy.Subscriber('/camera/depth/image', Image, self.process_depth_image, queue_size=1, buff_size=2 ** 24)

        # Subscribe to queues for receiving sensory data, primarily for bumps 
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.process_bump_sensing)
        self.sounds = rospy.Publisher('mobile_base/commands/sound', Sound, queue_size=10)

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 5 Hz
        self.rate = rospy.Rate(5)



        # Subscribe to topic for AR tags
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.process_ar_tags)

   
    
    
    def run(self):
        """
        - Control the state that the robot is currently in 
        - Run until Ctrl+C pressed
        :return: None
        """
        self.AR_curr = int(sys.argv[1])
        if (self.AR_curr == 5 or self.AR_curr == 6):
            self.AR_curr = self.AR_curr*10 + 1
        while not rospy.is_shutdown():
            move_cmd = Twist()
            if (self.state is "bumped" or self.state is "avoid_obstacle"):
                self.sounds.publish(Sound.ON)
                sec = 0
                if (self.state == "bumped" and not self.close_VERY):
                    print "bump when not very close to ar_tag"
                    sec = 5

                elif (self.close_VERY):
                    print "obstacle when very close to ar_tag!!"
                    sec = 15
                elif (self.close == False):
                    print "obstacle, not close to ar tag"
                    sec = 2
                else:
                    print "obstacle, moderately close to ar tag"
                    sec = 5
                rospy.sleep(sec)
                self.prev_state = 'avoid_obstacle'
                self.state = "go_to_pos"
                

            
            if (self.state == 'wait'):
                # just wait around 
                move_cmd = self.mover.wait()
                if (self.AR_curr != -1):
                    print "changing state to go_to_pos"
                    self.prev_state = 'wait'
                    self.state = 'go_to_pos'

            if (self.state == 'go_to_pos'):
                orienting = True 
                print self.ar_z
                while (not(self.AR_seen) or self.ar_z >= 1.5):
                    while (orienting):
                        pos = self.AR_ids[self.AR_curr]
                        dest_orientation = cm.orient(self.mapper.positionToMap(self.position), pos)
                        angle_dif = cm.angle_compare(self.orientation, dest_orientation)
                        if (abs(float(angle_dif)) < abs(math.radians(5)) and self.state is not "bumped"):
                            move_cmd = self.mover.go_to_pos("forward", self.position, self.orientation)
                            print "forward 1"
                            orienting = False
                            # self.execute_command(move_cmd)
                        else:
                            # Turn in the relevant direction
                            if angle_dif < 0:
                                print "left"
                                move_cmd = self.mover.go_to_pos("left", self.position, self.orientation)
                            else:
                                move_cmd = self.mover.go_to_pos("right", self.position, self.orientation)
                                print "right"
                            # self.cmd_vel.publish(move_cmd)
                            # self.rate.sleep()
                    if (not orienting):
                        if ((self.AR_curr > 10)):
                            print "big ar tag"
                            travel_time = 100
                            if (self.AR_curr == (Home*10) + 1):
                                travel_time = 20 #check on this
                            # for i in range(travel_time):
                            #     move_cmd = self.mover.go_to_pos("forward", self.position, self.orientation)
                            #     self.execute_command(move_cmd)
                            self.AR_curr = (self.AR_curr- 1) / 10
                            orienting = True
                if (self.AR_seen and self.ar_z < 1.5):
                    print "see AR"
                    self.sounds.publish(Sound.ON)
                    self.prev_state = 'go_to_pos'
                    self.state = 'go_to_AR'
            
            if (self.state == "go_to_AR"): 
                # parking the robot
                print "going to ar"
                
            # self.sounds.publish(Sound.ON)
                
            

    
    def execute_command(self, my_move):
        """
        - Just a function to decrease repetion when executing move commands
        :param: a move command with linear and angular velocity set, see move_scipt.py
        :return: None
        """
        print "self.state in exec command is:"
        print self.state
        if (self.state is not "bumped" or self.state is not "avoid_obstacle"):
            move_cmd = my_move
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()



    def park(self):
        """
        - Control the parking that the robot 
        :return: None
        """

        # goal distance between robot and ARTag before perfect parking 
        LL_DIST = 0.5 # m
        # distance between ARTag and robot when robot is almost touching it 
        CLOSE_DIST = 0.28 # m
        # desired accuracy when zeroing in on ARTag 
        X_ACC = 0.07 # m
        # parameters for limiting robots movement
        ALPHA_DIST_CLOSE = 0.01 # m
        ALPHA_RAD_CLOSE = radians(0.8) # radians

        # how long should the robot sleep
        SLEEP_TIME = 10
        # for hen robot is lost 
        OSC_LIM = 20

        # theshold for losing and finding the ARTag
        MAX_LOST_TAGS = 10
        MIN_FOUND_TAGS = 3

        # constants of proportionaly for setting speeds
        K_ROT = 2.5
        K_LIN = 0.25

        # distance between robot and parfet spot to park from 
        alpha_dist = 0 # m
        # radians between robot and angle to move 'alpha_dist'
        alpha = 0 # radians

        # used to determine how long to sleep 
        sleep_count = 0
        # determine robot velocity when lost
        osc_count = 0 
        # keep track of how long robot has been lost 
        lost_timer = None # s

        # arrays to save information about robot's history 
        past_orr = []
        past_pos = []
        past_xs = []

        # boolean 
        almost_perfet = False

        while not rospy.is_shutdown(): 
            while self.state is not "bumped" or self.state is not "avoid_obstacle":
                return -1
                # # only begin parking when the ARTag has been 
                # # located and saved in markers dictionary
                # if self.state2 is SEARCHING and len(self.markers) > 0:
                #     print "in SEARCHING"

                #     # used to decide what side of the robot the ARTag is on
                #     theta_org = self.ar_orientation

                #     # using the magnitude of the small angle 
                #     # between the robot and ARTag, beta, for most calculations 
                #     beta = abs(radians(180) - abs(theta_org))   
                #     self.state2 = ZERO_X


                # # handle event of ARTag being lost while parking sequence goes on
                # # this has high priority over other states
                # elif self.state2 is SEARCHING_2:
                #     print "in SEARCHING 2 - ar tag lost"

                #     # keep track of ar_x, which would only 
                #     # be updated when ARTag is in view
                #     del past_orr [:] # clear list of past positions
                #     past_xs.append(self.ar_x) 

                #     # if ar_x is being updated, then ARTag has been found, 
                #     # return to parking TODO: update this name
                #     if len(past_xs) > MIN_FOUND_TAGS:
                #         if not any(sum(1 for _ in g) > MAX_LOST_TAGS*0.5 for _, g in groupby(past_xs)):
                #             print "found tag again!"
                #             osc_count = 0 # clear counter for oscillations
                #             del past_xs [:] # clear list of past ar_xs
                #             self.state2 = ZERO_X
                    
                #     # if the ARTag has been lost for too long, 
                #     # return that parking was unsuccesful
                #     if rospy.Time.now() - lost_timer > rospy.Duration(5):
                #         print "cant find tag, going to return!"
                #         return -1

                #     # oscillate while looking for ARTag to 
                #     # maximize chances of finding it again
                #     osc_count+=1 
                #     osc_count = osc_count % OSC_LIM
                #     if osc_count < OSC_LIM*0.5:  
                #         self.execute_command(self.mover.twist(radians(-30)))
                #     else:
                #         self.execute_command(self.mover.twist(radians(30)))

                #         # used to decide what side of the robot the ARTag is on
                #         theta_org = self.ar_orientation


                # # turn to face the ARTag 
                # if self.state2 is ZERO_X:
                #     print "in zero x"

                #     # keep track of whether the ARTag is still in view or is lost
                #     past_xs.append(self.ar_x)
                #     if any(sum(1 for _ in g) > MAX_LOST_TAGS for _, g in groupby(past_xs)):
                #         lost_timer = rospy.Time.now() # track how long the ARTag has been lost 
                #         self.state2 = SEARCHING_2
                        
                    
                #     # turn until ar_x is almost 0
                #     elif abs(self.ar_x) > X_ACC:
                #         self.execute_command(self.mover.twist(-K_ROT*self.ar_x))
                    
                #     # triangulate distances and angles to guide 
                #     # robot's parking and move to next state
                #     else: 
                #         if almost_perfet == True:
                #             self.state2 = MOVE_PERF
                #         else:    
                #             alpha_dist = cm.third_side(self.ar_z, LL_DIST, beta) # meters
                #             alpha = cm.get_angle_ab(self.ar_z, alpha_dist, LL_DIST) # radians
                #             self.state2 = TURN_ALPHA


                # # turn away from AR_TAG by a small angle alpha
                # elif self.state2 is TURN_ALPHA:
                #     print "in turn alpha"

                #     # if robot is already close to ARTag, it should just park  
                #     if self.ar_z <= CLOSE_DIST*2.5: 
                #         print "dont need to turn - z distance is low"
                #         self.state2 = MOVE_PERF

                #     # alpha will be exceptionally high when LL_DIST 
                #     # is much greater than ar_z + alpha_dist - only need to park
                #     elif abs(alpha) > 100:
                #         print "dont need to turn - alpha is invalid"
                #         self.state2 = MOVE_PERF
                    
                #     # regular operation of just turning alpha
                #     else: 
                #         # keep track of how much robot has turned 
                #         # since it entered 'alpha' state
                #         past_orr.append(self.orientation)
                #         dif =  abs(self.orientation - past_orr[0])
                #         rad2go = abs(alpha) - abs(dif)
                #         print "alpha" + str(alpha)
                #         print degrees(rad2go)

                #         # want to always turn away from the ARTag until 
                #         # robot has almost turned alpha
                #         if rad2go > ALPHA_RAD_CLOSE: 
                #             if theta_org < 0:
                #                 self.execute_command(self.mover.twist(-K_ROT*rad2go)) # robot on left side
                #             else:
                #                 self.execute_command(self.mover.twist(K_ROT*rad2go)) # robot on right side
                #         else:
                #           del past_orr [:] # clear list of past orientations
                #           self.execute_command(self.mover.stop())
                #           self.state2 = MOVE_ALPHA


                # # move to a position that makes parking convenient
                # elif self.state2 == MOVE_ALPHA:
                #     print "in move alpha"
                #     # store info about ar_x as robot moves
                #     past_xs.append(self.ar_x)

                #     # keep track of how far robot has moved since it entered 'MOVE_ALPHA'
                #     past_pos.append(self.position)
                #     dist_traveled =  cm.dist_btwn(self.position, past_pos[0])
                #     dist2go = abs(alpha_dist) - abs(dist_traveled)

                #     # travel until the alpha_dist has been moved - need this to be very accurate
                #     if dist2go > ALPHA_DIST_CLOSE and dist2go > CLOSE_DIST*2.5:
                #         self.execute_command(self.mover.go_forward_K(K_LIN*alpha_dist))
                #     # dont need to do this anymore, rigjt up agains AR_TAG
                #     elif self.ar_z < CLOSE_DIST*2.5:
                #          self.state2 = MOVE_PERF

                #     # turn to face ARTag before moving directly to it 
                #     else: 
                #         print "ar_x" + str(self.ar_x)
                #         del past_orr [:] # clear list of past positions
                        
                #         # check if the ARTag data is valid before zeroing x
                #         if any(sum(1 for _ in g) > MAX_LOST_TAGS*2 for _, g in groupby(past_xs)):
                #             lost_timer = rospy.Time.now() # track how long the ARTag has been lost 
                #             self.state2 = SEARCHING_2
                #         # now zero ar_x
                        
                #         elif abs(self.ar_x) > X_ACC*4:
                #             self.state2 = ZERO_X
                #             almost_perfet = True
                #         else:
                #             self.state2 = MOVE_PERF

                # # move in a straight line to the ar tag 
                # elif self.state2 == MOVE_PERF:
                #     print "in move perf"

                #     print "ar_z" + str(self.ar_z)
                #     print "ar_x" + str(self.ar_x)

                #     # store info about ar_x as robot moves
                #     past_xs.append(self.ar_x)

                #     # check if the ARTag data is valid before turning to face it
                #     if any(sum(1 for _ in g) > MAX_LOST_TAGS*2 for _, g in groupby(past_xs)) and self.ar_z > CLOSE_DIST*2.5:
                #         lost_timer = rospy.Time.now() # track how long the ARTag has been lost 
                #         self.state2 = SEARCHING_2
                #     # now zero ar_x
                #     elif abs(self.ar_x) > X_ACC*2:
                #         self.state2 = ZERO_X
                #         almost_perfet = True

                #     # otherwise move to the ARTag     
                #     else:
                #         if self.ar_z > CLOSE_DIST:
                #             self.execute_command(self.mover.go_forward_K(K_LIN*self.ar_z))
                #         else:
                #             # set parameters for avoiding obstacles
                #             self.close = False
                #             self.close_VERY = True
                #             self.state2 = SLEEPING


                # # wait to recieve package 
                # elif self.state2 == SLEEPING:
                #     print "in sleeping"
                #     sleep_count+=1
                #     rospy.sleep(1)
                #     if sleep_count > SLEEP_TIME:
                #         self.state2 = BACK_OUT


                # # back out from the ARTag
                # elif self.state2 == BACK_OUT:  
                #     print "in back out"
                #     self.execute_command(self.mover.back_out())
                #     if self.ar_z > CLOSE_DIST*3:
                #         # set parameters for avoiding obstacles
                #         self.close_VERY = False
                #         self.state2 = DONE_PARKING


                # # done with the parking sequence!
                # elif self.state2 == DONE_PARKING:
                #     print "in done parking"
                #     self.execute_command(self.mover.stop())
                #     # return succesful parking 
                #     return 0

                self.rate.sleep()


    # ------------------ Functions telling us about the robot ---------------- #     


    def process_ar_tags(self, data):
        """
        Process the AR tag information.
        :param data: AlvarMarkers message telling you where multiple individual AR tags are
        :return: None
        """
        for marker in data.markers:
            if (marker.id == self.AR_curr):
                self.AR_seen = True
                pos = marker.pose.pose.position # what is this relative to -- robot at that time - who is the origin 

                distance = cm.dist((pos.x, pos.y, pos.z))

                self.ar_x = pos.x
                # print "X DIST TO AR TAG %0.2f" % self.ar_x
                self.ar_z = pos.z

                orientation = marker.pose.pose.orientation
                list_orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
                self.ar_orientation = tf.transformations.euler_from_quaternion(list_orientation)[0]
                self.markers[marker.id] = distance
        
    def print_markers(self):
        """
        Print a nicely-formatted presentation of all the tags ever seen
        :return: None
        """
        if len(self.markers) == 0:
            print "\nNo markers seen"
        else:
            print "\nMARKERS:"
            missing_ids = []
            for tag_id, dist in self.markers.iteritems():
                if dist is not None:
                    print "{}:\t{:.2f} m".format(tag_id, dist)
                else:
                    missing_ids.append(tag_id)
            if len(missing_ids) > 0:
                print "Previously seen tags:", ', '.join([str(i) for i in missing_ids])
        print '----------------------------------------------------'

    def process_ekf(self, data):
        """
        Process a message from the robot_pose_ekf and save position & orientation to the parameters
        :param data: PoseWithCovarianceStamped from EKF
        """
        # Extract the relevant covariances (uncertainties).
        # Note that these are uncertainty on the robot VELOCITY, not position
        cov = np.reshape(np.array(data.pose.covariance), (6, 6))
        x_var = cov[0, 0]
        y_var = cov[1, 1]
        rot_var = cov[5, 5]
        # You can print these or integrate over time to get the total uncertainty
        
        # Save the position and orientation
        pos = data.pose.pose.position
        extra_pos = [0, 0]
        extra_or = 0


        self.position = (pos.x + extra_pos[0], pos.y + extra_pos[1])
        orientation = data.pose.pose.orientation
        list_orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.orientation = tf.transformations.euler_from_quaternion(list_orientation)[-1] + extra_or


    #   OBSTACLE TWEAKING: the range of obstacle depth detected, the width of camera, area of obstacle      
    def bound_object(self, img_in):
        """
        - Draws a bounding box around the largest object in the scene and returns
        - Lets us know when obstacles have been seen
        - Lets us know when to avoid obstacles
        :param: Image described by an array
        :return: Image with bounding box 
        """
        img = np.copy(img_in)
        img = img[:-250, :]
        middle_seg = False
        img_height, img_width = img.shape[:2] # (480, 640) 

        # Get contours
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # Find the largest contour
            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            max_contour = contours[max_index]
            new_obstacle_pos = cm.centroid(max_contour)

            # show where largest obstacle is 
            cv2.drawContours(img, max_contour, -1, color=(0, 255, 0), thickness=3)
       
            # Draw rectangle bounding box on image
            x, y, w, h = cv2.boundingRect(max_contour)
   
            # only want to map obstacle if it is large enough 


            # obstacle must be even larger to get the state to be switched 
            if (w*h > 400):
                if (self.close_VERY == False):
                    print "avoiding obstacle"
                    self.prev_state = self.state
                    self.state = 'avoid_obstacle'
 
                else:
                    print "obstacle seen, not being avoided"       

        return img

    def process_depth_image(self, data):
        """ 
        - Use bridge to convert to CV::Mat type. (i.e., convert image from ROS format to OpenCV format)
        - Displays thresholded depth image 
        - Calls bound_object function on depth image
        :param: Data from depth camera
        :return: None
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)

            mask = cv2.inRange(cv_image, 0.1, .5)
            mask[:, 0:140] = 0
            mask[:, 500:] = 0
            # create a mask to restrict the depth that can be seen 
            im_mask = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            self.depth_image = im_mask

            # bound the largest object within this masked image 
            dst2 = self.bound_object(mask)

            # Normalize values to range between 0 and 1 for displaying
            norm_img = im_mask
            cv2.normalize(norm_img, norm_img, 0, 1, cv2.NORM_MINMAX)

            # Displays thresholded depth image   
            cv2.imshow('Depth Image', norm_img)    
            cv2.waitKey(3)

        except CvBridgeError, err:
            rospy.loginfo(err)

    def process_bump_sensing(self, data):
        """
        Simply sets state to bump and lets other functions handle it
        :param data: Raw message data from bump sensor 
        :return: None
        """
        if (data.state == BumperEvent.PRESSED):
            print "BUMP"
            
            self.prev_state = self.state
            self.state = 'bumped'
            
            print self.state

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown 
        :return: None
        """
        # Close CV Image windows
        cv2.destroyAllWindows()
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

if __name__ == '__main__':
    # try:
    robot = Main2()
    robot.run()
    print "success"

    # # uncomment for cleaner print outs!
    # except Exception, err:
    #     rospy.loginfo("You have an error!")
    #     print err
    
       

