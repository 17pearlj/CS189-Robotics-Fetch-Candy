"""
Milestone 2: Fetch Candy
Main script controlling robot functionality
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

class Main2:
    def __init__(self):
        # information about the robot's current position and orientation relative to start
        self.position = [0, 0]
        self.orientation = radians(0) # CCW +, radians 
        

        # states: wait, go_to_pos, go_to_AR, handle_AR
        self.state = 'wait'
        self.prev_state = 'wait'
        self.state2 = None

        # depth image for getting obstacles
        self.depth_image = []

        # key of AR_TAG we are seeking
        self.AR_curr = -1
        # dictionary for ar ids and coordinates
        self.AR_ids = {
            1: (2, 15), 
            2: (4, 24),
            3: (45, 24),
            4: (31, 14), 
            51: (35, 18), #fake location to get around table
            5: (23, 10),
            61: (35, 18), #fake location to get around table
            6: (25, 8),
            7: (9, 5)
        } 

        self.returning = False
        self.ar_orientation = 0
        self.ar_x = 0
        self.ar_z = 0
        self.close = False #if there's an obstacle and we are really close to the ar_tag, it's probably another robot
        self.close_VERY = False #if we are extremely close to the ar_tag, we are just going to park or get bumped
        self.AR_seen = False
        
        # mapping object will come from imported module 
        self.mapper = map_script.MapMaker()
        self.mapper.position = self.position
        self.mapper.orientation = self.orientation
        self.mover = move_script.MoveMaker()
        self.markers = {}

               

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
        # # Subscribe to topic for AR tags
        # rospy.Subscriber('/ar_pose_marker', AlvarMarkers, )
   
    
    
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
            print "my position %s" % str(self.mapper.positionToMap(self.position)) 
            move_cmd = Twist()
            while(self.state is not 'avoid_obstacle' or self.state is not 'bumped'):
                while (self.state == 'wait'):
                    # just wait around 
                    move_cmd = self.mover.wait()
                    if (self.AR_curr != -1):
                        print "changing state to go_to_pos"
                        self.prev_state = 'wait'
                        self.state = 'go_to_pos'

                if (self.state == 'go_to_pos'):
                    orienting = True 
                    while (not(self.AR_seen) or self.ar_x > .7):
                        while (orienting):
                            print "orienting"
                            pos = self.AR_ids[self.AR_curr]
                            print "pos %s" % str(pos)
                            dest_orientation = cm.orient(self.mapper.positionToMap(self.position), pos)
                            print "my position %s" % str(self.mapper.positionToMap(self.position))
                            angle_dif = cm.angle_compare(self.orientation, dest_orientation)
                            if (abs(float(angle_dif)) < abs(math.radians(5))):
                                move_cmd = self.mover.go_to_pos("forward", self.position, self.orientation)
                                print "forward"
                                orienting = False
                                
                            else:
                                # Turn in the relevant direction
                                if angle_dif < 0:
                                    move_cmd = self.mover.go_to_pos("left", self.position, self.orientation)
                                else:
                                    move_cmd = self.mover.go_to_pos("right", self.position, self.orientation)
                                self.cmd_vel.publish(move_cmd)
                                self.rate.sleep()
                        else:
                            if ((self.AR_curr == 51 or self.AR_curr == 61)):
                                for i in range(100):
                                    move_cmd = self.mover.go_to_pos("forward", self.position, self.orientation)
                                    self.cmd_vel.publish(move_cmd)
                                    self.rate.sleep()
                                self.AR_curr = (self.AR_curr- 1) / 10
                                orienting = True
                                
                            else: 
                                move_cmd = self.mover.go_to_pos("forward", self.position, self.orientation)
                                print "forward"
                                self.cmd_vel.publish(move_cmd)
                                self.rate.sleep()
                    if (self.AR_seen):
                        print "see AR"
                        self.sounds.publish(Sound.ON)
                        self.prev_state = 'go_to_pos'
                        self.state = 'go_to_AR'
                
                elif (self.state == "go_to_AR"): 
                    # -----------handle ar here!!--------#
                    self.state2 = "searching"
                    self.park()
                    # return from handle ar!
                    self.AR_seen = False
                    self.returning = not(self.returning)
                    if (self.returning):
                        self.AR_curr = Home
                        self.prev_state = 'go_to_AR'
                        self.state = 'go_to_pos'
                    else:
                        self.AR_curr = -1
                        self.prev_state = 'go_to_AR'
                        self.state = 'wait'
                self.cmd_vel.publish(move_cmd)
                self.rate.sleep()
                # self.sounds.publish(Sound.ON)
            if (self.state == "avoid_obstacle" or self.state == "bumped"):
                sec = 0
                if (self.state == bumped and not self.close_VERY):
                    self.execute_command(self.mover.bumped())
                if (self.close_VERY):
                    sec = 15
                if (self.close == False):
                    sec = 2
                else:
                    sec = 5
                rospy.sleep(sec)
                self.state = self.prev_state
                self.prev_state = 'avoid_obstacle'

    def execute_command(self, my_move):
        move_cmd = my_move
        self.cmd_vel.publish(move_cmd)
        self.rate.sleep()

    def park(self):
        """
        - Control the state that the robot is currently in 
        - Run until Ctrl+C pressed
        :return: None
        """
        # used to determine how long to sleep 
        count = 0

        # constant goal distance from robot before moving to part 
        ll_dist = 0.7 #m

        # arrays to save the orientation at a certain instance 
        past_orr = []
        past_pos = []
        past_xs = []

        # constant of proportionaly for angular speed 
        K_rot = 0.05
        # want to zero in on x quickly
        xK_rot = 2
        # desired accuracy when zeroing in on ARTag 
        xAcc = 0.07
        alpha_dist = 0
        timer2 = 0

        while not rospy.is_shutdown(): 
            # TODO: need to put checks 4 whether or not the artag has been recently seen 
            # begin when at least one AR_TAG has been found 
            if self.state2 is 'searching' and len(self.markers) > 0:
                # save the orientation wrt to AR_TAG
                print "found a new tag"
                theta = abs(self.ar_orientation)
                beta = abs(radians(180) - theta)
                self.state2 = 'zerox'
            elif self.state2 is 'searching2':
                print "lost tag looking for another"
                # should only go a certain angle each way - TODO
                self.execute_command(self.mover.twist(radians(-15)))
                if rospy.Time.now() - timer2 > rospy.Duration(5):
                    print "lost artag - need to return! and look for it "

            # turn to face the AR_TAG
            if self.state2 is 'zerox':
                past_xs.append(self.ar_x)
                # tag has been lost
                if any(sum(1 for _ in g) > 3 for _, g in groupby(past_xs)):
                    self.state2 = "searching2"
                    timer2 = rospy.Time.now()
                elif abs(self.ar_x) > xAcc:
                    print("x: %.2f" % self.ar_x)
                    print("twist velocity:")
                    print -1*xK_rot*self.ar_x 
                    self.execute_command(self.mover.twist(-0.8*xK_rot*self.ar_x))
                else:
                    print "zeroed x"
                    
                    # only want to do these calcs at the beginning when triangle initialized
                    print("beta before turning: %.2f" % degrees(beta)) # beta should be 0 or close to it on second time through 
                    print("self.ar_z: %.2f" % self.ar_z) # arz should be close to 0.5 which is current ll_dist  
                    alpha_dist = cm.third_side(self.ar_z, ll_dist, beta) # alpha dist should be tiny tiny 
                    print("alpha_dist in turn_alpha: %.2f" % alpha_dist) 
                    alpha = cm.get_angle_ab(self.ar_z, alpha_dist, ll_dist)
                    print("degrees alpha: %.2f" % degrees(alpha))
                    print("regular alpha: %.2f" % alpha)
                    self.state2 = 'turn_alpha'
            
            # turn away from AR_TAG by a small angle alpha
            elif self.state2 is 'turn_alpha':
                print "in turn_alpha"      
                if self.ar_z <= 0.4: 
                    print "dont need to turn - z distance is low"
                    self.state2 = 'move_perf'
                
                # can comment  everything else out 
                # note this comment out - the robot should always turn???
                # robot should not need to turn alpha or move alpha on second time through, 
                # should just go straight to zeroing in 
                elif alpha_dist <= 0.01: # note this change from 0.5 to 0.01
                    print "dont need to turn OR MOVE - alpha_dist is very low"
                    self.state2 = 'move_perf'

                
                # this should never happen though, as long as ll_dist is significant
                elif abs(alpha) > 100:
                    print "dont need to turn alpha is invalid"
                    self.state2 = 'move_perf'
                else: 
                    # save the orientation of the robot at this instance -- might want to look into this 
                    # if the robot needs to turn alpha more than once, past orr will not be valid! 
                    # TODO: keep track of how many time the robot does this loop 
                    past_orr.append(self.orientation)
                    dif =  abs(self.orientation - past_orr[0])
                    print("dif: %.2f" % degrees(dif))
                    dif2go = abs(alpha - dif)

                    # rotate until angled 'alpha' away from the orientation in 'zerox'
                    if dif2go > radians(3): #TODO: replace with SMALLANGLE constant 
                        print("dif2go: %.2f" % degrees(dif2go))
                        print("twist velocity:")
                        print 2*dif2go
                        self.execute_command(self.mover.twist(2*dif2go))
                    else:
                        # move to next state when this has happened -- no check?
                        print "dont need to turn much - go to move_alpha"
                        self.execute_command(self.mover.stop())
                        self.state = 'move_alpha'
# alpha dist starts increasing??           
# 0.34
# arz - 0.90
            # move to a position that will allow the robot to move straight towards the robot
            elif self.state2 == 'move_alpha':
                # want beta to be near 0, if beta is close to 180, then alpha will be 0.75m when z is low 
                print("beta in move alpha: %.2f" % degrees(beta)) 
                print("self.ar_z: %.2f" % self.ar_z)
                print("alpha_dist in move_alpha FROM TURN ALPHA: %.2f" % alpha_dist)
                # recalculating alpha_dist with knowledge that beta is 0 + alpha
                #alpha_dist = cm.third_side(self.ar_z, ll_dist, beta) # beta is going to be messed up after you turn alpha, so that will mess up alpha dist
                # simplest solution -- don't recalculate alpha_dist here just do it only before moving beta 
                #print("alpha_dist in move_alpha: %.2f" % alpha_dist) # should ideally be the same, prolly wont be 
                
                # this is ok if alpha_dist is constantly being recalculated with the  WRONG BETA - lol comment this out!
                # if alpha_dist > 0.05: 
                #     #TODO: replace with a constant for this accuracy
                #     print "moving to alpha dist"
                #     self.execute_command(self.mover.go_forward_K(alpha_dist))

                # more correct way 
                # position at time started moving ar_dist 
                past_pos.append(self.position)
                dist_traveled =  cm.dist_btwn(self.position, past_pos[0])
                print("dist_traveled: %.2f" % dist_traveled)
                dist2go = abs(alpha_dist) - abs(dist_traveled)
                if dist2go > 0.01:
                    print("dist2go: %.2f" % dist2go)
                    self.execute_command(self.mover.go_forward_K(0.5*alpha_dist))

                # this will always happen the first time 
                elif abs(self.ar_x) > xAcc:
                    print "back to zeroing x "
                    self.state2 = 'zerox'
                else: 
                    print "moving forward to artag"
                    self.state2 = "move_perf"

            # move in a straight line to the ar tag 
            elif self.state2 == 'move_perf':
                self.close = False
                self.close_VERY = True
                print self.state2
                print("self.ar_x: %.2f" % self.ar_z)
                print("self.ar_z: %.2f" % self.ar_x)
                past_xs.append(self.ar_x)
                # tag has been lost
                if any(sum(1 for _ in g) > 3 for _, g in groupby(past_xs)):
                    self.state = "searching2"
                    timer2 = rospy.Time.now()
                elif abs(self.ar_x) >= xAcc:
                    print "back to zeroing x "
                    self.state2 = 'zerox'
                else:
                    if self.ar_z > 0.25:
                        print "going to tag"
                        self.execute_command(self.mover.go_forward_K(0.25*self.ar_z))
                    else:
                        print "park_it"
                        self.state2 = "park_it"

            # wait to recieve package 
            elif self.state2 == "park_it":
               # self.execute_command(self.mover.stop())
                print "sleeping"
                count+=1
                print count
                rospy.sleep(1)
                if count > 10:
                    print "back out"
                    self.state2 = "back out"

            # backout 
            elif self.state2 == "back out":  
                    self.execute_command(self.mover.back_out())
                    print self.ar_z
                    if self.ar_z > 0.7:
                        self.state2 = 'done'

            # done with the parking sequence!
            elif self.state2 == "done":
                    self.execute_command(self.mover.stop())
                    print "done parking :)"  
                    self.close_VERY = False
                    return  

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
                print "X DIST TO AR TAG %0.2f" % self.ar_x
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
                if (self.obstacle_OFF == False and self.close_VERY == False):
                    print "avoiding obstacle"
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

            mask = cv2.inRange(cv_image, 0.1, .3)
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
            # cv2.imshow('Depth Image', norm_img)    
            # cv2.waitKey(3)

        except CvBridgeError, err:
            rospy.loginfo(err)

    def process_bump_sensing(self, data):
        """
        Simply sets state to bump and lets other functions handle it
        :param data: Raw message data from bump sensor 
        :return: None
        """

        if (data.state == BumperEvent.PRESSED):
            self.state = 'bumped'

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown 
        :return: None
        """
        # TODO: save the map image - maybe put somewhere else?

        # Close CV Image windows
        cv2.destroyAllWindows()
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(5)

if __name__ == '__main__':
    # try:
    robot = Main2()
    robot.run()
    print "success"

    # # uncomment for cleaner print outs!
    # except Exception, err:
    #     rospy.loginfo("You have an error!")
    #     print err
    
       

