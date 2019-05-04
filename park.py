"""
Milestone 1: Fetch Candy
Main script controlling robot functionality
"""

import random
import math
from math import radians, degrees
import cv2
import numpy as np

#imports for rospy
import rospy
from geometry_msgs.msg import Twist
import tf
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers

from itertools import groupby

#imports for other functions
import map_script
import move_script
import cool_math as cm 

# valid ids for AR Tags
VALID_IDS = range(18)
SMALL_ANGLE  = radians(8)



class Main:
    def __init__(self):
        # information about the robot's current position and orientation relative to start
        self.position = [0,0]
        self.orientation = 0 # CCW +, radians 

        # the initialized state, and prev state is 'wander'
        self.state2 = 'searching'
        #self.prev_state = 'wander'

        # depth image for getting obstacles
        self.depth_image = []

        # booleans that let robot know when and where to map
        self.obstacle_seen = False
        self.obstacle_depth = [-1, -1] # depth, segment (segment for map fun)

        self.AR_seen = False

        # booleans that help the robot avoid obstacles and react to bumps 
        self.obstacle_side = None 
        self.obstacle_OFF = False # do we care about obstacles or not?

        # tag_id: [distace, orientation] key-value pairs for ARTags
        self.markers = {} # dictionary holding all ARTags currently seen
        self.AR_q = {} # dictionary holding all ARTags ever seen
        

        # key of AR_TAG we are concerned with 
        self.AR_curr = -1
        # lets it be know that an ARTAG is very close 
        self.AR_close = False
        self.handle_AR_step = 0
        self.ar_orientation = None
        self.ar_x = None
        self.ar_z = None

        # mapping object will come from imported module 
        self.mapper = map_script.MapMaker()
        self.mapper.position = self.position
        self.mapper.orientation = self.orientation
        self.mapper.obstacle_depth = self.obstacle_depth
        # moving controls will come from imported module 
        self.mover = move_script.MoveMaker()
        self.mover.position = self.position
        self.mover.AR_close = self.AR_close
        self.mover.handle_AR_step = self.handle_AR_step

        # # ---- rospy stuff ----
        # Initialize the node
        rospy.init_node('Main', anonymous=False)

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
        timer = rospy.Time.now()
        while rospy.Time.now() - timer < rospy.Duration(1) or self.position is None:
            reset_odom.publish(Empty())

        # Subscribe to topic for AR tags
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.process_ar_tags)

        # Use a CvBridge to convert ROS image type to CV Image (Mat)
        self.bridge = CvBridge()
        # Subscribe to depth topic
        rospy.Subscriber('/camera/depth/image', Image, self.process_depth_image, queue_size=1, buff_size=2 ** 24)

        # Subscribe to queues for receiving sensory data, primarily for bumps 
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.process_bump_sensing)

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 5 Hz
        self.rate = rospy.Rate(5)
    
    def execute_command(self, my_move):
        move_cmd = my_move
        self.cmd_vel.publish(move_cmd)
        self.rate.sleep()


    
    def run(self):
        """
        - Control the state that the robot is currently in 
        - Run until Ctrl+C pressed
        :return: None
        """
        # used to determine how long to sleep 
        count = 0

        # constant goal distance from robot before moving to part 
        ll_dist = 0.5 #m

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
        count2 = 0
        self.state2 = 'searching'

        while not rospy.is_shutdown(): 
            
            
            # TODO: need to put checks 4 whether or not the artag has been recently seen 
            # begin when at least one AR_TAG has been found 
            if self.state2 is 'searching' and len(self.markers) > 0:
                # save the orientation wrt to AR_TAG
                print "found a new tag"
                theta_org = self.ar_orientation
                theta = abs(self.ar_orientation)
                beta = abs(radians(180) - theta)
                past_xs.append(self.ar_x)
                # tag has been lpst 
                if any(sum(1 for _ in g) > 10 for _, g in groupby(past_xs)):
                    timer2 = rospy.Time.now()
                    print timer2
                    print "going to searching 2"
                    self.state2 = 'searching2'

            elif self.state2 is 'searching2':
                print "lost tag looking for another"
                past_xs.append(self.ar_x)
                count2 = count2 +  1
                count3 = count2 % 20
                print count3
                # should only go a certain angle each way - TODO
                if count3 < 10:  
                    self.execute_command(self.mover.twist(radians(-30)))
                elif count3 >= 10:
                    self.execute_command(self.mover.twist(radians(30)))

                if len(past_xs) > 3:
                    if past_xs[-1] != past_xs[-2]:
                        print "found it again!"
                        self.state2 = 'zerox'

                
                elif rospy.Time.now() - timer2 > rospy.Duration(10):
                    print "lost artag - need to return! and look for it "
                    # return -1

            elif self.state2 is 'zerox':
                    print "in zerox"

            # turn to face the AR_TAG
#             if self.state2 is 'zerox':
#                 past_xs.append(self.ar_x)
#                 # tag has been lost
#                 if any(sum(1 for _ in g) > 10 for _, g in groupby(past_xs)):
#                     self.state2 = "searching2"
#                     timer2 = rospy.Time.now()
#                 elif abs(self.ar_x) > xAcc:
#                     print("x: %.2f" % self.ar_x)
#                     print("twist velocity:")
#                     print -1*xK_rot*self.ar_x 
#                     self.execute_command(self.mover.twist(-0.8*xK_rot*self.ar_x))
#                 else:
#                     print "zeroed x"
                    
#                     # only want to do these calcs at the beginning when triangle initialized
#                     print("beta before turning: %.2f" % degrees(beta)) # beta should be 0 or close to it on second time through 
#                     print("self.ar_z: %.2f" % self.ar_z) # arz should be close to 0.5 which is current ll_dist  
#                     alpha_dist = cm.third_side(self.ar_z, ll_dist, beta) # alpha dist should be tiny tiny 
#                     print("alpha_dist in turn_alpha: %.2f" % alpha_dist) 
#                     alpha = cm.get_angle_ab(self.ar_z, alpha_dist, ll_dist)
#                     print("degrees alpha: %.2f" % degrees(alpha))
#                     print("regular alpha: %.2f" % alpha)
#                     self.state2 = 'turn_alpha'
            
#             # turn away from AR_TAG by a small angle alpha
#             elif self.state2 is 'turn_alpha':
#                 past_xs.append(self.ar_x)
#                 # tag has been lost
#                 if any(sum(1 for _ in g) > 10 for _, g in groupby(past_xs)):
#                     self.state2 = "searching2"
#                     timer2 = rospy.Time.now()
#                 print "in turn_alpha"      
#                 if self.ar_z <= 0.4: 
#                     print "dont need to turn - z distance is low"
#                     self.state2 = 'move_perf'
                
#                 # can comment  everything else out 
#                 # note this comment out - the robot should always turn???
#                 # robot should not need to turn alpha or move alpha on second time through, 
#                 # should just go straight to zeroing in 
#                 elif alpha_dist <= 0.01: # note this change from 0.5 to 0.01
#                     print "dont need to turn OR MOVE - alpha_dist is very low"
#                     self.state2 = 'move_perf'

                
#                 # this should never happen though, as long as ll_dist is significant
#                 elif abs(alpha) > 100:
#                     print "dont need to turn alpha is invalid"
#                     self.state2 = 'move_perf'
#                 else: 
#                     # save the orientation of the robot at this instance -- might want to look into this 
#                     # if the robot needs to turn alpha more than once, past orr will not be valid! 
#                     # TODO: keep track of how many time the robot does this loop 
#                     past_orr.append(self.orientation)
#                     dif =  abs(self.orientation - past_orr[0])
#                     print("dif: %.2f" % degrees(dif))
#                     dif2go = abs(alpha - dif)

#                     # rotate until angled 'alpha' away from the orientation in 'zerox'
#                     if dif2go > radians(0.8): #TODO: replace with SMALLANGLE constant 
#                         print("dif2go in alpha: %.2f" % degrees(dif2go))
#                         print("twist velocity:")
#                         print 2*dif2go
#                         print("theta: %.2f" % degrees(theta_org))
#                         if theta_org < 0:
#                             print "left side"
#                             self.execute_command(self.mover.twist(-2*dif2go))
#                         else:
#                             print "right side"
#                             self.execute_command(self.mover.twist(2*dif2go))
#                     else:
#                         # move to next state when this has happened -- no check?
#                         print "dont need to turn much more - go to move_alpha"
#                         self.execute_command(self.mover.stop())
#                         self.state2 = 'move_alpha'
# # alpha dist starts increasing??           
# # 0.34
# # arz - 0.90
#             # move to a position that will allow the robot to move straight towards the robot
#             elif self.state2 == 'move_alpha':
#                 past_xs.append(self.ar_x)
#                 # tag has been lost
#                 if any(sum(1 for _ in g) > 20 for _, g in groupby(past_xs)):
#                     self.state2 = "searching2"
#                     timer2 = rospy.Time.now()
#                 # want beta to be near 0, if beta is close to 180, then alpha will be 0.75m when z is low 
#                 print("beta in move alpha: %.2f" % degrees(beta)) 
#                 print("self.ar_z: %.2f" % self.ar_z)
#                 print("alpha_dist in move_alpha FROM TURN ALPHA: %.2f" % alpha_dist)
#                 # recalculating alpha_dist with knowledge that beta is 0 + alpha
#                 #alpha_dist = cm.third_side(self.ar_z, ll_dist, beta) # beta is going to be messed up after you turn alpha, so that will mess up alpha dist
#                 # simplest solution -- don't recalculate alpha_dist here just do it only before moving beta 
#                 #print("alpha_dist in move_alpha: %.2f" % alpha_dist) # should ideally be the same, prolly wont be 
                
#                 # this is ok if alpha_dist is constantly being recalculated with the  WRONG BETA - lol comment this out!
#                 # if alpha_dist > 0.05: 
#                 #     #TODO: replace with a constant for this accuracy
#                 #     print "moving to alpha dist"
#                 #     self.execute_command(self.mover.go_forward_K(alpha_dist))

#                 # more correct way 
#                 # position at time started moving ar_dist 
#                 past_pos.append(self.position)
#                 dist_traveled =  cm.dist_btwn(self.position, past_pos[0])
#                 print("dist_traveled: %.2f" % dist_traveled)
#                 dist2go = abs(alpha_dist) - abs(dist_traveled)
#                 if dist2go > 0.01:
#                     print("dist2go: %.2f" % dist2go)
#                     self.execute_command(self.mover.go_forward_K(0.5*alpha_dist))

#                 # this will always happen the first time 
#                 elif abs(self.ar_x) > xAcc:
#                     print "back to zeroing x "
#                     self.state2 = 'zerox'
#                 else: 
#                     print "moving forward to artag"
#                     self.state2 = "move_perf"

#             # move in a straight line to the ar tag 
#             elif self.state2 == 'move_perf':
#                 print self.state2
#                 self.close = False
#                 self.close_VERY = True
#                 print("self.ar_x: %.2f" % self.ar_z)
#                 print("self.ar_z: %.2f" % self.ar_x)
#                 past_xs.append(self.ar_x)
#                 # tag has been lost -- higher threshold than when zeroing x bc are likely closer
#                 if any(sum(1 for _ in g) > 10 for _, g in groupby(past_xs)):
#                     self.state2 = "searching2"
#                     timer2 = rospy.Time.now()
#                 elif abs(self.ar_x) >= 0.20:
#                     print "back to zeroiing x "
#                     self.state2 = 'zerox'
#                 else:
#                     if self.ar_z > 0.20:
#                         print "going to tag"
#                         self.execute_command(self.mover.go_forward_K(0.25*self.ar_z))
#                     else:
#                         print "park_it"
#                         self.state2 = "park_it"

#             # wait to recieve package 
#             elif self.state2 == "park_it":
#                # self.execute_command(self.mover.stop())
#                 print "sleeping"
#                 count+=1
#                 print count
#                 rospy.sleep(1)
#                 if count > 10:
#                     print "back out"
#                     self.state2 = "back out"

#             # backout 
#             elif self.state2 == "back out":  
#                     self.execute_command(self.mover.back_out())
#                     print self.ar_z
#                     if self.ar_z > 0.7:
#                         self.state2 = 'done'

#             # done with the parking sequence!
#             elif self.state2 == "done":
#                     self.execute_command(self.mover.stop())
#                     print "done parking :)"
#                     self.close_VERY = False
#                     return

            self.rate.sleep()

# ------------------ Functions telling us about the robot and its environment ---------------- #     
    def process_ar_tags(self, data):
        """
        Process the AR tag information.
        :param data: AlvarMarkers message telling you where multiple individual AR tags are
        :return: None
        """

        # Set the position for all the markers that are in the received message
        for marker in data.markers:
            if marker.id in VALID_IDS:
                pos = marker.pose.pose.position # what is this relative to -- robot at that time - who is the origin 
                # for checking 
                # print "pos:"
                # print pos

                distance = cm.dist((pos.x, pos.y, pos.z))
                self.ar_x = pos.x
                self.ar_z = pos.z

                orientation = marker.pose.pose.orientation
                list_orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
                self.ar_orientation = tf.transformations.euler_from_quaternion(list_orientation)[0]


                # want to keep track of the distance between robot and AR_tag, but also robot's orientation at the time 
                # and whether or not the AR_tag has been visited 
                # going to do this in seperate dictionaries for now
                self.markers[marker.id] = distance
                self.AR_q[marker.id] = [pos, self.orientation, 'unvisited']

        # Set the position to None for all the previously seen markers that weren't in the message
        all_seen_ids = self.markers.keys()
        seen_now_ids = [m.id for m in data.markers]
        missing_ids = list(set(all_seen_ids) - set(seen_now_ids))

        # Lets us know about only ARTags seen currently, to update map as needed 
        self.markers.update(dict.fromkeys(missing_ids, None))
        if (len (self.markers) > 0):
            self.AR_seen = True
  
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
        self.position = (pos.x, pos.y)
        orientation = data.pose.pose.orientation
        list_orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.orientation = tf.transformations.euler_from_quaternion(list_orientation)[-1]
        # want the first element 
            
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
        NUM_SEGMENTS = 5 #segmenting robot vision by 5 (could be changed to 3 if lag/ not accurate)


        # Get contours
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # Find the largest contour
            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            max_contour = contours[max_index]
            new_obstacle_pos = cm.centroid(max_contour)
            obs_segment = int(math.floor(new_obstacle_pos[1]/128))
            # returns obstacle depth that will allow obstacle to be mapped 
            if new_obstacle_pos:
                #print "obstacle!"
                self.obstacle_depth =  [(self.depth_image[new_obstacle_pos[0]-200][new_obstacle_pos[1]]/0.2),
                                        obs_segment]

            # show where largest obstacle is 
            cv2.drawContours(img, max_contour, -1, color=(0, 255, 0), thickness=3)
       
            # Draw rectangle bounding box on image
            x, y, w, h = cv2.boundingRect(max_contour)
   
            # only want to map obstacle if it is large enough 
            if ((w*h > 200) | ((w*h > 100) and (obs_segment == 2))):
                self.obstacle_seen = True

            # # obstacle must be even larger to get the state to be switched - JK NOT SWITCHING RN
            # if ((w*h > 400) | ((w*h > 200) and (obs_segment == 2))):
            #     if (self.obstacle_OFF == False and (self.state2 is not 'handle_AR')):
            #         print "avoiding obstacle"
            #         self.state2 = 'avoid_obstacle'
            #         # Differentiate between left and right objects
            #         if (obs_segment < 1):  
            #             self.obstacle_side = 'left'
            #         else:
            #             self.obstacle_side = 'right'  
            #     else:
            #         print "obstacle seen, not being avoided"       

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

            mask = cv2.inRange(cv_image, 0.1, .4)
            mask[:, 0:70] = 0
            mask[:, 570:] = 0
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

        if (data.state == BumperEvent.PRESSED and self.obstacle_OFF == False):
            self.state2 = 'bumped'

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
        rospy.sleep(2)

if __name__ == '__main__':
    # try:
    robot = Main()
    robot.run()
    print "success"

    # # uncomment for cleaner print outs!
    # except Exception, err:
    #     rospy.loginfo("You have an error!")
    #     print err
    
       

