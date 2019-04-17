"""
2018-03: Created by Julia Ebert
"""

from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, PointStamped
import tf
from std_msgs.msg import Empty
import math
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
from math import radians, degrees
from kobuki_msgs.msg import BumperEvent
import numpy as np

# constant for speed 
LIN_SPEED = 0.2  # 0.2 m/s
ROT_SPEED = radians(45)  # 45 deg/s in radians/s

# Parameters for drawing shape
SIDE_LENGTH = 1  # meter
TURN_ANGLE = radians(90)  # 90 deg in radians


class LocalizedSquare:
    """
    Continuously draw squares. After every square, print out the robot's position
    as determined by the EKF package.

    Note that this is open loop control; the localization from the EKF position is
    not used for controlling the robot.
    """

    def __init__(self):

        # Initialize
        rospy.init_node('LocalizedSquare', 'time', anonymous=False)

        # Robot state (forward, left, right, reverse, stop)
        self.state = 'stop'
        self.state_change_time = rospy.Time.now()
        
        # Boolean to track whether bumper was pressed
        self.bump = False
        
        # Localization determined from EKF
        # Position is a geometry_msgs/Point object with (x,y,z) fields
        self.position = None
        # Orientation is the rotation in the floor plane from initial pose
        self.orientation = None

        # What to do you ctrl + c (call shutdown function written below)
        rospy.on_shutdown(self.shutdown)

        # Publish to topic for controlling robot
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Subscribe to bumper events topic
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.process_bump_sensing)

        # Subscribe to robot_pose_ekf for odometry/position information
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.process_ekf)

        # Set up the odometry reset publisher (publishing Empty messages here will reset odom)
        reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=1)
        # Reset odometry (these messages take about a second to get through)
        timer = rospy.Time.now()
        while rospy.Time.now() - timer < rospy.Duration(1) or self.position is None:
            reset_odom.publish(Empty())

        # 5 HZ
        self.rate = rospy.Rate(5)

    def run(self):
        """
        Run the robot until Ctrl+C is pressed, a bump is detected, or 4 squares are drawn
        :return: None
        """

        # How long to go forward/turn
        turn_dur = rospy.Duration(TURN_ANGLE/ROT_SPEED)  # seconds
        straight_dur = rospy.Duration(SIDE_LENGTH/(LIN_SPEED*2))  # seconds

        # Initialize by starting first side
        self.state = 'forward'
        self.state_change_time = rospy.Time.now()
        printed_position = False

        home = Twist()
        
        

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.bump:
            	print "bump"
                # return home
                # (1) re-orient itself toward the home position, and (2) move straight home.
                # angle headed = arctan (Yhome - Ycurrent/ Xhome - Xcurrent)
                n_Theta = math.atan(self.position[1] / self.position[0])
                home.angular.z = n_Theta
                print home.angular.z
                home.linear.x = 0.0
                for i in range(0,3):
                    self.cmd_vel.publish(home)
                print home.angular.z
                home.angular.z = radians(0)
                home.linear.x = 0.2

                while (self.position != (0,0)):
                    self.cmd_vel.publish(home)
                    
                self.bump = False

            if self.state == 'right' and now > self.state_change_time + turn_dur:
                # Finished turning corner; start next side
                self.state = 'forward'
                self.state_change_time = now
                printed_position = False
            elif self.state == 'forward' and now > self.state_change_time + straight_dur:
                # Finished side; start turning corner
                self.state = 'right'
                self.state_change_time = now

            # Print the EKF-determined position, if back to the first side
            if not printed_position:
                rospy.loginfo('({:.2f}, {:.2f})\t{:.1f} deg'.format(
                    self.position[0], self.position[1], degrees(self.orientation)))
                printed_position = True

            # Send appropriate movement command
            self.cmd_vel.publish(self.get_move_cmd())
        
            self.rate.sleep()

    def get_move_cmd(self):
        """
        Create movement command according to robot's state
        :return: Twist of movement to make
        """
        # Start with command with 0 linear and angular velocity (stop command)
        move_cmd = Twist()

        if self.state == 'reverse':
            move_cmd.linear.x = -1 * LIN_SPEED / 2.0
        elif self.state == 'forward':
            move_cmd.linear.x = LIN_SPEED
        elif self.state == 'left':
            move_cmd.angular.z = ROT_SPEED
        elif self.state == 'right':
            move_cmd.angular.z = -ROT_SPEED

        return move_cmd

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

    def process_bump_sensing(self, data):
        """
        If bump data is received, process the data
        data.bumper: LEFT (0), CENTER (1), RIGHT (2)
        data.state: RELEASED (0), PRESSED (1)
        :param data: Raw bump sensor data 
        :return: None
        """
        if data.state == BumperEvent.PRESSED:
            self.bump = True
        else:
           self.bump = False

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown 
        :return: None
        """
        # stop turtlebot
        rospy.loginfo("Stop Drawing Squares!")
        # publish a zeroed out Twist object
        self.cmd_vel.publish(Twist())
        # sleep before final shutdown
        rospy.sleep(2)


if __name__ == '__main__':
    try:
        robot = LocalizedSquare()
        robot.run()
    except Exception, err:
        rospy.loginfo("LocalizedSquare node terminated.")
        ex_type, ex, tb = sys.exc_info()
        traceback.print_tb(tb)
        print err

