"""
List of functions that are useful and can now be referenced more easily
"""


import random
import math
from math import radians, degrees
import cv2
import numpy as np


def centroid(contour):
    """
    Compute the (x,y) centroid position of the counter
    :param contour: OpenCV contour
    :return: Tuple of (x,y) centroid position
    """

    def centroid_x(c):
        """
        Get centroid x position
        :param c: OpenCV contour
        :return: x position or -1
        """
        M = cv2.moments(c)
        if M['m00'] == 0:
            return -1
        return int(M['m10'] / M['m00'])

    def centroid_y(c):
        """
        Get centroid y position
        :param c: OpenCV contour
        :return: y position or -1
        """
        M = cv2.moments(c)
        if M['m00'] == 0:
            return -1
        return int(M['m01'] / M['m00'])

    return centroid_x(contour), centroid_y(contour)

def dist(pos):
    """
    Get a the distance to a position from the origin. This can be any number of dimensions
    :param pos: Tuple of position
    :return: Float distance of position from origin
    """
    return np.sqrt(sum([i**2 for i in pos]))


def dist_btwn(pos1, pos2):
    """
    Get cartesian distance between the (x, y) positions
    :param pos1: (x, y) position 1
    :param pos2: (x, y) position 2
    :return: Distance (float)
    """
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)


def orient(curr_pos, goal_pos):
    """
    Get necessary heading to reach the goal
    :param curr_pos: (x, y) current position of the robot
    :param goal_pos: (x, y) goal position to orient toward
    """
    return math.atan2(
        goal_pos[1] - curr_pos[1],
        goal_pos[0] - curr_pos[0])


def angle_compare(curr_angle, goal_angle):
    """
    Determine the difference between the angles, normalized from -pi to +pi
    :param curr_angle: current angle of the robot, in radians
    :param goal_angle: goal orientation for the robot, in radians
    """
    pi2 = 2 * math.pi
    # Normalize angle difference
    angle_diff = (curr_angle - goal_angle) % pi2
    # Force into range 0-2*pi
    angle_diff = (angle_diff + pi2) % pi2
    # Adjust to range of -pi to pi
    if (angle_diff > math.pi):
        angle_diff -= pi2
    return angle_diff


def sign(val):
    """
    Get the sign of direction to turn if >/< pi
    :param val: Number from 0 to 2*pi
    :return: 1 if val < pi (left turn), else -1 (right turn)
    """
    if val < 0: 
        return -1
    elif val > 0:
        return 1
    return 0

def third_side(a, b, gamma):
    """
    returns the third side of a triangle given two sides and one angle 
    """
    return math.sqrt(a**2 + b**2 - (2 * a * b * math.cos(gamma)))

def get_angle_ab(a, b, c):
    """
    returns angle in radians that is between a and b given three sides 
    """
    if (a != 0 and b != 0 ):
        top = (a**2 + b**2) - c**2
        if top > 0:
            bottom = 2 * a * b
            both = top/bottom
            if abs(both) > 1:
                both = 1
                return math.acos(both)
            else:
                return math.acos(both)

        else:
            return -222
    else:
            return -222