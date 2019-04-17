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

def dist(self, pos):
    """
    Get a the distance to a position from the origin. This can be any number of dimensions
    :param pos: Tuple of position
    :return: Float distance of position from origin
    """
    return np.sqrt(sum([i**2 for i in pos]))