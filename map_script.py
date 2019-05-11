"""
Used for Mapping
"""

import map_util as mp
import math
from math import radians, degrees
import numpy as np
import time 

# ratio of world meters to map coordinates 
world_map_ratio = 0.2

class MapMaker:
    def __init__(self):
        # initialize MapDrawer object
        print "map initialized"
        # self.mapObj = mp.MapDrawer(self.positionToMap)
        self.calliber = [0, 17]
        # create blank array of negative ones to represent blank map 
        self.my_map = -np.ones((40,30))
        self.position  = [0,0]
        self.obstacle_depth = [-1, -1] # depth, segment (segment for map fun)

    def positionToMap(self, position, calliber):
        """
        turn EKF position in meters into map position in coordinates
        (r, c) -> (x, y)
        """

        step_x = int(position[0]/world_map_ratio)
        step_y = int(position[1]/world_map_ratio)
        
        return (step_x + calliber[0], step_y + calliber[1])

    def positionFromMap(self, position, calliber):
        """
        turn map positions back to EKF for display purposes
        (x, y) -> (r c)
        """

        step_x = (position[0] - calliber[0]) * world_map_ratio
        step_y = (position[1] - calliber[1]) * world_map_ratio
        
        return (step_x, step_y)

    def initializeMap(self):
        print "initialized map fun called"
        # first map update, need to do twice because it doesn't show up nicely the first time .p
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))
      
        # show map for this amount of time 
        time.sleep(0.001)

    