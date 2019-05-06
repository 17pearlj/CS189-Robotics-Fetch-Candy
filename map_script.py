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
        self.mapObj = mp.MapDrawer(self.positionToMap)
        self.calliber = [0, 17]
        # create blank array of negative ones to represent blank map 
        self.my_map = -np.ones((40,30))
        self.position  = [0,0]
        self.obstacle_depth = [-1, -1] # depth, segment (segment for map fun)

    def positionToMap(self, position):
        """
        turn EKF position in meters into map position in coordinates
        (r, c) -> (x, y)
        """

        step_x = int(position[0]/world_map_ratio)
        step_y = int(position[1]/world_map_ratio)
        
        return (step_x + 2, step_y + 15)

    def positionFromMap(self, position):
        """
        turn map positions back to EKF for display purposes
        (x, y) -> (r c)
        """

        step_x = (position[0] - self.calliber[0]) * world_map_ratio
        step_y = (position[1] - self.calliber[1]) * world_map_ratio
        
        return (step_x, step_y)

    def initializeMap(self):
        print "initialized map fun called"
        # first map update, need to do twice because it doesn't show up nicely the first time .p
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))
      
        # show map for this amount of time 
        time.sleep(0.001)

    def updateMapFree(self, current_pos_map):
        """
        Takes a (x, y) as a parameter.
        Calling UpdateMapDisplay on (r, c)
        """
        #print "map free"
        # update map with free position
        current_pos = self.positionFromMap(current_pos_map)

        # check that current pos in the map is within the bounds
        # TODO: Should this be <=300 and 400 or 30 and 40?? It's in (x, y)
        if (current_pos_map[0] <= 30 and current_pos_map[0] >= 0 and current_pos_map[1] <= 40 and current_pos_map[1] >= 0):
            # if the current position is ok, set it to be free and update and show the map 
            self.my_map[current_pos_map[0], current_pos_map[1]] = 0
            self.my_map[current_pos_map[0], current_pos_map[1]-1] = 0
            self.my_map[current_pos_map[0]-1, current_pos_map[1]-1] = 0
            self.my_map[current_pos_map[0]-1, current_pos_map[1]] = 0
            self.mapObj.UpdateMapDisplay(self.my_map, current_pos)
            # print "current map pos: %d, %d" % (current_pos_map[0], current_pos_map[1])
            time.sleep(0.0000001)    
    
    def updateMapObstacle(self):
        #print "update map obstacle"
        # position from (r,c) to (x, y)
        (pos_x, pos_y) = self.positionToMap(self.position)
        obstacle_pos = [-1, -1]

        # obstacle coordinate calculation using depth
        if (not(math.isnan(self.orientation)) and not(math.isnan(pos_x)) and not(math.isnan(pos_y))):
            time.sleep(0.0000001) 
            obstacle_pos[0] = int(pos_x + abs(self.obstacle_depth[0])*np.cos(self.orientation + radians(60 - 30*self.obstacle_depth[1])))
            obstacle_pos[1] = int(pos_y + abs(self.obstacle_depth[0])*np.sin(self.orientation + radians(60 - 30*self.obstacle_depth[1])))

            # loop through to free coordinates in front of obstacles
            for x in range(0, int(abs(pos_x - obstacle_pos[0]))):
                for y in range(0, int(abs(obstacle_pos[1] - pos_y))):
                    (x1, y1) = (min(obstacle_pos[0], pos_x) + x, 
                                min(obstacle_pos[1], pos_y) + y)
                    self.updateMapFree((x1, y1))
                    time.sleep(0.0000001) 
            
            # final check before mapping obstacle (may not be necessary)
            if (self.obstacle_depth[0] != -1):
                self.updateMapOccupied() 
                rtime.sleep(0.0000001) 
            else:
                self.updateMapFree((obstacle_pos[0], obstacle_pos[1])) 
                time.sleep(0.0000001) 
         

    def updateMapOccupied(self):
        """
        self.obstacle_pos is in (x, y)
        self.position is in (r, c)
        """
        #print "obstacle loop"

        # update map with position of obstacle and knowledge that that position will be occupied 
        current_pos = self.position
        current_orr = self.orientation
        obstacle_orr = self.orientation # to be changed
        current_pos_map = self.positionToMap(current_pos)
        obstacle_pos_map = self.obstacle_pos
        
        # check that obstacle pos in the map is ok
        if (current_pos_map[0] <= 30 and current_pos_map[0] >= 0 and current_pos_map[1] <= 40 and current_pos_map[1] >= 0):
                # if the obstacle position is ok, set it to be occupied
                self.my_map[obstacle_pos_map[0], obstacle_pos_map[1]] = 1
                self.my_map[obstacle_pos_map[0]-1, obstacle_pos_map[1]] = 1
                self.my_map[obstacle_pos_map[0]-1, obstacle_pos_map[1]-1] = 1
                self.my_map[obstacle_pos_map[0], obstacle_pos_map[1]-1] = 1
                # show the map, but still relative to current position 
                self.mapObj.UpdateMapDisplay(self.my_map, current_pos)
                time.sleep(0.00000001)
        else:
            # if the current position is not ok, let it be known that the values are off, do not change the map array
            print "obstacle map pos: %d, %d" % (obstacle_pos_map[0], obstacle_pos_map[1])


    def updateMapAR(self):
        print "map w artag"
        return 0
