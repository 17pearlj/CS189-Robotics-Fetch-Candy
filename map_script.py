import map_util as mp
import numpy as np
import time 

class MapMaker:
    def __init__(self):
        # initialize MapDrawer object
        self.mapObj = mp.MapDrawer(self.positionToMap)

        # create blank array of negative ones to represent blank map 
        self.my_map = -np.ones((40,30))
        self.position  = [0,0]

    def positionToMap(self, position):
        """
        turn EKF position in meters into map position in coordinates
        """
        # ratio of world meters to map coordinates 
        world_map_ratio = 0.2

        step_x = int(position[0]/world_map_ratio)
        step_y = int(position[1]/world_map_ratio)
        
        return (step_x + 2, step_y + 15)

    def initializeMap(self):
        # first map update, need to do twice because it doesn't show up nicely the first time .p
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))
      
        # show map for this amount of time 
        time.sleep(0.001)

    def updateMapFree(self):
        print "map free"
        # update map with current position and knowlege that this position is free
        current_pos = self.position
        current_pos_map = self.positionToMap(current_pos)

        # check that current pos in the map is within the bounds
        if (current_pos_map[0] <= 300 and current_pos_map[0] >= 0 and current_pos_map[1] <= 400 and current_pos_map[1] >= 0):
            # if the current position is ok, set it to be free and update and show the map 
            self.my_map[current_pos_map[0], current_pos_map[1]] = 0
            self.my_map[current_pos_map[0], current_pos_map[1]-1] = 0
            self.my_map[current_pos_map[0]-1, current_pos_map[1]-1] = 0
            self.my_map[current_pos_map[0]-1, current_pos_map[1]] = 0
            self.mapObj.UpdateMapDisplay(self.my_map, current_pos)
            print "current map pos: %d, %d" % (current_pos_map[0], current_pos_map[1])
            time.sleep(0.0000001)   
    
    def updateMapObstacle(self):
        print "map obstacle"
        return 0

    def updateMapAR(self):
        print "map w artag"
        return 0
