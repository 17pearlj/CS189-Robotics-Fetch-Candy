import cool_math as cm

if __name__ == '__main__':
    h = [0,1,2,3,4]
    tlist = ['a', 'b', 'c', 'd', 'e']
    jlist = [1, 3, 7, 3, 2]
    test_dict = {}
    # my_dict = {
    #     "id1": [1, 2, 3], "id2": [1, 2, 3], "id3": [1, 2, 3]
    #     }
        
    # print my_dict
    # hello = all(x[2] != 3 for x in my_dict.values())
    # print hello

    # for tnum in h:
    #     test_dict[tnum] = [tlist[tnum], jlist[tnum]]
    # #print test_dict

    # value = [None] * 2
    # sub_dict = {}
    # for value in test_dict.values():
    #     if value[1] == 3:
    #         print 0
            

    # sub_list = []
    # for item in test_dict.items():
    #     if (item[1][1] == 3):
    #         #sub_dict.update(test_dict[item]) 
    #         # use list of lists instead
    #         sub_list.append([item])
    
    # print sub_list
 
#     pos1 = (3, 20, 1)
#     pos2 = (3,4,5)
#     pos3 = (1,1,1)
#     orr = 1
#     key = 12
#     key1 = 13
#     key2 =14
#     my_pos = (0,0)
    
#  # new list of all the distances in the list of ARTags that have not been visited 
#     unvisited = [(key, [pos1, orr, 'hello']),
#                 (key1, [pos2, orr, 'hello']),
#                 (key2, [pos3, orr, 'hello'])]
#     close = []
#     for i in unvisited:
#         pos = (i[1][0][0] , i[1][0][1])
#         curr_dist = cm.dist_btwn(pos, my_po0s)
#         close.append([i, curr_dist])

#     # actually reorder from smallest to largest distance 
#     close.sort(reverse = True)

#     #print close
    
#     # close is sorted smallest to large, but its index in unvisited was saved 
#     print "the one:"
#     the_key = close[0][0][0]
#     print the_key
    
#     my_dict = dict(unvisited)

#     print "the chosen one:"
#     print my_dict.get(the_key)
    

    # returns [(1, 1, 1), 1, 'hello']
my_var = 18.456789
print("my var %.2f" % my_var)






























# old code

while (self.state == 'wander'):
                # just wandering around 
                move_cmd = self.mover.wander()
            
                # current location will always be free :)
                self.mapper.updateMapFree(self.position)
                
                # # this info will come from depth senor processing
                if (self.obstacle_seen == True):
                    self.mapper.updateMapObstacle()

                # # map the ARTAG using info from ARTAG sensor stored in self
                # elif (self.AR_seen == True): 
                #     self.mapper.updateMapAR()

                # if there are ARTags that have not yet been visited, choose one to visit 
                if (len(self.AR_q) is not 0 and all(x[2] == 'unvisited' for x in self.AR_q.values())):
                    self.AR_curr = self.mover.choose_AR(self.AR_q) 
                    print "CURRENT AR TAG:"
                    print self.AR_curr
                    # robot will now go to AR tag 
                    if self.AR_curr is not -1:
                        self.prev_state = 'wander'
                        self.state = 'go_to_AR'
                    else:
                        self.state = 'wander'

                self.cmd_vel.publish(move_cmd)
                self.rate.sleep()
                    

            # zero in on an ARTag 
            elif (self.state == 'go_to_AR'):
                move_cmd, self.AR_close, self.obstacle_OFF = self.mover.go_to_AR(self.AR_q, self.AR_curr, self.orientation)
                # only want to do the ARTag procedure when we are close enough to the AR tags 
                if (self.AR_close == True):
                    self.prev_state = 'go_to_AR'
                    self.state = 'handle_AR'
                      
            elif (self.state == 'handle_AR'):
                print "handle that"
                orient_more = None

                orient_more, move_cmd = self.mover.close_orient(self.AR_q, self.AR_curr, self.orientation)
                if (orient_more is 'good'):
                    move_cmd = self.mover.stop()
                    rospy.sleep(10)
                    orient_more = 'back_out'
        
                elif (orient_more == 'back_out'):
                    move_cmd = self.mover.back_out()
                    self.obstacle_OFF = False
                    self.state = 'wander'
                    self.handle_AR_step = 0
                    self.prev_state = 'handle_AR'



                # self.handle_AR_step = self.handle_AR_step + 1
                # print "SSS %d" % self.handle_AR_step
                # move_cmd = self.mover.handle_AR(self.AR_q, self.AR_curr, self.handle_AR_step)
                # # pause for 10 seconds
                
                # if (self.handle_AR_step == 5):
                #     print "enter"
                #     self.obstacle_OFF = False
                #     self.state = 'wander'
                #     self.handle_AR_step = 0
                #     self.prev_state = 'handle_AR'
                    
                    

