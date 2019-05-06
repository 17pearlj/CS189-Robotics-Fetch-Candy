import cool_math as cm
import math
import numpy

if __name__ == '__main__':
    h = [0,1,2,3,4]
    tlist = ['a', 'b', 'c', 'd', 'e']
    jlist = [1, 3, 7, 3, 2]
    test_dict = {}
#     # my_dict = {
#     #     "id1": [1, 2, 3], "id2": [1, 2, 3], "id3": [1, 2, 3]
#     #     }
        
#     # print my_dict
#     # hello = all(x[2] != 3 for x in my_dict.values())
#     # print hello

#     # for tnum in h:
#     #     test_dict[tnum] = [tlist[tnum], jlist[tnum]]
#     # #print test_dict

#     # value = [None] * 2
#     # sub_dict = {}
#     # for value in test_dict.values():
#     #     if value[1] == 3:
#     #         print 0
            

#     # sub_list = []
#     # for item in test_dict.items():
#     #     if (item[1][1] == 3):
#     #         #sub_dict.update(test_dict[item]) 
#     #         # use list of lists instead
#     #         sub_list.append([item])
    
#     # print sub_list
 
# #     pos1 = (3, 20, 1)
# #     pos2 = (3,4,5)
# #     pos3 = (1,1,1)
# #     orr = 1
# #     key = 12
# #     key1 = 13
# #     key2 =14
# #     my_pos = (0,0)
    
# #  # new list of all the distances in the list of ARTags that have not been visited 
# #     unvisited = [(key, [pos1, orr, 'hello']),
# #                 (key1, [pos2, orr, 'hello']),
# #                 (key2, [pos3, orr, 'hello'])]
# #     close = []
# #     for i in unvisited:
# #         pos = (i[1][0][0] , i[1][0][1])
# #         curr_dist = cm.dist_btwn(pos, my_po0s)
# #         close.append([i, curr_dist])

# #     # actually reorder from smallest to largest distance 
# #     close.sort(reverse = True)

# #     #print close
    
# #     # close is sorted smallest to large, but its index in unvisited was saved 
# #     print "the one:"
# #     the_key = close[0][0][0]
# #     print the_key
    
# #     my_dict = dict(unvisited)

# #     print "the chosen one:"
# #     print my_dict.get(the_key)
    

#     # returns [(1, 1, 1), 1, 'hello']
# # my_var = 18.456789
# # print("my var %.2f" % my_var)


# # get a side given two sides and one angle
# def third_side(a, b, gamma):
#     """
#     returns the third side of a triangle given two sides and one angle 
#     """
#     return math.sqrt(a**2 + b**2 - (2 * a * b * math.cos(gamma)))

# # alpha dist starts increasing??           
# # 0.34
# # arz - 0.90
# #alpha_dist = cm.third_side(self.ar_z, ll_dist, beta)

# # decreaseses as z decreases 
# # print cm.third_side(0.9, 0.5, math.radians(89))
# # print cm.third_side(0.8, 0.5, math.radians(89))
# # print cm.third_side(0.7, 0.5, math.radians(89))
# # print cm.third_side(0.6, 0.5, math.radians(89))

# # decreaseses as z decreases 
# # print cm.third_side(0.25, 0.5, math.radians(180))
# # print cm.third_side(0.25, 0.5, math.radians(170))
# # print cm.third_side(0.25, 0.5, math.radians(10))
# # print cm.third_side(0.25, 0.5, math.radians(0))
# # print cm.third_side(0.001, 0.5, math.radians(0.001))

# # print cm.third_side(0.5, 0.5, math.radians(30))
# # print cm.third_side(0.5, 0.5, math.radians(20))
# # print cm.third_side(0.5, 0.5, math.radians(15))
# # print cm.third_side(0.5, 0.5, math.radians(1))



# # def get_angle_ab(a, b, c):
# #     """
# #     returns angle in radians that is between a and b given three sides 
# #     """
# #     if (a != 0 and b != 0 ):
# #         top = (a**2 + b**2) - c**2
# #         if top > 0:
# #             bottom = 2 * a * b
# #             both = top/bottom
# #             if abs(both) > 1:
# #                 both = 1
# #                 return math.acos(both)
# #             else:
# #                 return math.acos(both)

# #         else:
# #             return -222
# #     else:
#             return -222

# # cm.get_angle_ab(self.ar_z, alpha_dist, ll_dist)

# #print math.degrees(get_angle_ab(0.5, 0.5, 0.1))




# # old code

# # while (state == 'wander'):
# #                 # just wandering around 
# #                 move_cmd = mover.wander()
            
# #                 # current location will always be free :)
# #                 mapper.updateMapFree(position)
                
# #                 # # this info will come from depth senor processing
# #                 if (obstacle_seen == True):
# #                     mapper.updateMapObstacle()

# #                 # # map the ARTAG using info from ARTAG sensor stored in self
# #                 # elif (AR_seen == True): 
# #                 #     mapper.updateMapAR()

# #                 # if there are ARTags that have not yet been visited, choose one to visit 
# #                 if (len(AR_q) is not 0 and all(x[2] == 'unvisited' for x in AR_q.values())):
# #                     AR_curr = mover.choose_AR(AR_q) 
# #                     print "CURRENT AR TAG:"
# #                     print AR_curr
# #                     # robot will now go to AR tag 
# #                     if AR_curr is not -1:
# #                         prev_state = 'wander'
# #                         state = 'go_to_AR'
# #                     else:
# #                         state = 'wander'

# #                 cmd_vel.publish(move_cmd)
# #                 rate.sleep()
                    

# #             # zero in on an ARTag 
# #             elif (state == 'go_to_AR'):
# #                 move_cmd, AR_close, obstacle_OFF = mover.go_to_AR(AR_q, AR_curr, orientation)
# #                 # only want to do the ARTag procedure when we are close enough to the AR tags 
# #                 if (AR_close == True):
# #                     prev_state = 'go_to_AR'
# #                     state = 'handle_AR'
                      
# #             elif (state == 'handle_AR'):
# #                 print "handle that"
# #                 orient_more = None

# #                 orient_more, move_cmd = mover.close_orient(AR_q, AR_curr, orientation)
# #                 if (orient_more is 'good'):
# #                     move_cmd = mover.stop()
# #                     rospy.sleep(10)
# #                     orient_more = 'back_out'
        
# #                 elif (orient_more == 'back_out'):
# #                     move_cmd = mover.back_out()
# #                     obstacle_OFF = False
# #                     state = 'wander'
# #                     handle_AR_step = 0
# #                     prev_state = 'handle_AR'



# #                 # handle_AR_step = handle_AR_step + 1
# #                 # print "SSS %d" % handle_AR_step
# #                 # move_cmd = mover.handle_AR(AR_q, AR_curr, handle_AR_step)
# #                 # # pause for 10 seconds
                
# #                 # if (handle_AR_step == 5):
# #                 #     print "enter"
# #                 #     obstacle_OFF = False
# #                 #     state = 'wander'
# #                 #     handle_AR_step = 0
# #                 #     prev_state = 'handle_AR'
                    
                    
# # theta = radians(44)
# # ar_x = 0.5
# # ar_z = 0.1
# # def run():
# #         """
# #         - Control the state that the robot is currently in 
# #         - Run until Ctrl+C pressed
# #         :return: None
# #         """
# #         count = 0
# #         # constant goal dist from robot, parallel distance 
# #         ll_dist = 0.5 #m
# #         # arrays to save the orientation at a certain distance 
# #         past_orr = []
# #         past_orr1 = []

# #         while not rospy.is_shutdown(): 
# #             #  local twist object will be shared by all the states 
# #             #rint degrees(ar_orientation)
# #             # move_cmd = Twist()

            
# #             K_rot = 0.05
# #             xK_rot = 2
# #             xAcc = 0.05


# #             if state is 'searching':
# #                 # go to next state on next loop through 
# #                 # orientation of ar_tag wrt robot
# #                 theta = abs(ar_orientation)
# #                 beta = abs(radians(180) - theta)
# #                 state = 'zerox'

# #             if state is 'zerox':
# #                 if abs(ar_x) > xAcc:
# #                     print("x: %.2f" % ar_x)
# #                     # need to change so it twists in the shortest way - proportional control later
# #                     # if ar_x > 0:)
# #                     print("twist velocity:")
# #                     print -1*xK_rot*ar_x 
# #                     #execute_command(mover.twist(-1*xK_rot*ar_x))
# #                     # elif ar_x < 0:
# #                     #     execute_command(mover.twist(-1*K_rot*ar_x)
# #                 else:
# #                     print "zeroed x"
# #                     state = 'turn_alpha'
            
# #             elif state is 'turn_alpha':
# #                 print "in turn_alpha"
# #                 print("beta: %.2f" % degrees(beta))
# #                 alpha_dist = cm.third_side(ar_z, ll_dist, beta) 
# #                 print("alpha_dist in turn_alpha: %.2f" % alpha_dist)
# #                 alpha = cm.get_angle_ab(ar_z, alpha_dist, ll_dist)
# #                 print("alpha: %.2f" % degrees(alpha))
# #                 if alpha_dist <= 0.5:
# #                     print "dont need to turn - alpha_dist is very low"
# #                     state = 'move_alpha'
# #                 elif alpha is -1000:
# #                     print "dont need to turn alpha is none"
# #                     state = 'move_alpha'
# #                 else: 
# #                     past_orr.append(orientation)
# #                     dif =  abs(orientation - past_orr[0])
# #                     print("dif: %.2f" % degrees(dif))
# #                     dif2go = abs(alpha - dif)

# #                     # rotate until angled 'alpha' to robot
# #                     if dif2go < radians(3):
                        
# #                         print("dif2go: %.2f" % degrees(dif2go))
# #                         print("twist velocity:")
# #                         print 4*K_rot*dif2go
# #                         #execute_command(mover.twist(4*K_rot*dif2go))
# #                     else:
# #                         # move to next state when this has happened -- no check?
# #                         print "dont need to turn much - go to move_alpha"
# #                         #execute_command(mover.stop())
# #                         state = 'move_alpha'
            
# # # 0.34
# # # arz - 0.90
# #             elif state == 'move_alpha':
# #                 print "innnnn move_alpha"
# #                 print("beta: %.2f" % degrees(beta))
# #                 print("ar_z: %.2f" % ar_z)
                
# #                 alpha_dist = cm.third_side(ar_z, ll_dist, beta)
# #                 print("alpha_dist in move_alpha: %.2f" % alpha_dist)
# #                 if alpha_dist > 0.05: 
# #                     #execute_command(mover.go_forward_K(alpha_dist))
# #                 elif abs(ar_x) > xAcc:
# #                     print "back to zeroing x "
# #                     state = 'zerox'
# #                 else: 
# #                     print "moving forward to artag"
# #                     state = "move_perf"
            
# #             elif state == 'move_perf':
# #                 # move to the ar tag 
# #                 print state
# #                 print("ar_z: %.2f" % ar_z)
# #                 if ar_z > 0.25:
# #                     #execute_command(mover.go_forward_K(ar_z))
# #                 else:
# #                     print "park_it"
# #                     state = "park_it"
            
# #             elif state == "park_it":
# #                 # wait to recieve package 
# #                 #execute_command(mover.stop())
# #                 print "slee[ing"
# #                 count+=1
# #                 print count
# #                 rospy.sleep(1)
# #                 if count > 10:
# #                     print "back out"
# #                     state = "back out"

# #             elif state == "back out":
# #                     # backout 
# #                     #execute_command(mover.back_out())
# #                     print ar_z
# #                     if ar_z > 0.7:
# #                         state = 'done'

# #             elif state == "done":
# #                     #execute_command(mover.stop())
# #                     print "done parking :)"



# # count2 = count2 +  1
# # count3 = count2 % 20

# count = 0 
# while True:
#     count+=1 
#     count = count % 20
#     print count


# print statements we like

# print("x: %.2f" % self.ar_x)
# print("twist velocity:")
# print -1*xK_rot*self.ar_x 

# print("beta before turning: %.2f" % degrees(beta)) # beta should be 0 or close to it on second time through 
#                     print("self.ar_z: %.2f" % self.ar_z) # arz should be close to 0.5 which is current ll_dist 
#                     print("alpha_dist in turn_alpha: %.2f" % alpha_dist) 
#                     print("degrees alpha: %.2f" % degrees(alpha))
#                     print("regular alpha: %.2f" % alpha)

def valid_list1(my_list, num):
    """
    returns True if the last num numbers in a list are different 
    """
    if num > len(my_list) + 1:
        help_list_sz = num - len(my_list)
        help_list = [0] * help_list_sz
        my_list = help_list + my_list
    my_sum = sum( my_list[-num: len(my_list) + 1])
    avg = my_sum/len(my_list)
    # list is not valid - not made of values that are being updated 
    if avg == my_list[-1]:
        return False
    else:
        return True

def valid_list(my_list, num):
    """
    returns True if the last num numbers in a list are different 
    """
    if num > len(my_list):
        num = len(my_list) - 1
    my_sum = sum( my_list[-num: len(my_list) + 1])
    avg = my_sum/len(my_list)
    # list is not valid - not made of values that are being updated 
    if avg == my_list[-1]:
        return False
    else:
        return True


a_list = list(numpy.linspace(1, 1.005))
b_list = [1,1,1]
# print a_list
print valid_list(a_list, 52)
