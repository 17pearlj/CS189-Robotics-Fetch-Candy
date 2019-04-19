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
print(" my var %.2f" % my_var)

