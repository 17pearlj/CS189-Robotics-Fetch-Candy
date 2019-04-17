if __name__ == '__main__':

    my_dict = {
        "id1": [1, 2, 3], "id2": [1, 2, 3], "id3": [1, 2, 3]
        }
        
    print my_dict
    hello = all(x[2] != 3 for x in my_dict.values())
    print hello

    i =[0,1,]
    for i[2] in my_dict.items():
        if i[2] == 3:
            print i[2]
