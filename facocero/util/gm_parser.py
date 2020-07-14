# gm_parser
# 
# Reads and writes 2d grid maps
#
#
# @author: dharabor
# @created: 2020-07-14
#

import sys

class gm_parser:
    
    def __init__(self):
        self.map_ = []
        self.height_ = 0
        self.width_ = 0

    def load(self, filename):
        map_fo = open(filename, "r")

        print("parsing map")
        if(self.__parse_header(map_fo) == -1):
            sys.stderr.write("err; invalid map header");
            return
        print(
        self.map_ = [False] * (int(self.height_) * int(self.width_))
        i = 0;
        while(True):
            char = map_fo.read(1)
            print
            if not char:
                break
            if(char == '\n'):
                print()
                continue
            if(char == '.'):
                self.map_[i] = True
            else:
                self.map_[i] = False
            i += 1
        
    def write(self, filename):

        print("write function begin")
        print("type octile")
        print("height " + str(self.height_))
        print("width " + str(self.width_))
        print("map")
 
        for y in range(0, int(self.height_)):
            for x in range(0, int(self.width_)):
                if(self.map_[y*int(self.width_) + x] == True):
                    print('.', end="")
                else:
                    print('@', end="")
            print()

    def __parse_header(self, map_fo):

        tmp = map_fo.readline().strip().split(" ")
        if(tmp[0] != "type" and tmp[1] != "octile"):
            print("not octile map")
            return -1
    
        for i in range(0, 2):
            tmp = map_fo.readline().strip().split(" ")
            if tmp[0] == "height" and len(tmp) == 2:
                self.height_ = tmp[1]
            elif tmp[0] == "width" and len(tmp) == 2:
                self.width_ = tmp[1]
            else:
                return -1

        tmp = map_fo.readline().strip()
        if(tmp != "map"):
            return -1
