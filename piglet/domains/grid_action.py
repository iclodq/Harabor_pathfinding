# domains/grid_action.py
# 
# describes a valid action in a grid domain and specifies its cost
# 
# @author: dharabor
# @created: 2020-07-15

import sys

MOVE_UP = 0
MOVE_LEFT = 1
MOVE_RIGHT = 2  
MOVE_DOWN = 3
MOVE_WAIT = 9

class grid_action:

    def __init__(self):
        self.move_ = MOVE_WAIT
        self.cost_ = 1

    def print(self):
        if(self.move_ == MOVE_UP):
            print("UP " + str(self.cost_))
        elif(self.move_ == MOVE_DOWN):
            print("DOWN " + str(self.cost_))
        elif(self.move_ == MOVE_LEFT):
            print("LEFT " + str(self.cost_))
        elif(self.move_ == MOVE_RIGHT):
            print("RIGHT " + str(self.cost_))
        else:
            print("WAIT " + str(self.cost_))
