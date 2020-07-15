# domains/grid_action.py
# 
# describes a valid action in a grid domain and specifies its cost
# 
# @author: dharabor
# @created: 2020-07-15

import sys

class grid_action:
    
    UP = 0
    LEFT = 1
    RIGHT = 2  
    DOWN = 3
    WAIT = 9

    def __init__(self):
        self.move_ = grid_action.WAIT
        self.cost_ = 1

