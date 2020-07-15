# domains/grid_expander.py
# 
# Expand function for the 4-connected gridmap domain.
#
# Given a current search node, the expander checks the set of valid grid actions 
# and generates search node successors for each.
#
# @author: dharabor
# @created: 2020-07-15
#

class gridmap_4c_expander:

    self.gridmap_

    def __init__(self, gridmap):
        self.gridmap_ = gridmap

    def expand(self, current):
    
        index = y*self.width_ + x

    
