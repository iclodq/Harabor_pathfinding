# search/search_node.py
# 
# Data structure that represents a domain-independent search node
#
# @author: dharabor
# @created: 2020-07-15
#

import sys

def class search_node:
    self.action_
    self.state_
    self.parent_
    self.g_
    self.depth_
    self.instance_

    __init__(self):
        # some default values for uninitialised nodes
        self.action_ = False 
        self.state_ = False 
        self.parent_ = False 
        self.g_ = sys.maxint
        self.depth_ = sys.maxint
        self.instance_ = sys.maxint

    
    
