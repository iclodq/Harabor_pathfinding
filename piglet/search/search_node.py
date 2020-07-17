# search/search_node.py
# 
# Data structure that represents a domain-independent search node
#
# @author: dharabor
# @created: 2020-07-15
#

import sys

class search_node:
    action_ = None
    state_ = None
    parent_ = None
    g_ = None
    depth_ = None
    instance_ = None

    def __init__(self):
        # some default values for uninitialised nodes
        self.action_ = False 
        self.state_ = False 
        self.parent_ = None
        self.g_ = sys.maxsize
        self.depth_ = sys.maxsize
        self.instance_ = sys.maxsize

    def __str__(self):
        return str(self.state_)

    def __repr__(self):
        return self.state_.__repr__()

    
