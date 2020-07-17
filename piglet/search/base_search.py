# search/base_search.py
# This module defines a base search class and what attribute/method a search class should have.
#
# @author: mike
# @created: 2020-07-15
#
import sys
from expanders.base_expander import base_expander
from search.search_node import search_node
from solution.solution import solution


class base_search:
    nodes_generated_: int = 0
    nodes_expanded_: int = 0
    runtime_: float = 0
    start_time_: float = 0
    time_limit_: int = sys.maxsize
    expander_: base_expander = None
    solution_: solution = None
    start_ = None
    goal_ = None

    def __init__(self):
        pass

    # Search the path between two state
    # @param start_state The start of the path
    # @param goal_state Then goal of the path
    # @return a list of locations between start and goal
    def get_path(self,start_state, goal_state):
        raise NotImplementedError()

    # Generate search_node objects for a given state
    # For this operatin we we need to know:
    # @param state: the state which the search node maps to
    # @param action: the action which generated the state (could be [None])
    # @param parent: the parent state (could be [None])
    def generate(self, state, action, parent: search_node):

        retval = search_node()
        retval.state_ = state
        retval.action_ = action
        if (parent == None):
            # initialise the node from scratch
            # NB: we usually do this only for the start node
            retval.g_ = 0
            retval.depth_ = 0
        else:
            # initialise the node based on its parent
            retval.g_ = parent.g_ + action.cost_
            retval.depth_ = parent.depth_ + 1
            retval.parent_ = parent
        return retval

    # extract the computed solution by following backpointers
    def solution(self, goal_node: search_node):
        tmp = goal_node
        depth = goal_node.depth_
        cost = goal_node.g_
        sol = []
        while (tmp != None):
            sol.append(tmp)
            tmp = tmp.parent_

        sol.reverse()
        return solution(sol,depth,cost)



