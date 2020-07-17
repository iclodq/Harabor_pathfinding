# search/tree_search.py
# 
# Implements the Tree-Search algorithm:
# Given an expander and an open list, this approach will 
# search until it finds the goal state.
# 
# The expansion order is determined by the type of open list.
# The task environment is determined by the expander
# 
# @author: dharabor
# @created: 2020-07-16
#
import time, sys
from search.base_search import base_search
from expanders.base_expander import base_expander


class tree_search(base_search):


    def __init__(self, open_list, expander:base_expander, time_limit: int = sys.maxsize):
        self.open_list_ = open_list
        self.expander_ = expander
        self.time_limit_ = time_limit

    # Search the path between two state
    # @param start_state The start of the path
    # @param goal_state Then goal of the path
    # @return a list of locations between start and goal
    def get_path(self,start_state, goal_state):
        self.start_ = start_state
        self.goal_ = goal_state
        self.start_time = time.process_time()
    
        start_node = self.generate(start_state, None, None)
        self.open_list_.push(start_node)

        # continue while there are still nods on OPEN
        while (len(self.open_list_) > 0):
            current = self.open_list_.pop()
            self.runtime_ = time.process_time() - self.start_time
            self.nodes_expanded_ +=1
            if self.runtime_ > self.time_limit_:
                print("Time out")
                break
            # goal test. if successful, return the solution
            if(current.state_ == goal_state):
                self.solution_ = self.solution(current)
                return self.solution_

            # expand the current node
            for succ in self.expander_.expand(current):
                # each successor is a (state, action) tuple which
                # which we map to a corresponding search_node and push
                # then push onto the OPEN list
                succ_node = self.generate(succ[0], succ[1], current)
                self.open_list_.push(succ_node)
                self.nodes_generated_+=1

        # OPEN list is exhausted and we did not find the goal
        # return failure instead of a solution
        return None

    # Print statistic information on screen
    def print_statistic(self):
        print(str(self))

    # Get statistic information
    # @return string Statistic information
    def get_statistic(self):
        return str(self)

    def __str__(self):
        str_ = "***************************\n"
        str_ += "Problem:\t{}\n".format(str(self.expander_))
        str_ += "Start:\t"
        str_ += str(self.start_)+"\n"
        str_ += "Goal:\t"
        str_ += str(self.goal_)+"\n"
        str_ += "Nodes expanded:\t{}\n".format(self.nodes_expanded_)
        str_ += "Nodes generated:\t{}\n".format(self.nodes_generated_)
        str_ += "Runtime:\t{}\n".format(self.runtime_)
        str_ += "***************************\n"

        return str_





