# solution/solution.py
# This module defines a base search class and what attribute/method a search class should have.
#
# @author: mike
# @created: 2020-07-16
#


class solution:
    cost_: int
    depth_: int
    paths_: list

    def __init__(self, path: list, depth: int, cost: int):
        self.cost_ = cost
        self.depth_ = depth
        self.paths_ = path

    def __str__(self):
        return "Solution:\t{} \nSolution cost:\t{} \nSolution depth:\t{}\n".format(self.paths_,self.cost_,self.depth_)

    def __repr__(self):
        return self.__str__()
