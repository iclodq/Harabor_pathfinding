# domains/n_puzzle.py
# This module implements a n_puzzle domain
#
# @author: mike
# @created: 2020-07-16

import math
from enum import IntEnum


class Puzzle_Actions(IntEnum):
    SWAP_UP = 0
    SWAP_LEFT = 1
    SWAP_RIGHT = 2
    SWAP_DOWN = 3
    START = -1

class puzzle_action:
    move_: int
    cost_: int
    def __init__(self, action: int, cost:int):
        self.move_ = action
        self.cost_ = cost


class puzzle_state:
    state_list_: list
    from_action_: int = -1
    x_index_: int

    def __init__(self, alist: list, x_index: int, from_action: int = Puzzle_Actions.START):
        self.state_list_ = alist
        self.from_action_ = from_action
        self.x_index_ = x_index

    def __eq__(self, other):
        return self.state_list_ == other.state_list_

    def __str__(self):
        width = math.sqrt(len(self.state_list_))
        str_ = "\n"
        for i in range(1,len(self.state_list_)+1):
            str_ += str(self.state_list_[i-1])
            if i%width == 0:
                str_ += "\n"
            else:
                str_ += "\t"
        return str_

    def __repr__(self):
        return Puzzle_Actions(self.from_action_).name



class n_puzzle:
    width_: int
    size_: int
    goal_: puzzle_state
    start_: puzzle_state
    swap_offset: list
    domain_file_: str = "Unknown"

    # Load a problem from a file
    # @param filename The path to a puzzle file
    def load(self, filename: str):
        self.domain_file_ = filename
        with open(filename) as file:

            type_line = file.readline().strip().split(" ")
            if len(type_line) < 2 or type_line[0] != "type" or type_line[1] != "n-puzzle":
                raise Exception("File type is not n-puzzle")
            width_line = file.readline().strip().split(" ")
            if len(width_line) < 2 or width_line[0] != "width":
                raise Exception("Can't read puzzle width")
            try:
                self.width_ = int(width_line[1])
                self.size_ = self.width_*self.width_
            except:
                raise Exception("Can't read puzzle width")

            puzzle_list = self.__parse_puzzle(file)

            try:
                x_index = puzzle_list.index("x")
            except:
                raise Exception('Cannot find white space "x" in the puzzle')

            self.start_ = puzzle_state(puzzle_list, x_index)
            goal_list = list(range(1, self.width_*self.width_)) + ["x"]
            self.goal_ = puzzle_state(goal_list, self.width_-1)
            if not self.is_solvable():
                raise Exception("The given puzzle is insolvable!")

        self.__init_swap_offset()


    # @return puzzle_state The start state of the n-puzzle
    def start_state(self):
        return self.start_

    # @return puzzle_state The goal state of the n-puzzle
    def goal_state(self):
        return self.goal_

    # @return bool True if the puzzle is solvable
    def is_solvable(self):
        inversions = self.__get_inversion()
        x_row_bottom = self.width_ - self.start_state().x_index_//self.width_
        if self.width_%2 == 0: #even
            return x_row_bottom%2 != inversions%2
        else: #odd
            return inversions%2 == 0


    # Return is the new index/location valid
    # @param old_index The old index for x(white space)
    # @param new_index The new index for x(white space)
    # @return bool True if new_index is valid
    def is_valid_move(self,old_index:int, new_index:int):
        if new_index < 0 or new_index >= self.width_* self.width_:
            return False
        curr_x = old_index // self.width_
        curr_y = old_index % self.width_
        next_x = new_index // self.width_
        next_y = new_index % self.width_
        return abs(next_x - curr_x) + abs(next_y - curr_y) < 2


    def __str__(self):
        return self.domain_file_

    def __parse_puzzle(self, file):
        puzzle_list = []
        for i in range(0, self.width_):
            line = file.readline().strip().strip(",").split(",")
            if len(line) != self.width_:
                raise Exception("The width of puzzle line {} not equal to puzzle width".format(i))
            for char in line:
                if char.isnumeric():
                    num = int(char)
                    if num <= 0 or num >= self.size_:
                        raise Exception("Number {} not in range 1~{}".format(num, self.size_ - 1))
                else:
                    num = "x"

                if num in puzzle_list:
                    raise Exception("You can't have two {} in one puzzle".format(num))
                puzzle_list.append(num)
        return puzzle_list

    def __init_swap_offset(self):
        self.swap_offset = [None]*4
        self.swap_offset[Puzzle_Actions.SWAP_UP] = -1*self.width_
        self.swap_offset[Puzzle_Actions.SWAP_DOWN] = self.width_
        self.swap_offset[Puzzle_Actions.SWAP_LEFT] = -1
        self.swap_offset[Puzzle_Actions.SWAP_RIGHT] = 1

    def __get_inversion(self):
        count = 0
        for i in range(0,self.size_):
            if self.start_state().state_list_[i] =="x":
                continue
            for j in range(i+1,self.size_):
                if self.start_state().state_list_[j] == "x":
                    continue
                if self.start_state().state_list_[i] > self.start_state().state_list_[j]:
                    count += 1
        return count




