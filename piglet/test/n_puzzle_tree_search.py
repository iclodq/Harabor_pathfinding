from domains import n_puzzle
from expanders.n_puzzle_expander import n_puzzle_expander
from search.tree_search import tree_search
from _collections import deque
import os

file_folder = os.path.dirname(os.path.abspath(__file__))
inputfile = os.path.join(file_folder, "n_puzzle/sample_9_puzzle")

class queue(deque):
    def push(self,item):
        self.appendleft(item)


puzzle = n_puzzle.n_puzzle()
puzzle.load(inputfile)

open_list = queue()

expander = n_puzzle_expander(puzzle)
search = tree_search(open_list, expander)
path = search.get_path(puzzle.start_state(), puzzle.goal_state())
search.print_statistic()
print(path)
