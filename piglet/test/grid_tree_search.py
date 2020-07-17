from domains import gridmap
from expanders.grid_expander import grid_expander
from search.tree_search import tree_search
from _collections import deque
import os

file_folder = os.path.dirname(os.path.abspath(__file__))
inputfile = os.path.join(file_folder, "gridmap/empty-16-16.map")

class queue(deque):
    def push(self,item):
        self.append(item)

gm = gridmap.gridmap()
gm.load(inputfile)
open_list = queue()

expander = grid_expander(gm)
search = tree_search(open_list, expander)
path = search.get_path((1,2),(10,2))
print(path)
search.print_statistic()



