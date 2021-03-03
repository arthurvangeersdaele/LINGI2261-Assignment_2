# -*-coding: utf-8 -*
'''NAMES OF THE AUTHOR(S): Gaël Aglin <gael.aglin@uclouvain.be>'''
from search import *
import sys
import time

goal_state = None
#################
# Problem class #
#################
class Blocks(Problem):

    def successor(self, state):
        cur_pos = state.position
        successors = []
        

    def goal_test(self, state):
        


###############
# State class #
###############
class State:
    def __init__(self, grid):
        self.nbr = len(grid)
        self.nbc = len(grid[0])
        self.grid = grid

    def __str__(self):
        n_sharp = self.nbc + 2
        s = ("#" * n_sharp) + "\n"
        for i in range(self.nbr):
            s += "#"
            for j in range(self.nbc):
                s = s + str(self.grid[i][j])
            s += "#"
            if i < self.nbr - 1:
                s += '\n'
        return s + "\n" + "#" * n_sharp


######################
# Auxiliary function #
######################
def readInstanceFile(filename):
    grid_init, grid_goal = map(lambda x: [[c for c in l.rstrip('\n')[1:-1]] for l in open(filename + x)], [".init", ".goalinfo"])
    return grid_init[1:-1], grid_goal[1:-1]

######################
# Heuristic function #
######################
def heuristic(node):
    h = 0.0
    # ...
    # compute an heuristic value
    # ...
    return h

##############################
# Launch the search in local #
##############################
#Use this block to test your code in local
# Comment it and uncomment the next one if you want to submit your code on INGInious
instances_path = "instances/"
instance_names = ['a01','a02','a03','a04','a05','a06','a07','a08','a09','a10','a11']

for instance in [instances_path + name for name in instance_names]:
    grid_init, grid_goal = readInstanceFile(instance)
    init_state = State(grid_init)
    goal_state = State(grid_goal)
    problem = Blocks(init_state)

    # example of bfs tree search
    startTime = time.perf_counter()
    node, nb_explored, remaining_nodes = depth_first_graph_search(problem)
    endTime = time.perf_counter()

    # example of print
    path = node.path()
    path.reverse()

    print('Number of moves: ' + str(node.depth))
    for n in path:
        print(n.state)  # assuming that the __str__ function of state outputs the correct format
        print()
    print("* Execution time:\t", str(endTime - startTime))
    print("* Path cost to goal:\t", node.depth, "moves")
    print("* #Nodes explored:\t", nb_explored)
    print("* Queue size at goal:\t",  remaining_nodes)



# ####################################
# # Launch the search for INGInious  #
# ####################################
# #Use this block to test your code on INGInious
# instance = sys.argv[1]
# grid_init, grid_goal = readInstanceFile(instance)
# init_state = State(grid_init)
# goal_state = State(grid_goal)
# problem = Blocks(init_state)

# # example of bfs graph search
# startTime = time.perf_counter()
# node, nb_explored, remaining_nodes = astar_graph_search(problem, heuristic)
# endTime = time.perf_counter()

# # example of print
# path = node.path()
# path.reverse()

# print('Number of moves: ' + str(node.depth))
# for n in path:
#     print(n.state)  # assuming that the __str__ function of state outputs the correct format
#     print()
# print("* Execution time:\t", str(endTime - startTime))
# print("* Path cost to goal:\t", node.depth, "moves")
# print("* #Nodes explored:\t", nb_explored)
# print("* Queue size at goal:\t",  remaining_nodes)