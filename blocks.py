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
        # cur_pos = state.position
        successors = []
        for position in state.positions.keys():
            v_pos, h_pos = position
            if state.grid[v_pos, h_pos + 1] == ' ':  # can move to the right
                suc = state.get_successor(position, (v_pos, h_pos + 1))
                successors.append(suc)
            if state.grid[v_pos, h_pos - 1] == ' ':  # can move to the left
                suc = state.get_successor(position, (v_pos, h_pos - 1))
                successors.append(suc)
        return successors

    def goal_test(self, state):
        return state.remaining_targets == 0 if state.remaining_targets != -1 else state.get_score() == 0


###############
# State class #
###############
class State:
    def __init__(self, grid, pos=None, remaining_targets=-1):
        self.nbr = len(grid)
        self.nbc = len(grid[0])
        self.grid = grid
        if remaining_targets != -1:
            self.remaining_targets = remaining_targets
        else:
            self.remaining_targets = self.get_remaining_targets()
        if pos is None:
            self.positions = self.get_positions()

    def get_positions(self):
        positions = {}
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                if self.grid[row, col] != '#' and self.grid[row, col] != ' ':
                    # positions.append([self.grid[row, col], [row, col]])
                    positions[(row, col)] = self.grid[row, col]
        return positions

    def get_successor(self, old_pos, move):
        successor = None
        old_v, old_h = old_pos
        move_v, move_h = move

        # Duplicate self.grid in new_grid
        new_grid = []
        for row in self.grid:
            new_grid.append(row[:])

        # Duplicate positions in new positions
        new_positions = dict(self.positions)  # self.positions[:]

        i = 0

        # Check until if the block will fall
        if self.grid[move_v - 1][move_h] == ' ':
            while self.grid[move_v - 1 - i][move_h] == ' ':  # Find vertical position after the block has fallen
                i += 1

        # Updating new_grid with new_pos of block
        target_places = goal_state.positions
        target_place_reached = False
        if (move_v - i, move_h) in target_places:
            new_grid[move_v - i][move_h] = '@'
            target_place_reached = True
        else:
            new_grid[move_v - i][move_h] = new_grid[old_v][old_h]
        new_grid[old_v][old_h] = ' '

        # Updating positions with new positions
        new_positions[move] = new_positions[old_pos]
        del new_positions[old_pos]

        # if self.grid[move_v][move_h] == ' ' and self.grid[move_v - 1][move_h] != ' ':  # The block does NOT fall
        #
        #     # Updating new_grid with new_pos of block
        #     new_grid[move_v][move_h] = new_grid[old_v][old_h]
        #     new_grid[old_v][old_h] = ' '
        #
        #     # Updating positions with new positions
        #     new_positions[move] = new_positions[old_pos]
        #     del new_positions[old_pos]
        #     # for i in range(len(new_positions)):
        #     #     if new_positions[i][0] == block and new_positions[i][1] == old_pos:
        #     #         new_positions[i] = [block, move]
        #
        # elif self.grid[move_v][move_h] == ' ' and self.grid[move_v - 1][move_h] == ' ':  # The block will fall
        #
        #     # Check until which vertical position the block will fall
        #     i = 0
        #     while self.grid[move_v - 1 - i][move_h] == ' ':
        #         i += 1
        #
        #     # Updating new_grid with new_pos of block
        #     new_grid[move_v - i][move_h] = new_grid[old_v][old_h]
        #     new_grid[old_v][old_h] = ' '
        #
        #     # Updating positions with new positions
        #     new_positions[move] = new_positions[old_pos]
        #     del new_positions[old_pos]
        #     # for i in range(len(new_positions)):
        #     #
        #     #     if new_positions[i][0] == block and new_positions[i][1] == old_pos:
        #     #         new_positions[i] = [block, [move_v - i, move_h]]

        # Applying gravity to make potential block on top of old_pos fall ...
        self.apply_gravity_to_others(new_grid, new_positions, old_pos)

        # Generating the successor

        if target_place_reached:
            successor = State(new_grid, pos=new_positions, remaining_targets=self.remaining_targets - 1)
        else:
            successor = State(new_grid, pos=new_positions, remaining_targets=self.remaining_targets)
            
        return successor


    def apply_gravity_to_others(self, new_grid, new_positions, old_pos):
        old_v_pos, old_h_pos = old_pos
        for i in range(old_v_pos - 1, -1, -1):
            if new_grid[i][old_h_pos] == ' ' or new_grid[i][old_h_pos] == '#':
                break
            else:
                new_grid[i+1][old_h_pos] = new_grid[i][old_h_pos]

                new_positions[(i+1, old_h_pos)] = new_positions[(i, old_h_pos)]
                del new_positions[(i, old_h_pos)]
        return

    def get_remaining_targets(self):
        target_places = goal_state.positions
        counter = len(target_places)
        for row, col in target_places.keys():
            if self.grid[row][col] == '@':
                counter -= 1
        return counter

    def get_score(self):
        score = 0
        target_places = goal_state.positions
        for position in target_places.keys():
            v_pos, h_pos = position
            dist = self.get_closest_block_distance(target_places[position], h_pos)
            score += dist
        return score

    def get_closest_block_distance(self, target, h_pos):
        best = self.nbc
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                if self.grid[row, col] == target.lower() and abs(col - h_pos) < best:
                    best = abs(col - h_pos)
        return best

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

    def __hash__(self):
        return hash(str(self.grid))

    def __eq__(self, other):
        return self.grid == other.grid

    def __ne__(self, other):
        # Not strictly necessary, but to avoid having both x==y and x!=y
        # True at the same time
        # This method comes form the internet
        return not (self == other)


######################
# Auxiliary function #
######################
def readInstanceFile(filename):
    grid_init, grid_goal = map(lambda x: [[c for c in l.rstrip('\n')[1:-1]] for l in open(filename + x)],
                               [".init", ".goalinfo"])
    return grid_init[1:-1], grid_goal[1:-1]


######################
# Heuristic function #
######################
def heuristic(node):
    # h = 0.0
    state = node.state
    h = float(state.get_score())
    # ...
    # compute an heuristic value
    # ...
    return h


##############################
# Launch the search in local #
##############################
# Use this block to test your code in local
# Comment it and uncomment the next one if you want to submit your code on INGInious
instances_path = "instances/"
instance_names = ['a01', 'a02', 'a03', 'a04', 'a05', 'a06', 'a07', 'a08', 'a09', 'a10', 'a11']

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
    print("* Queue size at goal:\t", remaining_nodes)

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
