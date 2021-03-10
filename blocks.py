# -*-coding: utf-8 -*
"""NAMES OF THE AUTHOR(S): Gaël Aglin <gael.aglin@uclouvain.be>"""
from search import *
import sys
import time
#import numpy as np
import multiprocessing

goal_state = None
max_score = -1


#################
# Problem class #
#################
class Blocks(Problem):

    def successor(self, state):
        # cur_pos = state.position
        successors = []
        for position, block in state.positions.items():
            v_pos, h_pos = position
            if state.grid[v_pos][h_pos] == '@':
                del state.positions
                continue
            if (h_pos + 1 < state.nbc) and state.grid[v_pos][h_pos + 1] == ' ':  # can move to the right
                suc = state.get_successor(position, (v_pos, h_pos + 1))
                if suc is not None:
                    successors.append(("right", suc))
            if (h_pos - 1 >= 0) and state.grid[v_pos][h_pos - 1] == ' ':  # can move to the left
                suc = state.get_successor(position, (v_pos, h_pos - 1))
                if suc is not None:
                    successors.append(("left", suc))
        return successors

    def goal_test(self, state):
        return state.remaining_targets == 0  # if state.remaining_targets != -1 else state.get_score() == 0


###############
# State class #
###############
def apply_gravity_to_others(new_grid, new_positions, old_pos):
    old_v_pos, old_h_pos = old_pos
    target_places = goal_state.positions
    for i in range(old_v_pos - 1, -1, -1):
        if new_grid[i][old_h_pos] in [' ', '#', '@']:
            break
        else:
            target_place_reached = False
            if (i + 1, old_h_pos) in target_places and target_places[(i + 1, old_h_pos)] == (
                    new_grid[i][old_h_pos]).upper():
                new_grid[i + 1][old_h_pos] = '@'
                target_place_reached = True
            else:
                new_grid[i + 1][old_h_pos] = new_grid[i][old_h_pos]
            new_grid[i][old_h_pos] = ' '

            if not target_place_reached:
                new_positions[(i + 1, old_h_pos)] = new_positions[(i, old_h_pos)]
            del new_positions[(i, old_h_pos)]
    return


class State:
    def __init__(self, grid, pos=None, remaining_targets=-1, is_goal_state=False):
        self.nbr = len(grid)
        self.nbc = len(grid[0])
        self.grid = grid
        if not is_goal_state:
            if remaining_targets != -1:
                self.remaining_targets = remaining_targets
            else:
                self.remaining_targets = goal_state.remaining_targets  # self.get_remaining_targets()
        else:
            self.remaining_targets = self.goal_state_get_remaining_targets()
            global max_score
            max_score = (self.nbc * self.nbr) ** 4
        if pos is None:
            self.positions = self.get_positions()
        else:
            self.positions = pos

    def get_positions(self):
        positions = {}
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                if self.grid[row][col] != '#' and self.grid[row][col] != ' ':
                    # positions.append([self.grid[row, col], [row, col]])
                    positions[(row, col)] = self.grid[row][col]
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
        if (move_v + 1) < self.nbr and self.grid[move_v + 1][move_h] == ' ':
            while (move_v + 1 + i) < self.nbr and self.grid[move_v + 1 + i][move_h] == ' ':
                # Find vertical position after the block has fallen
                i += 1

        # CONDITION: (self.grid[8][5] == 'b' and self.grid[6][6] == 'b' and move_h == 5 and move_v == 6 and i == 1)
        # or (self.grid[8][3] == 'a' and self.grid[6][3] == 'a' and move_h == 3 and move_v == 3 and i == 1) or
        # self.grid[7][3] == '@' or self.grid[7][5] == '@' Updating new_grid with new_pos of block
        target_places = goal_state.positions
        target_place_reached = False
        if (move_v + i, move_h) in target_places and target_places[(move_v + i, move_h)] == (
                new_grid[old_v][old_h]).upper():
            new_grid[move_v + i][move_h] = '@'
            target_place_reached = True
        else:
            new_grid[move_v + i][move_h] = new_grid[old_v][old_h]
        new_grid[old_v][old_h] = ' '

        # Updating positions with new positions
        if not target_place_reached:
            new_positions[(move_v + i, move_h)] = new_positions[old_pos]
        del new_positions[old_pos]

        # Applying gravity to make potential block on top of old_pos fall ...
        apply_gravity_to_others(new_grid, new_positions, old_pos)

        # Generating the successor

        if target_place_reached and self.remaining_targets > 0:
            successor = State(new_grid, pos=new_positions, remaining_targets=self.remaining_targets - 1)
            # if (sys.gettrace() is not None) or (len(sys.argv) > 1 and sys.argv[1] in ['astar', 'bfs_graphs']):
            #     from tabulate import tabulate
            #     print(tabulate(np.array(new_grid)), "\n")
        else:
            successor = State(new_grid, pos=new_positions, remaining_targets=self.remaining_targets)

        return successor if successor.get_score() != -1 else None

    def goal_state_get_remaining_targets(self):
        counter = 0
        for row in self.grid:
            for slot in row:
                if slot not in [' ', '#']:
                    counter += 1
        return counter

    # def get_remaining_targets(self):
    #     target_places = goal_state.positions
    #     counter = len(target_places)
    #     for row, col in target_places.keys():
    #         if self.grid[row][col] == '@':
    #             counter -= 1
    #     return counter

    def get_heuristic_score(self):
        tmp = self.get_score()
        return tmp if tmp != -1 else max_score

    def get_score(self):
        score = 0
        for position, block in goal_state.positions.items():
            v_pos, h_pos = position
            if self.grid[v_pos][h_pos] != '@':
                dist = self.get_closest_block_distance(block, h_pos, v_pos)
                if dist == -1:
                    return -1
                score += dist
        return score

    def get_closest_block_distance(self, target, h_pos, v_pos):
        best = -1  # self.nbc
        for (row, col), block in self.positions.items():
            if block == target.lower() and row <= v_pos and ((best == -1) or abs(col - h_pos) < best):
                if abs(col - h_pos) > 0:
                    best = abs(col - h_pos)  # if abs(col - h_pos) > 0 else 2
                elif best == -1:
                    best = 1
                else:
                    continue
        # for row in range(len(self.grid)):
        #     for col in range(len(self.grid[0])):
        #         if self.grid[row][col] == target.lower() and row <= v_pos and abs(col - h_pos) < best:
        #             best = abs(col - h_pos)
        return int(best)

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
        return str(self.grid) == str(other.grid)

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
    h = float(state.get_heuristic_score())
    # ...
    # compute an heuristic value
    # ...
    return h


def run_in_proc(instance, init, queue, h=None):
    with open(str(sys.argv[1]) + '_out.txt', 'w') as f:
        original_stdout = sys.stdout
        sys.stdout = f

        print("==================", instance, "==================")

        grid_init, grid_goal = init
        global goal_state
        goal_state = State(grid_goal, is_goal_state=True)
        init_state = State(grid_init)
        problem = Blocks(init_state)

        start_time = time.perf_counter()
        if h is None:
            node, nb_explored, remaining_nodes = breadth_first_graph_search(problem)
        else:
            node, nb_explored, remaining_nodes = astar_graph_search(problem, h)
        end_time = time.perf_counter()
        queue.put((node, nb_explored, remaining_nodes, end_time - start_time))

        print("==================", "===========", "==================")

        sys.stdout = original_stdout


def local_run():
    q_multi_proc = multiprocessing.Queue()

    ##############################
    # Launch the search in local #
    ##############################
    # Use this block to test your code in local
    # Comment it and uncomment the next one if you want to submit your code on INGInious

    instances_path = "instances/"
    instance_names = ['a01', 'a02', 'a03', 'a04', 'a05', 'a06', 'a07', 'a08', 'a09', 'a10']
    headers = ['Instance', 'Timeout?', 'Number of moves', 'Execution time', 'Path cost to goal', '#Nodes explored',
               'Queue size at goal']
    content = []

    for instance in [instances_path + name for name in instance_names]:
        grid_init, grid_goal = readInstanceFile(instance)
        if instance == 'instances/a05':
            continue
        # global goal_state
        # goal_state = State(grid_goal, is_goal_state=True)
        # init_state = State(grid_init)
        # problem = Blocks(init_state)
        print('Instance: ', instance)

        # example of bfs tree search
        # startTime = time.perf_counter()
        # if len(sys.argv) > 1:
        #     if sys.argv[1] == 'astar':
        #         node, nb_explored, remaining_nodes = astar_graph_search(problem, heuristic)
        #     elif sys.argv[1] == 'bfs_graph':
        #         node, nb_explored, remaining_nodes = breadth_first_graph_search(problem)
        #     else:
        #         node, nb_explored, remaining_nodes = None, -1, -1
        # else:
        #     node, nb_explored, remaining_nodes = None, -1, -1
        #
        # if (node, nb_explored, remaining_nodes) == (None, -1, -1):
        #     node, nb_explored, remaining_nodes = depth_first_graph_search(problem)
        # endTime = time.perf_counter()
        if sys.gettrace() is None:
            timeout = 75  # in SECONDS
        else:
            timeout = 3600 * 24

        if len(sys.argv) > 1:
            if sys.argv[1] == 'astar':
                p = multiprocessing.Process(target=run_in_proc,
                                            args=(instance, (grid_init, grid_goal), q_multi_proc, heuristic))
            elif sys.argv[1] == 'bfs_graph':
                p = multiprocessing.Process(target=run_in_proc, args=(instance, (grid_init, grid_goal), q_multi_proc))
            else:
                p = multiprocessing.Process(target=run_in_proc, args=(instance, (grid_init, grid_goal), q_multi_proc))
        else:
            p = multiprocessing.Process(target=run_in_proc, args=(instance, (grid_init, grid_goal), q_multi_proc))

        p.start()
        p.join(timeout=timeout)
        p.terminate()

        if p.exitcode == 0:
            return_value = q_multi_proc.get()

            node, nb_explored, remaining_nodes, execution_time = return_value

            # example of print
            path = node.path()
            path.reverse()

            print('Number of moves: ' + str(node.depth))
            # for n in path:
            #     print(n.state)  # assuming that the __str__ function of state outputs the correct format
            #     print()
            print("* Execution time:\t", str(execution_time))
            print("* Path cost to goal:\t", node.depth, "moves")
            print("* #Nodes explored:\t", nb_explored)
            print("* Queue size at goal:\t", remaining_nodes)
            content.append(
                [str(instance), 'NO', str(node.depth), str(execution_time), str(node.depth), str(nb_explored),
                 str(remaining_nodes)])
        else:
            print('Instance', instance, "timed out after", str(timeout) + "s")
            content.append(
                [str(instance), 'YES', '/', '> ' + str(timeout) + 's', '/', '/',
                 '/'])
    from tabulate import tabulate
    print(tabulate(content, headers=headers))


def INGInious_run(search='astar'):
    ####################################
    # Launch the search for INGInious  #
    ####################################
    # Use this block to test your code on INGInious
    instance = sys.argv[1]
    grid_init, grid_goal = readInstanceFile(instance)
    global goal_state
    goal_state = State(grid_goal, is_goal_state=True)
    init_state = State(grid_init)
    problem = Blocks(init_state)

    # example of bfs graph search
    start_time = time.perf_counter()
    if search == 'astar':
        node, nb_explored, remaining_nodes = astar_graph_search(problem, heuristic)
    else:
        node, nb_explored, remaining_nodes = breadth_first_graph_search(problem)
    end_time = time.perf_counter()

    # example of print
    path = node.path()
    path.reverse()

    print('Number of moves: ' + str(node.depth))
    for n in path:
        print(n.state)  # assuming that the __str__ function of state outputs the correct format
        print()
    print("* Execution time:\t", str(end_time - start_time))
    print("* Path cost to goal:\t", node.depth, "moves")
    print("* #Nodes explored:\t", nb_explored)
    print("* Queue size at goal:\t", remaining_nodes)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] != 'astar' and sys.argv[1] != 'bfs_graph':
        INGInious_run(search='astar')
    else:
        local_run()
