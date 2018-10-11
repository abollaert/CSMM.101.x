#! /usr/bin/python2
from sys import argv
from math import sqrt
from Queue import Queue
from Queue import PriorityQueue
from time import time

import resource

class Statistics:

    def __init__(self):
        self.__nodes_expanded = 0
        self.__depth = 0
        self.__max_search_depth = 0

    def node_expanded(self, node):
        self.__nodes_expanded += 1

    def node_visited(self, node):
        if node.get_depth() > self.__max_search_depth:
            self.__max_search_depth = node.get_depth()

    def get_nodes_expanded(self):
        return self.__nodes_expanded

    def get_max_search_depth(self):
        return self.__max_search_depth

""" Models a board state. """
class BoardState:

    """ Creates a new board state from a state string or a grid. """
    def __init__(self, state, previous_state, depth, move):
        if isinstance(state, str):
            tiles = state.split(",")

            dimension = int(sqrt(len(tiles)))

            self.__grid = [[int(tiles[row * dimension + col]) for col in range(dimension)] for row in range(dimension)]
        elif isinstance(state, list):
            self.__grid = state

        self.__previous_state = previous_state
        self.__depth = depth
        self.__move = move

        hash_string = ""

        for row in range(len(self.__grid)):
            for col in range(len(self.__grid)):
                hash_string += str(self.__grid[row][col])

        self.__hash = int(hash_string)

    def __get_blank_location(self):
        for row in range(len(self.__grid)):
            for col in range(len(self.__grid[row])):
                if self.__grid[row][col] == 0:
                    return (row, col)

        return None

    def __copy_grid(self):
        return [[self.__grid[row][col] for col in range(len(self.__grid))] for row in range(len(self.__grid))]

    """ Returns the possible next states. """
    def get_possible_next_states(self):
        next_states = []
        blank_row, blank_col = self.__get_blank_location()

        # Decide if we can go up.
        if blank_row != 0:
            new_grid = self.__copy_grid()

            new_grid[blank_row][blank_col] = new_grid[blank_row - 1][blank_col]
            new_grid[blank_row - 1][blank_col] = 0

            next_states.append(("Up", new_grid))

        # Decide if we can go down.
        if blank_row != len(self.__grid) - 1:
            new_grid = self.__copy_grid()

            new_grid[blank_row][blank_col] = new_grid[blank_row + 1][blank_col]
            new_grid[blank_row + 1][blank_col] = 0

            next_states.append(("Down", new_grid))

        # Left.
        if blank_col != 0:
            new_grid = self.__copy_grid()

            new_grid[blank_row][blank_col] = new_grid[blank_row][blank_col - 1]
            new_grid[blank_row][blank_col - 1] = 0

            next_states.append(("Left", new_grid))

        # Right.
        if blank_col != len(self.__grid) - 1:
            new_grid = self.__copy_grid()

            new_grid[blank_row][blank_col] = new_grid[blank_row][blank_col + 1]
            new_grid[blank_row][blank_col + 1] = 0

            next_states.append(("Right", new_grid))

        return next_states

    def is_goal_state(self):
        for row in range(len(self.__grid)):
            for col in range(len(self.__grid[row])):
                if self.__grid[row][col] != row * len(self.__grid) + col:
                    return False

        return True

    def get_previous_state(self):
        return self.__previous_state

    def get_path(self):
        path = []
        state = self

        while state.__move != None:
            path.insert(0, state.__move)
            state = state.get_previous_state()

        return path

    def get_depth(self):
        return self.__depth

    def distance(self):
        distance = 0

        for row in range(len(self.__grid)):
            for col in range(len(self.__grid)):
                value = self.__grid[row][col]

                target_position_row = value // len(self.__grid)
                target_position_col = value % len(self.__grid)

                distance = distance + abs(row - target_position_row) + abs(col - target_position_col)

        return distance

    def __hash__(self):
        return self.__hash

def run_bfs(initial_state):
    frontier = Queue()

    search(initial_state, lambda: frontier.get(), lambda state: frontier.put(state), lambda: frontier.empty(), False)

def run_dfs(initial_state):
    frontier = list()

    search(initial_state, lambda: frontier.pop(), lambda state: frontier.append(state), lambda: len(frontier) == 0, True)

def run_ast(initial_state):
    frontier = PriorityQueue()

    search(initial_state, lambda: frontier.get()[1], lambda state: frontier.put((state.distance(), state)), lambda: frontier.empty(), False)

def search(initial_state, next_state_function, frontier_add_function, is_frontier_empty, reverse):
    start_time = time()

    visited = set()
    frontier = set()
    statistics = Statistics()

    frontier_add_function(initial_state)
    frontier.add(hash(initial_state))

    goal_state = None

    while not is_frontier_empty():
        current_state = next_state_function()
        visited.add(hash(current_state))
        frontier.remove(hash(current_state))

        if current_state.is_goal_state():
            goal_state = current_state

            break
        else:
            next_states = current_state.get_possible_next_states()

            if reverse:
                next_states = reversed(next_states)

            statistics.node_expanded(current_state)

            for direction, grid in next_states:
                new_state = BoardState(grid, current_state, current_state.get_depth() + 1, direction)
                new_state_hash = hash(new_state)

                if not new_state_hash in visited and not new_state_hash in frontier:
                    statistics.node_visited(new_state)
                    frontier_add_function(new_state)
                    frontier.add(new_state_hash)

    if goal_state != None:
        output_file = open("output.txt", "w")

        path = goal_state.get_path()

        output_file.write("path_to_goal: %s\n" % (path))
        output_file.write("cost_of_path: %i\n" % (len(path)))
        output_file.write("nodes_expanded: %i\n" % (statistics.get_nodes_expanded()))
        output_file.write("search_depth: %i\n" % (current_state.get_depth()))
        output_file.write("max_search_depth: %i\n" % (statistics.get_max_search_depth()))

        mem = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
        run_time = time() - start_time

        output_file.write("running_time: %s\n" % (str(run_time)))
        output_file.write("max_ram_usage: %s\n" % (str(mem)))

        output_file.close()

def main():
     algorithm = argv[1]
     initial_state_string = argv[2]

     initial_board_state = BoardState(initial_state_string, None, 0, None)

     if algorithm == "bfs":
         run_bfs(initial_board_state)
     elif algorithm == "dfs":
         run_dfs(initial_board_state)
     elif algorithm == "ast":
         run_ast(initial_board_state)

if __name__ == "__main__":
    main()