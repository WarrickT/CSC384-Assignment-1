#   You may only add standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files
from typing import Callable, Union

import os                       # For time functions
import math                     # For infinity

from src import (
    # For search engine implementations
    SearchEngine, SearchNode, SearchStatistics,
    # For Sokoban-specific implementations
    SokobanState,
    sokoban_goal_state,
    UP, DOWN, LEFT, RIGHT,
    # You may further import any constants you may need.
    # See `search_constants.py`
)


# SOKOBAN HEURISTICS
def heur_alternate(state: 'SokobanState') -> float:
    """
    Returns a heuristic value with the goal of improving upon
    the flaws inherent to a heuristic that uses Manhattan distance
    and produce a more accurate estimate of the distance from the
    current state to the goal state.

    You must explain your heuristic via inline comments.

    :param state: A SokobanState object representing the current
                  state in a game of Sokoban.
    :return: An estimate of the distance from the current
             SokobanState to the goal state.
    """
    # TODO: IMPLEMENT

    """

    1. Checking for deadlocks 

    - Corner deadlocks 
    - Outer wall deadlocks 
    
    """
    h = 0 

    obs = state.obstacles
    storage = state.storage
    width = state.width
    height = state.height
    boxes = state.boxes 
    unstored_boxes = boxes - storage 

    if not unstored_boxes: 
        return 0 
    
    # Precomputing the storage 
    storage_x = {s[0] for s in storage}
    storage_y = {s[1] for s in storage }

    # 

    # Checking the storage in the perimeters
    has_storage_top = (0 in storage_y)
    has_storage_bottom = ((height - 1) in storage_y)
    has_storage_left = (0 in storage_x)
    has_storage_right = ((width - 1) in storage_x)

    # Exhaustive Search for Deadlocking 

    for b_x, b_y in unstored_boxes: 
        # If the direction has obstacle or boundary 
        is_blocked_north = (b_x, b_y-1) in obs or b_y-1 < 0 
        is_blocked_south = (b_x, b_y+1) in obs or b_y+1 >= height 
        is_blocked_west = (b_x-1, b_y) in obs or b_x-1 < 0
        is_blocked_east = (b_x+1, b_y) in obs or b_x+1 >= width 

        if (is_blocked_north or is_blocked_south) and (is_blocked_west or is_blocked_east):
            return float('inf')

        
        if (b_y == 0 and not has_storage_top) or (b_y == height - 1 and not has_storage_bottom) or \
        (b_x == 0 and not has_storage_left) or (b_x == width - 1 and not has_storage_right):
            return float('inf')
        
        # Manhattan distance between the box and storage spots 
        for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
            # Check the three neighbors that would form a 2x2 square with our box
            adj_h = (b_x + dx, b_y)
            adj_v = (b_x, b_y + dy)
            diag  = (b_x + dx, b_y + dy)
            
            if (adj_h in obs or adj_h in boxes) and \
            (adj_v in obs or adj_v in boxes) and \
            (diag in obs or diag in boxes):
                if (b_x, b_y) not in storage:
                    return float('inf')
        
        # h += min(abs(b_x - s_x) + abs(b_y - s_y) for s_x, s_y in storage)
        min_s = 1e9
        for s_x, s_y in storage:
            d = abs(b_x - s_x) + abs(b_y - s_y)
            if d < min_s: min_s = d
        h += min_s

    min_robot_dist = float('inf')
    for r_x, r_y in state.robots: 
        for b_x, b_y in unstored_boxes: 
            dist = abs(r_x - b_x) + abs(r_y - b_y)
            if dist < min_robot_dist: 
                min_robot_dist = dist 
    
    if min_robot_dist != float('inf'):
            h += (min_robot_dist - 1) * 0.3 

    return h 

def heur_zero(state: 'SokobanState') -> float:
    """
    This function is used in A* to perform a uniform cost search
    by returning zero.

    :param state: A SokobanState object representing the current
                  state in a game of Sokoban.
    :return: The zero value.
    """
    return 0

def heur_manhattan_distance(state: 'SokobanState') -> float:
    # IMPLEMENT
    """
    Returns an admissible - i.e. optimistic - heuristic by never
    overestimating the cost to transition from the current state to the goal state.
    The sum of the Manhattan distances between each box that has yet to be stored
    and the storage point nearest to it qualifies as such a heuristic.

    You may assume there are no obstacles on the grid when calculating distances.
    You must implement this function exactly as specified.

    :param state: A SokobanState object representing the current
                  state in a game of Sokoban.
    :return: An admissible estimate of the distance from the
             current SokobanState to the goal state.
    """
    # TODO: IMPLEMENT
    manhattan_sum = 0.0
    
    # Loop through every box 
    unstored_boxes = state.boxes - state.storage 

    # Take the lowest Manhattan distance from that loop 
    for b_x, b_y in unstored_boxes: 
        closest_dist = min(abs(b_x - s_x) + abs(b_y - s_y) for s_x, s_y in state.storage)
        manhattan_sum += closest_dist 
        
    # Just return the sum of the Manhattan distances between each box 

    # Return a  float 
    return manhattan_sum 
    # raise NotImplementedError("You must implement heur_manhattan_distance.")

def fval_function(node: 'SearchNode', weight: float) -> float:
    """
    Returns the f-value of the state contained in node
    based on weight, to be used in Anytime Weighted A* search.

    :param node: A SearchNode object containing a SokobanState object
    :param weight: The weight used in Anytime Weighted A* search.
    :return: The f-value of the state contained in node.
    """
    # TODO: IMPLEMENT
    raise NotImplementedError("You must implement fval_function.")

# SEARCH ALGORITHMS
def weighted_astar(
        initial_state: 'SokobanState',
        heur_fn: Callable,
        weight: float,
        timebound: int) -> tuple[Union['SokobanState', bool], 'SearchStatistics']:
    """
    Returns a tuple of the goal SokobanState and a SearchStatistics object
    by implementing weighted A* search as defined in the handout.

    If no goal state is found, returns a tuple of False and a SearchStatistics
    object.

    :param initial_state: The initial SokobanState of the game of Sokoban.
    :param heur_fn: The heuristic function used in weighted A* search.
    :param weight: The weight used in calculating the heuristic.
    :param timebound: The time bound used in weighted A* search, in seconds.
    :return: A tuple consisting of the goal SokobanState or False if such a state
             is not found, and a SearchStatistics object.
    """
    # TODO: Implement
    raise NotImplementedError("You must implement weighted_astar.")

def iterative_astar( # uses f(n)
        initial_state: 'SokobanState',
        heur_fn: Callable,
        weight: float = 1,
        timebound: int = 5) -> tuple[Union['SokobanState', bool], 'SearchStatistics']:
    """
    Returns a tuple of the goal SokobanState and a SearchStatistics object
    by implementing realtime iterative A* search as defined in the handout.

    If no goal state is found, returns a tuple of False and a SearchStatistics
    object.

    Refer to test_alternate_fun in autograder.py to see how to initialize a search.

    :param initial_state: The initial SokobanState of the game of Sokoban.
    :param heur_fn: The heuristic function used in realtime iterative A* search.
    :param weight: The weight used in calculating the heuristic.
    :param timebound: The time bound used in realtime iterative A* search, in seconds.
    :return: A tuple consisting of the goal SokobanState or False if such a state
             is not found, and a SearchStatistics object.
    """
    # TODO: IMPLEMENT
    raise NotImplementedError("You must implement iterative_astar.")

def iterative_gbfs( # uses h(n)
        initial_state: 'SokobanState',
        heur_fn: Callable,
        timebound: int = 5) -> tuple[Union['SokobanState', bool], 'SearchStatistics']:
    """
    Returns a tuple of the goal SokobanState and a SearchStatistics object
    by implementing iterative greedy best-first search as defined in the handout.

    :param initial_state: The initial SokobanState of the game of Sokoban.
    :param heur_fn: The heuristic function used in iterative greedy best-first search.
    :param timebound: The time bound used in iterative greedy best-first search, in seconds.
    :return: A tuple consisting of the goal SokobanState or False if such a state
             is not found, and a SearchStatistics object.
    """
    # TODO: IMPLEMENT
    raise NotImplementedError("You must implement iterative_gbfs.")
