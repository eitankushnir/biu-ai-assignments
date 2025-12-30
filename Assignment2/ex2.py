from __future__ import generators
from collections import deque
import numpy as np
import re
import math
import random
import sys
import ext_plant
import time
import copy
import operator, math, random, copy, sys, os.path, bisect, inspect

id = ["328149547"]

class Controller:
    """This class is a controller for the ext_plant game."""

    def __init__(self, game: ext_plant.Game):
        """Initialize controller for given game model."""
        self.original_game = game
        self.problem = game.get_problem()
        self.rows, self.cols = self.problem['Size']
        self.walls = self.problem['Walls']

        self.active_robot_ids = self._get_best_robots()
        self.EFFICIENCY_FACTOR = self.problem['robot_chosen_action_prob'][self.active_robot_ids[-1]]
        # Eff factor is the p of the worst active robot
        
        self.plan = []
        self.cached_plan = None
        self.last_state = None
        self.last_action = None

  
    def choose_next_action(self, state):
        """ Choose the next action given a state."""
        # unpack the state
        current_need = state[3]

        is_reset = False
        if self.last_state and current_need > self.last_state[3] or not self.last_state:
            is_reset = True
            self.last_state = None
            self.last_action = None
            self.plan = []
        
        if not self.plan:
            # If i didnt cache the astar solution then that's what we do at the start.
            if not self.cached_plan:
                pruned = self._prune_problem(self.original_game.get_max_steps() - self.original_game.get_current_steps())
                self.plan = self.solve_problem(pruned, 'astar')
                self.cached_plan = self.plan.copy()

            elif is_reset:
                remaining_steps = self.original_game.get_max_steps() - self.original_game.get_current_steps()
                should_continue_aster = len(self.cached_plan) <= remaining_steps * self.EFFICIENCY_FACTOR

                if should_continue_aster:
                    self.plan = self.cached_plan.copy()

                # Not enough steps to continue 
                else:
                    reprune = self._prune_problem(remaining_steps)
                    self.plan = self.solve_problem(reprune, 'gbfs')

                # New plan generation failed.
                if not self.plan:
                    return "RESET"

            # Finished the plan but not all plants were watered
            if not self.plan:
                self.last_state = None
                return "RESET"

        # Check for fixes after last step
        if self.last_action and self.last_state and not is_reset:
            recovery_steps = self._check_and_handle_fail(self.last_state, self.last_action, state)
            if recovery_steps:
                for step in reversed(recovery_steps):
                    self.plan.insert(0, step)

        next_action = self.plan[0]

        # Check for blocking bots
        unblocking_move = self._check_blocking_dumbot(state, next_action)
        if unblocking_move:
            if unblocking_move == "RESET":
                self.last_state = None
            return unblocking_move

        action = self.plan.pop(0)
        self.last_action = action
        self.last_state = state
        return action

    def _check_blocking_dumbot(self, state, next_action_str):
        try:
            parts = next_action_str.split('(')
            action_type = parts[0].strip()
            rid = int(parts[1].replace(')', '').strip())
        except: return None

        if action_type not in ["UP", "DOWN", "LEFT", "RIGHT"]:
            return None

        active_bot = next((r for r in state[0] if r[0] == rid), None)
        if not active_bot:
            return None

        target_pos = self._get_expected_pos(active_bot[1], action_type)
        blocker = next((r for r in state[0] if r[1] == target_pos), None)

        if blocker and blocker[0] not in self.active_robot_ids:
            blocker_id = blocker[0]
            for move_dir in ["UP", "DOWN", "LEFT", "RIGHT"]:
                if move_dir in self.plan[1]:
                    continue
                next_pos = self._get_expected_pos(blocker[1], move_dir)
                if self._is_valid_cell(next_pos, state):
                    return f"{move_dir}({blocker_id})"

            # When a blocker cannot be moved, make him a wall and cause a recalculation.
            self.problem['Walls'].add(blocker[1]) # Add new robot as wall.
            self.problem['Plants'] = self._get_reachable_plants(self.problem)
            self.cached_plan = []
            self.last_state = None
            return "RESET"
        return None


    def _check_and_handle_fail(self, prev_state, action_str, current_state):
        
        try:
            parts = action_str.split('(')
            action_type = parts[0].strip()
            rid = int(parts[1].replace(')', '').strip())
        except:
            return []

        curr_robot = next(r for r in current_state[0] if r[0] == rid)
        prev_robot = next(r for r in prev_state[0] if r[0] == rid)

        if not curr_robot or not prev_robot:
            return []

        # If a pour did not succeed we have to ignore it and keep going.
                
        # If a load did not succeed we just try to load again. (if the tap still has water)
        if action_type == 'LOAD' and curr_robot[2] <= prev_robot[2]:
            return [action_str]

        elif action_type in ["UP", "DOWN", "LEFT", "RIGHT"]:
            # On a move we need to check for slipping, and move back to the correct position. if we stayed we just repeat the move
            curr_pos = curr_robot[1]
            prev_pos = prev_robot[1]

            # Stay = retry last action again
            if prev_pos == curr_pos:
                return [action_str]

            # Moved wrongly = fix position + retry action
            expected_pos = self._get_expected_pos(prev_pos, action_type)
            if expected_pos == curr_pos:
                return []

            correction = self._get_move_command(curr_pos, prev_pos, rid)
            if correction:
                return [correction, action_str]

        return []


    def _get_move_command(self, start, end, rid):
        dr = end[0] - start[0]
        dc = end[1] - start[1]
        if dr == -1:
            return f"UP ({rid})"
        if dr == 1:
            return f"DOWN ({rid})"
        if dc == -1:
            return f"LEFT ({rid})"
        if dc == 1:
            return f"RIGHT ({rid})"
        return None

    def _get_expected_pos(self, pos, direction):
        r, c = pos
        if direction == "UP":
            return (r - 1, c)
        elif direction == "DOWN":
            return (r + 1, c)
        elif direction == "LEFT":
            return (r, c - 1)
        elif direction == "RIGHT":
            return (r, c + 1)

        return pos

    def _is_valid_cell(self, pos, state):
        if pos in self.walls:
            return False

        if not (0 <= pos[0] < self.rows and 0 <= pos[1] < self.cols):
            return False

        for r in state[0]:
            if r[1] == pos:
                return False

        return True


    def solve_problem(self, problem, algo):
        return solve_problems(problem, algo)

    def _get_best_robots(self):
        probs = self.problem['robot_chosen_action_prob']

        # Get all bots with p > 0.9
        sorted_probs = sorted(probs.items(), key=lambda x: x[1], reverse=True)
        high_quality = [rid for rid, p in sorted_probs if p > 0.9]

        # if there are non get the bot with best p
        if not high_quality:
            return [sorted_probs[0][0]]

        # else get the 3 bots with > 0.9
        return high_quality[:3]

    def _prune_problem(self, horizon):
        optimized_prob = self.problem.copy()

        optimized_prob['Size'] = self.problem['Size']
        optimized_prob['Walls'] = self.problem['Walls']
        
        optimized_prob['Robots'] = {
                rid: self.problem['Robots'][rid]
                for rid in self.active_robot_ids
        }

        plants = self.problem['Plants']
        taps = self.problem['Taps']
        rewards_map = self.problem['plants_reward']

        optimized_prob['Plants'] = self._get_reachable_plants(self.problem)
        optimized_prob['Taps'] = taps

        if not taps: 
            return optimized_prob

        tap_coords = list(taps.keys())
        hub_r = np.mean([t[0] for t in tap_coords])
        hub_c = np.mean([t[1] for t in tap_coords])

        plant_metrics = []
        for pos, need in plants.items():
            dist = abs(pos[0] - hub_r) + abs(pos[1] - hub_c)
            cost = 2 * dist + need

            rewards = rewards_map.get(pos, [0])
            avg_rwd = sum(rewards) / len(rewards)

            ratio = avg_rwd / (cost + 1)

            plant_metrics.append({
                'pos': pos,
                'need': need,
                'cost': cost,
                'ratio': ratio
            })

        total_work = sum(p['cost'] for p in plant_metrics)

        # Reduce total steps to accout for failure / needing to move other robots away.
        capcity = horizon * self.EFFICIENCY_FACTOR

        if total_work > capcity:
            plant_metrics.sort(key=lambda x: x['ratio'], reverse=True)

            keep_plants = {}
            current_load = 0
            for p in plant_metrics:
                if current_load + p['cost'] < capcity:
                    keep_plants[p['pos']] = p['need']
                    current_load += p['cost']

            optimized_prob["Plants"] = keep_plants

        return optimized_prob

    def _get_reachable_cells(self, start_positions):
        """
        Performs a BFS/Flood Fill to find all cells reachable from start_positions
        considering the walls.
        """
        queue = list(start_positions)
        visited = set(start_positions)
        
        rows, cols = self.problem['Size']
        walls = self.problem['Walls']
        
        # Directions: Up, Down, Left, Right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        head = 0
        while head < len(queue):
            r, c = queue[head]
            head += 1
            
            for dr, dc in directions:
                nr, nc = r + dr, c + dc
                
                # Check bounds
                if 0 <= nr < rows and 0 <= nc < cols:
                    pos = (nr, nc)
                    # Check walls and visited
                    if pos not in walls and pos not in visited:
                        visited.add(pos)
                        queue.append(pos)
                        
        return visited

    def _get_reachable_plants(self, problem):
        robot_starts = []
        for rid in self.active_robot_ids:
            r_data = problem['Robots'][rid]
            robot_starts.append((r_data[0], r_data[1]))

        reachable_by_bots = self._get_reachable_cells(robot_starts)

        tap_starts = list(problem['Taps'].keys())
        reachable_by_taps = self._get_reachable_cells(tap_starts)

        valid_plants = dict()
        for pos, need in problem['Plants'].items():
            if pos in reachable_by_bots and pos in reachable_by_taps:
                valid_plants[pos] = need

        return valid_plants
    




# ASTAR CODE
"""Search (Chapters 3-4)

The way to use this code is to subclass Problem to create a class of problems,
then create problem instances and solve them with calls to the various search
functions."""



# ______________________________________________________________________________

class Problem:
    """The abstract class for a formal problem.  You should subclass this and
    implement the method successor, and possibly __init__, goal_test, and
    path_cost. Then you will create instances of your subclass and solve them
    with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal.  Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def successor(self, state):
        """Given a state, return a sequence of (action, state) pairs reachable
        from this state. If there are many successors, consider an iterator
        that yields the successors one at a time, rather than building them
        all at once. Iterators will work fine within the framework."""
        abstract

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal, as specified in the constructor. Implement this
        method if checking against a single self.goal is not enough."""
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self):
        """For optimization problems, each state has a value.  Hill-climbing
        and related algorithms try to maximize this value."""
        abstract


# ______________________________________________________________________________

class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state.  Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node.  Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        "Create a search tree Node, derived from a parent by an action."
        update(self, state=state, parent=parent, action=action,
               path_cost=path_cost, depth=0)
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def path(self):
        "Create a list of nodes from the root to this node."
        x, result = self, [self]
        while x.parent:
            result.append(x.parent)
            x = x.parent
        return result

    def expand(self, problem):
        "Return a list of nodes reachable from this node. [Fig. 3.8]"
        return [Node(next, self, act,
                     problem.path_cost(self.path_cost, self.state, act, next))
                for (act, next) in problem.successor(self.state)]

    def __eq__(self, other):
        return (self.f == other.f)

    def __ne__(self, other):
        return not (self == other)

    def __lt__(self, other):
        return (self.f < other.f)

    def __gt__(self, other):
        return (self.f > other.f)

    def __le__(self, other):
        return (self < other) or (self == other)

    def __ge__(self, other):
        return (self > other) or (self == other)


# ______________________________________________________________________________
## Uninformed Search algorithms

def tree_search(problem, fringe):
    """Search through the successors of a problem to find a goal.
    The argument fringe should be an empty queue.
    Don't worry about repeated paths to a state. [Fig. 3.8]"""
    fringe.append(Node(problem.initial))
    while fringe:
        node = fringe.pop()
        if problem.goal_test(node.state):
            return node
        fringe.extend(node.expand(problem))
    return None


def breadth_first_tree_search(problem):
    "Search the shallowest nodes in the search tree first. [p 74]"
    return tree_search(problem, FIFOQueue())


def depth_first_tree_search(problem):
    "Search the deepest nodes in the search tree first. [p 74]"
    return tree_search(problem, Stack())


def graph_search(problem, fringe):
    """Search through the successors of a problem to find a goal.
    The argument fringe should be an empty queue.
    If two paths reach a state, only use the best one. [Fig. 3.18]"""
    closed = {}
    expanded = 0
   
    fringe.append(Node(problem.initial))
    while fringe:
        node = fringe.pop()
        if problem.goal_test(node.state):
            return node, expanded
        if node.state not in closed:
            closed[node.state] = True
            fringe.extend(node.expand(problem))
            expanded += 1
    return None


def breadth_first_graph_search(problem):
    "Search the shallowest nodes in the search tree first. [p 74]"
    return graph_search(problem, FIFOQueue())


def depth_first_graph_search(problem):
    "Search the deepest nodes in the search tree first. [p 74]"
    return graph_search(problem, Stack())


def depth_limited_search(problem, limit=50):
    "[Fig. 3.12]"

    def recursive_dls(node, problem, limit):
        cutoff_occurred = False
        if problem.goal_test(node.state):
            return node
        elif node.depth == limit:
            return 'cutoff'
        else:
            for successor in node.expand(problem):
                result = recursive_dls(successor, problem, limit)
                if result == 'cutoff':
                    cutoff_occurred = True
                elif result != None:
                    return result
        if cutoff_occurred:
            return 'cutoff'
        else:
            return None

    # Body of depth_limited_search:
    return recursive_dls(Node(problem.initial), problem, limit)


def iterative_deepening_search(problem):
    "[Fig. 3.13]"
    for depth in range(sys.maxsize):
        result = depth_limited_search(problem, depth)
        if result != 'cutoff':
            return result


# ______________________________________________________________________________
# Informed (Heuristic) Search

def best_first_graph_search(problem, f):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have depth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = memoize(f, 'f')
    return graph_search(problem, PriorityQueue(min, f))


greedy_best_first_graph_search = best_first_graph_search


# Greedy best-first search is accomplished by specifying f(n) = h(n).

def astar_search(problem, h=None):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search.
    Uses the pathmax trick: f(n) = max(f(n), g(n)+h(n))."""
    h = h or problem.h

    def f(n):
        return max(getattr(n, 'f', -infinity), n.path_cost + h(n))

    return best_first_graph_search(problem, f)


# ______________________________________________________________________________
## Other search algorithms

def recursive_best_first_search(problem):
    "[Fig. 4.5]"

    def RBFS(problem, node, flimit):
        if problem.goal_test(node.state):
            return node
        successors = expand(node, problem)
        if len(successors) == 0:
            return None, infinity
        for s in successors:
            s.f = max(s.path_cost + s.h, node.f)
        while True:
            successors.sort(lambda x, y: x.f - y.f)  # Order by lowest f value
            best = successors[0]
            if best.f > flimit:
                return None, best.f
            alternative = successors[1]
            result, best.f = RBFS(problem, best, min(flimit, alternative))
            if result is not None:
                return result

    return RBFS(Node(problem.initial), infinity)


def hill_climbing(problem):
    """From the initial node, keep choosing the neighbor with highest value,
    stopping when no neighbor is better. [Fig. 4.11]"""
    current = Node(problem.initial)
    while True:
        neighbor = argmax(expand(node, problem), Node.value)
        if neighbor.value() <= current.value():
            return current.state
        current = neighbor


def exp_schedule(k=20, lam=0.005, limit=100):
    "One possible schedule function for simulated annealing"
    return lambda t: if_(t < limit, k * math.exp(-lam * t), 0)


def simulated_annealing(problem, schedule=exp_schedule()):
    "[Fig. 4.5]"
    current = Node(problem.initial)
    for t in xrange(sys.maxint):
        T = schedule(t)
        if T == 0:
            return current
        next = random.choice(expand(node.problem))
        delta_e = next.path_cost - current.path_cost
        if delta_e > 0 or probability(math.exp(delta_e / T)):
            current = next


def online_dfs_agent(a):
    "[Fig. 4.12]"
    pass  #### more


def lrta_star_agent(a):
    "[Fig. 4.12]"
    pass  #### more


"""Provide some widely useful utilities. Safe for "from utils import *".

"""

def raiseNotDefined():
    fileName = inspect.stack()[1][1]
    line = inspect.stack()[1][2]
    method = inspect.stack()[1][3]

    print("*** Method not implemented: %s at line %s of %s" % (method, line, fileName))
    sys.exit(1)


# ______________________________________________________________________________
# Compatibility with Python 2.2 and 2.3

# The AIMA code is designed to run in Python 2.2 and up (at some point,
# support for 2.2 may go away; 2.2 was released in 2001, and so is over
# 3 years old). The first part of this file brings you up to 2.4
# compatibility if you are running in Python 2.2 or 2.3:

# try: bool, True, False ## Introduced in 2.3
# except NameError:
#     class bool(int):
#         "Simple implementation of Booleans, as in PEP 285"
#         def __init__(self, val): self.val = val
#         def __int__(self): return self.val
#         def __repr__(self): return ('False', 'True')[self.val]
#
#     True, False = bool(1), bool(0)
#
# try: sum ## Introduced in 2.3
# except NameError:
#     def sum(seq, start=0):
#         """Sum the elements of seq.
#         >>> sum([1, 2, 3])
#         6
#         """
#         return reduce(operator.add, seq, start)

try:
    enumerate  ## Introduced in 2.3
except NameError:
    def enumerate(collection):
        """Return an iterator that enumerates pairs of (i, c[i]). PEP 279.
        >>> list(enumerate('abc'))
        [(0, 'a'), (1, 'b'), (2, 'c')]
        """
        ## Copied from PEP 279
        i = 0
        it = iter(collection)
        while 1:
            yield (i, it.next())
            i += 1

try:
    reversed  ## Introduced in 2.4
except NameError:
    def reversed(seq):
        """Iterate over x in reverse order.
        >>> list(reversed([1,2,3]))
        [3, 2, 1]
        """
        if hasattr(seq, 'keys'):
            raise ValueError("mappings do not support reverse iteration")
        i = len(seq)
        while i > 0:
            i -= 1
            yield seq[i]

try:
    sorted  ## Introduced in 2.4
except NameError:
    def sorted(seq, cmp=None, key=None, reverse=False):
        """Copy seq and sort and return it.
        >>> sorted([3, 1, 2])
        [1, 2, 3]
        """
        seq2 = copy.copy(seq)
        if key:
            if cmp == None:
                cmp = __builtins__.cmp
            seq2.sort(lambda x, y: cmp(key(x), key(y)))
        else:
            if cmp == None:
                seq2.sort()
            else:
                seq2.sort(cmp)
        if reverse:
            seq2.reverse()
        return seq2

try:
    set, frozenset  ## set builtin introduced in 2.4
except NameError:
    try:
        import sets  ## sets module introduced in 2.3

        set, frozenset = sets.Set, sets.ImmutableSet
    except (NameError, ImportError):
        class BaseSet:
            "set type (see http://docs.python.org/lib/types-set.html)"

            def __init__(self, elements=[]):
                self.dict = {}
                for e in elements:
                    self.dict[e] = 1

            def __len__(self):
                return len(self.dict)

            def __iter__(self):
                for e in self.dict:
                    yield e

            def __contains__(self, element):
                return element in self.dict

            def issubset(self, other):
                for e in self.dict.keys():
                    if e not in other:
                        return False
                return True

            def issuperset(self, other):
                for e in other:
                    if e not in self:
                        return False
                return True

            def union(self, other):
                return type(self)(list(self) + list(other))

            def intersection(self, other):
                return type(self)([e for e in self.dict if e in other])

            def difference(self, other):
                return type(self)([e for e in self.dict if e not in other])

            def symmetric_difference(self, other):
                return type(self)([e for e in self.dict if e not in other] +
                                  [e for e in other if e not in self.dict])

            def copy(self):
                return type(self)(self.dict)

            def __repr__(self):
                elements = ", ".join(map(str, self.dict))
                return "%s([%s])" % (type(self).__name__, elements)

            __le__ = issubset
            __ge__ = issuperset
            __or__ = union
            __and__ = intersection
            __sub__ = difference
            __xor__ = symmetric_difference


        class frozenset(BaseSet):
            "A frozenset is a BaseSet that has a hash value and is immutable."

            def __init__(self, elements=[]):
                BaseSet.__init__(elements)
                self.hash = 0
                for e in self:
                    self.hash |= hash(e)

            def __hash__(self):
                return self.hash


        class set(BaseSet):
            "A set is a BaseSet that does not have a hash, but is mutable."

            def update(self, other):
                for e in other:
                    self.add(e)
                return self

            def intersection_update(self, other):
                for e in self.dict.keys():
                    if e not in other:
                        self.remove(e)
                return self

            def difference_update(self, other):
                for e in self.dict.keys():
                    if e in other:
                        self.remove(e)
                return self

            def symmetric_difference_update(self, other):
                to_remove1 = [e for e in self.dict if e in other]
                to_remove2 = [e for e in other if e in self.dict]
                self.difference_update(to_remove1)
                self.difference_update(to_remove2)
                return self

            def add(self, element):
                self.dict[element] = 1

            def remove(self, element):
                del self.dict[element]

            def discard(self, element):
                if element in self.dict:
                    del self.dict[element]

            def pop(self):
                key, val = self.dict.popitem()
                return key

            def clear(self):
                self.dict.clear()

            __ior__ = update
            __iand__ = intersection_update
            __isub__ = difference_update
            __ixor__ = symmetric_difference_update

# ______________________________________________________________________________
# Simple Data Structures: infinity, Dict, Struct

infinity = 1.0e400


def Dict(**entries):
    """Create a dict out of the argument=value arguments. 
    >>> Dict(a=1, b=2, c=3)
    {'a': 1, 'c': 3, 'b': 2}
    """
    return entries


class DefaultDict(dict):
    """Dictionary with a default value for unknown keys."""

    def __init__(self, default):
        self.default = default

    def __getitem__(self, key):
        if key in self: return self.get(key)
        return self.setdefault(key, copy.deepcopy(self.default))

    def __copy__(self):
        copy = DefaultDict(self.default)
        copy.update(self)
        return copy


class Struct:
    """Create an instance with argument=value slots.
    This is for making a lightweight object whose class doesn't matter."""

    def __init__(self, **entries):
        self.__dict__.update(entries)

    def __cmp__(self, other):
        if isinstance(other, Struct):
            return cmp(self.__dict__, other.__dict__)
        else:
            return cmp(self.__dict__, other)

    def __repr__(self):
        args = ['%s=%s' % (k, repr(v)) for (k, v) in vars(self).items()]
        return 'Struct(%s)' % ', '.join(args)


def update(x, **entries):
    """Update a dict; or an object with slots; according to entries.
    >>> update({'a': 1}, a=10, b=20)
    {'a': 10, 'b': 20}
    >>> update(Struct(a=1), a=10, b=20)
    Struct(a=10, b=20)
    """
    if isinstance(x, dict):
        x.update(entries)
    else:
        x.__dict__.update(entries)
    return x


# ______________________________________________________________________________
# Functions on Sequences (mostly inspired by Common Lisp)
# NOTE: Sequence functions (count_if, find_if, every, some) take function
# argument first (like reduce, filter, and map).

def removeall(item, seq):
    """Return a copy of seq (or string) with all occurences of item removed.
    >>> removeall(3, [1, 2, 3, 3, 2, 1, 3])
    [1, 2, 2, 1]
    >>> removeall(4, [1, 2, 3])
    [1, 2, 3]
    """
    if isinstance(seq, str):
        return seq.replace(item, '')
    else:
        return [x for x in seq if x != item]


def unique(seq):
    """Remove duplicate elements from seq. Assumes hashable elements.
    >>> unique([1, 2, 3, 2, 1])
    [1, 2, 3]
    """
    return list(set(seq))


def product(numbers):
    """Return the product of the numbers.
    >>> product([1,2,3,4])
    24
    """
    return reduce(operator.mul, numbers, 1)


def count_if(predicate, seq):
    """Count the number of elements of seq for which the predicate is true.
    >>> count_if(callable, [42, None, max, min])
    2
    """
    f = lambda count, x: count + (not not predicate(x))
    return reduce(f, seq, 0)


def find_if(predicate, seq):
    """If there is an element of seq that satisfies predicate; return it.
    >>> find_if(callable, [3, min, max])
    <built-in function min>
    >>> find_if(callable, [1, 2, 3])
    """
    for x in seq:
        if predicate(x): return x
    return None


def every(predicate, seq):
    """True if every element of seq satisfies predicate.
    >>> every(callable, [min, max])
    1
    >>> every(callable, [min, 3])
    0
    """
    for x in seq:
        if not predicate(x): return False
    return True


def some(predicate, seq):
    """If some element x of seq satisfies predicate(x), return predicate(x).
    >>> some(callable, [min, 3])
    1
    >>> some(callable, [2, 3])
    0
    """
    for x in seq:
        px = predicate(x)
        if px: return px
    return False


def isin(elt, seq):
    """Like (elt in seq), but compares with is, not ==.
    >>> e = []; isin(e, [1, e, 3])
    True
    >>> isin(e, [1, [], 3])
    False
    """
    for x in seq:
        if elt is x: return True
    return False


# ______________________________________________________________________________
# Functions on sequences of numbers
# NOTE: these take the sequence argument first, like min and max,
# and like standard math notation: \sigma (i = 1..n) fn(i)
# A lot of programing is finding the best value that satisfies some condition;
# so there are three versions of argmin/argmax, depending on what you want to
# do with ties: return the first one, return them all, or pick at random.


def argmin(seq, fn):
    """Return an element with lowest fn(seq[i]) score; tie goes to first one.
    >>> argmin(['one', 'to', 'three'], len)
    'to'
    """
    best = seq[0];
    best_score = fn(best)
    for x in seq:
        x_score = fn(x)
        if x_score < best_score:
            best, best_score = x, x_score
    return best


def argmin_list(seq, fn):
    """Return a list of elements of seq[i] with the lowest fn(seq[i]) scores.
    >>> argmin_list(['one', 'to', 'three', 'or'], len)
    ['to', 'or']
    """
    best_score, best = fn(seq[0]), []
    for x in seq:
        x_score = fn(x)
        if x_score < best_score:
            best, best_score = [x], x_score
        elif x_score == best_score:
            best.append(x)
    return best


def argmin_random_tie(seq, fn):
    """Return an element with lowest fn(seq[i]) score; break ties at random.
    Thus, for all s,f: argmin_random_tie(s, f) in argmin_list(s, f)"""
    best_score = fn(seq[0]);
    n = 0
    for x in seq:
        x_score = fn(x)
        if x_score < best_score:
            best, best_score = x, x_score;
            n = 1
        elif x_score == best_score:
            n += 1
            if random.randrange(n) == 0:
                best = x
    return best


def argmax(seq, fn):
    """Return an element with highest fn(seq[i]) score; tie goes to first one.
    >>> argmax(['one', 'to', 'three'], len)
    'three'
    """
    return argmin(seq, lambda x: -fn(x))


def argmax_list(seq, fn):
    """Return a list of elements of seq[i] with the highest fn(seq[i]) scores.
    >>> argmax_list(['one', 'three', 'seven'], len)
    ['three', 'seven']
    """
    return argmin_list(seq, lambda x: -fn(x))


def argmax_random_tie(seq, fn):
    "Return an element with highest fn(seq[i]) score; break ties at random."
    return argmin_random_tie(seq, lambda x: -fn(x))


# ______________________________________________________________________________
# Statistical and mathematical functions

def histogram(values, mode=0, bin_function=None):
    """Return a list of (value, count) pairs, summarizing the input values.
    Sorted by increasing value, or if mode=1, by decreasing count.
    If bin_function is given, map it over values first."""
    if bin_function: values = map(bin_function, values)
    bins = {}
    for val in values:
        bins[val] = bins.get(val, 0) + 1
    if mode:
        return sorted(bins.items(), key=lambda v: v[1], reverse=True)
    else:
        return sorted(bins.items())


def log2(x):
    """Base 2 logarithm.
    >>> log2(1024)
    10.0
    """
    return math.log10(x) / math.log10(2)


def mode(values):
    """Return the most common value in the list of values.
    >>> mode([1, 2, 3, 2])
    2
    """
    return histogram(values, mode=1)[0][0]


def median(values):
    """Return the middle value, when the values are sorted.
    If there are an odd number of elements, try to average the middle two.
    If they can't be averaged (e.g. they are strings), choose one at random.
    >>> median([10, 100, 11])
    11
    >>> median([1, 2, 3, 4])
    2.5
    """
    n = len(values)
    values = sorted(values)
    if n % 2 == 1:
        return values[n / 2]
    else:
        middle2 = values[(n / 2) - 1:(n / 2) + 1]
        try:
            return mean(middle2)
        except TypeError:
            return random.choice(middle2)


def mean(values):
    """Return the arithmetic average of the values."""
    return sum(values) / float(len(values))


def stddev(values, meanval=None):
    """The standard deviation of a set of values.
    Pass in the mean if you already know it."""
    if meanval == None: meanval = mean(values)
    return math.sqrt(sum([(x - meanval) ** 2 for x in values]) / (len(values) - 1))


def dotproduct(X, Y):
    """Return the sum of the element-wise product of vectors x and y.
    >>> dotproduct([1, 2, 3], [1000, 100, 10])
    1230
    """
    return sum([x * y for x, y in zip(X, Y)])


def vector_add(a, b):
    """Component-wise addition of two vectors.
    >>> vector_add((0, 1), (8, 9))
    (8, 10)
    """
    return tuple(map(operator.add, a, b))


def probability(p):
    "Return true with probability p."
    return p > random.uniform(0.0, 1.0)


def num_or_str(x):
    """The argument is a string; convert to a number if possible, or strip it.
    >>> num_or_str('42')
    42
    >>> num_or_str(' 42x ')
    '42x'
    """
    if isnumber(x): return x
    try:
        return int(x)
    except ValueError:
        try:
            return float(x)
        except ValueError:
            return str(x).strip()


def normalize(numbers, total=1.0):
    """Multiply each number by a constant such that the sum is 1.0 (or total).
    >>> normalize([1,2,1])
    [0.25, 0.5, 0.25]
    """
    k = total / sum(numbers)
    return [k * n for n in numbers]


## OK, the following are not as widely useful utilities as some of the other
## functions here, but they do show up wherever we have 2D grids: Wumpus and
## Vacuum worlds, TicTacToe and Checkers, and markov decision Processes.

orientations = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def turn_right(orientation):
    return orientations[orientations.index(orientation) - 1]


def turn_left(orientation):
    return orientations[(orientations.index(orientation) + 1) % len(orientations)]


def distance(t1, t2):  # (ax, ay), (bx, by)):
    "The distance between two (x, y) points."
    return math.hypot((t1.ax - t2.bx), (t1.ay - t2.by))


def distance2(t1, t2):  # ((ax, ay), (bx, by)):
    "The square of the distance between two (x, y) points."
    return (t1.ax - t2.bx) ** 2 + (t1.ay - t2.by) ** 2


def clip(vector, lowest, highest):
    """Return vector, except if any element is less than the corresponding
    value of lowest or more than the corresponding value of highest, clip to
    those values.
    >>> clip((-1, 10), (0, 0), (9, 9))
    (0, 9)
    """
    return type(vector)(map(min, map(max, vector, lowest), highest))


# ______________________________________________________________________________
# Misc Functions

def printf(format, *args):
    """Format args with the first argument as format string, and write.
    Return the last arg, or format itself if there are no args."""
    sys.stdout.write(str(format) % args)
    return if_(args, args[-1], format)


def caller(n=1):
    """Return the name of the calling function n levels up in the frame stack.
    >>> caller(0)
    'caller'
    >>> def f(): 
    ...     return caller()
    >>> f()
    'f'
    """
    import inspect
    return inspect.getouterframes(inspect.currentframe())[n][3]


def memoize(fn, slot=None):
    """Memoize fn: make it remember the computed value for any argument list.
    If slot is specified, store result in that slot of first argument.
    If slot is false, store results in a dictionary."""
    if slot:
        def memoized_fn(obj, *args):
            if hasattr(obj, slot):
                return getattr(obj, slot)
            else:
                val = fn(obj, *args)
                setattr(obj, slot, val)
                return val
    else:
        def memoized_fn(*args):
            if not memoized_fn.cache.has_key(args):
                memoized_fn.cache[args] = fn(*args)
            return memoized_fn.cache[args]

        memoized_fn.cache = {}
    return memoized_fn


def if_(test, result, alternative):
    """Like C++ and Java's (test ? result : alternative), except
    both result and alternative are always evaluated. However, if
    either evaluates to a function, it is applied to the empty arglist,
    so you can delay execution by putting it in a lambda.
    >>> if_(2 + 2 == 4, 'ok', lambda: expensive_computation())
    'ok'
    """
    if test:
        if callable(result): return result()
        return result
    else:
        if callable(alternative): return alternative()
        return alternative


def name(object):
    "Try to find some reasonable name for the object."
    return (getattr(object, 'name', 0) or getattr(object, '__name__', 0)
            or getattr(getattr(object, '__class__', 0), '__name__', 0)
            or str(object))


def isnumber(x):
    "Is x a number? We say it is if it has a __int__ method."
    return hasattr(x, '__int__')


def issequence(x):
    "Is x a sequence? We say it is if it has a __getitem__ method."
    return hasattr(x, '__getitem__')


def print_table(table, header=None, sep=' ', numfmt='%g'):
    """Print a list of lists as a table, so that columns line up nicely.
    header, if specified, will be printed as the first row.
    numfmt is the format for all numbers; you might want e.g. '%6.2f'.
    (If you want different formats in differnt columns, don't use print_table.)
    sep is the separator between columns."""
    justs = [if_(isnumber(x), 'rjust', 'ljust') for x in table[0]]
    if header:
        table = [header] + table
    table = [[if_(isnumber(x), lambda: numfmt % x, x) for x in row]
             for row in table]
    maxlen = lambda seq: max(map(len, seq))
    sizes = map(maxlen, zip(*[map(str, row) for row in table]))
    for row in table:
        for (j, size, x) in zip(justs, sizes, row):
            print(getattr(str(x), j)(size), sep),
        print()


def AIMAFile(components, mode='r'):
    "Open a file based at the AIMA root directory."
    import utils
    dir = os.path.dirname(utils.__file__)
    return open(apply(os.path.join, [dir] + components), mode)


def DataFile(name, mode='r'):
    "Return a file in the AIMA /data directory."
    return AIMAFile(['..', 'data', name], mode)


# ______________________________________________________________________________
# Queues: Stack, FIFOQueue, PriorityQueue

class Queue:
    """Queue is an abstract class/interface. There are three types:
        Stack(): A Last In First Out Queue.
        FIFOQueue(): A First In First Out Queue.
        PriorityQueue(lt): Queue where items are sorted by lt, (default <).
    Each type supports the following methods and functions:
        q.append(item)  -- add an item to the queue
        q.extend(items) -- equivalent to: for item in items: q.append(item)
        q.pop()         -- return the top item from the queue
        len(q)          -- number of items in q (also q.__len())
    Note that isinstance(Stack(), Queue) is false, because we implement stacks
    as lists.  If Python ever gets interfaces, Queue will be an interface."""

    def __init__(self):
        abstract

    def extend(self, items):
        for item in items: self.append(item)


def Stack():
    """Return an empty list, suitable as a Last-In-First-Out Queue."""
    return []


class FIFOQueue(Queue):
    """A First-In-First-Out Queue."""

    def __init__(self):
        self.A = [];
        self.start = 0

    def append(self, item):
        self.A.append(item)

    def __len__(self):
        return len(self.A) - self.start

    def extend(self, items):
        self.A.extend(items)

    def pop(self):
        e = self.A[self.start]
        self.start += 1
        if self.start > 5 and self.start > len(self.A) / 2:
            self.A = self.A[self.start:]
            self.start = 0
        return e


class PriorityQueue(Queue):
    """A queue in which the minimum (or maximum) element (as determined by f and
    order) is returned first. If order is min, the item with minimum f(x) is
    returned first; if order is max, then it is the item with maximum f(x)."""

    def __init__(self, order=min, f=lambda x: x):
        update(self, A=[], order=order, f=f)

    def append(self, item):
        bisect.insort(self.A, (self.f(item), item))

    def __len__(self):
        return len(self.A)

    def pop(self):
        if self.order == min:
            return self.A.pop(0)[1]
        else:
            return self.A.pop()[1]


## Fig: The idea is we can define things like Fig[3,10] later.
## Alas, it is Fig[3,10] not Fig[3.10], because that would be the same as Fig[3.1]
Fig = {}


class WateringProblem(Problem):
    """This class implements a pressure plate problem"""

    def __init__(self, initial):
        """
        Intialize the state of the function, static values (like tap positions) are stored in self,
        Dynamic values are stored in the state tuple, which is
        ( robots, taps, plants)
        Robots - a tuple for each robot (row, col, carry, capacity)
        Plants - a tuple of the amount of water the i'th plant needs
        Taps - a tuple of the amount of water held by the i'th type
        """

        # Constant values should be kept inside self and not in the state.
        self.rows, self.cols = initial["Size"]
        self.walls = frozenset(initial["Walls"])

        # Store robot data.
        # Contant data (ids) goes into self, dynamic data (propreties) goes into state tuple
        robots = sorted(initial["Robots"].items())
        self.robot_ids = tuple(r[0] for r in robots)
        initial_robots = tuple(r[1] for r in robots)

        # Store tap data.
        # Contant data (positions) goes into self, dynamic data (wu count) goes into state tuple
        taps = sorted(initial["Taps"].items())
        self.tap_coords = tuple(t[0] for t in taps)
        initial_tap_counts = tuple(t[1] for t in taps)

        # Store plant data.
        # Contant data (positions) goes into self, dynamic data (wu needed) goes into state tuple
        plants = sorted(initial["Plants"].items())
        self.plant_coords = tuple(p[0] for p in plants)
        initial_plant_needs = tuple(p[1] for p in plants)

        # Maps to link tap positions to tap indexes in the state (and plants)
        # Useful for lookup when we go by position (i.e check if a robot is on a plant or tap)
        self.tap_map = {coords: i for i, coords in enumerate(self.tap_coords)}
        self.plant_map = {coords: i for i, coords in enumerate(self.plant_coords)}

        # Pre calculate distances tap->plant and plant-> plant
        self.tap_plant_dists = [
            [abs(t[0] - p[0]) + abs(t[1] - p[1]) for p in self.plant_coords]
            for t in self.tap_coords
        ]
        self.plant_plant_dists = [
            [abs(t[0] - p[0]) + abs(t[1] - p[1]) for p in self.plant_coords]
            for t in self.plant_coords
        ]

        # Pre calculate all the string literals for actions
        self.directions = [
            [
                (f"RIGHT({rid})", 0, 1),
                (f"LEFT({rid})", 0, -1),
                (f"UP({rid})", -1, 0),
                (f"DOWN({rid})", 1, 0),
            ]
            for rid in self.robot_ids
        ]
        self.load_strs = [f"LOAD({rid})" for rid in self.robot_ids]
        self.pour_strs = [f"POUR({rid})" for rid in self.robot_ids]

        # Initialize empty cache
        self.cache = dict()

        # Create the initial state: a tuple of all the dynamic data.
        initial_state = (initial_robots, initial_tap_counts, initial_plant_needs)

        Problem.__init__(self, initial_state)

    def successor(self, state):
        """Generates the successor states returns [(action, achieved_states, ...)]"""

        # Extract state data in the currect variables.
        # robots - a tuple of robots tuples (row, col, water held, water cap)
        # taps - a tuple of numbers, water remaning in a certain tap
        # plants - a tuple of number, water required to complete plant
        robots, taps, plants = state
        successors = []

        # Calculate some values so we can know if we should terminate early
        total_tap_water = sum(taps)
        total_robot_water = sum(r[2] for r in robots)
        total_water = total_tap_water + total_robot_water
        total_need = sum(plants)
        # NO STATES LEFT CANNOT COMPLETE
        if total_need > total_water:
            return []

        # Very optimized successor function for the simple case of 1 robot
        if len(robots) == 1:
            return self._single_robot_successor(state, total_need)

        # All invalid positions to go to
        occupied = {(r[0], r[1]) for r in robots} | self.walls

        # All MOVE actions, per robot per direction
        num_robots = len(robots)
        for i in range(num_robots):
            r_row, r_col, r_wu, r_cap = robots[i]
            curr_coords = (r_row, r_col)

            # POUR ACTION - IF WE CAN POUR, WE JUST DO, NO NEED TO BRANCH FROM IT
            if curr_coords in self.plant_map:
                plant_idx = self.plant_map[curr_coords]
                needed_wu = plants[plant_idx]
                if needed_wu > 0 and r_wu > 0:
                    # Update robots
                    new_robot = (r_row, r_col, r_wu - 1, r_cap)
                    new_robots = robots[:i] + (new_robot,) + robots[i + 1 :]

                    # Update plants
                    new_plants = (
                        plants[:plant_idx] + (needed_wu - 1,) + plants[plant_idx + 1 :]
                    )

                    # New state
                    new_state = (tuple(new_robots), taps, tuple(new_plants))
                    if new_state not in self.cache:
                        successors.append((self.pour_strs[i], new_state))
            

            # LOAD ACTION
            if curr_coords in self.tap_map:
                tap_idx = self.tap_map[curr_coords]
                available_wu = taps[tap_idx]
                # Can take water
                if available_wu > 0 and r_wu < r_cap:
                    # Update robots
                    new_robot = (r_row, r_col, r_wu + 1, r_cap)
                    new_robots = robots[:i] + (new_robot,) + robots[i + 1 :]

                    # Update taps
                    new_taps = (
                        taps[:tap_idx] + (available_wu - 1,) + taps[tap_idx + 1 :]
                    )

                    # New State
                    new_state = (tuple(new_robots), tuple(new_taps), plants)

                    # Not need to branch again from a state with already been to.
                    if new_state not in self.cache:
                        successors.append((self.load_strs[i], new_state))

            for action, d_row, d_col in self.directions[i]:
                new_row = r_row + d_row
                new_col = r_col + d_col

                # Check that we are inside bounds
                if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                    # Check that new position is not a wall or another robot
                    if (new_row, new_col) not in occupied:
                        new_robot = (new_row, new_col, r_wu, r_cap)

                        # Remake the robots tuple
                        new_robots = robots[:i] + (new_robot,) + robots[i + 1 :]

                        new_state = (tuple(new_robots), taps, plants)

                        # Not need to branch again from a state with already been to.
                        if new_state not in self.cache:
                            successors.append((action, new_state))
        return successors

    def _single_robot_successor(self, state, total_need):
        """
        Generates the successor states when there is only one robot
        returns [(action, achieved_states, ...)]
        """
        successors = []
        robots, taps, plants = state

        r_row, r_col, r_wu, r_cap = robots[0]
        # Single robot, ON TAP, CAN GET MORE, DOESNT HAVE ENOUGH FOR PLANTS
        # Best (and only action he should take) is getting water
        if (r_row, r_col) in self.tap_coords:
            t_idx = self.tap_map[(r_row, r_col)]
            t_amount = taps[t_idx]
            if t_amount > 0 and r_wu < r_cap and r_wu < total_need:
                # Update robots
                new_robot = (r_row, r_col, r_wu + 1, r_cap)
                new_robots = (new_robot,)

                # Update taps
                new_taps = taps[:t_idx] + (t_amount - 1,) + taps[t_idx + 1 :]

                # New State
                new_state = (tuple(new_robots), tuple(new_taps), plants)
                # if not in cache, return this only, o.w. continue checking other options.
                if new_state not in self.cache:
                    return [(self.load_strs[0], new_state)]

        # Single robot, ON PLANT, HAS WATER
        # Best (and only action) should be to keep watering
        if (r_row, r_col) in self.plant_coords:
            p_idx = self.plant_map[(r_row, r_col)]
            p_need = plants[p_idx]
            if p_need > 0 and r_wu > 0:
                new_robot = (r_row, r_col, r_wu - 1, r_cap)
                new_robots = (new_robot,)

                new_plants = plants[:p_idx] + (p_need - 1,) + plants[p_idx + 1 :]

                new_state = (new_robots, taps, new_plants)
                # if not in cache, return this only, o.w. continue checking other options.
                if new_state not in self.cache:
                    return [(self.pour_strs[0], new_state)]

        # CANT LOAD, CANT WATER, SHOULD JUST MOVE IN ANY DIRECTION IT CAN

        # MOVE ACTIONS
        for action, d_row, d_col in self.directions[0]:
            new_row = r_row + d_row
            new_col = r_col + d_col

            # Check that we are inside bounds
            if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                # Check that new position is not a wall or another robot
                if (new_row, new_col) not in self.walls:
                    new_robot = (new_row, new_col, r_wu, r_cap)

                    # Remake the robots tuple
                    new_robots = (new_robot,)

                    new_state = (new_robots, taps, plants)
                    if new_state not in self.cache:
                        successors.append((action, new_state))

        return successors

    def goal_test(self, state):
        """given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        # Any checks if there are any non-zero elements inside state[2] which is plants.
        return not any(state[2])

    def h_astar(self, node):
        """This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        if node.state in self.cache:
            return self.cache[node.state]

        robots, taps, plants = node.state

        active_tap_indecies = []
        active_plant_indecies = []

        pours_required = 0
        total_wu_in_tap = 0
        current_wu_carried = 0

        for i, t in enumerate(taps):
            if t > 0:
                total_wu_in_tap += t
                active_tap_indecies.append(i)

        for i, p in enumerate(plants):
            if p > 0:
                pours_required += p
                active_plant_indecies.append(i)

        current_wu_carried = sum(r[2] for r in robots)
        total_wu_world = current_wu_carried + total_wu_in_tap

        # Cannot complete world because there is not enough water
        if total_wu_world < pours_required:
            return float("inf")
        if pours_required == 0:
            return 0

        # Minimum number of LOADs
        load_required = max(0, pours_required - current_wu_carried)

        # Get The distance we need to travel to water the plants
        max_min_dist = 0
        min_min_dist = float("inf")

        for p_idx in active_plant_indecies:
            p_loc = self.plant_coords[p_idx]
            min_dist_to_plant = float("inf")

            for r in robots:
                r_row, r_col, r_wu = r[0], r[1], r[2]

                if r_wu > 0:
                    d_r_p = abs(r_row - p_loc[0]) + abs(r_col - p_loc[1])
                    if d_r_p < min_dist_to_plant:
                        min_dist_to_plant = d_r_p
                else:
                    # Must load, ROBOT -> TAP -> PLANT
                    best_tap_path = float("inf")
                    for t_idx in active_tap_indecies:
                        t_loc = self.tap_coords[t_idx]

                        d_r_t = abs(r_row - t_loc[0]) + abs(r_col - t_loc[1])
                        d_p_t = self.tap_plant_dists[t_idx][p_idx]
                        path_len = d_r_t + d_p_t
                        if path_len < best_tap_path:
                            best_tap_path = path_len

                    if best_tap_path < min_dist_to_plant:
                        min_dist_to_plant = best_tap_path

            if min_dist_to_plant == float("inf"):
                return float("inf")

            if min_dist_to_plant > max_min_dist:
                max_min_dist = min_dist_to_plant
            if min_dist_to_plant < min_min_dist:
                min_min_dist = min_dist_to_plant

        # Get the minimum distance to a tap.
        min_tap_dist = 0
        if load_required > 0 and active_tap_indecies:
            min_tap_dist = float("inf")
            for r in robots:
                r_row, r_col = r[0], r[1]
                for t_idx in active_tap_indecies:
                    t_loc = self.tap_coords[t_idx]
                    dist = abs(r_row - t_loc[0]) + abs(r_col - t_loc[1])
                    if dist < min_tap_dist:
                        min_tap_dist = dist

        elif load_required > 0 and not active_tap_indecies:
            return float("inf")

        diameter = (
            0  # Max distance between 2 plants, useful to know when we have 1 robot
        )
        if len(robots) == 1 and len(active_plant_indecies) > 1:
            min_sum = float("inf")
            max_sum = float("-inf")
            min_diff = float("inf")
            max_diff = float("-inf")

            for p_idx in active_plant_indecies:
                p_row, p_col = self.plant_coords[p_idx]
                u = p_row + p_col
                v = p_row - p_col
                if u < min_sum:
                    min_sum = u
                if u > max_sum:
                    max_sum = u
                if v > max_diff:
                    max_diff = v
                if v < min_diff:
                    min_diff = v

            u_range = max_sum - min_sum
            v_range = max_diff - min_diff
            # The maximum distance between 2 plants that need watering + the cost to get to the closest plant.
            # Single robot must do all of this on its own.
            diameter = max(u_range, v_range) + min_min_dist

        # Take the largest movement option
        move_cost = max(max_min_dist, min_tap_dist, diameter)

        heuristic = pours_required + load_required + move_cost
        self.cache[node.state] = heuristic
        return heuristic

    def h_gbfs(self, node):
        """This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        robots, taps, plants = node.state

        total_need = sum(plants)
        if total_need == 0:
            return 0

        active_plant_indecies = [i for i, p in enumerate(plants) if p > 0]
        active_tap_indecies = [i for i, count in enumerate(taps) if count > 0]

        current_water = sum(r[2] for r in robots)
        useful_water = min(current_water, total_need)

        min_dist_to_useful_action = float("inf")

        for r_row, r_col, r_wu, _ in robots:
            dist_to_closest_plant = float("inf")

            for p_idx in active_plant_indecies:
                p_loc = self.plant_coords[p_idx]
                d = abs(r_row - p_loc[0]) + abs(r_col - p_loc[1])
                if d < dist_to_closest_plant:
                    dist_to_closest_plant = d

            if dist_to_closest_plant == float("inf"):
                continue

            if r_wu > 0:
                if dist_to_closest_plant < min_dist_to_useful_action:
                    min_dist_to_useful_action = dist_to_closest_plant
            elif active_tap_indecies:
                closest_tap_dist = float("inf")

                for t_idx in active_tap_indecies:
                    t_loc = self.tap_coords[t_idx]
                    d_tap = abs(r_row - t_loc[0]) + abs(r_col - t_loc[1])
                    if d_tap < closest_tap_dist:
                        closest_tap_dist = d_tap

                full_trip_length = closest_tap_dist + dist_to_closest_plant
                if full_trip_length < min_dist_to_useful_action:
                    min_dist_to_useful_action = full_trip_length

            # Something went wrong if this is the case. Should not be true if solvable.
        if min_dist_to_useful_action == float("inf"):
            return 0

        # We give high weight to total_need, this way the algorithm will always prioritize
        # actions that reduce the total_need.
        # We give our water more weight so robots will always fill up at a tap before moving on
        return (total_need * 100) - (useful_water * 10) + min_dist_to_useful_action


def create_watering_problem(game):
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)

def run_problem(func, targs=(), kwargs=None):
    if kwargs is None:
        kwargs = {}
    result = (-3, "default")
    try:
        result = func(*targs, **kwargs)

    except Exception as e:
        result = (-3, e)
    return result

def solve_problems(problem, algorithm):
    

    try:
        p = create_watering_problem(problem)
    except Exception as e:
        print("Error creating problem: ", e)
        return None

    if algorithm == "gbfs":
        result = run_problem((lambda p: greedy_best_first_graph_search(p, p.h_gbfs)),targs=[p])
    else:
        result = run_problem((lambda p: astar_search(p, p.h_astar)), targs=[p])

    if result and isinstance(result[0], Node):
        solve = result[0].path()[::-1]
        solution = [pi.action for pi in solve][1:]
        return solution
    else:
        return []
