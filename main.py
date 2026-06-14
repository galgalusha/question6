import heapq
from collections import defaultdict

# ─────────────────────────────────────────────
# Planning problem definition
# ─────────────────────────────────────────────

# 3x3 grid  (row, col);  '#' = wall
GRID = [
    ['.', '.', '.'],
    ['.', '#', '.'],
    ['.', '.', '.'],
]
ROWS  = 3
COLS  = 3
START = (1, 0)   # robot start position
GOAL  = (1, 2)   # goal position

ACTIONS = ['left', 'right', 'up', 'down']
ACTION_DELTAS = {
    'left':  ( 0, -1),
    'right': ( 0, +1),
    'up':    (-1,  0),
    'down':  (+1,  0),
}


def manhattan(pos):
    return abs(pos[0] - GOAL[0]) + abs(pos[1] - GOAL[1])


def is_valid(pos):
    r, c = pos
    return 0 <= r < ROWS and 0 <= c < COLS and GRID[r][c] != '#'


# ─────────────────────────────────────────────
# SearchState  — the single "variable" used in V
# ─────────────────────────────────────────────

class SearchState:
    def __init__(self, state, plan, g):
        self.state = state
        self.plan  = list(plan)
        self.g     = g
        self.h     = manhattan(state)
        self.f     = self.g + self.h
        self._done_expanding = False
    @property
    def domain(self):
        return set()


# ─────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────

class Node:
    def __init__(self, assignment, vars_set, C, unassigned):
        self.assignment = assignment
        self._vars = set(vars_set)
        self.C = C
        self.unassigned = unassigned

    @property
    def vars(self):
        if not self._vars:
            return set()
        ss = next(iter(self._vars))
        return set() if ss._done_expanding else set(self._vars)

    @vars.setter
    def vars(self, value):
        self._vars = set(value)


# ─────────────────────────────────────────────
# Functions required by the BnB algorithm
# ─────────────────────────────────────────────

def ChooseVariable(vars_n):
    ss = next(iter(vars_n))
    return SearchState(ss.state, ss.plan, ss.g)


def ChooseValue(n, v, i):
    action  = ACTIONS[i]
    dr, dc  = ACTION_DELTAS[action]
    new_pos = (v.state[0] + dr, v.state[1] + dc)
    new_ss  = SearchState(new_pos, v.plan + [action], v.g + 1)
    if i == k - 1:
        next(iter(n._vars))._done_expanding = True
    return new_ss


def ComputeOptimistic(n):
    ss = next(iter(n._vars))
    if ss is None or not is_valid(ss.state):
        return float('inf')
    return ss.f


def NewNode(assignment, vars_n, C, unassigned):
    if assignment:
        # assignment is always a single (v_copy, successor) pair
        _, latest = next(iter(assignment))
        new_vars = {latest}
    else:
        new_vars = set(vars_n)
    return Node(assignment, new_vars, C, defaultdict(set))


def Complete(n):
    ss = next(iter(n._vars))
    return ss is not None and ss.state == GOAL


def B(n):
    ss = next(iter(n._vars))
    if ss is None or not is_valid(ss.state):
        return float('inf')
    return ss.f


def PreviouslyAssigned(n, v):
    return set(n.assignment)


# ─────────────────────────────────────────────
# Priority Queue  (ordered by B)
# ─────────────────────────────────────────────

class PriorityQueue:
    def __init__(self):
        self._heap    = []
        self._counter = 0  # tie-breaker

    def push(self, node):
        heapq.heappush(self._heap, (B(node), self._counter, node))
        self._counter += 1

    def pop(self):
        _, _, node = heapq.heappop(self._heap)
        return node

    def empty(self):
        return len(self._heap) == 0


# ─────────────────────────────────────────────
# Main algorithm  (unchanged from pseudo-code)
# ─────────────────────────────────────────────

k = 4      # number of children nodes to try expanding

def DeferredUnordered(V, C):
# Input: Variables Set V, Constraints C
# Output: Best assignmenet

# Initialize variables do
    OPEN = PriorityQueue()
    U = {v: set(v.domain) for v in V}           # U(v) <- Domain(v) for all v in V
    Best = float('inf'); nBest = NewNode(set(), set(V), C, U)
    OPEN.push(nBest)
# done

    while not OPEN.empty():
        n = OPEN.pop()
        if Complete(n):                          # A solution! e.g., Vars(n) = ∅
            if B(n) < Best:
                nBest = n; Best = B(n)
        else:                                    # node n not a complete solution
          # Choose a Variable
            v = ChooseVariable(n.vars)           # heuristically select variable to expand

            if v in n.vars:
                n.vars = n.vars - {v}            # variable previously unassigned, remove from list
                NewAssignment = set(n.assignment)
            else:                                # remove its assignment
                NewAssignment = set(n.assignment) - PreviouslyAssigned(n, v)
          # done
          # Generate new nodes
            for i in range(k):
                h = ChooseValue(n, v, i)
                n.unassigned[v] = n.unassigned[v] - {h}
                n_new = NewNode(NewAssignment | {(v, h)}, n.vars, C, n.unassigned)
                B_new = ComputeOptimistic(n_new)
                if B_new < Best:                 # if yes, add to OPEN, otherwise gets pruned
                    OPEN.push(n_new)
          # done
          # Push n back?
            if n.vars or any(len(n.unassigned.get(v, set())) > 0 for v in V):
                OPEN.push(n)                     # current node still not done expanding
          # done
    return nBest.assignment


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────

if __name__ == '__main__':
    initial_ss = SearchState(START, [], 0)
    V = {initial_ss}
    C = set()

    result = DeferredUnordered(V, C)

    if result:
        _, best_ss = next(iter(result))
        print('Plan found:', best_ss.plan)
        print('Steps     :', len(best_ss.plan))
    else:
        print('No plan found')
