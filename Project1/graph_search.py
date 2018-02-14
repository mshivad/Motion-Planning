#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import heapq
import matplotlib.pyplot as plotter
from math import hypot
import copy
import math

_DEBUG = False
_DEBUG_END = True
_ACTIONS = ['u','d','l','r']
_ACTIONS_2 = ['u','d','l','r','ne','nw','sw','se']
_X = 1
_Y = 0
_GOAL_COLOR = 0.75
_INIT_COLOR = 0.25
_PATH_COLOR_RANGE = _GOAL_COLOR-_INIT_COLOR
_VISITED_COLOR = 0.9


class GridMap:
    '''
    Class to hold a grid map for navigation. Reads in a map.txt file of the format
    0 - free cell, x - occupied cell, g - goal location, i - initial location.
    Additionally provides a simple transition model for grid maps and a convience function
    for displaying maps.
    '''
    def __init__(self, map_path=None):
        '''
        Constructor. Makes the necessary class variables. Optionally reads in a provided map
        file given by map_path.

        map_path (optional) - a string of the path to the file on disk
        '''
        self.rows = None
        self.cols = None
        self.goal = None
        self.init_pos = None
        self.occupancy_grid = None
        if map_path is not None:
            self.read_map(map_path)



    def read_map(self, map_path):
        '''
        Read in a specified map file of the format described in the class doc string.

        map_path - a string of the path to the file on disk
        '''
        map_file = file(map_path,'r')
        lines = [l.rstrip().lower() for l in map_file.readlines()]
        map_file.close()
        self.rows = len(lines)
        self.cols = max([len(l) for l in lines])
        #self.rows = 4
        #self.cols = 4
        if _DEBUG:
            print 'rows', self.rows
            print 'cols', self.cols
            print lines
        self.occupancy_grid = np.zeros((self.rows, self.cols), dtype=np.bool)
        for r in xrange(self.rows):
            for c in xrange(self.cols):
                if lines[r][c] == 'x':
                    self.occupancy_grid[r][c] = True
                    #print "done"
                if lines[r][c] == 'g':
                    self.goal = (r,c)
                    #print "love"
                elif lines[r][c] == 'i':
                    self.init_pos = (r,c)
                    #print "you"
        else :
            print "path not found"

    def is_goal(self,s):
        '''
        Test if a specifid state is the goal state

        s - tuple describing the state as (row, col) position on the grid.

        Returns - True if s is the goal. False otherwise.
        '''
        return (s[_X] == self.goal[_X] and
                s[_Y] == self.goal[_Y])

    def transition(self, s, a):
        '''
        Transition function for the current grid map.

        s - tuple describing the state as (row, col) position on the grid.
        a - the action to be performed from state s

        returns - s_prime, the state transitioned to by taking action a in state s.
        If the action is not valid (e.g. moves off the grid or into an obstacle)
        returns the current state.
        '''
        new_pos = list(s[:])
        # Ensure action stays on the board
        if a == 'u':
            if s[_Y] > 0:
                new_pos[_Y] -= 1
        elif a == 'd':
            if s[_Y] < self.rows - 1:
                new_pos[_Y] += 1
        elif a == 'l':
            if s[_X] > 0:
                new_pos[_X] -= 1
        elif a == 'r':
            if s[_X] < self.cols - 1:
                new_pos[_X] += 1
        elif a == 'ne':
            if s[_Y] > 0 and s[_X] < self.cols - 1:
                new_pos[_Y] -= 1
                new_pos[_X] += 1
        elif a == 'nw':
            if s[_Y] > 0 and s[_X] > 0:
                new_pos[_Y] -= 1
                new_pos[_X] -= 1
        elif a == 'se':
            if s[_Y] < self.rows - 1 and s[_X] < self.cols - 1:
                new_pos[_Y] += 1
                new_pos[_X] += 1
        elif a == 'sw':
            if s[_Y] < self.rows - 1 and s[_X] > 0:
                new_pos[_Y] += 1
                new_pos[_X] -= 1
        else:
            print 'Unknown action:', str(a)

        # Test if new position is clear
        if self.occupancy_grid[new_pos[0], new_pos[1]]:
            s_prime = tuple(s)
        else:
            s_prime = tuple(new_pos)
        return s_prime

    def bfs_simulator(self, s, a, bfs_actions, action_probability, neighbour_probability, ortho_probability, simulate = False):
        #n0=GridMap(s,a);

        distribution = []
        # bfs_actions = ['u','u','u','u','r','r','r','r']
        if bfs_actions == _ACTIONS:
            if a == 'u' or a == 'd':
                state = self.transition(s, a)
                distribution.append((state, action_probability))
                print distribution
                state = self.transition(s, 'l')
                distribution.append((state, neighbour_probability))
                state = self.transition(s, 'r')
                distribution.append((state, neighbour_probability))
            else:
                state = self.transition(s, a)
                distribution.append((state, action_probability))
                state = self.transition(s, 'u')
                distribution.append((state, neighbour_probability))
                state = self.transition(s, 'd')
                distribution.append((state, neighbour_probability))


        elif (bfs_actions == _ACTIONS_2):
            state = self.transition(s,a)
            distribution.append((state,action_probability))
            if a == 'u':
                state = self.transition(s,'nw')
                distribution.append((state,neighbour_probability))
                state = self.transition(s, 'ne')
                distribution.append((state, neighbour_probability))
                state = self.transition(s, 'l')
                distribution.append((state, ortho_probability))
                state = self.transition(s, 'r')
                distribution.append((state, ortho_probability))
            elif a == 'r':
                state = self.transition(s,'ne')
                distribution.append((state,neighbour_probability))
                state = self.transition(s, 'se')
                distribution.append((state, neighbour_probability))
                state = self.transition(s, 'u')
                distribution.append((state, ortho_probability))
                state = self.transition(s, 'd')
                distribution.append((state, ortho_probability))
            elif a == 'd':
                state = self.transition(s,'se')
                distribution.append((state,neighbour_probability))
                state = self.transition(s, 'sw')
                distribution.append((state, neighbour_probability))
                state = self.transition(s, 'l')
                distribution.append((state, ortho_probability))
                state = self.transition(s, 'r')
                distribution.append((state, ortho_probability))
            elif a == 'l':
                state = self.transition(s,'nw')
                distribution.append((state,neighbour_probability))
                state = self.transition(s, 'sw')
                distribution.append((state, neighbour_probability))
                state = self.transition(s, 'u')
                distribution.append((state, ortho_probability))
                state = self.transition(s, 'd')
                distribution.append((state, ortho_probability))
            elif a == 'ne':
                state = self.transition(s,'u')
                distribution.append((state,neighbour_probability))
                state = self.transition(s, 'r')
                distribution.append((state, neighbour_probability))
                state = self.transition(s, 'nw')
                distribution.append((state, ortho_probability))
                state = self.transition(s, 'se')
                distribution.append((state, ortho_probability))
            elif a == 'nw':
                state = self.transition(s,'u')
                distribution.append((state,neighbour_probability))
                state = self.transition(s, 'l')
                distribution.append((state, neighbour_probability))
                state = self.transition(s, 'ne')
                distribution.append((state, ortho_probability))
                state = self.transition(s, 'sw')
                distribution.append((state, ortho_probability))
            elif a == 'se':
                state = self.transition(s,'d')
                distribution.append((state,neighbour_probability))
                state = self.transition(s, 'r')
                distribution.append((state, neighbour_probability))
                state = self.transition(s, 'sw')
                distribution.append((state, ortho_probability))
                state = self.transition(s, 'ne')
                distribution.append((state, ortho_probability))
            elif a == 'sw':
                state = self.transition(s,'d')
                distribution.append((state,neighbour_probability))
                state = self.transition(s, 'l')
                distribution.append((state, neighbour_probability))
                state = self.transition(s, 'se')
                distribution.append((state, ortho_probability))
                state = self.transition(s, 'nw')
                distribution.append((state, ortho_probability))

        if simulate:
            random=np.random.uniform(0,1)
            result = None
            total_value = 0
            for value in distribution:
                total_value += value[1]
                if total_value > random:
                    return value[0]
            return result
        return distribution

        i = np.random.uniform(0,1)
        if (i > 0.0 and i < 0.8):
            b=distribution[0][0]
            return b
        elif (i > 0.81 and i < 0.9):
            b = distribution[1][0]
            return b
        else:
            b = distribution[2][0]
            return b



    def findValues(self,goal_reward,normal_reward,corner_reward=None):
            value_iteration= np.zeros([self.rows,self.cols])
            for r in range (0,self.rows):
                for c in range (0,self.cols):
                    nr = self.rows - 1
                    nc = self.cols - 1
                    if ((r == 0) and (c == 0 or c == nc)) or ((r == nr) and (c == 0 or c == nc)):
                        if corner_reward is None:
                            value_iteration[r][c] = normal_reward
                        else:
                            value_iteration[r][c] = corner_reward
                    else:
                        value_iteration[r][c] = normal_reward
            value_iteration[self.goal[0]][self.goal[1]] = goal_reward
            return value_iteration

    def value_iteration(self, discount_factor, value, bfs_actions, action_probability, neighbour_probability, ortho_probability):
            previous_value = value
            current_value = np.zeros([self.rows,self.cols])
            policy = np.empty([self.rows,self.cols], dtype=object)
            no_iteration = 0
            while(True):
                no_iteration += 1
                for i in range (0,self.rows):
                    for j in range (0,self.cols):
                        temp_value = None
                        if (not self.occupancy_grid[i][j]):
                            for a in bfs_actions:
                                total = 0
                                distance = self.bfs_simulator((i, j), a, bfs_actions, action_probability, neighbour_probability, ortho_probability)
                                for prob in distance:
                                    total += prob[1]*previous_value[prob[0][0]][prob[0][1]]
                                if (total > temp_value) or (temp_value is None):
                                    temp_value = total
                                    policy[i][j] = a
                            current_value[i][j] = value[i][j] + discount_factor * temp_value
                if (no_iteration % 10 == 0):
                    print (no_iteration)
                if self.check(previous_value,current_value):
                    print ("Number of Iteration:" + str(no_iteration))
                    return current_value,policy
                previous_value = current_value
                current_value = np.zeros([self.rows,self.cols])


    def check(self, value1, value2):
        for r in range(self.rows):
            for c in range(self.cols):
                    if value1[r][c] != value2[r][c]:
                        return False
        return True

    def find_policy(self, default = 'd', actionset = None, randomize = False):
        policy = np.empty([self.rows, self.cols], dtype=object)
        for r in range(self.rows):
            for c in range(self.cols):
                if (not self.occupancy_grid[r][c]):
                    if (randomize):
                        policy[r][c] = np.random.choice(actionset)
                    else:
                        policy[r][c] = default
        return policy


    def PI(self, discount_factor, value, actionset, prob_action, prob_neighbour, prob_ortho, policy):
        old_policy = policy
        prev_value = value
        current_value = np.zeros([self.rows, self.cols])
        new_policy = np.empty([self.rows, self.cols], dtype=object)
        num_iterations = 0
        while (True):
            while (True):
                num_iterations += 1
                for r in range(self.rows):
                    for c in range(self.cols):
                        if (not self.occupancy_grid[r][c]):
                            tot = 0
                            dist = self.bfs_simulator((r, c), old_policy[r][c], actionset, prob_action,
                                                             prob_neighbour, prob_ortho)
                            for prob in dist:
                                tot += prob[1] * prev_value[prob[0][0]][prob[0][1]]
                            current_value[r][c] = value[r][c] + discount_factor * tot
                        else:
                            current_value[r][c] = -10000
                if (self.check(prev_value, current_value)):
                    # print("Breaking at: " + str(num_iterations))
                    break
                prev_value = current_value
                current_value = np.zeros([self.rows, self.cols])

            for r in range(self.rows):
                for c in range(self.cols):
                    temp = None
                    if (not self.occupancy_grid[r][c]):
                        for a in actionset:
                            tot = 0
                            dist = self.bfs_simulator((r, c), a, actionset, prob_action, prob_neighbour,
                                                             prob_ortho)
                            for prob in dist:
                                tot += prob[1] * prev_value[prob[0][0]][prob[0][1]]
                            if (tot > temp) or (temp is None):
                                temp = tot
                                new_policy[r][c] = a
            if self.check(old_policy, new_policy):
                print("Number of Iterations to Converge:" + str(num_iterations))
                return current_value, new_policy
            old_policy = new_policy
            new_policy = np.empty([self.rows, self.cols], dtype=object)
            prev_value = current_value
            current_value = np.zeros([self.rows, self.cols])

    def display_values(self, values=[]):
        display_value = copy.deepcopy(values)
        for r in range(self.rows):
            for c in range(self.cols):
                if (not self.occupancy_grid[r][c]):
                    if values [r][c] < 0:
                        values [r][c] = abs(values[r][c])
                        display_value[r][c] = -(values[r][c] ** (0.2))
                    else:
                        display_value[r][c] = values[r][c] ** (0.2)
        cmap = plotter.cm.get_cmap("inferno")
        cmap.set_under('w')
        mi_ma = self.mm(display_value)
        imgplot = plotter.imshow(display_value, cmap=cmap, vmin=mi_ma[0], vmax=mi_ma[1])
        imgplot.set_interpolation('nearest')
        for r in range(self.rows):
            for c in range(self.cols):
                if (self.occupancy_grid[r][c]):
                    val = ' '
                else:
                    val = str(round(values[r][c], 3))
                plotter.text(c, r, val, ha='center', va='center', fontsize=6, color='cyan', weight='bold')
        plotter.show()

    def display_policy(self, values=[], policy=[]):
        display_value = copy.deepcopy(values)
        for r in range(self.rows):
            for c in range(self.cols):
                if (not self.occupancy_grid[r][c]):
                    display_value[r][c] = values[r][c] ** (0.2)
        cmap = plotter.cm.get_cmap("inferno")
        cmap.set_under('w')
        mi_ma = self.mm(display_value)
        imgplot = plotter.imshow(display_value, cmap=cmap, vmin=mi_ma[0], vmax=mi_ma[1])
        imgplot.set_interpolation('nearest')
        for r in range(self.rows):
            for c in range(self.cols):
                if (policy[r][c] == 'u'):
                    dc = 0
                    dr = -0.25
                elif (policy[r][c] == 'l'):
                    dc = 0
                    dr = 0.25
                elif (policy[r][c] == 'r'):
                    dc = -0.25
                    dr = 0
                elif (policy[r][c] == 'd'):
                    dc = 0.25
                    dr = 0
                elif (policy[r][c] == 'ne'):
                    dc = 0.25
                    dr = -0.25
                elif (policy[r][c] == 'se'):
                    dc = -0.25
                    dr = -0.25
                elif (policy[r][c] == 'nw'):
                    dc = 0.25
                    dr = 0.25
                elif (policy[r][c] == 'sw'):
                    dc = -0.25
                    dr = 0.25
                if ((policy[r][c] is not None) and (self.goal != (r, c))):
                    plotter.arrow(c, r, dc, dr, lw=2, head_width=0.07, head_length=0.05, color='cyan')
        plotter.show()

    def mm(self, values):
        min = 10000
        max = -10000
        for r in range(self.rows):
            for c in range(self.cols):
                if(not self.occupancy_grid[r][c]):
                    if (values[r][c] < min):
                        min = values[r][c]
                    elif (values[r][c] > max):
                        max = values[r][c]
        return (min, max)

    def display_map(self, path=[], visited={}):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        '''
        display_grid = np.array(self.occupancy_grid, dtype=np.float32)

        # Color all visited nodes if requested
        for v in visited:
            display_grid[v] = _VISITED_COLOR
        # Color path in increasing color from init to goal
        if path is not None:
            for i, p in enumerate(path):
                disp_col = _INIT_COLOR + _PATH_COLOR_RANGE*(i+1)/len(path)
                display_grid[p] = disp_col

        display_grid[self.init_pos] = _INIT_COLOR
        display_grid[self.goal] = _GOAL_COLOR

        # Plot display grid for visualization
        imgplot = plotter.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('spectral')
        plotter.show()

    def uninformed_heuristic_euclidean(self, s):
        '''
        Example of how a heuristic may be provided. This one is admissable, but dumb.

        s - tuple describing the state as (row, col) position on the grid.

        returns - floating point estimate of the cost to the goal from state s
        '''
        ed = np.linalg.norm(np.array(s) - np.array(self.goal))
        return ed

    def uninformed_heuristic_manhattan(self, s):
        '''
        Example of how a heuristic may be provided. This one is admissable, but dumb.

        s - tuple describing the state as (row, col) position on the grid.

        returns - floating point estimate of the cost to the goal from state s
        '''
        #md = abs (x1 - x) + abs (y1 - y)
        md = abs (s[_X] - self.goal[_X]) + abs (s[_Y] - self.goal[_Y])
        return md

class SearchNode:
    def __init__(self, s, A, parent=None, parent_action=None, cost=0):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''
        self.parent = parent
        self.cost = cost
        self.parent_action = parent_action
        self.state = s[:]
        self.actions = A[:]

    def __str__(self):
        '''
        Return a human readable description of the node
        '''
        return str(self.state) + ' ' + str(self.actions)+' '+str(self.parent)+' '+str(self.parent_action)

class PriorityQ:
    '''
    Priority queue implementation with quick access for membership testing
    Setup currently to only with the SearchNode class
    '''
    def __init__(self):
        # type: () -> object
        '''
        Initialize an empty priority queue
        '''
        self.l = [] # list storing the priority q
        self.s = set() # set for fast membership testing

    def __contains__(self, x):
        '''
        Test if x is in the queue
        '''
        return x in self.s

    def push(self, x, cost):
        '''
        Adds an element to the priority queue.
        If the state already exists, we update the cost
        '''
        if x.state in self.s:
        #if x in self.s:
            return self.replace(x, cost)
        heapq.heappush(self.l, (cost, x))
        self.s.add(x.state)
        #self.s.add(x)

    def pop(self):
        # type: () -> object
        '''
        Get the value and remove the lowest cost element from the queue
        '''
        x = heapq.heappop(self.l)
        self.s.remove(x[1].state)
        #self.s.remove(x[1])
        return x[1]

    def peak(self):
        '''
        Get the value of the lowest cost element in the priority queue
        '''
        x = self.l[0]
        return x[1]

    def __len__(self):
        '''
        Return the number of elements in the queue
        '''
        return len(self.l)

    def replace(self, x, new_cost):
        '''
        Removes element x from the q and replaces it with x with the new_cost
        '''
        for y in self.l:
            if x.state == y[1].state:
            #if x == y[1] :
                self.l.remove(y)
                self.s.remove(y[1].state)
                #self.s.remove(y[1])
                break
        heapq.heapify(self.l)
        self.push(x, new_cost)

    def get_cost(self, x):
        '''
        Return the cost for the search node with state x.state
        '''
        for y in self.l:
            if x.state == y[1].state:
            #if x == y[1] :
                return y[0]

    def __str__(self):
        '''
        Return a string of the contents of the list
        '''
        return str(self.l)

def path(node):
        goal = []
        cost = 0
        while node is not None:
            #goal = [node.state] + goal
            if node.parent_action is not None:
                goal = [node.parent_action] + goal
            cost = cost + node.cost
            #print cost, goal
            node = node.parent

        return goal

def dfs(init_state, f, is_goal, actions):
    '''
    Perform depth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    # Fill me in!

    frontier = []  # Search stack
    n0 = SearchNode(init_state, actions)
    print n0
    visited = []
    frontier.append(n0)
    while len(frontier) > 0:
        # Peak last element
        print len(frontier)
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (path(n_i), visited)
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    n_prime = SearchNode(s_prime, actions, n_i, a)
                    print n_prime
                    frontier.append(n_prime)

                    print n_prime
    print "path not found"
    return (None, visited)

def dfsdepth(init_state, f, is_goal, actions, m):
    frontier = []  # Search stack
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.append(n0)
    while len(frontier) > 0:
        # Peak last element
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            #print ("appended" + str(n_i.state))
            if is_goal(n_i.state):
                return (path(n_i), visited)
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    n_prime = SearchNode(s_prime, actions, n_i, a)
                    if len(path(n_prime)) <= m:
                        frontier.append(n_prime)

    return (None, visited)

def iddfs(init_state, f, is_goal, actions, m):
    '''
    Perform depth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    # Fill me in!
    ans = []
    for i in range(0, m+1):
        ans = dfsdepth(init_state, f, is_goal, actions, i)
        if ans[0] is not None:
            return ans
    return ans


def bfs(init_state, f, is_goal, actions):
    '''
    Perform breadth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    # Fill me in!
    frontier = []  # Search stack
    n0 = SearchNode(init_state, actions)
    print n0
    visited = []
    frontier.append(n0)
    while len(frontier) > 0:
        # Peak last element
        #print frontier[0]
        n_i = frontier.pop(0)
        if n_i.state not in visited:
            visited.append(n_i.state)
            #print visited
            if is_goal(n_i.state):
                return (path(n_i), visited)
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    n_prime = SearchNode(s_prime, actions, n_i, a)
                    frontier.append(n_prime)
                    print n_prime
    print "path not found"
    return (None, visited)

def g(state):

    #print 'my value of state is' , state
    if (state.parent is not None and state.parent_action is not None):
        if state.parent_action == 'u' or state.parent_action == 'd' or state.parent_action =='l' or state.parent_action == 'r':
            state.cost = state.parent.cost + 1.0
        elif state.parent_action == 'ne' or state.parent_action == 'nw' or state.parent_action =='se' or state.parent_action == 'sw':
            state.cost = state.parent.cost + 1.5
    else:
        state.cost = 0
        #print action
        #state=state.parent
        #print cost
    return state.cost


def uniform_cost_search(init_state, f, is_goal, actions):
    # Fill me in!
    frontier = PriorityQ()
    n0 = SearchNode(init_state, actions)
    frontier.push(n0, g(n0))
    visited = []
    while len(frontier) > 0:
        s = frontier.pop()
        print s.state, s.cost
        if s.state not in visited:
            visited.append(s.state)
        #print visited
            if is_goal(s.state):
            #print path(s)
                return (path(s), visited)
        #visited.append(s.state)
        #visited[s] = s.cost
            else:
                for a in actions:
                    s_prime = f(s.state, a)
            #print s_prime, a
                    n_prime = SearchNode(s_prime, actions, s, a)
                    g(n_prime)
            #n_prime.cost = g(n_prime)
            #print(n_prime.cost)
                    # f(s_prime) = g(s_prime) + h(s_prime)
                    if n_prime.state not in visited and n_prime.state not in frontier:
                        frontier.push(n_prime, n_prime.cost)
                    elif (frontier.get_cost(n_prime) > n_prime.cost):
                        frontier.replace(n_prime, n_prime.cost)
            #elif n_prime in frontier and g(n_prime) < frontier.get_cost(n_prime):
             #   frontier.replace(n_prime, g(n_prime))
            #print n_prime.cost
            #print n_prime
    print "path not found"
    return (None, visited)



def a_star_search(init_state, f, is_goal, actions, h):
    '''
    init_state - value of the initial state
    f - transition function takes input state (s), action (a), returns s_prime = f(s, a)
        returns s if action is not valid
    is_goal - takes state as input returns true if it is a goal state
        actions - list of actions available
    h - heuristic function, takes input s and returns estimated cost to goal
        (note h will also need access to the map, so should be a member function of GridMap)
    '''
    # Fill me in!
    frontier = PriorityQ()
    n0 = SearchNode(init_state, actions)
    frontier.push(n0, g(n0) + h(n0.state))
    visited = []
    while len(frontier) > 0:
        s = frontier.pop()
        #print s.state, s.cost
        if s.state not in visited:
            visited.append(s.state)
            print visited
            if is_goal(s.state):
            #print path(s)
                return (path(s), visited)
            else:
                for a in actions:
                    s_prime = f(s.state, a)
            #print s_prime, a
                    n_prime = SearchNode(s_prime, actions, s, a)
                    g(n_prime)
                    l = n_prime.cost + h(n_prime.state)
            #print g(n_prime), h(n_prime.state), l, s_prime
            #print l, s_prime
                    if (n_prime.state not in visited and n_prime.state not in frontier):
                        frontier.push(n_prime, l)
                    elif (frontier.get_cost(n_prime) > l):
                        frontier.replace(n_prime, l)
            #print frontier
            #print n_prime
            #print n_prime.cost
    print "path not found"
    return (None, visited)


def backpath(node):
    '''
    Function to determine the path that lead to the specified search node

    node - the SearchNode that is the end of the path

    returns - a tuple containing (path, action_path) which are lists respectively of the states
    visited from init to goal (inclusive) and the actions taken to make those transitions.
    '''
    path = []
    action_path = []
    # Fill me in!
    return (path, action_path)



