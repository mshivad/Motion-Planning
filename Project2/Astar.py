import numpy as np
import heapq

class SearchNode:
    def __init__(self, s, A, parent=None, cost=0):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''
        self.parent = parent
        self.cost = cost
        self.state = tuple(s[:])
        self.actions = A

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
            return self.replace(x, cost)
        heapq.heappush(self.l, (cost, x))
        self.s.add(x.state)

    def pop(self):
        '''
        Get the value and remove the lowest cost element from the queue
        '''
        x = heapq.heappop(self.l)
        self.s.remove(x[1].state)
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
                self.l.remove(y)
                self.s.remove(y[1].state)
                break
        heapq.heapify(self.l)
        self.push(x, new_cost)

    def get_cost(self, x):
        '''
        Return the cost for the search node with state x.state
        '''
        for y in self.l:
            if x.state == y[1].state:
                return y[0]

def getCost(node):
    if(node.parent is not None):
        dist = np.linalg.norm(np.array(node.parent.state) - np.array(node.state))
        node.cost = node.parent.cost + dist
    else:
        node.cost = 0
    return node.cost

def h(node, goals):
    dist = 10000
    for goal in goals:
        euclidean = np.linalg.norm(node.state - goal.vertex)
        if(euclidean < dist):
            dist = euclidean
    return dist

def path(node):
    togoal = []
    while node is not None:
        togoal = [node.state] + togoal
        node = node.parent
    return togoal

def a_star_search(init_states, nodes , goal_states):
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
    frontier = PriorityQ()  # Search stack
    goalset = []
    for goal in goal_states:
        goalset.append(tuple(goal.vertex))
    for state in init_states:
        n0 = SearchNode(state.vertex, state.adjacent)
        n0.cost = 0
        frontier.push(n0, getCost(n0) + h(n0, goal_states))
    visited = []
    #print(h(n0.state))
    while len(frontier) > 0:
        # Peak last element
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            # print ("appended")
            if n_i.state in goalset:
                print("Cost to Goal: " + str(n_i.cost))
                return path(n_i)
            else:
                if n_i.actions is not None:
                    for a in n_i.actions:
                        # s_prime = f(n_i.state, a)
                        n_prime = SearchNode(a.vertex, a.adjacent, n_i)
                        getCost(n_prime)
                        if (n_prime not in frontier and n_prime.state not in visited):
                            frontier.push(n_prime, n_prime.cost + h(n_prime, goal_states))
                        elif (frontier.get_cost(n_prime) > n_prime.cost + h(n_prime, goal_states)):
                            frontier.replace(n_prime, n_prime.cost + h(n_prime, goal_states))

    return None