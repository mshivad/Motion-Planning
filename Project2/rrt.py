#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import matplotlib.pyplot as plotter
from math import pi,sqrt,cos,sin,atan2
from collisions import PolygonEnvironment
import time
import random
from operator import sub

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

class TreeNode:
    def __init__(self, state, parent=None):
        self.state = state
        self.children = []
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)

class RRTSearchTree:
    def __init__(self, init):
        self.root = TreeNode(init)
        self.nodes = [self.root]
        self.edges = []

    def find_nearest(self, s_query):
        min_d = 1000000
        nn = self.root
        for n_i in self.nodes:
            d = np.linalg.norm(s_query - n_i.state)
            if d < min_d:
                nn = n_i
                min_d = d
        return (nn, min_d)

    def near_goal(self, goal, robot):
        tempv = robot.fk(goal)
        goalee = tempv[3]
        min_d = 1000000
        nn = self.root
        for n_i in self.nodes:
            tempv = robot.fk(n_i.state)
            n_iee = tempv[3]
            d = np.linalg.norm(goalee - n_iee)
            if d < min_d:
                nn = n_i
                min_d = d
        return (nn, min_d)

    def add_node(self, node, parent):
        self.nodes.append(node)
        self.edges.append((parent.state, node.state))
        node.parent = parent
        parent.add_child(node)

    def get_states_and_edges(self):
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

    def get_back_path(self, n):
        path = []
        while n.parent is not None:
            path.append(n.state)
            n = n.parent
        path.reverse()
        return path

class RRT:

    def __init__(self, num_samples, num_dimensions=2, step_length = 1, lims = None,
                 connect_prob = 0.1, collision_func=None):
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.connect_prob = connect_prob

        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = self.fake_in_collision

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in xrange(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)
            print self.limits

        self.ranges = self.limits[:,1] - self.limits[:,0]
        self.found_path = False

    def build_rrt(self, init, goal, p):
        '''
        Build the rrt from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.T = RRTSearchTree(init)
        for each in range(1,self.K):
            randconfig = self.sample()
            #print randconfig
            self.extend(randconfig)
            lastnode = self.T.nodes[-1]
            if np.linalg.norm(lastnode.state - self.goal)<= self.epsilon:
                finalpath=self.T.get_back_path(self.T.nodes[-1])
                return finalpath
        if (len(self.goal) < 3):
            gnear = self.T.find_nearest(self.goal)
            return None
        else:
            g = self.T.near_goal(self.goal, p)
            return self.T.get_back_path(g[0])
        #print Final_Path


    def build_rrt_connect(self, init, goal):
        '''
        Build the rrt connect from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.T = RRTSearchTree(init)
        result = _TRAPPED
        randconfig = self.sample()
        for each in range(0,self.K):
            if (result == _TRAPPED) or (result == _REACHED):
                randconfig = self.sample()
            result = self.extend(randconfig)
            if (result == _REACHED) and (np.linalg.norm(self.T.nodes[-1].state - self.goal) <= self.epsilon):
                finalpath=self.T.get_back_path(self.T.nodes[-1])
                return finalpath
        return None

    def build_rrt_bi_connect(self, init, goal):
        self.init = np.array(init)
        self.goal = np.array(goal)
        self.T = RRTSearchTree(init)
        self.T1 = RRTSearchTree(init)
        self.Tb = RRTSearchTree(goal)
        T1 = self.Ta
        T2 = self.Tb
        result1 = _TRAPPED
        result2 = _TRAPPED
        randconfig = self.sample()
        for k in range(self.K):
            if (result1 == _TRAPPED) or (result1 == _REACHED):
                randconfig = self.sample()
            result1 = _ADVANCED
            self.T = T1
            while (result1 == _ADVANCED):
                result1 = self.extend(randconfig)
            if np.array_equal(T1.nodes[-1].state, T2.nodes[-1].state):
                plan1 = self.Ta.get_back_path(self.Ta.nodes[-1])
                plan2 = self.Tb.get_back_path(self.Tb.nodes[-1])
                plan2.reverse()
                plan = plan1 + plan2
                self.T.nodes = self.Ta.nodes + self.Tb.nodes
                self.T.edges = self.Ta.edges + self.Tb.edges
                return plan
            res2 = _ADVANCED
            self.T = T2
            while (res2 == _ADVANCED):
                res2 = self.extend(T1.nodes[-1].state)
            if np.array_equal(T1.nodes[-1].state, T2.nodes[-1].state):
                plan1 = self.Ta.get_back_path(self.Ta.nodes[-1])
                plan2 = self.Tb.get_back_path(self.Tb.nodes[-1])
                plan2.reverse()
                plan = plan1 + plan2
                self.T.nodes = self.Ta.nodes + self.Tb.nodes
                self.T.edges = self.Ta.edges + self.Tb.edges
                return plan
            temp = T1
            T1 = T2
            T2 = temp
        self.T.nodes = self.Ta.nodes + self.Tb.nodes
        self.T.edges = self.Ta.edges + self.Tb.edges
        return None

    def sample(self):
        '''
        Sample a new configuration and return
        '''
        # Return goal with connect_prob probability
        # Fill me in!
        randnumb = random.random()
        if(randnumb <= self.connect_prob):
            return self.goal
        else:
            new_config = []
            for value in range (0,self.n,1):
                new_config.append(random.uniform(self.limits[value][0],self.limits[value][1]))
            return np.array(new_config)

    def extend(self, q):
        '''
        Perform rrt extend operation.
        q - new configuration to extend towards
        '''
        # Fill me in!
        (nearnode,dist) = self.T.find_nearest(q)
        new_configuration = self.new_config_generator(nearnode.state , q, dist)   # new_config is (x_, y_) not a node
        if self.in_collision(new_configuration) == False:
            new_node = TreeNode(new_configuration)
            self.T.add_node(new_node,nearnode)
            if np.linalg.norm(np.array(new_configuration) - np.array(q)) == 0:
                return _REACHED
            else:
                return _ADVANCED
        return _TRAPPED

    def extend_connect(self, q):
        (neartnode,dist) = self.T.find_nearest(q)
        while(dist > self.epsilon):
            new_configuration = self.new_config_generator(nearnode.state , q, dist)
            if self.in_collision(new_configuration) == False:
                new_node = TreeNode(new_configuration)
                self.T.add_node(new_node,nearnode)
                if((np.linalg.norm(new_node.state - self.goal))<= self.epsilon):
                    new_node = TreeNode(self.goal)
                    self.T.add_node(new_node,nearnode)
                    return _REACHED
                else:
                    nearnode = new_node
                    dist = np.linalg.norm(new_node.state - q)
            else:
                return _ADVANCED
        if(dist <= self.epsilon):
            if self.in_collision(q) == False:
                new_node = TreeNode(q)
                self.T.add_node(new_node,nearnode)
                if((np.linalg.norm(new_node.state - self.goal))<= self.epsilon):
                    new_node = TreeNode(self.goal)
                    self.T.add_node(new_node,nearnode)
                    return _REACHED
            return  _ADVANCED


    def extend_bi_connect(self, Tree, q):
        (nearestnode,dist) = Tree.find_nearest(q)
        while(dist > self.epsilon):
            new_config = self.new_config_generator(nearestnode.state , q, dist)
            if self.in_collision(new_config) == False:
                new_tree_node = TreeNode(new_config)
                Tree.add_node(new_tree_node,nearestnode)
                nearestnode = new_tree_node
                dist = np.linalg.norm(new_tree_node.state - q)
            else:
                break
        if(dist <= self.epsilon):
            if self.in_collision(q) == False:
                new_tree_node = TreeNode(q)
                Tree.add_node(new_tree_node,nearestnode)
                return _REACHED
        return  _ADVANCED


    def new_config_generator(self, near,random,distance):
        if distance < self.epsilon:
            return random
        else:
            result = near + self.epsilon*((random-near)/distance)   #vector manipulation.
            return result


    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False

def test_rrt_env(num_samples=2500, step_length=0.18, env='./env1.txt', connect= 1):
    '''
    create an instance of PolygonEnvironment from a description file and plan a path from start to goal on it using an RRT

    num_samples - number of samples to generate in RRT
    step_length - step size for growing in rrt (epsilon)
    env - path to the environment file to read
    connect - If True run rrt_connect

    returns plan, planner - plan is the set of configurations from start to goal, planner is the rrt used for building the plan
    '''
    pe = PolygonEnvironment()
    pe.read_env(env)

    dims = len(pe.start)
    start_time = time.time()

    rrt = RRT(num_samples,
              dims,
              step_length,
              lims = pe.lims,
              connect_prob = 0.05,
              collision_func=pe.test_collisions)
    if connect==1:
        plan = rrt.build_rrt_connect(pe.start, pe.goal)
    elif connect==0:
        plan = rrt.build_rrt(pe.start, pe.goal, pe.robot)
    elif connect ==2:
        plan = rrt.build_rrt_bi_connect(pe.start, pe.goal)

    run_time = time.time() - start_time
    print 'plan:', plan
    print 'run_time =', run_time
    #print 'plan length =', len(plan)
    pe.draw_plan(plan, rrt, dynamic_plan=True)
    plotter.pause(10000)
    return plan, rrt

