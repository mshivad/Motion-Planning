#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import matplotlib.pyplot as plotter
from math import pi,sqrt,cos,sin,atan2
from collisions import PolygonEnvironment
import time
from Astar import a_star_search
from rrt import RRT
import random
import copy

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

class TreeNode:

    def __init__(self, vertex):
        self.vertex = vertex
        self.adjacent = []

    def add_adjacent(self, a):
        if(self.adjacent is None):
            self.adjacent = [a]
        else:
            self.adjacent.append(a)

class PRMSearchTree:

    def __init__(self):
        self.nodes = []
        self.edges = []

    def add_node(self, node):
        self.nodes.append(node)

    def get_states_and_edges(self):
        states = np.array([n.vertex for n in self.nodes])
        return (states, self.edges)

class PRM:

    def __init__(self, collision_func, num_dimensions, num_samples = 300, step_length = 2, lims=None, localrad=20, var=15, no_closest_neighbours = 4):
        self.in_collision = collision_func
        self.epsilon = step_length
        self.localrad = localrad
        self.K = num_samples
        self.n = num_dimensions
        self.variance = var
        self.Nq = no_closest_neighbours
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in xrange(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)


    def is_collision_freepath(self,q1,q2):
        distance = np.linalg.norm(np.array(q1) - np.array(q2))
        while distance >= 0:
            new_config = self.directed_random_configuration(q1 , q2, distance)
            if self.in_collision(new_config):
                return True
            distance -= self.epsilon
        return False


    def directed_random_configuration(self, near, rand, distance):
        if distance < self.epsilon:
            return rand
        else:
            new = near + self.epsilon * ((rand - near) / distance)
            return new

    def sample(self):
        randsample = []
        for i in range(0,self.n):
            randsample.append(np.random.uniform(self.limits[i][0],self.limits[i][1]))
        return np.array(randsample)

    def gaussiansample(self):
        while True:
            collisions = 0
            sample1 = []
            for i in range(0, self.n):
                sample1.append(np.random.uniform(self.limits[i][0], self.limits[i][1]))
            sample2 = np.random.normal(sample1, self.variance)
            if(self.in_collision(sample1) and not self.in_collision(sample2)):
                return np.array(sample2)
            elif ( not self.in_collision(sample1) and self.in_collision(sample2)):
                return np.array(sample1)

    def find_neighbours(self, node):
        neighbours = []
        for map_node in self.T.nodes:
            if np.linalg.norm(map_node.vertex - node) <= self.localrad:
                if not self.is_collision_freepath(map_node.vertex, node):
                    neighbours.append(map_node)
        if (len(neighbours) < self.Nq):
            return neighbours
        return neighbours[0:self.Nq]

    def find_neighbours_no_cc(self, node):
        neighbours = []
        for map_node in self.T.nodes:
            dist = np.linalg.norm(map_node.vertex - node)
            if dist <= self.localrad:
                neighbours.append((dist, map_node))
        sorted_near = sorted(neighbours)
        limiter = self.Nq
        result = []
        if (len(neighbours) < self.Nq):
            limiter = len(neighbours)
        for i in range(limiter):
            result.append(sorted_near[i][1])
        return result

    def rrt_local(self, pe, start, end):
        dims = len(start.vertex)
        lims = []
        for i in range(dims):
            if start.vertex[i] < end.vertex[i]:
                min = start.vertex[i] - 20
                max = end.vertex[i] + 20
            else:
                max = start.vertex[i] + 20
                min = end.vertex[i] - 20
            lims.append([min,max])

        rrt = RRT(50,
                  dims,
                  self.epsilon,
                  lims=np.array(lims),
                  connect_prob=.15,
                  collision_func=pe.test_collisions)
        plan = rrt.build_rrt(start.vertex, end.vertex, pe.robot)
        return plan

    def build_prm_rrt(self, pe):
        self.T = PRMSearchTree()
        for k in range(self.K):
            new = self.sample()
            #new = self.gaussiansample()
            if not self.in_collision(new):
                # near = self.find_neighbours(new)
                newnode = TreeNode(new)
                self.T.add_node(newnode)
                # print("added" + str(len(self.T.node)))
        prmnodes = self.T.nodes
        added = []
        for nn in prmnodes:
            added.append(nn)
            near = self.find_neighbours_no_cc(nn.vertex)
            for ne in near:
                if ne not in added:
                    localplan = self.rrt_local(pe,nn,ne)
                    localnodes = []
                    if(localplan is not None):
                        if(len(localplan) != 0):
                            for rrtcoord in localplan:
                                localnodes.append(TreeNode(tuple(rrtcoord)))
                            if (len(localnodes) == 0):
                                print("empty")
                            nn.add_adjacent(localnodes[0])
                            localnodes[0].add_adjacent(nn)
                            self.T.edges.append((nn.vertex, localnodes[0].vertex))
                            ne.add_adjacent(localnodes[-1])
                            localnodes[-1].add_adjacent(ne)
                            self.T.edges.append((ne.vertex, localnodes[-1].vertex))
                            # self.T.node.append(localplan[0])
                            for i in range(1, len(localnodes)):
                                localnodes[i - 1].add_adjacent(localnodes[i])
                                localnodes[i].add_adjacent(localnodes[i - 1])
                                # self.T.node.append(localplan[i])
                                self.T.edges.append((localnodes[i - 1].vertex, localnodes[i].vertex))
        return None

    def build_prm(self, gaussian = False):

        self.T = PRMSearchTree()
        for k in range(self.K):
            if(gaussian):
                new = self.gaussiansample()
            else:
                new = self.sample()
            if not self.in_collision(new):
                # near = self.find_neighbours(new)
                newnode = TreeNode(new)
                self.T.add_node(newnode)
                # print("added" + str(len(self.T.node)))
        added = []
        for nn in self.T.nodes:
            # print("h")
            added.append(nn)
            near = self.find_neighbours(nn.vertex)
            for i in range(len(near)):
                if near[i] not in added:
                    nn.add_adjacent(near[i])
                    near[i].add_adjacent(nn)
                    self.T.edges.append((nn.vertex, near[i].vertex))
                    # print(len(self.T.edges))
        return None


def test_prm_env(num_samples=500, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=False, rrtlp=True):
        pe = PolygonEnvironment()
        pe.read_env(env)

        dims = len(pe.start)
        start_time = time.time()

        prm = PRM(pe.test_collisions, dims, num_samples, step_length, pe.lims, localrad=localrad, var=variance,
                  no_closest_neighbours=5)
        if rrtlp:
            prm.build_prm_rrt(pe)
        else:
            prm.build_prm(gaussian)
        init_states = []
        goal_states = []
        for i in range(0, 2):
            init_states.append(prm.T.nodes[np.random.randint(0, len(prm.T.nodes))])
            goal_states.append(prm.T.nodes[np.random.randint(0, len(prm.T.nodes))])

        pe.goal = goal_states[0].vertex
        pe.start = init_states[0].vertex
        for g in goal_states:
            pe.addngoals.append(g.vertex)
        for i in init_states:
            pe.addnstart.append(i.vertex)
        plan = a_star_search(init_states, prm.T.nodes, goal_states)
        print 'plan:', plan
        # print 'plan length =', len(plan)
        # print 'run_time =', run_time
        pe.draw_plan(plan, prm, dynamic_plan=True)
        plotter.pause(10000)
        return plan, prm


