import numpy as np
import matplotlib.pyplot as plt
import random


class Node:
    def __init__(self,start):
        self.val = start
        self.children = []


class RRT:
    def __init__(self, qinit, k, delta, d):
        self.qinit = qinit
        self.k = k
        self.delta = delta
        self.d = d

        self.g = Node(qinit)
    
    def distance(self, p1, p2):
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    def random_configuration(self):
        print("Random config")
        #Generate random position in the domain
        x = random.randrange(self.d[0][0], self.d[0][1])
        y = random.randrange(self.d[1][0], self.d[1][1])
        print("Random:", x,y)
        return (x,y)
    
    def nearest_vertex(self, pos):
        # find the vertex in g closest to given pos
        print("Nearest vertex")
        closest = self.g
        dist = self.distance(self.g.val, pos)
        print(closest)
        print(dist)
        for n in self.g.children: 
            print(n)
        return closest
            
        
    def new_configuration(self, q_start, q_direction):
        print("new config")
        # generate new tree config by moving delta from one vertex to another
        

    
    def go(self):
        # Run RRT
        print("GO")
        for i in range(0, self.k):
            qrand = self.random_configuration()
            qnear = self.nearest_vertex(qrand)
            qnew = self.new_configuration(qnear, qrand)


d = [(0,100),(0,100)]
qinit = (50,50)
delta = 1
k = 3
Task1 = RRT(qinit, k, delta, d)
Task1.go()