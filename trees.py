import numpy as np
import matplotlib.pyplot as plt
import random


class Node:
    def __init__(self,start):
        self.parent = None
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
        print(q_start)
        print(q_direction)
        qsx = q_start.val[0]
        qsy = q_start.val[1]
        qdx = q_direction[0]
        qdy = q_direction[1]
        ## CHECK DIV BY 0
        # generate new tree config by moving delta from one vertex to another
        theta = np.arctan((qdy - qsy)/(qdx - qsx))
        print(theta)
        print(theta/np.pi, "radians")
        print(theta*180/np.pi, "deg")
        y = self.delta * np.sin(theta)
        x = self.delta * np.cos(theta)
        print(x, y)
        if (qdx-qsx)<0: 
            new_x = qsx - x
            new_y = qsy - y
        else: 
            new_x = qsx + x
            new_y = qsy + y
        print(new_x, new_y)
        plt.scatter(qsx, qsy, color='r')
        plt.scatter(qdx, qdy, color='g')
        plt.scatter(new_x, new_y, color='k')
        plt.title('RED = start, GREEN = dir, BLACK = new')
        plt.show()


    
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
k = 10
Task1 = RRT(qinit, k, delta, d)
Task1.go()