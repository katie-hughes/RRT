import numpy as np
import matplotlib.pyplot as plt
import random


class Node:

    def __init__(self,val):
        self.parent = None
        self.val = val
        self.children = []
    def add_parent(self, p):
        self.parent = p
    def add_child(self, c):
        self.children.append(c)
    def pr(self,lvl=0):
        if len(self.children)==0:
            print(f"Val {lvl}:{self.val}")
        else: 
            for c in self.children: 
                c.pr(lvl=lvl+1)


class RRT:
    def __init__(self, qinit, k, delta, d):
        self.qinit = qinit
        self.k = k
        self.delta = delta
        self.d = d

        self.g = Node(qinit)
        self.g.pr()
    
    def distance(self, p1, p2):
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    def random_configuration(self):
        print("Random config")
        #Generate random position in the domain
        x = random.randrange(self.d[0][0], self.d[0][1])
        y = random.randrange(self.d[1][0], self.d[1][1])
        print("Random:", x,y)
        return (x,y)
    
    def nearest_vertex(self, nd, pos):
        # find the vertex in g closest to given pos
        print("Nearest vertex")
        print('position', pos)
        print('node', nd)
        print('node children',nd.children)
        closest_node = nd
        closest_dist = self.distance(closest_node.val, pos)
        print('initial dist is', closest_dist)
        for n in closest_node.children: 
            print(n)
            subtree_dist, subtree_node = self.nearest_vertex(n, pos)
            if subtree_dist < closest_dist: 
                closest_dist = subtree_dist
                closest_node = subtree_node
        print("closest dist is", closest_dist)
        return closest_dist, closest_node
            
        
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
        """
        plt.scatter(qsx, qsy, color='r')
        plt.scatter(qdx, qdy, color='g')
        plt.scatter(new_x, new_y, color='k')
        plt.title('RED = start, GREEN = dir, BLACK = new')
        plt.show()
        """
        plt.plot([qsx, new_x], [qsy, new_y], color='b')
        new_node = Node((new_x, new_y))
        q_start.add_child(new_node)
        new_node.add_parent(q_start)
        print('val',q_start.val)
        print('children',q_start.children)


    
    def go(self):
        # Run RRT
        print("GO")
        plt.scatter(self.qinit[0], self.qinit[1], color='k')
        for i in range(0, self.k):
            print()
            print()
            self.g.pr()
            qrand = self.random_configuration()
            qnear_dist, qnear = self.nearest_vertex(self.g, qrand)
            qnew = self.new_configuration(qnear, qrand)
        self.g.pr()
        plt.show()


d = [(0,100),(0,100)]
qinit = (50,50)
delta = 1
k = 50
Task1 = RRT(qinit, k, delta, d)
Task1.go()