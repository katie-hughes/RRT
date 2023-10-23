import numpy as np
import matplotlib.pyplot as plt
import random
import imageio.v2 as imageio
import math
import argparse

def distance(p1, p2):
    return np.linalg.norm(np.array(p2)-np.array(p1))

class Node:
    def __init__(self,val,parent):
        self.parent = parent
        self.val = val
        self.children = []
    def add_child(self, c):
        self.children.append(c)
    def pr(self,lvl=0):
        if len(self.children)==0:
            print(f"Val {lvl}:{self.val}")
        else: 
            for c in self.children: 
                c.pr(lvl=lvl+1)

class RRT:
    def __init__(self, qinit, k, delta, d, verbose=False):
        # starting position
        self.qinit = qinit
        # number of steps
        self.k = k
        # step size
        self.delta = delta
        # domain of graph
        self.d = d
        # print out extra info
        self.verbose = verbose
        # keep track of the graph structure of connected nodes
        self.g = Node(qinit,parent=None)

        # plotting 
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.axes.set_xlim3d(left=self.d[0][0], right=self.d[0][1]) 
        ax.axes.set_ylim3d(bottom=self.d[1][0], top=self.d[1][1]) 
        ax.axes.set_zlim3d(bottom=self.d[2][0], top=self.d[2][1]) 
        self.fig = fig
        self.ax = ax

        # set a random goal position that is at least 10 units away from the start
        dst = 0 
        goalx = -1
        goaly = -1
        goalz = -1
        while dst < 10: 
            goalx = random.randrange(self.d[0][0], self.d[0][1])
            goaly = random.randrange(self.d[1][0], self.d[1][1])
            goalz = random.randrange(self.d[2][0], self.d[2][1])
            self.goal = (goalx, goaly,goalz)
            dst = distance(self.goal, self.qinit)
        # buffer for reaching goal
        self.buff = 2*self.delta
        
    def random_configuration(self):
        #Generate random position in the domain
        x = random.randrange(self.d[0][0], self.d[0][1])
        y = random.randrange(self.d[1][0], self.d[1][1])
        z = random.randrange(self.d[2][0], self.d[2][1])
        if self.verbose: print("Random:", x,y,z)
        return (x,y,z)
    
    def nearest_vertex(self, nd, pos):
        # find the vertex in g closest to given pos
        closest_node = nd
        closest_dist = distance(closest_node.val, pos)
        for n in closest_node.children: 
            subtree_dist, subtree_node = self.nearest_vertex(n, pos)
            if subtree_dist < closest_dist: 
                closest_dist = subtree_dist
                closest_node = subtree_node
        return closest_dist, closest_node
            
        
    def new_configuration(self, q_start, q_direction):
        qs_array = np.array(q_start.val)
        qd_array = np.array(q_direction)
        # generate new tree config by moving delta from one vertex to another
        vector = qd_array - qs_array
        unit_vector = vector / np.linalg.norm(vector)
        new_point = qs_array + unit_vector
        new_x = new_point[0]
        new_y = new_point[1]
        new_z = new_point[2]
        if self.verbose: print(f'New candidate point:{new_x},{new_y},{new_z}')
        if not ((self.d[0][0] < new_x < self.d[0][1]) and 
                (self.d[1][0] < new_y < self.d[1][1]) and 
                (self.d[2][0] < new_y < self.d[2][1])):
            if self.verbose: print("OUT OF BOUNDS")
            return None
        if self.verbose: print("drawing line")
        self.ax.plot([q_start.val[0], new_x], [q_start.val[1], new_y], zs=[q_start.val[2],new_z], color='b')
        new_node = Node(val=(new_x, new_y, new_z), parent=q_start)
        q_start.add_child(new_node)
        if distance((new_x, new_y, new_z), self.goal) < self.buff: 
            print("Goal Reached!!")
            return new_node
        else: 
            return None
    
    def drawPath(self, node, co): 
        current = node.val
        self.ax.plot([co[0], current[0]], [co[1], current[1]], zs=[co[2], current[2]], color='r')
        if node.parent is not None: 
            self.drawPath(node.parent, current)
    
    def go(self):
        # Run RRT
        if self.verbose: print("GO")
        for i in range(0, self.k):
            if self.verbose: print(f'\n')
            if (i % 1000 == 0):print(f'{i}/{self.k}')
            qrand = self.random_configuration()
            qnear_dist, qnear = self.nearest_vertex(self.g, qrand)
            qnew = self.new_configuration(qnear, qrand)
            if qnew is not None: 
                print(f"Finished after {i} iterations!")
                self.drawPath(qnew, self.goal)
                break

        self.ax.scatter(self.qinit[0], self.qinit[1], self.qinit[2],
                        color='r',
                        label=f'Start\n{self.qinit}')
        self.ax.scatter(self.goal[0], self.goal[1], self.goal[2],
                        color='orange',
                        label=f'End\n{self.goal}')

        plt.legend(loc='upper left', bbox_to_anchor=(-0.35, 0.5))
        

def main():
    parser = argparse.ArgumentParser(
                    prog='RRT',
                    description='Implements a 3D RRT path planning algorithm')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print out debug messages')
    args = parser.parse_args()

    # some parameters of the RRT
    # domain of the graph: 50x50x50
    d = [(0,50),(0,50),(0,50)]
    # starting position
    qinit = (10,10,10)
    # increment distance
    delta = 2
    # number of iterations
    k = 10000

    Task1 = RRT(qinit, k, delta, d, verbose=args.verbose)
    Task1.go()
    plt.savefig('images/3d.png')
    plt.show()

if __name__ == "__main__":
    main()