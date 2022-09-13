import numpy as np
import matplotlib.pyplot as plt
import random



def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


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

class Circle: 
    def __init__(self, center, radius):
        self.c = center
        self.r = radius
    def inside(self, point): 
        return (distance(center, point) <= self.r)



class RRT:
    def __init__(self, qinit, k, delta, d, circles=False):
        self.qinit = qinit
        self.k = k
        self.delta = delta
        self.d = d

        self.g = Node(qinit)
        self.g.pr()

        self.circles = []
        self.doCircles = False
        fig, ax = plt.subplots()
        self.fig = fig
        self.ax = ax

        # Plot the starting point
        ax.scatter(qinit[0], qinit[1], color='r')

        if circles: 
            self.doCircles = True
            print("Running with circles!")
            c1 = Circle((45,45), 4)
            self.circles.append(c1)

            print("Circles:", c1.c, c1.r)

            
            circle1 = plt.Circle(c1.c, c1.r, color='k')
            
            self.ax.add_patch(circle1)
            #ax.scatter([d[0][0], d[0][1], qinit[0]], [d[1][0], d[1][1], qinit[1]])
            self.ax.set_aspect('equal')
            self.fig.show()
            #plt.show()
            
    
    
    
    def random_configuration(self):
        #print("Random config")
        #Generate random position in the domain
        x = random.randrange(self.d[0][0], self.d[0][1])
        y = random.randrange(self.d[1][0], self.d[1][1])
        print("Random:", x,y)
        return (x,y)
    
    def nearest_vertex(self, nd, pos):
        # find the vertex in g closest to given pos
        #print("Nearest vertex")
        #print('position', pos)
        #print('node', nd)
        #print('node children',nd.children)
        closest_node = nd
        closest_dist = distance(closest_node.val, pos)
        #print('initial dist is', closest_dist)
        for n in closest_node.children: 
            #print(n)
            subtree_dist, subtree_node = self.nearest_vertex(n, pos)
            if subtree_dist < closest_dist: 
                closest_dist = subtree_dist
                closest_node = subtree_node
        #print("closest dist is", closest_dist)
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
        if self.doCircles: 
            slope = (new_y - qsy) / (new_x - qsx)
            print("SLOPE:", slope)
            b = new_y - slope*new_x
            print("B:", b)
            tan_slope = -1./slope
            for c in self.circles: 
                tan_b = c.c[1] - tan_slope*c.c[0]
                x_int = (b - tan_b)/(tan_slope - slope)
                y_int = tan_slope*x_int+tan_b
                print("x and y int are", x_int, y_int)
                print("X lims are", qsx, new_x)
                print("y lims are", qsy, new_y)
                """
                plt.scatter(qsx, qsy, color='r')
                plt.scatter(new_x, new_y, color='b')
                plt.plot([qsx, new_x], [qsy, new_y], color='purple')
                plt.scatter(x_int, y_int, color='green')
                plt.scatter(c.c[0], c.c[1], color='orange')
                plt.plot([x_int, c.c[0]], [y_int, c.c[1]], color='k')
                plt.plot([x_int, new_x], [y_int, new_y], color='gray')
                plt.gca().set_aspect('equal', adjustable='box')
                plt.show()
                """
                if distance((x_int, y_int), c.c) < c.r: 
                    print("INTERSECT!!!!")
                    return False
                if qsx < x_int < new_x and qsy < y_int < new_y: 
                    print("\nINTERSEECITON2!!!\n")
                    return False
        self.ax.plot([qsx, new_x], [qsy, new_y], color='b')
        new_node = Node((new_x, new_y))
        q_start.add_child(new_node)
        new_node.add_parent(q_start)
        print('val',q_start.val)
        print('children',q_start.children)
        return True


    
    def go(self):
        # Run RRT
        print("GO")
        #plt.scatter(self.qinit[0], self.qinit[1], color='k')
        for i in range(0, self.k):
            print()
            print()
            #self.g.pr()
            qrand = self.random_configuration()
            qnear_dist, qnear = self.nearest_vertex(self.g, qrand)
            qnew = self.new_configuration(qnear, qrand)
        #self.g.pr()
        plt.show()


d = [(0,100),(0,100)]
qinit = (50,50)
delta = 1
k = 300
#Task1 = RRT(qinit, k, delta, d)
#Task1.go()





Task2 = RRT(qinit, k, delta, d, circles=True)
Task2.go()