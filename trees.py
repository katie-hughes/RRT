import numpy as np
import matplotlib.pyplot as plt
import random
import imageio
import math


def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def inBetween(a, b, c): 
    #checks if c is in between a and b
    ax = a[0]
    ay = a[1]
    bx = b[0]
    by = b[1]
    cx = c[0]
    cy = c[1]
    if ax < bx: 
        if ay < by: 
            return (ax < cx < bx and ay < cx < by)
        else: 
            return (ax < cx < bx and by < cx < ay)
    else: 
        if ay < by: 
            return (bx < cx < ax and ay < cx < by)
        else: 
            return (bx < cx < ax and by < cx < ay)



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
    def __init__(self, qinit, k, delta, d, circles=False, img=False):
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


        dst = 0 
        goalx = -1
        goaly = -1
        while dst < 10: 
            goalx = random.randrange(self.d[0][0], self.d[0][1])
            goaly = random.randrange(self.d[1][0], self.d[1][1])
            self.goal = (goalx, goaly)
            dst = distance(self.goal, self.qinit)
        #print(f"GOAL: {self.goal}")
        self.buff = 0.5

        self.maxCircleRad = 15
        self.nCircles = 10

        self.doImage = False
        self.imgMap = []

        if circles: 
            self.doCircles = True
            print("Running with circles!")
            for n in range(0, self.nCircles): 
                x = random.randrange(self.d[0][0], self.d[0][1])
                y = random.randrange(self.d[1][0], self.d[1][1])
                rad = random.randrange(1, self.maxCircleRad)
                c = Circle((x,y), rad)
                if (distance(c.c, self.goal) < c.r) or (distance(c.c, self.qinit) < c.r):
                    print('Circle intersected! Not including')
                else: 
                    self.circles.append(c)
                    circle = plt.Circle(c.c, c.r, color='k')
                    self.ax.add_patch(circle)
        if img: 
            self.doImage = True
            image = imageio.imread('N_map.png')
            self.goal = (60, 60) 
            print(image)
            print(image.shape)
            image = np.flipud(image)
            ax.imshow(image, cmap="gray", origin='lower')
            self.imgMap = image

        # Plot the starting point and goal 
        ax.scatter(qinit[0], qinit[1], color='r', label='Start='+str(self.qinit))
        ax.scatter(self.goal[0], self.goal[1], color='orange', label='end='+str(self.goal))
        self.ax.set_aspect('equal')
        
                    
            
            
    
    
    
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
        #print("new config")
        #print(q_start)
        #print(q_direction)
        qsx = q_start.val[0]
        qsy = q_start.val[1]
        qdx = q_direction[0]
        qdy = q_direction[1]
        ## CHECK DIV BY 0
        # generate new tree config by moving delta from one vertex to another
        if qdx == qsx: 
            return None
        theta = np.arctan((qdy - qsy)/(qdx - qsx))
        #print(theta*180/np.pi, "deg")
        y = self.delta * np.sin(theta)
        x = self.delta * np.cos(theta)
        #print(x, y)
        if (qdx-qsx)<0: 
            new_x = qsx - x
            new_y = qsy - y
        else: 
            new_x = qsx + x
            new_y = qsy + y
        print('New candidate point:', new_x, new_y)
        slope = (new_y - qsy) / (new_x - qsx)
        b = new_y - slope*new_x
        if self.doCircles: 
            tan_slope = -1./slope
            for c in self.circles: 
                tan_b = c.c[1] - tan_slope*c.c[0]
                x_int = (b - tan_b)/(tan_slope - slope)
                y_int = tan_slope*x_int+tan_b
                if distance((new_x, new_y), c.c) < c.r:
                    # The new point is inside of the circle (bad)
                    print("New point intersects!")
                    return None
                if distance((x_int, y_int), c.c) < c.r: 
                    # Intersection point is less than radius
                    if inBetween((qsx, qsy), (new_x, new_y), (x_int, y_int)):
                        # Intersection point is on the line segment
                        print("Line intersects!")
                        return None
        if self.doImage: 
            # very naive approach
            pixels = []
            if new_x > qsx: 
                if new_y > qsy: 
                    if 0 < slope < 1: 
                        print("ONE")
                        pixels += self.bres1((qsx, qsy), (new_x, new_y), slope, b)
                    else: 
                        print("TWO")
                        pixels += self.bres2((qsx, qsy), (new_x, new_y), slope, b)
                else: 
                    if 0 < np.abs(slope) < 1: 
                        print('THREE')
                        pixels += self.bres3((qsx, qsy), (new_x, new_y), slope, b)
                    else: 
                        print("FOUR")
                        pixels += self.bres4((qsx, qsy), (new_x, new_y), slope, b)
            else: 
                print("nope")
                return None

            print("Checking pixels")
            for p in pixels: 
                if self.pixelOccupied(p):
                   print("bad pixel", p) 
                   return None
                else: 
                   print("ok pixel", p)
        print("drawing line")
        self.ax.plot([qsx, new_x], [qsy, new_y], color='b')
        new_node = Node((new_x, new_y))
        q_start.add_child(new_node)
        new_node.add_parent(q_start)
        if distance((new_x, new_y), self.goal) < self.buff: 
            print("WE HAVE REACHED GOAL!")
            return new_node
        else: 
            return None

    def pixelOccupied(self, point): 
        # 0 is black, 255 is white 
        px = self.imgMap[point[0]][point[1]]
        print(px)
        return (px == 0)
    
    def bres1(self, start, end, slope, b): 
        # startx < endx and starty < endy and 0 < slope < 1
        # iterate through x
        print(f"Start: {start}")
        print(f"End: {end}")
        curr_x = math.floor(start[0])
        final_x = math.floor(end[0])
        pixels = []
        while curr_x < final_x+1: 
            curr_y = slope*curr_x + b
            curr_y = math.floor(curr_y)
            print(f"current point:({curr_x},{curr_y})")
            # check above and below
            pixels.append((curr_x, curr_y-1))
            pixels.append((curr_x, curr_y))
            pixels.append((curr_x, curr_y+1))
            curr_x += 1
        return pixels

    def bres2(self, start, end, slope, b): 
        # startx < endx and starty < endy and slope > 1
        # iterate through y 
        print(f"Start: {start}")
        print(f"End: {end}")
        curr_y = math.floor(start[1])
        final_y = math.floor(end[1])
        pixels = []
        while curr_y < final_y+1: 
            curr_x = (curr_y - b)/slope
            curr_x = math.floor(curr_x)
            print(f"current point:({curr_x},{curr_y})")
            # check above and below
            pixels.append((curr_x-1, curr_y))
            pixels.append((curr_x, curr_y))
            pixels.append((curr_x+1, curr_y))
            curr_y += 1
        return pixels

    def bres3(self, start, end, slope, b): 
        # startx < endx and starty > endy and 0 < |slope| < 1
        # iterate through x
        print(f"Start: {start}")
        print(f"End: {end}")
        curr_x = math.floor(start[0])
        final_x = math.floor(end[0])
        pixels = []
        while curr_x < final_x+1: 
            curr_y = slope*curr_x + b
            curr_y = math.floor(curr_y)
            print(f"current point:({curr_x},{curr_y})")
            # check above and below
            pixels.append((curr_x, curr_y-1))
            pixels.append((curr_x, curr_y))
            pixels.append((curr_x, curr_y+1))
            curr_x += 1
        return pixels

    def bres4(self, start, end, slope, b): 
        # startx < endx and starty > endy and |slope| > 1
        # iterate through y 
        print(f"Start: {start}")
        print(f"End: {end}")
        curr_y = math.floor(start[1])
        final_y = math.floor(end[1])
        pixels = []
        while curr_y > final_y+1: 
            curr_x = (curr_y - b)/slope
            curr_x = math.floor(curr_x)
            print(f"current point:({curr_x},{curr_y})")
            # check above and below
            pixels.append((curr_x-1, curr_y))
            pixels.append((curr_x, curr_y))
            pixels.append((curr_x+1, curr_y))
            curr_y -= 1
        return pixels



    def drawPath(self, node, co): 
        current = node.val
        self.ax.plot([co[0], current[0]], [co[1], current[1]], color='r')
        if node.parent is not None: 
            self.drawPath(node.parent, current)

    
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
            if qnew is not None: 
                self.drawPath(qnew, self.goal)
                break
        #self.g.pr()

        print(f"Start: {self.qinit}")
        print(f"End: {self.goal}")
        
        plt.legend()
        plt.show()


d = [(0,100),(0,100)]
qinit = (40,40)
delta = 10
k = 1000
#Task1 = RRT(qinit, k, delta, d)
#Task1.go()





#Task2 = RRT(qinit, k, delta, d, circles=True)
#Task2.go()


Task3 = RRT(qinit, k, delta, d, img=True)
Task3.go()