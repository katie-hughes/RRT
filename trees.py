import numpy as np
import matplotlib.pyplot as plt
import random
import imageio.v2 as imageio
import math
import argparse

def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def inBetween(a, b, c): 
    #checks if c is in between a and b
    ax, ay = a
    bx, by = b
    cx, cy = c
    if ax < bx: 
        if ay < by: 
            return (ax < cx < bx and ay < cy < by)
        else: 
            return (ax < cx < bx and by < cy < ay)
    else: 
        if ay < by: 
            return (bx < cx < ax and ay < cy < by)
        else: 
            return (bx < cx < ax and by < cy < ay)


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

class Circle: 
    def __init__(self, center, radius):
        self.c = center
        self.r = radius
    def inside(self, point): 
        return (distance(self.c, point) <= self.r)

class RRT:
    def __init__(self, qinit, k, delta, d, circles=False, img=False, verbose=False):
        self.qinit = qinit
        self.k = k
        self.delta = delta
        self.d = d
        self.verbose = verbose

        self.g = Node(qinit,parent=None)
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
        self.buff = 2*self.delta

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
                    if self.verbose:
                        print('Circle intersected! Not including')
                else: 
                    self.circles.append(c)
                    circle = plt.Circle(c.c, c.r, color='k')
                    self.ax.add_patch(circle)
        if img: 
            self.doImage = True
            image = imageio.imread('N_map.png')
            if len(image.shape) > 2:
                # make pixels black/white
                image = image[:,:,:1]
                image = image.reshape([image.shape[0], image.shape[1]])
            self.goal = (60, 60) 
            image = np.flipud(image)
            ax.imshow(image, cmap="gray", origin='lower')
            self.imgMap = image

        # Plot the starting point and goal  
        self.ax.set_aspect('equal')
        
    def random_configuration(self):
        #Generate random position in the domain
        x = random.randrange(self.d[0][0], self.d[0][1])
        y = random.randrange(self.d[1][0], self.d[1][1])
        if self.verbose:
            print("Random:", x,y)
        return (x,y)
    
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
        qsx = q_start.val[0]
        qsy = q_start.val[1]
        qdx = q_direction[0]
        qdy = q_direction[1]
        ## CHECK DIV BY 0
        # generate new tree config by moving delta from one vertex to another
        if qdx == qsx:
            if self.verbose:
                print("SLOPE 0 returning")
            return None
        theta = np.arctan((qdy - qsy)/(qdx - qsx))
        y = self.delta * np.sin(theta)
        x = self.delta * np.cos(theta)
        if (qdx-qsx)<0: 
            new_x = qsx - x
            new_y = qsy - y
        else: 
            new_x = qsx + x
            new_y = qsy + y
        if self.verbose:
            print('New candidate point:', new_x, new_y)
        if not ((self.d[0][0] < new_x < self.d[0][1]) and (self.d[1][0] < new_y < self.d[1][1])):
            if self.verbose:
                print("OUT OF BOUDNS")
            return None
        if (new_x == qsx or new_y == qsy): 
            if self.verbose:
                print("dont want to deal with this case")
            return None
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
                    if self.verbose:
                        print("New point intersects!")
                    return None
                if distance((x_int, y_int), c.c) < c.r: 
                    # Intersection point is less than radius
                    if inBetween((qsx, qsy), (new_x, new_y), (x_int, y_int)):
                        # Intersection point is on the line segment
                        if self.verbose:
                            print("Line intersects!")
                        return None
        if self.doImage: 
            # very naive approach
            pixels = []
            if new_x > qsx: 
                if new_y > qsy: 
                    if 0 < np.abs(slope) < 1: 
                        if self.verbose: print("ONE")
                        pixels += self.bres1((qsx, qsy), (new_x, new_y), slope, b)
                    else: 
                        if self.verbose: print("TWO")
                        pixels += self.bres2((qsx, qsy), (new_x, new_y), slope, b)
                else: 
                    if 0 < np.abs(slope) < 1: 
                        if self.verbose: print('THREE')
                        pixels += self.bres3((qsx, qsy), (new_x, new_y), slope, b)
                    else: 
                        if self.verbose: print("FOUR")
                        pixels += self.bres4((qsx, qsy), (new_x, new_y), slope, b)
            else: 
                if new_y > qsy: 
                    if 0 < np.abs(slope) < 1: 
                        if self.verbose: print("FIVE")
                        pixels += self.bres3((new_x, new_y), (qsx, qsy), slope, b)
                    else: 
                        if self.verbose: print("SIX")
                        pixels += self.bres4((new_x, new_y), (qsx, qsy), slope, b)
                else: 
                    if 0 < np.abs(slope) < 1: 
                        if self.verbose: print('SEVEN')
                        pixels += self.bres1((new_x, new_y), (qsx, qsy), slope, b)
                    else: 
                        if self.verbose: print("EIGHT")
                        pixels += self.bres2((new_x, new_y), (qsx, qsy), slope, b)

            if self.verbose: print("Checking pixels")
            if self.verbose: print(pixels)
            if len(pixels) == 0: 
                if self.verbose: print("PIXELS EMPTY")
                exit()
            for p in pixels: 
                if self.pixelOccupied(p):
                   return None
                else:
                    pass 
        if self.verbose: print("drawing line")
        self.ax.plot([qsx, new_x], [qsy, new_y], color='b')
        new_node = Node(val=(new_x, new_y), parent=q_start)
        q_start.add_child(new_node)
        if distance((new_x, new_y), self.goal) < self.buff: 
            print("Goal Reached!!")
            return new_node
        else: 
            return None

    def pixelOccupied(self, point): 
        # 0 is black, 255 is white 
        px = self.imgMap[point[1]][point[0]]
        return (px == 0)
    
    def bres1(self, start, end, slope, b): 
        # startx < endx and starty < endy and 0 < slope < 1
        # iterate through x
        if self.verbose: print(f"Start: {start}")
        if self.verbose: print(f"End: {end}")
        curr_x = math.floor(start[0])
        final_x = math.floor(end[0])
        pixels = []
        while curr_x < final_x+1: 
            curr_y = slope*curr_x + b
            curr_y = math.floor(curr_y)
            if self.verbose: print(f"current point:({curr_x},{curr_y})")
            # check above and below
            pixels.append((curr_x, curr_y-1))
            pixels.append((curr_x, curr_y))
            pixels.append((curr_x, curr_y+1))
            curr_x += 1
        return pixels

    def bres2(self, start, end, slope, b): 
        # startx < endx and starty < endy and slope > 1
        # iterate through y 
        if self.verbose: print(f"Start: {start}")
        if self.verbose: print(f"End: {end}")
        curr_y = math.floor(start[1])
        final_y = math.floor(end[1])
        pixels = []
        while curr_y < final_y+1: 
            curr_x = (curr_y - b)/slope
            curr_x = math.floor(curr_x)
            if self.verbose: print(f"current point:({curr_x},{curr_y})")
            # check above and below
            pixels.append((curr_x-1, curr_y))
            pixels.append((curr_x, curr_y))
            pixels.append((curr_x+1, curr_y))
            curr_y += 1
        return pixels

    def bres3(self, start, end, slope, b): 
        # startx < endx and starty > endy and 0 < |slope| < 1
        # iterate through x
        if self.verbose: print(f"Start: {start}")
        if self.verbose: print(f"End: {end}")
        curr_x = math.floor(start[0])
        final_x = math.floor(end[0])
        pixels = []
        while curr_x < final_x+1: 
            curr_y = slope*curr_x + b
            curr_y = math.floor(curr_y)
            if self.verbose: print(f"current point:({curr_x},{curr_y})")
            # check above and below
            pixels.append((curr_x, curr_y-1))
            pixels.append((curr_x, curr_y))
            pixels.append((curr_x, curr_y+1))
            curr_x += 1
        return pixels

    def bres4(self, start, end, slope, b): 
        # startx < endx and starty > endy and |slope| > 1
        # iterate through y 
        if self.verbose: print(f"Start: {start}")
        if self.verbose: print(f"End: {end}")
        curr_y = math.floor(start[1])
        final_y = math.floor(end[1])
        pixels = []
        while curr_y > final_y-1: 
            curr_x = (curr_y - b)/slope
            curr_x = math.floor(curr_x)
            if self.verbose: print(f"current point:({curr_x},{curr_y})")
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
        if self.verbose: print("GO")
        for i in range(0, self.k):
            if self.verbose: print(f'\n')
            if (i % 100 == 0):print(f'{i}/{self.k}')
            qrand = self.random_configuration()
            qnear_dist, qnear = self.nearest_vertex(self.g, qrand)
            qnew = self.new_configuration(qnear, qrand)
            if qnew is not None: 
                self.drawPath(qnew, self.goal)
                break

        self.ax.scatter(self.qinit[0], self.qinit[1], color='r',
                        label=f'Start\n{self.qinit}')
        self.ax.scatter(self.goal[0], self.goal[1], color='orange',
                        label=f'End\n{self.goal}')

        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        

def main():
    parser = argparse.ArgumentParser(
                    prog='RRT',
                    description='Implements a 2D RRT path planning algorithm')
    parser.add_argument('-t', '--task', choices=[1,2,3], default=3, type=int,
                        help="Which task of the RRT to implement.\n \
                              1: basic functionality\n \
                              2: RRT with circles\n \
                              3: RRT on image\n")
    parser.add_argument('-v', '--verbose', action='store_true', help='Print out debug messages')
    args = parser.parse_args()

    # some parameters of the RRT
    # domain of the graph: 100x100
    d = [(0,100),(0,100)]
    # starting position
    qinit = (40,40)
    # increment distance
    delta = 1
    # number of iterations
    k = 10000

    print(f"You have chosen task {args.task}!")
    if args.task == 1:
        Task1 = RRT(qinit, k, delta, d, verbose=args.verbose)
        Task1.go()
        plt.savefig('images/task1.png')
        plt.show()
    elif args.task == 2:
        Task2 = RRT(qinit, k, delta, d, circles=True, verbose=args.verbose)
        Task2.go()
        plt.savefig('images/task2.png')
        plt.show()
    elif args.task == 3:
        Task3 = RRT(qinit, k, delta, d, img=True, verbose=args.verbose)
        Task3.go()
        plt.savefig('images/task3.png')
        plt.show()

if __name__ == "__main__":
    main()