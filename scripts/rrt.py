#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import random

from matplotlib.pyplot import rcParams

np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
# rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 22

# tree node
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None # parent node reference


# RRT Algorithm
class RRTAlgorithm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1]) # root pos
        self.goal = treeNode(goal[0], goal[1]) # goal pos
        self.nearestNode = None 
        self.iterations = min(numIterations, 2000)
        self.grid = grid # map
        self.rho = stepSize # len of each branch
        self.path_distance = 0 # total path distance
        self.nearestDist = 10000 # dist to nearest node
        self.numWayPoints = 0 # number of way points
        self.wayPoints = [] # the waypoints
        
    # add point to nearest node
    def addChild(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            # add goal node to child of nearest node
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(locationX, locationY)    
            # add temp node to child of nearest node
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode
            
    
    # sample a random point within grid limits
    def sampleAPoint(self):
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
        point = np.array([x, y])
        return point

    # steer a distance stepsize from start to end
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho * self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        
        if point[0] >= self.grid.shape[1]:
            point[0] = self.grid.shape[1]
        if point[1] >= self.grid.shape[0]:
            point[1] = grid.shape[0]

        return point    

    # check if obstacle lies between start node and end point of edge
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])

        for i in range(self.rho):
            testPoint[0] = locationStart.locationX + i*u_hat[0]
            testPoint[1] = locationStart.locationY + i*u_hat[1]
            
            # grid[y, x]
            if self.grid[round(testPoint[1]), round(testPoint[0])] >= 1: # check test point lies within obstacle
                return True
            
        return False
    
    # find unit vector b/w 2 points
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v / np.linalg.norm(v)
        return u_hat

    # find nearest node from given unconnected point
    def findNearest(self, root, point):
        # return if root is null
        if not root:
            return
        # find dist b/w root and point
        dist = self.distance(root, point)
        # if this is lower than nearest distance then set this as nearest node and update nearest distance
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        # recursively call by iteratinf through children
        for child in root.children:
            self.findNearest(child, point)

    # find euclidean dist b/w a node and XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return dist

    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        return False

    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    # trace path from goal to start
    def retraceRRTPath(self, goal):
        # end the recursion when goal node reaches start node
        if goal.locationX == self.randomTree.locationX:
            return
        # add 1 to number of waypoints
        self.numWayPoints += 1
        # insert current point to waypoints array
        currentPoint = np.array([goal.locationX, goal.locationY])
        self.wayPoints.insert(0, currentPoint)
        # add step size (rho) to path distance
        self.path_distance += self.rho
        # recursive call ...
        self.retraceRRTPath(goal.parent)

# end class

def RRT(grid, start, goal, numIterations, stepSize):
    goalRegion = plt.Circle((
        goal[0], goal[1]), 
        stepSize,
        color='b',
        fill= False
        )

    fig = plt.figure("RRT Algorithm")
    plt.imshow(grid, cmap='binary', origin='lower')
    plt.plot(start[0], start[1], 'ro')
    plt.plot(goal[0], goal[1], 'bo')
    ax = fig.gca()
    ax.add_patch(goalRegion)
    plt.xlabel('X-axis $(m)$')
    plt.xlabel('Y-axis $(m)$')

    # Begin
    rrt = RRTAlgorithm(start=start, goal=goal, numIterations=numIterations, grid=grid, stepSize=stepSize)

    for i in range(rrt.iterations):
        #reset nearest values
        rrt.resetNearestValues()
        print("Iteration", i)

        #algorithm
        point = rrt.sampleAPoint()
        rrt.findNearest(rrt.randomTree, point)
        new = rrt.steerToPoint(rrt.nearestNode, point)
        bool = rrt.isInObstacle(rrt.nearestNode, new)
        if bool == False:
            rrt.addChild(new[0], new[1])
            plt.pause(0.10)
            plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go', linestyle="-")

            # if goal found, append to path
            if rrt.goalFound(new):
                rrt.addChild(goal[0], goal[1])
                print("Goal Found!")
                break

    rrt.retraceRRTPath(rrt.goal)
    rrt.wayPoints.insert(0,start)
    print("Number of way points: ", rrt.numWayPoints)
    print("Path Distance (m): ", rrt.path_distance)
    print("Waypoints: ", rrt.wayPoints)

    #plot waypoints
    for i in range(len(rrt.wayPoints) - 1):
        plt.plot([rrt.wayPoints[i][0], rrt.wayPoints[i+1][0]], 
                [rrt.wayPoints[i][1], rrt.wayPoints[i+1][1]], 
                'ro', 
                linestyle="-"
                )
        plt.pause(0.10)
    plt.show()

    return rrt.wayPoints

if __name__ == "__main__":

    grid = np.load('image.npy')
    

    start = np.array([132.0, 132.0])
    goal = np.array([400.0, 300])

    numIterations = 2000
    stepSize = 1

    waypoints = RRT(grid, start, goal, numIterations, stepSize)

    print(waypoints)

    
