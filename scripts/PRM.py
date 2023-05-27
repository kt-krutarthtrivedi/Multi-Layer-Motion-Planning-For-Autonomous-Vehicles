from collections import defaultdict
import sys
import math
from turtle import Screen
import pygame
from global_vars import*
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from sklearn.neighbors import NearestNeighbors
from algorithm import euc_dist
from Dijkstra import Graph, dijkstra, to_array
import shapely.geometry

class PRMController:
    def __init__(self, numOfRandomCoordinates, allObs, current, destination,win_height,win_width):
        self.numOfCoords = numOfRandomCoordinates
        self.coordsList = np.array([])
        self.allObs = allObs
        self.current = np.array(current)
        self.destination = np.array(destination)
        self.graph = Graph()
        # self.utils = Utils()
        self.solutionFound = False
        self.obsmap = []
        self.win_width = win_width
        self.win_height = win_height

    def runPRM(self,screen,initialRandomSeed=0):
        seed = initialRandomSeed
        # Keep resampling if no solution found
        pygame.draw.circle(screen, BLACK, (200,200), 100)
        # Create new obstacle map
        self.obsmap = self.extend_obstacles()
        # print('watch this',self.obsmap)
        while(not self.solutionFound):
            print("Trying with random seed {}".format(seed))
            np.random.seed(seed)
            
            # Generate n random samples called milestones
            self.genCoords()

            # Check if milestones are collision free
            self.checkIfCollisonFree(screen)

            # Link each milestone to k nearest neighbours.
            # Retain collision free links as local paths.
            self.findNearestNeighbour()

            # Search for shortest path from start to end node - Using Dijksta's shortest path alg
            final_path = self.shortestPath(self.current,self.destination)

            seed = np.random.randint(1, 100000)
            # self.coordsList = np.array([])
            # self.graph = Graph()

        # print("Collision Free Points",self.collisionFreePoints)
        return final_path,self.collisionFreePoints

    def genCoords(self, maxSizeOfMap=WINDOW_WIDTH):
        coordsList_x = np.random.randint(
            self.win_height, size=(self.numOfCoords, 1))

        coordsList_y = np.random.randint(
            self.win_width, size=(self.numOfCoords, 1))

        self.coordsList = np.column_stack((coordsList_x, coordsList_y))

        # Adding begin and end points
        self.current = self.current.reshape(1, 2)
        self.destination = self.destination.reshape(1, 2)
        self.coordsList = np.concatenate(
            (self.coordsList, self.current, self.destination), axis=0)
        # print("coordlist",self.coordsList)

    def checkIfCollisonFree(self,screen):
        collision = False
        self.collisionFreePoints = np.array([])
        for point in self.coordsList:
            collision = self.checkPointCollision(point)
            if(not collision):
                if(self.collisionFreePoints.size == 0):
                    self.collisionFreePoints = point
                else:
                    self.collisionFreePoints = np.vstack(
                        [self.collisionFreePoints, point])
        # self.plotPoints(self.collisionFreePoints,screen)
        # self.collisionFreePoints = np.vstack(
        #             [self.collisionFreePoints, self.destination])
        # print("look",self.collisionFreePoints)

    def findNearestNeighbour(self, k=10):
        X = self.collisionFreePoints
        knn = NearestNeighbors(n_neighbors=k)
        knn.fit(X)
        distances, indices = knn.kneighbors(X)
        self.collisionFreePaths = np.empty((1, 2), int)

        for i, p in enumerate(X):
            # Ignoring nearest neighbour - nearest neighbour is the point itself
            for j, neighbour in enumerate(X[indices[i][1:]]):
                start_line = p
                end_line = neighbour
                if(not self.checkPointCollision(start_line) and not self.checkPointCollision(end_line)):
                    if(not self.checkLineCollision(start_line, end_line)):
                        self.collisionFreePaths = np.concatenate(
                            (self.collisionFreePaths, p.reshape(1, 2), neighbour.reshape(1, 2)), axis=0)

                        a = str(self.findNodeIndex(p))
                        b = str(self.findNodeIndex(neighbour))
                        self.graph.add_node(a)
                        self.graph.add_edge(a, b, distances[i, j+1])
                        x = [p[0], neighbour[0]]
                        y = [p[1], neighbour[1]]
                        # plt.plot(x, y)
        # print("wow",self.graph.nodes)

    def shortestPath(self,start,destination):
        # print("look",self.current,self.destination)
        self.current = np.array(start)
        self.destination = np.array(destination)
        self.current = self.current.reshape(1, 2)
        self.destination = self.destination.reshape(1, 2)
        self.startNode = str(self.findNodeIndex(self.current))
        self.endNode = str(self.findGoalNodeIndex(self.destination))
        # if self.endNode in self.graph.nodes:
            # print("haaaaaa")
        # print("Yaay",self.collisionFreePoints[int(self.endNode)])

        dist, prev = dijkstra(self.graph, self.startNode)

        pathToEnd = to_array(prev, self.endNode)

        if(len(pathToEnd) > 1):
            self.solutionFound = True
        else:
            return

        # Plotting shorest path
        pointsToDisplay = [(self.findPointsFromNode(path))
                           for path in pathToEnd]

        x = [int(item[0]) for item in pointsToDisplay]
        y = [int(item[1]) for item in pointsToDisplay]
        # Add line
        # plt.plot(x, y, c="blue", linewidth=3.5)

        pointsToEnd = [list(self.findPointsFromNode(path))
                       for path in pathToEnd]
        print("****Output****")

        print("The quickest path from {} to {} is:  \n with a distance of {}".format(
            self.collisionFreePoints[int(self.startNode)],
            self.collisionFreePoints[int(self.endNode)],
            str(dist[self.endNode])
        )
        )
        return pointsToEnd

    def extend_obstacles(self):
        final_obs = []
        for obstacle in self.allObs:
            addition= []
            padding = 1.5
            for obs in obstacle:
                new_obs1 = (obs[0]-padding,obs[1])
                new_obs2 = (obs[0],obs[1]-padding)
                new_obs3 = (obs[0]+padding,obs[1])
                new_obs4 = (obs[0],obs[1]+padding)
                new_obs5 = (obs[0]-padding,obs[1]-padding)
                new_obs6 = (obs[0]+padding,obs[1]+padding)
                new_obs7 = (obs[0]+padding,obs[1]-padding)
                new_obs8 = (obs[0]-padding,obs[1]+padding)
                if new_obs1 not in obstacle and new_obs1 not in addition:
                    addition.append(new_obs1)
                if new_obs2 not in obstacle and new_obs2 not in addition:
                    addition.append(new_obs2)
                if new_obs3 not in obstacle and new_obs3 not in addition:
                    addition.append(new_obs3)
                if new_obs4 not in obstacle and new_obs4 not in addition:
                    addition.append(new_obs4)
                if new_obs5 not in obstacle and new_obs5 not in addition:
                    addition.append(new_obs5)
                if new_obs6 not in obstacle and new_obs6 not in addition:
                    addition.append(new_obs6)
                if new_obs7 not in obstacle and new_obs7 not in addition:
                    addition.append(new_obs7)
                if new_obs8 not in obstacle and new_obs8 not in addition:
                    addition.append(new_obs8)
            obstacle+=addition
            # print("see this",obstacle)
            new_list_minx = []
            new_list_maxx = []
            sorted_by_x = sorted(obstacle)
            minX = sorted_by_x[0][0]
            maxX = sorted_by_x[-1][0]
            for obs in obstacle:
                if obs[0]==minX:
                    new_list_minx.append(obs)
                if obs[0]==maxX:
                    new_list_maxx.append(obs)
            sorted_by_second_minX = sorted(new_list_minx, key=lambda tup: tup[1])
            sorted_by_second_maxX = sorted(new_list_maxx, key=lambda tup: tup[1])
            
            res = [(sorted_by_second_minX[0][0]*15,sorted_by_second_minX[0][1]*15),(sorted_by_second_minX[-1][0]*15,sorted_by_second_minX[-1][1]*15),
            (sorted_by_second_maxX[0][0]*15,sorted_by_second_maxX[0][1]*15),(sorted_by_second_maxX[-1][0]*15,sorted_by_second_maxX[-1][1]*15)]
            # print("res",res)
            final_obs.append(res) 
        return final_obs

            
    def checkLineCollision(self, start_line, end_line):
        collision = False
        line = shapely.geometry.LineString([start_line, end_line])
        for obs in self.obsmap:
            # print("obstacle",obs)
            obstacleShape = shapely.geometry.Polygon(obs)
            collision = line.intersects(obstacleShape)
            if(collision):
                return True
        return False



    def findNodeIndex(self, p):
        # print("jo",np.where((self.collisionFreePoints == p).all(axis=1)))
        return np.where((self.collisionFreePoints == p).all(axis=1))[0][0]

    def findGoalNodeIndex(self, p):
        dist = 20
        # print("previous p",p)
        p = list(p[0])
        # print("p",p)
        index=0
        for i in range(len(self.collisionFreePoints)):
            pt = (self.collisionFreePoints[i,0],self.collisionFreePoints[i,1])
            # print("fiii",pt,p)
            d = euc_dist(pt,p)
            if d<dist:
                dist = d
                index = i
        return index

    def findPointsFromNode(self, n):
        return self.collisionFreePoints[int(n)]

    def plotPoints(self, points,screen):
        # x = [item[0] for item in points]
        # y = [item[1] for item in points]
        for pos in points:
            pygame.draw.circle(screen, BLACK, pos, 4)
        # plt.scatter(x, y, c="black", s=1)

    def checkCollision(self, obs, point):
        point = shapely.geometry.Point(point)
        obstacleShape = shapely.geometry.Polygon(obs).convex_hull
        # print("point",(p_x,p_y),"and ",obs)
        if point.intersects(obstacleShape):
            # print("point",point,"colliding with",obstacleShape)
            return True
            
        else:
            return False

    def checkPointCollision(self, point):
        for obs in self.obsmap:
            collision = self.checkCollision(obs, point)
            if(collision):
                return True
        return False
    


    
