import math


class SafeInterval(object):
    '''Creates a point on a coordinate plane with values x and y.'''

    def __init__(self, openBound = 0, closeBound = float('inf')):
        '''Defines x and y variables'''
        self.openBound = openBound
        self.closeBound = closeBound

    def __str__(self):
        return "SafeInterval(%s,%s)"%(self.openBound, self.closeBound)


class Point(object):
    '''Creates a point on a coordinate plane with values x and y.'''

    def __init__(self, x, y, z, safeIntervals=[]):
        '''Defines x and y variables'''
        self.x = x
        self.y = y
        self.z = z
        self.safeIntervals = safeIntervals

    def __str__(self):
        return "Point(%s,%s,%s)"%(self.x, self.y, self.z)

    def distance(self, other):
        dx = self.x - other.X
        dy = self.y - other.Y
        dz = self.z - other.Z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def equals(self, other):
        return self.distance(other) == 0

class Edge(object):
    def __init__(self, p1, p2):
        '''Defines x and y variables'''
        self.point1 = p1
        self.point2 = p2


class Graph(object):
    def __init__(self, edges=[]):
        self.listOfEdges = edges

    def getNeighborPoints(self, point):
        neighbors = []
        for edge in self.getNeighborEdges(point):
            if edge.point1.equals(point):
                neighbors.append(edge.point2)
            elif edge.point2.equals(point):
                neighbors.append(edge.point1)
        return neighbors

    def getNeighborEdges(self, point):
        neighbors = []
        for edge in self.listOfEdges:
            if edge.point1.equals(point):
                neighbors.append(edge)
            elif edge.point2.equals(point):
                neighbors.append(edge)
        return neighbors


class Robot(object):
    def __init__(self, start, goal):
        self.goal = goal
        self.currentLocation = start

    def isAtGoal(self):
        return self.currentLocation.equals(self.goal)

    def updateCurrentLocation(self, point):
        self.currentLocation = point









