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

    def __init__(self, x, y, z, isGoal=False, isStart=False):
        '''Defines x and y variables'''
        self.X = x
        self.Y = y
        self.Z = z
        self.isGoal = isGoal
        self.isStart = isStart

    def __str__(self):
        return "Point(%s,%s,%s)"%(self.X, self.Y, self.Z)

    def distance(self, other):
        dx = self.X - other.X
        dy = self.Y - other.Y
        dz = self.Z - other.Z
        return math.sqrt(dx**2 + dy**2 + dz**2)

class Edge(object):
    def __init__(self, p1, p2, safeIntervals=[]):
        '''Defines x and y variables'''
        self.point1 = p1
        self.point2 = p2
        self.safeIntervals = safeIntervals


class Graph(object):
    def __init__(self, edges=[]):
        self.ListOfEdges = edges



