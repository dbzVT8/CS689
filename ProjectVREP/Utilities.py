import math

INFINITY = float('inf')
ROBOT_VELOCITY = 0.5
class SafeInterval(object):
    '''Creates a point on a coordinate plane with values x and y.'''

    def __init__(self, startTime = 0, endTime = INFINITY):
        '''Defines x and y variables'''
        self.startTime = startTime
        self.endTime = endTime

    def __str__(self):
        return "SafeInterval(%s,%s)"%(self.startTime, self.endTime)


class Point(object):
    '''Creates a point on a coordinate plane with values x and y.'''

    def __init__(self, x, y, z, safeIntervals=[SafeInterval(0,INFINITY)], visited=False):
        '''Defines x and y variables'''
        self.x = x
        self.y = y
        self.z = z
        self.safeIntervals = safeIntervals
        self.visited = visited

    def __str__(self):
        return "Point: (%s,%s,%s)"%(self.x, self.y, self.z)

    def distance(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def equals(self, other):
        return self.distance(other) == 0

class State(object):

    def __init__(self, point, safeInterval = SafeInterval(), arrivalTime = 0, cost = 0):
        self.point = point;
        self.safeInterval = safeInterval
        self.arrivalTime = arrivalTime
        self.cost = cost
        self.lowestCost = INFINITY
        self.earliestArrival = INFINITY

class Edge(object):
    def __init__(self, p1, p2):
        '''Defines x and y variables'''
        self.point1 = p1
        self.point2 = p2


class Graph(object):
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

    def __init__(self, edges=[]):
        self.listOfEdges = edges

class Robot(object):
    def __init__(self, start, goal):
        self.goal = goal
        self.start = start
        self.currentLocation = start

    def isAtGoal(self):
        return self.currentLocation.equals(self.goal)

    def updateCurrentLocation(self, point):
        self.currentLocation = point

PARKING = 0
ROAD = .2
AIRWAY = 5

def getCubeGraph():
    p1 = Point(-3,-3,ROAD)
    park1 = Point(-4, -3, PARKING)
    p2 = Point(3,-3,ROAD)
    park2 = Point(4, -3, PARKING)
    p3 = Point(3,3,ROAD)
    park3 = Point(4, 3, PARKING)
    p4 = Point(-3, 3, ROAD)
    park4 = Point(-4, 3, PARKING)

    p5 = Point(-3,3,AIRWAY)
    p6 = Point(3,3,AIRWAY)
    p7 = Point(-3, -3, AIRWAY)
    p8 = Point(3, -3, AIRWAY)

    edges = [Edge(p1, p2), Edge(p1, p4), Edge(p1, p7), Edge(p1, park1), Edge(p1, p1),
             Edge(p2,p8), Edge(p2, p3), Edge(p2, park2),Edge(p2, p2),
             Edge(p3,p4), Edge(p3,p6), Edge(p3, park3), Edge(p3,p3),
             Edge(p4,p5), Edge(p4, park4),Edge(p4,p4),
             Edge(p5,p6), Edge(p5,p7),Edge(p5,p5),
             Edge(p6,p8),Edge(p6,p6),
             Edge(p7,p8),Edge(p7,p7),
             Edge(p8,p8),
             Edge(park1, park1),
             Edge(park2, park2),
             Edge(park3, park3),
             Edge(park4, park4)]

    return Graph(edges)


def getCubeGraphRobots():
    return [Robot(State(Point(-4, -3, PARKING)),State(Point(4, 3, PARKING)))]
            #Robot(State(4,3, PARKING), State(-4,3,PARKING))]

def timeToTraverse(point1, point2):
    dist = point1.distance(point2)
    return dist/ROBOT_VELOCITY

def getMaxTime(graph):
    maxTime = 0
    for edge in graph.listOfEdges:
        timeEdge = timeToTraverse(edge.point1, edge.point2)
        if(timeEdge > maxTime):
            maxTime = timeEdge
    return maxTime
