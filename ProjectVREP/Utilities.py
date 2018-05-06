import math

INFINITY = float('inf')
ROBOT_VELOCITY = 0.5
ROBOT_DIAMETER = 0.5


class SafeInterval(object):
    '''Creates a point on a coordinate plane with values x and y.'''

    def __init__(self, startTime=0, endTime=INFINITY):
        '''Defines x and y variables'''
        self.startTime = startTime
        self.endTime = endTime

    def __str__(self):
        return "SafeInterval(%s,%s)" % (self.startTime, self.endTime)


class Point(object):
    # Creates a point on a coordinate plane with values x and y.

    def __init__(self, x, y, z, visited=False, id = 0):
        # Defines x and y variables
        self.x = x
        self.y = y
        self.z = z
        self.visited = visited
        self.id = id

    def copy(point):
        return Point(point.x, point.y, point.z, point.visited, point.id)

    def __str__(self):
        return "Point: ID: %s (%s,%s,%s)" % (self.id, self.x, self.y, self.z)

    def distance(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def equals(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z


class State(object):

    def __init__(self, point, safeInterval=SafeInterval(), arrivalTime=0, cost=0):
        self.point = point;
        self.safeInterval = safeInterval
        self.arrivalTime = arrivalTime
        self.cost = cost
        self.lowestCost = INFINITY
        self.earliestArrival = INFINITY


class Edge(object):
    def __init__(self, p1, p2):
        # Defines x and y variables
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
        for edge in self.edges:
            if edge.point1.equals(point):
                neighbors.append(edge)
            elif edge.point2.equals(point):
                neighbors.append(edge)
        return neighbors

    def getGraphPointFromPoint(self, point):
        for p in self.points:
            if (p.equals(point)):
                return p
        return

    def __init__(self, edges=[], points=[]):
        self.edges = edges
        self.points = points
        pointIdCounter = 1
        for point in points:
            point.id = pointIdCounter
            pointIdCounter += 1


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

p1 = Point(-3, -3, ROAD)
p2 = Point(3, -3, ROAD)
p3 = Point(3, 3, ROAD)
p4 = Point(-3, 3, ROAD)

park1 = Point(-4, -3, PARKING)
park2 = Point(4, -3, PARKING)
park3 = Point(4, 3, PARKING)
park4 = Point(-4, 3, PARKING)

p5 = Point(-3, 3, AIRWAY)
p6 = Point(3, 3, AIRWAY)
p7 = Point(-3, -3, AIRWAY)
p8 = Point(3, -3, AIRWAY)

def getCubeGraph():
    edges = [Edge(p1, p2), Edge(p1, p4), Edge(p1, p7), Edge(p1, park1), Edge(p1, p1),
             Edge(p2, p8), Edge(p2, p3), Edge(p2, park2), Edge(p2, p2),
             Edge(p3, p4), Edge(p3, p6), Edge(p3, park3), Edge(p3, p3),
             Edge(p4, p5), Edge(p4, park4), Edge(p4, p4),
             Edge(p5, p6), Edge(p5, p7), Edge(p5, p5),
             Edge(p6, p8), Edge(p6, p6),
             Edge(p7, p8), Edge(p7, p7),
             Edge(p8, p8),
             Edge(park1, park1),
             Edge(park2, park2),
             Edge(park3, park3),
             Edge(park4, park4)]

    return Graph(edges, [p1, p2, p3, p4, p5, p6, p7, p8, park1, park2, park3, park4])


def getCubeGraphRobots():
    pt1 = Point(-4, -3, PARKING)
    pt1.id = park1.id
    pt2 = Point(4, -3, PARKING)
    pt2.id = park2.id
    pt3 = Point(4, 3, PARKING)
    pt3.id = park3.id
    pt4 = Point(-4, 3, PARKING)
    pt4.id = park4.id
    return [Robot(State(Point.copy(pt1)), State(Point.copy(pt3))),
            Robot(State(Point.copy(pt3)), State(Point.copy(pt1))),
            Robot(State(Point.copy(pt4)), State(Point.copy(pt2))),
            Robot(State(Point.copy(pt2)), State(Point.copy(pt4)))]


def timeToTraverse(point1, point2):
    dist = point1.distance(point2)
    return dist / ROBOT_VELOCITY


def getMaxTime(graph):
    maxTime = 0
    for edge in graph.edges:
        timeEdge = timeToTraverse(edge.point1, edge.point2)
        if (timeEdge > maxTime):
            maxTime = timeEdge
    return maxTime

def initGlobalSafeIntervals(graph):
    safeIntervalDict = {}
    for pt in graph.points:
        safeIntervalDict[pt.id] = [SafeInterval(0, INFINITY)]
    return safeIntervalDict

def printSafeIntervals(safeIntervals):
    for id, intervals in safeIntervals.iteritems():
        print str(id),
        for interval in intervals:
            print"\t[" + str(interval.startTime) + "," + str(interval.endTime) + "] "
