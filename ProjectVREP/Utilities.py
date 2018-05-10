import math

INFINITY = float('inf')
ROBOT_VELOCITY = 0.763  #m/s
SIM_TIMESTEP = 0.5  #s
TIMESTEP_DIST = ROBOT_VELOCITY * SIM_TIMESTEP
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
    
    def unitVector(self, other):
        dist = self.distance(other)
        vector = [(self.x - other.x)/dist,
                  (self.y - other.y)/dist,
                  (self.z - other.z)/dist]
        return vector

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
    def __init__(self, p1, p2, id = 0):
        # Defines x and y variables
        self.point1 = p1
        self.point2 = p2
        self.id = id


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

    def getEdge(self, point1, point2):
        for edge in self.edges:
            if ((edge.point2.equals(point1) and edge.point1.equals(point2))
                    or( edge.point1.equals(point1) and edge.point2.equals(point2))):
                return edge

    def getGraphPointFromPoint(self, point):
        for p in self.points:
            if (p.equals(point)):
                return p
        return

    def __init__(self, edges=[], points=[]):
        self.edges = edges
        self.points = points
        pointIdCounter = 1
        edgeIdCounter = 1000
        for point in points:
            point.id = pointIdCounter
            pointIdCounter += 1

        for edge in edges:
            edge.id = edgeIdCounter
            edgeIdCounter += 1


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
ROAD = 0.2
AIRWAY = 2.2

p1 = Point(-.5, -.5, ROAD)
p2 = Point(.5, -.5, ROAD)
p3 = Point(.5, .5, ROAD)
p4 = Point(-.5, .5, ROAD)

park1 = Point(-2, -1, PARKING)
park2 = Point(2, -1, PARKING)
park3 = Point(2, 1, PARKING)
park4 = Point(-2, 1, PARKING)

p5 = Point(-1, 1, AIRWAY)
p6 = Point(1, 1, AIRWAY)
p7 = Point(-1, -1, AIRWAY)
p8 = Point(1, -1, AIRWAY)

p9 = Point(-1,0,ROAD)
p10 = Point(1,0,ROAD)
p11 = Point(0,1,ROAD)
p12 = Point(0,-1,ROAD)

def getCubeGraph():
    edges = [Edge(p1, p2), Edge(p1, p4), Edge(p1, p7), Edge(p1, park1), Edge(p1, p1),
             Edge(p2, p8), Edge(p2, p3), Edge(p2, park2), Edge(p2, p2),
             Edge(p3, p4), Edge(p3, p6), Edge(p3, park3), Edge(p3, p3),
             Edge(p4, p5), Edge(p4, park4), Edge(p4, p4),
             Edge(p5, p6), Edge(p5, p7), Edge(p5, p5),
             Edge(p6, p8), Edge(p6, p6),
             Edge(p7, p8), Edge(p7, p7),
             Edge(p8, p8)]

    return Graph(edges, [p1, p2, p3, p4, p5, p6, p7, p8, p9,p10,p11,p12,park1, park2, park3, park4])


def getSmallSquareGraph():
    edges = [Edge(p1, p9), Edge(p1, p12), Edge(p1,p1), Edge(park1, p1),
             Edge(p12, p2), Edge(p12,p12),
             Edge(p2, p10), Edge(p2,p2), Edge(park2, p2),
             Edge(p10,p3), Edge(p3,p3), Edge(park3, p3),
             Edge(p11, p4), Edge(p11, p11),
             Edge(p4, p9), Edge(p4,p4), Edge(park4,p4),
             Edge(p9,p9)]

    return Graph(edges, [p1, p2, p3, p4, p5, p6, p7, p8, p9,p10,p11,p12,park1, park2, park3, park4])


def getCubeGraphRobots():
    pt1 = Point(-2, -1, PARKING)
    pt1.id = park1.id
    pt2 = Point(2, -1, PARKING)
    pt2.id = park2.id
    pt3 = Point(2, 1, PARKING)
    pt3.id = park3.id
    pt4 = Point(-2, 1, PARKING)
    pt4.id = park4.id
    return [Robot(State(Point.copy(pt1)), State(Point.copy(pt2))),
            Robot(State(Point.copy(pt3)), State(Point.copy(pt1))),
            Robot(State(Point.copy(pt4)), State(Point.copy(pt2))),
            Robot(State(Point.copy(pt2)), State(Point.copy(pt1)))]


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

def initGlobalEdgeSafeIntervals(graph):
    safeIntervalEdgeDict = {}
    for edge in graph.edges:
        safeIntervalEdgeDict[edge.id] = [SafeInterval(0, INFINITY)]
    return safeIntervalEdgeDict

def printSafeIntervals(safeIntervals):
    for id, intervals in safeIntervals.iteritems():
        print str(id),
        for interval in intervals:
            print"\t[" + str(interval.startTime) + "," + str(interval.endTime) + "] "


class OpenItem(object):
    def __init__(self, state, cost, heuristic, startTime):
        self.state = state
        self.cost = cost
        self.heuristic = heuristic
        self.startTime = startTime

def getHeuristic(point1, point2):
    distance = point1.distance(point2)
    return distance

def getItemWithSmallestEValue(list):
    itemSmallestEValue = list[0]
    for item in list:
        if((item.heuristic + item.cost) < (itemSmallestEValue.heuristic + itemSmallestEValue.cost)):
            itemSmallestEValue = item
    return itemSmallestEValue

def getEarliestArrivalTime(intervalList):
    earliestArrival = INFINITY
    for interval in intervalList:
        if interval.startTime < earliestArrival:
            earliestArrival = interval.startTime
    return earliestArrival

def calculateCost(maxTime, arrivalTimeDiff):
    return arrivalTimeDiff/maxTime

def appendPointToPath(path, point):
    path.append(point.x)
    path.append(point.y)
    path.append(point.z)

def discretizePath(path):
    newPath = []

    for i, state in enumerate(path):
        if (i > 0) and (i < len(path)):
            pt1 = path[i-1].point
            pt2 = state.point
            dist = pt1.distance(pt2)
            print("pt1: " + str(pt1) + ", pt2: " + str(pt2) + "-->" + str(dist))
            if (dist > 0):
                unit = pt2.unitVector(pt1)
                numSteps = int(round(dist/ROBOT_VELOCITY/SIM_TIMESTEP))
                
                #Fill in timesteps that move the entire step distance
                remainingDist = dist
                stepPoint = pt1.copy()
                for j in xrange(numSteps-1):
                    stepPoint.x = stepPoint.x + unit[0]*TIMESTEP_DIST
                    stepPoint.y = stepPoint.y + unit[1]*TIMESTEP_DIST
                    stepPoint.z = stepPoint.z + unit[2]*TIMESTEP_DIST
                    appendPointToPath(newPath, stepPoint)
                    remainingDist = remainingDist - TIMESTEP_DIST
                    
                
                #Fill in last timestep with remaining distance
                stepPoint.x = stepPoint.x + unit[0]*remainingDist
                stepPoint.y = stepPoint.y + unit[1]*remainingDist
                stepPoint.z = stepPoint.z + unit[2]*remainingDist
                appendPointToPath(newPath, stepPoint)
            else:
                appendPointToPath(newPath, pt2)
    return newPath
    
    
