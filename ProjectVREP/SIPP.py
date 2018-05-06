from Utilities import Point
from Utilities import Graph
from Utilities import State
from Utilities import SafeInterval
import Utilities


class OpenItem(object):
    def __init__(self, state, cost, heuristic, startTime):
        self.state = state
        self.cost = cost
        self.heuristic = heuristic
        self.startTime = startTime

def getHeuristic(point1, point2):
    return point1.distance(point2)

def getItemWithSmallestEValue(list):
    itemSmallestEValue = list[0]
    for item in list:
        if((item.heuristic + item.cost) < (itemSmallestEValue.heuristic + itemSmallestEValue.cost)):
            itemSmallestEValue = item
    return itemSmallestEValue

def getEarliestArrivalTime(intervalList):
    earliestArrival = Utilities.INFINITY
    for interval in intervalList:
        if interval.startTime < earliestArrival:
            earliestArrival = interval.startTime
    return earliestArrival

def calculateCost(maxTime, arrivalTimeDiff):
    return arrivalTimeDiff/maxTime

def getSuccessors(state, arrivalTime, robot):
    successors = []
    neighbors = graph.getNeighborPoints(state.point)
    for v in neighbors:
        intervalList = safeIntervalDict[v.id]
        dt = Utilities.timeToTraverse(state.point, v)
        for safeInterval in intervalList:
            if ((safeInterval.endTime > (arrivalTime + dt)) and
                    (safeInterval.startTime < state.safeInterval.endTime)):
                earliestArrival = safeInterval.startTime#getEarliestArrivalTime(intervalList)
                cost = calculateCost(Utilities.getMaxTime(graph), earliestArrival - arrivalTime)
                if v.z != Utilities.PARKING or v.equals(robot.goal.point):
                    successor = State(v, safeInterval, earliestArrival, cost)
                    successors.append(successor)
    return successors

def updateSafeIntervals(path):
    totalElapsedTime = 0
    timeToPassNode = Utilities.ROBOT_DIAMETER/Utilities.ROBOT_VELOCITY
    for i in xrange(len(path)-1):
        pt1 = path[i].state.point
        pt2 = path[i+1].state.point
        edgeDistance = pt1.distance(pt2)
        timeToTraverseEdge = Utilities.timeToTraverse(pt1, pt2)
        pt1SafeIntervals = safeIntervalDict[pt1.id]
        if(len(pt1SafeIntervals) == 1 and i == 0):
            interval = pt1SafeIntervals[0]
            start = totalElapsedTime + timeToPassNode/2
            end = interval.endTime
            totalElapsedTime = totalElapsedTime + timeToTraverseEdge
            interval = SafeInterval(start, end)
            safeIntervalDict[pt1.id] = [interval]
        else:
            start = totalElapsedTime - timeToPassNode/2
            end = totalElapsedTime + timeToPassNode/2
            newSafeIntervals = []
            for interval in pt1SafeIntervals:
                if interval.startTime <= start and interval.endTime >= end:
                    newSafeIntervals.append(SafeInterval(interval.startTime,start))
                    newSafeIntervals.append(SafeInterval(end, interval.endTime))
                    totalElapsedTime = totalElapsedTime + timeToTraverseEdge
                else:
                    newSafeIntervals.append(interval)
            safeIntervalDict[pt1.id] = newSafeIntervals


def SIPP(robot):
    OPEN = [OpenItem(robot.start, 0, getHeuristic(robot.start.point, robot.goal.point), 0)]
    PATH = []
    robot.start.lowestCost = 0
    robot.start.earliestArrival = 0

    while not robot.goal.point.visited:
        smallestEValue = getItemWithSmallestEValue(OPEN)
        print(smallestEValue.state.point)
        print("Start time: " + str(smallestEValue.state.safeInterval.startTime) + "| end time: " + str(smallestEValue.state.safeInterval.endTime))
        print()
        smallestEValue.state.point.visited = True
        PATH.append(smallestEValue)
        if (smallestEValue.state.point.equals(robot.goal.point)):
            robot.goal.point.visited = True
        #print("")
        #print("Current " + str(smallestEValue.state.point) + "->" + str(smallestEValue.state.cost))
        OPEN.remove(smallestEValue)
        successors = getSuccessors( smallestEValue.state, smallestEValue.startTime, robot)
        for s in successors:
            #print("\tSuccessor " + str(s.point) + "->" + str(s.cost))
            if not s.point.visited:
                s.lowestCost = s.earliestArrival = Utilities.INFINITY
            if ((s.lowestCost > smallestEValue.cost + s.cost) or
                    (s.earliestArrival > s.arrivalTime)):

                s.lowestCost = min(smallestEValue.cost + s.cost, s.lowestCost)
                s.earliestArrival = min(s.earliestArrival, s.arrivalTime)
                heuristic = getHeuristic(s.point, robot.goal.point)
                newItem = OpenItem(s, smallestEValue.cost + s.cost,
                                   smallestEValue.cost + s.cost +
                                   heuristic, s.arrivalTime)
                #print("\tHeuristic: " + str(heuristic))

                NEW_OPEN = []

                for item in OPEN:
                    if(not (item.state.point.equals(s.point) and
                            item.cost >= s.lowestCost and
                            item.state.arrivalTime >= s.earliestArrival)):
                        NEW_OPEN.append(item)

                OPEN = NEW_OPEN
                OPEN.append(newItem)
    print("PATH:")
    for p in PATH:
        print(str(p.state.point))

    return PATH


safeIntervalDict = {}
graph = Utilities.getCubeGraph()
robots = Utilities.getCubeGraphRobots()
safeIntervalDict = Utilities.initGlobalSafeIntervals(graph)
counter = 0
for robot in robots:
    print("")
    print("robot: " + str(counter) + " Start: " + str(robot.start.point) + " Goal: " + str(robot.goal.point))
    # for point in graph.points:
    #     #     point.visited = False
    print("")
    path = SIPP(robot)
    updateSafeIntervals(path)
    Utilities.printSafeIntervals(safeIntervalDict)
    counter = counter+1












