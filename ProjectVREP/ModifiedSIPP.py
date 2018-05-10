from Utilities import *
import Utilities



def bestNodeOfThemAll(possibleNodes, localIntervalDict, goal, localEdgeIntervalDict, lastVisited):
    smallestCost = INFINITY #get the node with the smallest cost
    smallestCostNode = 0
    for node in possibleNodes:
        edge = graph.getEdge(node.point, lastVisited.point)
        cost = len(localIntervalDict[node.point.id]) * len(localEdgeIntervalDict[edge.id]) * node.point.distance(goal.point)
        if cost < smallestCost:
            smallestCostNode = node
            smallestCost = cost
    return smallestCostNode

def getNextNode(lastVisitedNode, neighbors, elapsedTime, goal):
    possibleNodes = []
    localIntervalDict = {}
    localEdgeIntervalDict = {}
    timeToPassNode = Utilities.ROBOT_DIAMETER/Utilities.ROBOT_VELOCITY #Time to get past a node
    for neighbor in neighbors:
        timeToNeighbor = timeToTraverse(lastVisitedNode.point, neighbor) #time from last node to the neighbor
        safeIntervals = safeIntervalDict[neighbor.id] #get all safe intervals for the neighbor
        edge = graph.getEdge(neighbor, lastVisitedNode.point) #get the edge from last node to neighbor
        safeEdgeIntervals = safeEdgeIntervalDict[edge.id] #get the edge intervals
        start = elapsedTime + timeToNeighbor - timeToPassNode / 2
        end = elapsedTime + timeToNeighbor + timeToPassNode / 2
        newSafeIntervals = []
        newEdgeSafeIntervals = []
        isAdded = False
        for interval in safeIntervals:
            if interval.startTime <= start and interval.endTime >= end:
                newSafeIntervals.append(SafeInterval(interval.startTime, start)) #the interval is good, split the existing interval
                newSafeIntervals.append(SafeInterval(end, interval.endTime))
                elapsedTime = end
                isAdded = True #flag to show that an interval was indeed found
            else:
                newSafeIntervals.append(interval)
        if isAdded:
            isAdded = False
            for interval in safeEdgeIntervals:
                if interval.startTime <= elapsedTime and interval.endTime >= elapsedTime + timeToNeighbor:
                    newEdgeSafeIntervals.append(SafeInterval(interval.startTime, elapsedTime))#interval is good, split the existing edge interval
                    newEdgeSafeIntervals.append(SafeInterval(elapsedTime + timeToNeighbor, interval.endTime))
                    isAdded = True #flag to show that an interval was found
                else:
                    newEdgeSafeIntervals.append(interval)


        if isAdded: #only if a valid edge & node interval was found
            localIntervalDict[neighbor.id] = newSafeIntervals
            localEdgeIntervalDict[edge.id] = newEdgeSafeIntervals
            possibleNodes.append(State(neighbor, SafeInterval(), elapsedTime))

    bestNode = bestNodeOfThemAll(possibleNodes, localIntervalDict, goal, localEdgeIntervalDict, lastVisitedNode) #out of the nodes we found, pick the best
    safeIntervalDict[bestNode.point.id] = localIntervalDict[bestNode.point.id]
    edge = graph.getEdge(bestNode.point, lastVisitedNode.point)
    safeEdgeIntervalDict[edge.id] = localEdgeIntervalDict[edge.id]
    return bestNode


def SIPP(robot):
    PATH = [robot.start]
    elapsedTime = 0
    robot.start.lowestCost = 0
    robot.start.earliestArrival = 0

    print(robot.start.point)
    while not robot.goal.point.visited:
        lastVisitedNode = PATH[len(PATH) - 1] #get the last visited node
        neighbors = graph.getNeighborPoints(lastVisitedNode.point) #get the neighbors of the node
        nextNodeInGraph = getNextNode(lastVisitedNode, neighbors, elapsedTime, robot.goal) #determine what the next node in the path should be

        elapsedTime = nextNodeInGraph.arrivalTime #set elapsed time to the arrival time of the next node in the path
        nextNodeInGraph.point.visited = True
        if(nextNodeInGraph.point.equals(robot.goal.point)):
            robot.goal.point.visited = True

        PATH.append(nextNodeInGraph)
        print(nextNodeInGraph.point)
        print("Elapsed Time: " + str(elapsedTime))


    return PATH

safeIntervalDict = {}
graph = Utilities.getCubeGraph()
robots = Utilities.getCubeGraphRobots()
safeIntervalDict = Utilities.initGlobalSafeIntervals(graph)
safeEdgeIntervalDict = Utilities.initGlobalEdgeSafeIntervals(graph)
counter = 0
for robot in robots:
    print("")
    print("robot: " + str(counter) + " Start: " + str(robot.start.point) + " Goal: " + str(robot.goal.point))
    # for point in graph.points:
    #     #     point.visited = False
    print("")
    path = SIPP(robot)
    discretizedPath = Utilities.discretizePath(path)
    for i in xrange(0, len(discretizedPath), 3):
        print("x: " + str(discretizedPath[i]) +
              ", y: " + str(discretizedPath[i+1]) +
              ", z: " + str(discretizedPath[i+2]))
    Utilities.printSafeIntervals(safeIntervalDict)
    counter = counter+1












