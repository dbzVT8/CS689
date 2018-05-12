import vrep
from datetime import datetime
import time
import Utilities

def bestNodeOfThemAll(possibleNodes, localIntervalDict, goal, localEdgeIntervalDict, lastVisited):
    smallestCost = Utilities.INFINITY #get the node with the smallest cost
    smallestCostNode = None
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
        timeToNeighbor = Utilities.timeToTraverse(lastVisitedNode.point, neighbor) #time from last node to the neighbor
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
                newSafeIntervals.append(Utilities.SafeInterval(interval.startTime, start)) #the interval is good, split the existing interval
                newSafeIntervals.append(Utilities.SafeInterval(end, interval.endTime))
                elapsedTime = end
                isAdded = True #flag to show that an interval was indeed found
            else:
                newSafeIntervals.append(interval)
        if isAdded:
            isAdded = False
            for interval in safeEdgeIntervals:
                if interval.startTime <= elapsedTime and interval.endTime >= elapsedTime + timeToNeighbor:
                    newEdgeSafeIntervals.append(Utilities.SafeInterval(interval.startTime, elapsedTime))#interval is good, split the existing edge interval
                    newEdgeSafeIntervals.append(Utilities.SafeInterval(elapsedTime + timeToNeighbor, interval.endTime))
                    isAdded = True #flag to show that an interval was found
                else:
                    newEdgeSafeIntervals.append(interval)


        if isAdded: #only if a valid edge & node interval was found
            localIntervalDict[neighbor.id] = newSafeIntervals
            localEdgeIntervalDict[edge.id] = newEdgeSafeIntervals
            possibleNodes.append(Utilities.State(neighbor, Utilities.SafeInterval(), elapsedTime))

    bestNode = bestNodeOfThemAll(possibleNodes, localIntervalDict, goal, localEdgeIntervalDict, lastVisitedNode) #out of the nodes we found, pick the best
    safeIntervalDict[bestNode.point.id] = localIntervalDict[bestNode.point.id]
    edge = graph.getEdge(bestNode.point, lastVisitedNode.point)
    safeEdgeIntervalDict[edge.id] = localEdgeIntervalDict[edge.id]
    return bestNode


def SIPP(robot):
    path = [robot.start]
    elapsedTime = 0
    robot.start.lowestCost = 0
    robot.start.earliestArrival = 0

    print(robot.start.point)
    while not robot.goal.point.visited:
        lastVisitedNode = path[len(path) - 1] #get the last visited node
        neighbors = graph.getNeighborPoints(lastVisitedNode.point) #get the neighbors of the node
        nextNodeInGraph = getNextNode(lastVisitedNode, neighbors, elapsedTime, robot.goal) #determine what the next node in the path should be

        elapsedTime = nextNodeInGraph.arrivalTime #set elapsed time to the arrival time of the next node in the path
        nextNodeInGraph.point.visited = True
        if(nextNodeInGraph.point.equals(robot.goal.point)):
            robot.goal.point.visited = True

        path.append(nextNodeInGraph)

    return path


safeIntervalDict = {}
graph = Utilities.getCubeGraph()
robots = Utilities.getCubeGraphRobots()
safeIntervalDict = Utilities.initGlobalSafeIntervals(graph)
safeEdgeIntervalDict = Utilities.initGlobalEdgeSafeIntervals(graph)

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,-500000,5) # Connect to V-REP, set a very large time-out for blocking commands
if clientID!=-1:
    print ('Connected to remote API server')

    emptyBuff = bytearray()
    
    # Get robots
    scriptNames = ["Init", "Init#0", "Init#1", "Init#2"]
    robotNames = ["Quad", "Quad#0", "Quad#1", "Quad#2"]
    robotPaths = []

    # robots.remove(robots[2])
    # robots.remove(robots[2])
    for robot in robots:
        path = Utilities.discretizePath(SIPP(robot))
        path.insert(0, len(path))
        robotPaths.append(path)

    print "node"
    Utilities.printSafeIntervals(safeIntervalDict)
    print"Edge"
    Utilities.printSafeIntervals(safeEdgeIntervalDict)

    if len(robots) <= len(robotNames):
        # Start the simulation:
        time.sleep(5)
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        for i, robot in enumerate(robots):
            vrep.simxCallScriptFunction(clientID,scriptNames[i],vrep.sim_scripttype_childscript,'setPath',[],robotPaths[i],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

        initializing = True
        while initializing:
            initializing = False
            for i, robot in enumerate(robots):
                ret,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(clientID, scriptNames[i], vrep.sim_scripttype_childscript,
                                                                                         'initialized', [], [], [], emptyBuff,
                                                                                         vrep.simx_opmode_oneshot_wait)
                if retInts[0] == 0:
                    initializing = True
                    break

        for i, robot in enumerate(robots):
            vrep.simxCallScriptFunction(clientID,scriptNames[i],vrep.sim_scripttype_childscript,'startSim',[],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

        print "Start: " + str(datetime.now())
        
        # Wait until path completed:
        runningPath = True
        while runningPath:
            runningPath = False
            for i, robot in enumerate(robots):
                ret,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(clientID, scriptNames[i],vrep.sim_scripttype_childscript,
                                                                                         'finished',[],[],[],emptyBuff,
                                                                                         vrep.simx_opmode_oneshot_wait)
                if retInts[0] == 0:
                    runningPath = True
                    break
    
        print "End: " + str(datetime.now())
    
        # Stop simulation:
        time.sleep(15)
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print "Error: Add " + str(len(robots) - len(robotNames)) + " more VREP robots!"
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

"""counter = 0
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
    counter = counter+1"""
