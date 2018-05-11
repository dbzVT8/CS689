import vrep
from datetime import datetime
import Utilities
import ModifiedSIPP

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,-500000,5) # Connect to V-REP, set a very large time-out for blocking commands
if clientID!=-1:
    print ('Connected to remote API server')

    emptyBuff = bytearray()
    
    # Get robots
    robotNames = ["Quad", "Quad#0", "Quad#1", "Quad#2"]
    robotHandles = []
    robotPaths = []
    safeIntervalDict = {}
    graph = Utilities.getCubeGraph()
    robots = Utilities.getCubeGraphRobots()
    safeIntervalDict = Utilities.initGlobalSafeIntervals(graph)
    safeEdgeIntervalDict = Utilities.initGlobalEdgeSafeIntervals(graph)

    for i, robot in enumerate(robots):
        path = Utilities.discretizePath(ModifiedSIPP.SIPP(robot, graph))
        path.insert(0, len(path))
        robotPaths.append(path)

    if len(robots) <= len(robotNames):
        # Start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        
        # Retrieve robot handles and set their paths:
        #for i, robot in enumerate(robots):
        ret,quad=vrep.simxGetObjectHandle(clientID, robotNames[i] ,vrep.simx_opmode_oneshot_wait)
        vrep.simxCallScriptFunction(clientID,'Init',vrep.sim_scripttype_childscript,'setPath',[],robotPaths[0],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        path = robotPaths[0]
        initPos = [ path[1], path[2], path[3]] #Skip the first item b/c it contains the path length
        vrep.simxCallScriptFunction(clientID,'Init',vrep.sim_scripttype_childscript,'setInitLocation',[],initPos,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        robotHandles.append(quad) 
        
        #for i in xrange(robots):
        vrep.simxCallScriptFunction(clientID,'Init',vrep.sim_scripttype_childscript,'start',[],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        print "Start: ",str(datetime.now())
        
        # Wait until path completed:
        runningPath=True
        while runningPath:
            ret,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'Init',vrep.sim_scripttype_childscript,'finished',[],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
            runningPath=retInts[0]==0
    
        print "End: ",str(datetime.now())
    
        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print "Error: Add " + str(len(robots) - len(robotNames)) + " more VREP robots!"
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
