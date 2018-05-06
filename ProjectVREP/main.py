import vrep

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,-500000,5) # Connect to V-REP, set a very large time-out for blocking commands
if clientID!=-1:
    print ('Connected to remote API server')

    emptyBuff = bytearray()

    # Start the simulation:
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)

    # Load a robot instance:    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'loadRobot',[],[0,0,0,0],['d:/v_rep/qrelease/release/test.ttm'],emptyBuff,vrep.simx_opmode_oneshot_wait)
    #    robotHandle=retInts[0]
    
    # Retrieve some handles:
    ret,quad1=vrep.simxGetObjectHandle(clientID,'Quad_Init',vrep.simx_opmode_oneshot_wait)
    #res,target1=vrep.simxGetObjectHandle(clientID,'Quad_target',vrep.simx_opmode_oneshot_wait)

    ret,pos=vrep.simxGetObjectPosition(clientID, quad1, -1, vrep.simx_opmode_oneshot_wait)
    print "Position: (",round(pos[0], 2),round(pos[1], 2),round(pos[2], 2),")"
    
    vrep.simxCallScriptFunction(clientID,'Init',vrep.sim_scripttype_childscript,'setPath',[],[35, 0.25, 0.25, 0.5, 0.5, 0.5, 0.25, 0.5, 0, 0.5, -0.25, 0.5, -0.5, 0.25, -0.5, 0, -0.5, -0.25, -0.5, -0.5, -0.5, -0.5, -0.25, -0.5, 0, -0.5, 0.25, -0.5, 0.5, -0.25, 0.25, 0, 0, 0.25, -0.25],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    
    # Wait until path completed:
    runningPath=True
    while runningPath:
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'Init',vrep.sim_scripttype_childscript,'finished',[],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        runningPath=retInts[0]==0

    # Stop simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')