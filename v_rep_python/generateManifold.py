#######################################################################
#################### ABOUT THE FILE - FUNCTION ########################
#######################################################################
# This file currently gets 100 poses from all the visual sensors in the 
# active scene, at intervals of 1 sec.
# It saves the images in directories inside images folder, named after
# the handle number of the visual sensor they belong to
########################################################################

# When running Python 3.x, add a 'b' prefix to strings, e.g.:
# 'hello world' becomes b'hello world'

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time	
from PIL import Image
import os
import math
import random
import numpy as np

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res, joint_objs, x, y, joint_names=vrep.simxGetObjectGroupData(clientID,vrep.sim_object_joint_type, 0,vrep.simx_opmode_oneshot_wait)
    if res==vrep.simx_return_ok:
        print ('Number of joints in the scene: ',len(joint_objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    res, vs_objs, x, y, vs_names=vrep.simxGetObjectGroupData(clientID,vrep.sim_object_visionsensor_type, 0, vrep.simx_opmode_oneshot_wait)
    if res==vrep.simx_return_ok:
        print ('Number of vision sensors in the scene: ',len(vs_objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    # for handle in objs:
    #     JointPosition=vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_oneshot_wait)
    #     print(JointPosition)

    startVar=vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    for pose in range(0,10):
        # setting random pose
        for joint_handle in joint_objs:
            pos = random.randint(1, 360)
            vrep.simxSetJointTargetPosition(clientID, joint_handle, math.radians(pos), vrep.simx_opmode_oneshot_wait)

        # capturing image of this pose
        time.sleep(1.0)
        # startVar=vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
        for index in range(0, len(vs_objs)):
            vs_handle=vs_objs[index]
            objName=vs_names[index]
            print('pose:'+str(pose),'handle:'+str(vs_handle), 'name:'+objName)
            if not os.path.exists('images/'+objName+'/'):
                os.makedirs('images/'+objName+'/')
            
            #readSensor=vrep.simxReadVisionSensor(clientID, handle, vrep.simx_opmode_oneshot_wait)
            #print('readSensor= ', readSensor)
            # Getting the image
            data=vrep.simxGetVisionSensorImage(clientID,vs_handle,0,vrep.simx_opmode_oneshot_wait)
            if data[0]==vrep.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
                # print('resolution', data[1])
                width=data[1][0]
                height=data[1][1]
                #print('size of image', len(data[2]))
                imageData=np.array(data[2]).reshape([height, width, 3])
                # print imageData.shape
                img = Image.fromarray(np.uint8(imageData))
                #img = img.rotate(180)
                img = img.transpose(1)
                img.save('images/'+objName+'/image'+str(pose)+'.png')
            else:
                print ('received data was not OK')
        #pauseVar=vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
        #time.sleep(0.1)

    # Now close the connection to V-REP:

    stopVar=vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    print('Stop Simulation', stopVar)
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
