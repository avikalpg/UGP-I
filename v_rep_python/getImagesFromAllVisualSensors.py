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
from PIL import Image
import os

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
	
print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_object_visionsensor_type,vrep.simx_opmode_oneshot_wait)
    if res==vrep.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    print(objs)
    startVar=vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    print('Start Simulation:', startVar)
    #time.sleep(10);
    startTime=time.time()

    for pose in range(0, 100):
        time.sleep(1)
        # Getting the handle of one of the vision sensors
        for handle in objs:
            print('pose:'+str(pose),'handle:'+str(handle))
            if not os.path.exists('images/'+str(handle)+'/'):
                os.makedirs('images/'+str(handle)+'/')
            
            #readSensor=vrep.simxReadVisionSensor(clientID, handle, vrep.simx_opmode_oneshot_wait)
            #print('readSensor= ', readSensor)
            # Getting the image
            data=vrep.simxGetVisionSensorImage(clientID,handle,0,vrep.simx_opmode_oneshot_wait)
            if data[0]==vrep.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
                print('resolution', data[1])
                width=data[1][0]
                height=data[1][1]
                #print('size of image', len(data[2]))
                img=Image.new('RGB', (width, height))
                imageData=[]
                for h in range(0,height):
                    for w in range(0,width):
                        imageData.append((data[2][(3*(w))+(3*width*(height-1-h))], data[2][1+(3*(w))+(3*width*(height-1-h))], data[2][2+(3*(w))+(3*width*(height-1-h))]))
                        #print('Image size', len(imageData))
                img.putdata(imageData)
                #img = img.rotate(180)
                img.save('images/'+str(handle)+'/image'+str(pose)+'.png')
            else:
                print ('received data was not OK')
                print (vrep.simx_return_ok)

    # Now close the connection to V-REP:
    pauseVar=vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    print('Pause Simulation', pauseVar)
    # stopVar=vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    # print('Stop Simulation', stopVar)
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
