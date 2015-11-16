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
import sys
# import math
import random
import pickle
import numpy as np

delayTime = 1.0 
source_node = 1
target_node = 21998

positions_location = "/home/avikalpg/Documents/acads/UGP-I/v_rep_python/images/joint_positions.txt"
images_location = '/home/avikalpg/Desktop/tmp/images_theta_'+str(source_node)+'_'+str(target_node)+'.txt'
# positions_location = sys.argv[1]
# images_location = sys.argv[2]

with open(positions_location, 'rb') as f:
    angles = pickle.load(f)
with open(images_location, 'rb') as f:
    images = pickle.load(f)

angles = np.array(angles)
angles = np.split(angles, 2, 2)
angles = np.array(angles[1])
angles = angles.squeeze()
# print angles*180/np.pi

#########################################
############ Temporary code #############
# images = ['image10000.png', 'image10001.png', 'image10010.png', 'image10110.png', 'image10120.png']
#########################################

joints = []
for image in images:
    index = image.split('e')
    index = index[1].split('.')
    joints.append(angles[int(index[0])])
joints = np.array(joints)
print joints


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

    startVar=vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    
    for pose in joints:
        # setting random pose
        count = 0
        for joint_handle in joint_objs:
            pos = pose[count]
            vrep.simxSetJointTargetPosition(clientID, joint_handle, pos, vrep.simx_opmode_oneshot_wait)
            count = count + 1#(count + 2)%3

        # capturing image of this pose
        time.sleep(delayTime)
        #time.sleep(0.1)

    # Now close the connection to V-REP:

    stopVar=vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    print('Stop Simulation', stopVar)
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
