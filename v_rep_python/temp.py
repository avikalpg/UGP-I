import pickle
import numpy as np
import scipy.io
import sys

fileName = 'images/joint_positions.txt'
with open(fileName, 'rb') as f:
    angles = pickle.load(f)
angles = np.array(angles)
angles = np.split(angles, 2, 2)
angles = np.array(angles[1])
angles = angles.squeeze()

name = fileName.split('.')[0]

scipy.io.savemat('/home/avikalpg/Documents/acads/UGP-I/v_rep_python/jointAngles.mat', mdict={'data': angles})
