import pickle
import numpy as np
import scipy.io
import sys

fileName = sys.argv[1]
with open(fileName, 'rb') as f:
    data = pickle.load(f)

name = fileName.split('.')[0]

scipy.io.savemat('/home/avikalpg/Documents/acads/UGP-I/v_rep_python/'+name+'.mat'), mdict={'data': data})
