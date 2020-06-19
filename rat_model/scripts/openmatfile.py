#!/usr/bin/env python
from scipy.io import loadmat
"""
x = loadmat('model_output_H.mat')
lon = x['plotRaster']
lat = x['lat']
# one-liner to read a single variable
lon = loadmat('test.mat')['lon']
"""
data = loadmat("model_output_H.mat")

print(data)
