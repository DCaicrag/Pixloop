#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep  9 10:34:20 2023

@author: rolando
"""

import numpy as np
import csv
from pandas import read_csv
import matplotlib.pyplot as plt

# with open('prueba2_waypoints.csv') as csvfile:
#     wpreader = csv.reader(csvfile)

wp = read_csv('prueba2_waypoints.csv')

x = wp.loc[:,'x'].to_numpy()
y = wp.loc[:,'y'].to_numpy()

plt.plot(x,y,'x')
plt.grid()
plt.title('waypoints')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')