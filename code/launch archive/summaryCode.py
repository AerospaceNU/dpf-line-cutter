# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import matplotlib.pyplot as plt
import numpy as np

def csvRead(filepath):
    data = np.genfromtxt(filepath, delimiter=',', skip_header=1)
    return data

def filterData(array, before, after):
    deployedRow = np.where(array[:,0] == 2)[0][0]
    landedRow = np.where(array[:,0] == 5)[0][0]
    return array[deployedRow-before:landedRow+after]

def stateTransitionRows(array):
    rows = []
    for i in range(1,6):
        rows.append(np.where(array[:,0] == i)[0][0])
    return rows

def plotData(array, headers, lineColor, disreef1, disreef2):
    timeCol = 1
    numPlots = array.shape[1]
    plotColumns = 1
    stateChangeRows = stateTransitionRows(array)
    print('State changes at times: ', end=' ')
    for i in range(1,6):
        print(array[stateChangeRows[i-1], timeCol], end='  ')
    print()
    fig, axs = plt.subplots(numPlots, plotColumns, figsize=(16, numPlots*4))
    for i in range(numPlots):
        currentAx = axs[i]
        currentAx.plot(array[:,timeCol], array[:,i], linewidth=0.5, color=lineColor)
        currentAx.set_title(headers[i])
        for row in stateChangeRows:
            currentAx.axvline(array[row,timeCol], color='black', linestyle='dotted', linewidth=0.5)
        if (headers[i] == 'Smooth altitude (m)'):
            if (disreef1 > 0):
                currentAx.axhline(disreef1, color='black', linestyle='dotted', linewidth=0.5)
            if (disreef2 > 0):
                currentAx.axhline(disreef2, color='black', linestyle='dotted', linewidth=0.5)
    fig.tight_layout(pad=3.0)
            
'''
Plots line cutter data in a series of vertically arranged numpy subplots.

Parameters
----------
filepath : str
           Specifies the relative path to the data csv
lineColor : str
            One of the numpy supported colors, as seen at https://matplotlib.org/stable/gallery/color/named_colors.html. 
            Default is 'r' (red).
before: int
        Number of data points before deployment detection to include in graph
after: int
       Number of data points after landing detection to include in graph
disreef1: int
        Altitude of first disreef in meters (to plot a horizontal line on the graph), should be >0
disreef2: int
       Altitude of second disreef in meters (to plot a horizontal line on the graph), should be >0
headers : list
          The list of strings to use as subplot headings for the data, in column order.
          Default is ['State', 'Time (ms)', 'Pressure [Pa]',  'Altitude (m)', 
                      'Smooth altitude (m)', 'Delta altitude (m/s)', 'Smooth delta altitude (m/s)',
                       'Temperature (C)', 'Accel X', 'Accel Y', 'Accel Z', 'Batt Sense', 
                       'Cut Sense 1', 'Cut Sense 2', 'Current Sense', 'Photoresistor']
'''
def blackMagic(filepath, lineColor='r', before=1200, after=500, disreef1=-1, disreef2=-1,
               headers=['State', 'Time (ms)', 'Pressure [Pa]', 
                      'Altitude (m)', 'Smooth altitude (m)', 'Delta altitude (m/s)', 'Smooth delta altitude (m/s)',
                       'Temperature (C)', 'Accel X', 'Accel Y', 'Accel Z',
                       'Batt Sense', 'Cut Sense 1', 'Cut Sense 2', 'Current Sense', 'Photoresistor']):
    array = csvRead(filepath)
    plotData(filterData(array, before, after), headers, lineColor, disreef1, disreef2)