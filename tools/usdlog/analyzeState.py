# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: jsschell
"""
import CF_functions as cff
import matplotlib.pyplot as plt
import re
import argparse
import numpy as np
import numpy.ma as ma

parser = argparse.ArgumentParser()
parser.add_argument("file", type=str, help="logfile")
args = parser.parse_args()

# decode binary log data
logData = cff.decode(args.file)

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# number of columns and rows for suplot
plotCols = 1;
plotRows = 4;

# current plot for simple subplot usage
plotCurrent = 0;

# new figure
plt.figure(0)


plotCurrent += 1
plt.subplot(plotRows, plotCols, plotCurrent)
plt.plot(logData['tick'], logData['gyro.x'], '-', label='X')
plt.plot(logData['tick'], logData['gyro.y'], '-', label='Y')
plt.plot(logData['tick'], logData['gyro.z'], '-', label='Z')
plt.xlabel('RTOS Ticks')
plt.ylabel('Gyroscope [°/s]')
plt.legend(loc=9, ncol=3, borderaxespad=0.)


plotCurrent += 1
plt.subplot(plotRows, plotCols, plotCurrent)
plt.plot(logData['tick'], logData['acc.x'], '-', label='X')
plt.plot(logData['tick'], logData['acc.y'], '-', label='Y')
plt.plot(logData['tick'], logData['acc.z'], '-', label='Z')
plt.xlabel('RTOS Ticks')
plt.ylabel('Accelerometer [g]')
plt.legend(loc=9, ncol=3, borderaxespad=0.)

plotCurrent += 1
plt.subplot(plotRows, plotCols, plotCurrent)
plt.plot(logData['tick'], logData['stateEstimateZ.x'], '-', label='X')
plt.plot(logData['tick'], logData['stateEstimateZ.y'], '-', label='Y')
plt.plot(logData['tick'], logData['stateEstimateZ.z'], '-', label='Z')

mocapDt = np.diff(logData['locSrvZ.time'])
mask = np.concatenate(([1], (mocapDt == 0)))
mocapX = ma.masked_array(logData['locSrvZ.x'], mask=mask).compressed()
mocapY = ma.masked_array(logData['locSrvZ.y'], mask=mask).compressed()
mocapZ = ma.masked_array(logData['locSrvZ.z'], mask=mask).compressed()

mocapT = ma.masked_array(logData['tick'], mask=mask).compressed()
mocapVX = np.diff(mocapX) / (np.diff(mocapT) / 1000.0)
mocapVY = np.diff(mocapY) / (np.diff(mocapT) / 1000.0)
mocapVZ = np.diff(mocapZ) / (np.diff(mocapT) / 1000.0)

print(mocapDt)

plt.plot(mocapT, mocapX, '.', label='ext X')
plt.plot(mocapT, mocapY, '.', label='ext Y')
plt.plot(mocapT, mocapZ, '.', label='ext Z')

plt.xlabel('RTOS Ticks')
plt.ylabel('Position [mm]')
plt.legend(loc=9, ncol=3, borderaxespad=0.)

plotCurrent += 1
plt.subplot(plotRows, plotCols, plotCurrent)
plt.plot(logData['tick'], logData['stateEstimateZ.vx'], '-', label='X')
plt.plot(logData['tick'], logData['stateEstimateZ.vy'], '-', label='Y')
plt.plot(logData['tick'], logData['stateEstimateZ.vz'], '-', label='Z')

plt.plot(mocapT[1:], mocapVX, '.', label='ext X')
plt.plot(mocapT[1:], mocapVY, '.', label='ext Y')
plt.plot(mocapT[1:], mocapVZ, '.', label='ext Z')

plt.xlabel('RTOS Ticks')
plt.ylabel('Velocity [mm/s]')
plt.legend(loc=9, ncol=3, borderaxespad=0.)


plt.show()