import CF_functions as cff
import matplotlib.pyplot as plt
import re
import argparse
import numpy as np
import numpy.ma as ma
import math

# decompress a quaternion, see quatcompress.h in firmware
def quatdecompress(comp):
  q = np.zeros(4)
  mask = (1 << 9) - 1
  i_largest = comp >> 30
  sum_squares = 0
  for i in range(3, -1, -1):
    if i != i_largest:
      mag = comp & mask
      negbit = (comp >> 9) & 0x1
      comp = comp >> 10
      q[i] = mag / mask / math.sqrt(2)
      if negbit == 1:
        q[i] = -q[i]
      sum_squares += q[i] * q[i]
  q[i_largest] = math.sqrt(1.0 - sum_squares)
  return q

# convert quaternion to (roll, pitch, yaw) Euler angles using Tait-Bryan convention
# (yaw, then pitch about new pitch axis, then roll about new roll axis)
# assume q = [x,y,z,w] order
def quat2rpy(q):
  rpy = np.zeros(3)
  rpy[0] = math.atan2(2 * (q[3] * q[0] + q[1] * q[2]), 1 - 2 * (q[0] * q[0] + q[1] * q[1]))
  rpy[1] = math.asin(2 * (q[3] * q[1] - q[0] * q[2]))
  rpy[2] = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))
  return rpy


parser = argparse.ArgumentParser()
parser.add_argument("file", type=str, help="logfile")
args = parser.parse_args()

# decode binary log data
logData = cff.decode(args.file)

time = logData['tick'] / 1e6 # us -> s

# compute additional data (mocap points)
mocapDt = np.diff(logData['locSrvZ.time'])
mask = np.concatenate(([1], (mocapDt == 0)))
mocapPos = [
  ma.masked_array(logData['locSrvZ.x'] / 1000.0, mask=mask).compressed(),
  ma.masked_array(logData['locSrvZ.y'] / 1000.0, mask=mask).compressed(),
  ma.masked_array(logData['locSrvZ.z'] / 1000.0, mask=mask).compressed()]

mocapT = ma.masked_array(time, mask=mask).compressed()
mocapV = [np.diff(p) / np.diff(mocapT) for p in mocapPos]

# compute rotations

quatState = np.array([quatdecompress(int(c)) for c in logData['stateEstimateZ.quat']])
# quatState = [[
#   logData['stateEstimate.qx'][i],
#   logData['stateEstimate.qy'][i],
#   logData['stateEstimate.qz'][i],
#   logData['stateEstimate.qw'][i]] for i in range(len(time))]
state_rpy = np.array([quat2rpy(q) for q in quatState])


mocapQuat = ma.masked_array(logData['locSrvZ.quat'], mask=mask).compressed()
quatMocap = np.array([quatdecompress(int(c)) for c in mocapQuat])
mocap_rpy = np.array([quat2rpy(q) for q in quatMocap])

# set window background to white
# plt.rcParams['figure.facecolor'] = 'w'
    
# number of columns and rows for suplot
plotCols = 3
plotRows = 3

# current plot for simple subplot usage
plotCurrent = 0

for i, axis in enumerate(['x', 'y', 'z']):

  plotCurrent = i + 1
  plt.subplot(plotRows, plotCols, plotCurrent)
  plt.plot(time, logData['stateEstimateZ.' + axis] / 1000.0, '-', label='state')
  plt.plot(time, logData['ctrltargetZ.' + axis] / 1000.0, '-', label='target')
  plt.plot(mocapT, mocapPos[i], '.', label='mocap')
  plt.xlabel('Time [s]')
  plt.ylabel('Position {} [m]'.format(axis))
  plt.legend(loc=9, ncol=3, borderaxespad=0.)

  plotCurrent = i + 4
  plt.subplot(plotRows, plotCols, plotCurrent)
  plt.plot(time, logData['stateEstimateZ.v' + axis] / 1000.0, '-', label='state')
  plt.plot(time, logData['ctrltargetZ.v' + axis] / 1000.0, '-', label='target')
  plt.plot(mocapT[1:], mocapV[i], '.', label='mocap')
  plt.xlabel('Time [s]')
  plt.ylabel('Velocity {} [m/s]'.format(axis))
  plt.legend(loc=9, ncol=3, borderaxespad=0.)

  plotCurrent = i + 7
  plt.subplot(plotRows, plotCols, plotCurrent)
  plt.plot(time, np.degrees(state_rpy[:,i]), '-', label='state')
  plt.plot(mocapT, np.degrees(mocap_rpy[:,i]), '.', label='mocap')
  plt.xlabel('Time [s]')
  plt.ylabel('Angle {} [deg]'.format(axis))
  plt.legend(loc=9, ncol=3, borderaxespad=0.)

plt.show()
