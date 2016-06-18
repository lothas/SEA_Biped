# -*- coding: utf-8 -*-
"""
Created on Fri Jun 17 19:49:08 2016

@author: Jonathan Spitz
"""

"""
ldr.py
Display analog data from Arduino using Python (matplotlib)
Author: Mahesh Venkitachalam
Website: electronut.in
http://electronut.in/plotting-real-time-data-from-arduino-using-python/
"""

import serial
import matplotlib.pyplot as plt 

# plot class
class AnalogPlot:
  # constr
  def __init__(self, strPort, maxLen):
      # open serial port
      self.ser = serial.Serial(strPort, 115200)

      self.time = []
      self.data = []
      self.plotHandles = []
      self.maxLen = maxLen
      self.count = 0

  # add to buffer
  def addToBuf(self, buf, val):
      if len(buf) < self.maxLen:
          buf.append(val)
      else:
          buf.pop()
          buf.appendleft(val)

  # add data
  def add(self, data):
      assert(len(data) >= 2)
      self.addToBuf(self.time, data[0])
      for i in range(1,len(data)):
          self.addToBuf(self.data[i-1], data[i])

  # update plot
  def update(self, frameNum):
      print('updating plot...')
      if self.count < 2000:
          try:
              for i in range(8):
                  line = self.ser.readline()
              # Get float numbers by splitting line (by white-spaces)
              data = [float(val) for val in line.split()]
              # print data
              if(len(data) >= 2):
                  self.add(data)
                  plotHandle = 0
                  for i in range(1,len(data)):
                      plotHandle = self.plotHandles[i-1][0]
                      plotHandle.set_data(self.time, self.data[i-1])
                  axes = plt.gca()
                  axes.set_xlim([min(self.time),max(self.time)])
                  self.count += 1
          except KeyboardInterrupt:
              print('exiting')
      else:
          pass
      
      return self.plotHandles[0], 

  # clean up
  def close(self):
      # close serial
      self.ser.flush()
      self.ser.close()