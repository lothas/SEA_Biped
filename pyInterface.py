# -*- coding: utf-8 -*-
"""
Created on Wed Jun 15 13:36:27 2016

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
from collections import deque

import matplotlib.pyplot as plt 
import matplotlib.animation as animation

    
# plot class
class AnalogPlot:
  # constr
  def __init__(self, strPort, maxLen):
      # open serial port
      self.ser = serial.Serial(strPort, 115200)

      self.time = []
      self.data = []
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
  def update(self, frameNum, plotHandles):
      if self.count < 2000:
          try:
              line = self.ser.readline()
              # Get float numbers by splitting line (by white-spaces)
              data = [float(val) for val in line.split()]
              # print data
              if(len(data) >= 2):
                  self.add(data)
                  plotHandle = 0
                  for i in range(1,len(data)):
                      plotHandle = plotHandles[i-1][0]
                      plotHandle.set_data(self.time, self.data[i-1])
                  axes = plt.gca()
                  axes.set_xlim([min(self.time),max(self.time)])
                  self.count += 1
          except KeyboardInterrupt:
              print('exiting')
      else:
          pass
      
      return plotHandles[0], 

  # clean up
  def close(self):
      # close serial
      self.ser.flush()
      self.ser.close()    

# main() function
def main():
#  # create parser
#  parser = argparse.ArgumentParser(description="LDR serial")
#  # add expected arguments
#  parser.add_argument('--port', dest='port', required=True)
#
#  # parse args
#  args = parser.parse_args()
#  
#  #strPort = '/dev/tty.usbserial-A7006Yqh'
#  strPort = args.port
  strPort = 'COM4'

  print('reading from serial port %s...' % strPort)

  # plot parameters
  analogPlot = AnalogPlot(strPort, 100)

  print('initializing...')
  # get sample data
  line = analogPlot.ser.readline()
  # Get float numbers by splitting line (by white-spaces)
  data = [float(val) for val in line.split()]
  
  # set up animation
  fig = plt.figure()
  ax = plt.axes(xlim=(0, 200), ylim=(-1000, 1100))
  plotHandles = []
  analogPlot.time = deque([data[0]])
  for i in range(1,len(data)):
      analogPlot.data.append(deque([data[i]]))
      plotHandles.append(ax.plot([],[]))
  print('plotting data...')
  anim = animation.FuncAnimation(fig, analogPlot.update, 
                                 fargs=[plotHandles], 
                                 interval=50)

  # show plot
  plt.show()
  
  # clean up
  analogPlot.close()

  print('exiting.')
  

# call main
if __name__ == '__main__':
  main()