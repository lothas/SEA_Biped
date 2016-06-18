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

from collections import deque

import matplotlib.pyplot as plt 
import matplotlib.animation as animation
from analog_plot import AnalogPlot

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
  analogPlot.plotHandles = plotHandles
  print('plotting data...')
  anim = animation.FuncAnimation(fig, analogPlot.update, interval=40)

  # show plot
  plt.show()
  
  # clean up
  analogPlot.close()

  print('exiting.')
  

# call main
if __name__ == '__main__':
  main()