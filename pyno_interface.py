# -*- coding: utf-8 -*-
"""
Created on Fri Jun 17 17:50:08 2016

@author: Jonathan Spitz
http://blog.rcnelson.com/building-a-matplotlib-gui-with-qt-designer-part-1/
http://stackoverflow.com/questions/36665850/matplotlib-animation-inside-your-own-pyqt4-gui
"""

from collections import deque

from PyQt4.uic import loadUiType
 
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import (
    FigureCanvasQTAgg as FigureCanvas,
    NavigationToolbar2QT as NavigationToolbar)
    
import serial
from matplotlib.animation import TimedAnimation
from matplotlib.lines import Line2D
    
import matplotlib.pyplot as plt 
import matplotlib.animation as animation
from analog_plot import AnalogPlot
    
Ui_MainWindow, QMainWindow = loadUiType('pyno_interface.ui')

class Main(QMainWindow, Ui_MainWindow):
    def __init__(self, ):
        super(Main, self).__init__()
        self.setupUi(self)
        
    def addmpl(self, fig):
        self.canvas = FigureCanvas(fig)
        self.plot_window_vlayout.addWidget(self.canvas)
        self.canvas.draw()
        self.toolbar = NavigationToolbar(self.canvas, 
                self.plot_window, coordinates=True)
        self.plot_window_vlayout.addWidget(self.toolbar)
        
        
class SerialFigCanvas(FigureCanvas, TimedAnimation):
    def __init__(self, strPort, bufferSize):
        # The data
        self.time = []
        self.data = []
        self.bufferSize = bufferSize
        
        # The window
        self.fig = Figure(figsize=(5,5), dpi=100)
        ax1 = self.fig.add_subplot(111)
        # ax1 settings
        ax1.set_xlabel('time')
        ax1.set_ylabel('raw data')
        
        # The serial connection
        self.strPort = strPort
        self.ser = serial.Serial(strPort, 115200)
        print('reading from serial port %s...' % self.strPort)
        
        # get sample data
        print('initializing...')
        line = self.ser.readline()
        # Get float numbers by splitting line (by white-spaces)
        data = [float(val) for val in line.split()]
        
        self.n_plots = len(data)-1
        self.lines = []
        self.time = deque([data[0]])
        for i in range(self.n_plots):
            self.data.append(deque([data[i+1]]))
            self.lines.append(Line2D(self.time, self.data[-1], color='blue'))
            ax1.add_line(self.lines[-1])
        
        self.count = 0

        ax1.set_xlim(0, 1000)
        ax1.set_ylim(0, 4)
        self.axes = ax1

        FigureCanvas.__init__(self, self.fig)
        TimedAnimation.__init__(self, self.fig, interval = 20, blit = True)

    def _draw_frame(self, framedata):
        i = framedata
        print(i)

        if self.count < 2000:
            try:
                # Read serial data
                for j in range(8):
                    line = self.ser.readline()
                    
                # Get float numbers by splitting line (by white-spaces)
                data = [float(val) for val in line.split()]
                
                # print data
                if(len(data) >= 2):
                    plotmax = 0
                    plotmin = 0
                    self.add(data)
                    
                    time = list(self.time)
                    for j in range(self.n_plots):
                        line = list(self.data[j])
                        if min(line)<plotmin:
                            plotmin = min(line)
                        if max(line)>plotmax:
                            plotmax = max(line)
                        self.lines[j].set_data(time, line)
                        
                    self.axes.set_xlim([min(time),max(time)])
                    self.axes.set_ylim([plotmin,plotmax])
                    self.count += 1
            except KeyboardInterrupt:
                print('exiting')
        else:
            pass
        
        self._drawn_artists = self.lines

    def new_frame_seq(self):
        return iter(range(len(self.time)))

    def _init_draw(self):
        for l in self.lines:
            l.set_data([], [])
            
    # add to buffer
    def addToBuf(self, buf, val):
        if len(buf) < self.bufferSize:
            buf.append(val)
        else:
            buf.popleft()
            buf.append(val)
    
    # add data
    def add(self, data):
        assert(len(data) >= 2)
        self.addToBuf(self.time, data[0])
        for i in range(1,len(data)):
            self.addToBuf(self.data[i-1], data[i])
            
    # clean up
    def close(self):
        # close serial
        self.ser.flush()
        self.ser.close()
            
        
if __name__ == '__main__':
    import sys
    from PyQt4 import QtGui
    import numpy as np
        
    # set up animation
    fig = plt.figure()
    ax = plt.axes(xlim=(0, 200), ylim=(-1000, 1100))
    print('plotting data...')
    
    app = QtGui.QApplication(sys.argv)
    main = Main()
#    main.addmpl(fig)
    myFigCanvas = SerialFigCanvas('COM4',100)
    main.plot_window_vlayout.addWidget(myFigCanvas)
    
    main.show()
    
    # clean up
    print('exiting.') 
    sys.exit(app.exec_())