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
    
Ui_MainWindow, QMainWindow = loadUiType('pyno_interface.ui')


class Main(QMainWindow, Ui_MainWindow):
    def __init__(self, portStr, bufferSize, lineSkip):
        super(Main, self).__init__()
        self.setupUi(self)
        
        # Add plotting area
        self.canvas = SerialFigCanvas(portStr,
                                      bufferSize,
                                      lineSkip,
                                      self.console)
        self.plot_window_vlayout.addWidget(self.canvas)
        
        self.canvas.draw()
        self.toolbar = NavigationToolbar(self.canvas, 
                self.plot_window, coordinates=True)
        self.plot_window_vlayout.addWidget(self.toolbar)
        
        # Connect buttons
        self.start_plot_button.clicked.connect(self.startPlot)
        self.stop_plot_button.clicked.connect(self.stopPlot)
        self.clear_plot_button.clicked.connect(self.clearPlot)
        self.save_to_matlab.clicked.connect(self.saveToMATLAB)
        self.motor_fwd_button.clicked.connect(self.canvas.sendMotorFwd)
        self.motor_bwd_button.clicked.connect(self.canvas.sendMotorBwd)
        self.motor_sine_button.clicked.connect(self.canvas.sendMotorSine)
        self.emergency_button.clicked.connect(self.canvas.sendEmergencyStop)
        
    def closeEvent(self, event):
        exit()
        
    def startPlot(self, item):
        self.canvas.doPlot = True
        
    def stopPlot(self, item):
        self.canvas.doPlot = False
        
    def clearPlot(self, item):
        self.canvas.clearBuffers()
        
    def saveToMATLAB(self, item):
        print("Saving data for MATLAB to pyno_data.txt")
        with open("pyno_data.txt", "w") as text_file:
            timeStr = "{0}".format(list(self.canvas.time))
            text_file.write(timeStr[1:len(timeStr)-1])
            for i in range(self.canvas.n_plots):                
                dataStr = "{0}".format(list(self.canvas.data[i]))
                text_file.write("\n" + dataStr[1:len(dataStr)-1])
        
        
class SerialFigCanvas(FigureCanvas, TimedAnimation):
    def __init__(self, strPort, bufferSize, lineSkip, console):
        # The data
        self.time = []
        self.data = []
        self.bufferSize = bufferSize
        self.lineSkip = lineSkip
        
        # The window
        self.fig = Figure(figsize=(5,5), dpi=100)
        ax1 = self.fig.add_subplot(111)
        # ax1 settings
        ax1.set_xlabel('time')
        ax1.set_ylabel('raw data')
        self.console = console
        
        # The serial connection
        self.strPort = strPort
        self.ser = serial.Serial(strPort, 115200)
        print('reading from serial port %s...' % self.strPort)
        
        # get sample data
        print('initializing...')
        line = self.ser.readline()
        # Get float numbers by splitting line (by white-spaces)
        data = [float(val) for val in line.split()]
        
        colors = ['blue','red','black','green']
        self.n_plots = len(data)-1
        self.lines = []
        self.time = deque([data[0]])
        for i in range(self.n_plots):
            self.data.append(deque([data[i+1]]))
            self.lines.append(Line2D(self.time, self.data[-1], color=colors[i%4]))
            ax1.add_line(self.lines[-1])
        
        self.doPlot = True

        ax1.set_xlim(0, 1000)
        ax1.set_ylim(0, 4)
        self.axes = ax1

        FigureCanvas.__init__(self, self.fig)
        TimedAnimation.__init__(self, self.fig, interval = 20, blit = True)

    def _draw_frame(self, framedata):
#        i = framedata
#        print(i)

        try:
            # Read serial data
            for j in range(self.lineSkip):
                line = self.read_serial()
            
            if self.doPlot:
                # Get float numbers by splitting line (by white-spaces)
                data = [float(val) for val in line.split()]
                
                # print data
                if(len(data) >= 2):
                    plotMax = 0
                    plotMin = 0
                    self.add(data)
                    
                    time = list(self.time)
                    for j in range(self.n_plots):
                        line = list(self.data[j])
                        if min(line)<plotMin:
                            plotMin = min(line)
                        if max(line)>plotMax:
                            plotMax = max(line)
                        self.lines[j].set_data(time, line)
                        
                    plotDelta = max(plotMax - plotMin, 1)
                    self.axes.set_xlim([min(time),max(time)])
                    self.axes.set_ylim([plotMin-0.1*plotDelta,
                                        plotMax+0.1*plotDelta])
                                      
                    self.draw()
                    
        except KeyboardInterrupt:
            print('exiting')
        
        self._drawn_artists = self.lines

    def new_frame_seq(self):
        return iter(range(len(self.time)))

    def _init_draw(self):
        for l in self.lines:
            l.set_data([], [])
            
    def read_serial(self):
        line = self.ser.readline()
        if line[0].isalpha():
            # This is a text output message
            self.display_message(line) # send message to console and
            return self.read_serial() # read next line
        else:
            # This is a time-stamped data message
            return line
        
    def display_message(self, line):
        self.console.insertPlainText(line)
        self.console.moveCursor(QtGui.QTextCursor.End)
        
    def send_message(self, line):
        self.ser.write(line)
        
    def sendEmergencyStop(self, item):
        self.send_message('se')
        
    def sendMotorFwd(self, item):
        self.send_message('smf5e')
            
    def sendMotorBwd(self, item):
        self.send_message('smb5e')
        
    def sendMotorSine(self, item):
        self.send_message('sse')
        
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
            
    # clear buffers
    def clearBuffers(self):
        last_val = self.time[-1]
        self.time.clear()
        self.time.append(last_val)
        for i in range(self.n_plots):
            last_val = self.data[i][-1]
            self.data[i].clear()
            self.data[i].append(last_val)
            
    # clean up
    def close(self):
        # close serial
        self.ser.flush()
        self.ser.close()
            
        
if __name__ == '__main__':
    import sys
    from PyQt4 import QtGui
    
    # Interface configuration
    portStr = 'COM19'
    bufferSize = 200
    lineSkip = 6
        
    # set up animation
    print('plotting data...')
    app = QtGui.QApplication(sys.argv)
    main = Main(portStr, bufferSize, lineSkip)
    main.show()
    
    # clean up
    print('exiting.') 
    sys.exit(app.exec_())