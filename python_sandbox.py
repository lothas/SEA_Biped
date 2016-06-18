# -*- coding: utf-8 -*-
"""
Created on Fri Jun 17 17:50:08 2016

@author: Jonathan Spitz
http://blog.rcnelson.com/building-a-matplotlib-gui-with-qt-designer-part-1/
http://stackoverflow.com/questions/36665850/matplotlib-animation-inside-your-own-pyqt4-gui
"""

class DataBuffer():
    def __init__(self, bufferSize):
        self.bufferSize = bufferSize
        self.data = [0 for i in range(bufferSize+1)]
        
        self.read_i = 0
        self.write_i = 0
        
    def __getitem__(self, pos):
        if pos<0:
            return self.data[(self.read_i+pos-1)%self.bufferSize+1]
        else:
            return self.data[(self.read_i+pos)%self.bufferSize]
            
    def __len__(self):
        return self.bufferSize
        
    def push(self, val):
        self.data[self.write_i] = val
        self.write_i += 1
        if self.write_i>self.bufferSize:
            self.write_i = 0
            
        if self.write_i == self.read_i:
            # We just wrote over previous buffer data
            self.read_i += 1
            if self.read_i>self.bufferSize:
                self.read_i = 0
            
    def clear(self):
        self.read_i = 0
        self.write_i = 0
        
    def get_list(self):
        if self.write_i<self.read_i:
            list = self.data[self.read_i:]+self.data[:self.write_i]
        elif self.write_i>self.read_i:
            list = self.data[self.read_i:self.write_i]
        else:
            list = []
        return list
        