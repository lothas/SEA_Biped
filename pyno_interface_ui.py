# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'pyno_interface.ui'
#
# Created: Fri Jun 17 17:50:00 2016
#      by: PyQt4 UI code generator 4.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1024, 768)
        MainWindow.setMinimumSize(QtCore.QSize(640, 480))
        MainWindow.setMaximumSize(QtCore.QSize(1920, 1080))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8("../../../../../Python27/Lib/site-packages/xy/img/gnuplot.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.plot_window = QtGui.QWidget(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plot_window.sizePolicy().hasHeightForWidth())
        self.plot_window.setSizePolicy(sizePolicy)
        self.plot_window.setObjectName(_fromUtf8("plot_window"))
        self.horizontalLayout.addWidget(self.plot_window)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.start_plot_button = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.start_plot_button.sizePolicy().hasHeightForWidth())
        self.start_plot_button.setSizePolicy(sizePolicy)
        self.start_plot_button.setMaximumSize(QtCore.QSize(200, 16777215))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(_fromUtf8("icons/play.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.start_plot_button.setIcon(icon1)
        self.start_plot_button.setObjectName(_fromUtf8("start_plot_button"))
        self.verticalLayout.addWidget(self.start_plot_button)
        self.stop_plot_button = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.stop_plot_button.sizePolicy().hasHeightForWidth())
        self.stop_plot_button.setSizePolicy(sizePolicy)
        self.stop_plot_button.setMaximumSize(QtCore.QSize(200, 16777215))
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(_fromUtf8("icons/stop2.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.stop_plot_button.setIcon(icon2)
        self.stop_plot_button.setObjectName(_fromUtf8("stop_plot_button"))
        self.verticalLayout.addWidget(self.stop_plot_button)
        self.clear_plot_button = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.clear_plot_button.sizePolicy().hasHeightForWidth())
        self.clear_plot_button.setSizePolicy(sizePolicy)
        self.clear_plot_button.setMaximumSize(QtCore.QSize(200, 16777215))
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(_fromUtf8("icons/wipe2.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.clear_plot_button.setIcon(icon3)
        self.clear_plot_button.setObjectName(_fromUtf8("clear_plot_button"))
        self.verticalLayout.addWidget(self.clear_plot_button)
        self.save_to_matlab = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.save_to_matlab.sizePolicy().hasHeightForWidth())
        self.save_to_matlab.setSizePolicy(sizePolicy)
        self.save_to_matlab.setMaximumSize(QtCore.QSize(200, 16777215))
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(_fromUtf8("icons/save.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.save_to_matlab.setIcon(icon4)
        self.save_to_matlab.setObjectName(_fromUtf8("save_to_matlab"))
        self.verticalLayout.addWidget(self.save_to_matlab)
        self.label = QtGui.QLabel(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setMaximumSize(QtCore.QSize(200, 16777215))
        self.label.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout.addWidget(self.label)
        self.buffer_size_input = QtGui.QLineEdit(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.buffer_size_input.sizePolicy().hasHeightForWidth())
        self.buffer_size_input.setSizePolicy(sizePolicy)
        self.buffer_size_input.setMaximumSize(QtCore.QSize(200, 16777215))
        self.buffer_size_input.setInputMask(_fromUtf8(""))
        self.buffer_size_input.setText(_fromUtf8(""))
        self.buffer_size_input.setMaxLength(32767)
        self.buffer_size_input.setAlignment(QtCore.Qt.AlignCenter)
        self.buffer_size_input.setObjectName(_fromUtf8("buffer_size_input"))
        self.verticalLayout.addWidget(self.buffer_size_input)
        self.horizontalLayout.addLayout(self.verticalLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1024, 21))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Arduino - Python interface", None))
        self.start_plot_button.setText(_translate("MainWindow", "Start plotting", None))
        self.stop_plot_button.setText(_translate("MainWindow", "Stop plotting", None))
        self.clear_plot_button.setText(_translate("MainWindow", "Clear plot", None))
        self.save_to_matlab.setText(_translate("MainWindow", "Save to MATLAB", None))
        self.label.setText(_translate("MainWindow", "Buffer size", None))
        self.buffer_size_input.setPlaceholderText(_translate("MainWindow", "200", None))

