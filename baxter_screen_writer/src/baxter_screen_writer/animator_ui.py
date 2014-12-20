# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'animator.ui'
#
# Created: Sat Dec 20 15:47:50 2014
#      by: PyQt4 UI code generator 4.10.4
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
        MainWindow.resize(504, 280)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayout = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.confusionValueLabel = QtGui.QLabel(self.centralwidget)
        self.confusionValueLabel.setText(_fromUtf8(""))
        self.confusionValueLabel.setObjectName(_fromUtf8("confusionValueLabel"))
        self.gridLayout.addWidget(self.confusionValueLabel, 0, 1, 1, 1)
        self.label_13 = QtGui.QLabel(self.centralwidget)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.gridLayout.addWidget(self.label_13, 1, 0, 1, 1)
        self.label_10 = QtGui.QLabel(self.centralwidget)
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.gridLayout.addWidget(self.label_10, 3, 1, 1, 1)
        self.confusionValueSlider = QtGui.QSlider(self.centralwidget)
        self.confusionValueSlider.setOrientation(QtCore.Qt.Horizontal)
        self.confusionValueSlider.setObjectName(_fromUtf8("confusionValueSlider"))
        self.gridLayout.addWidget(self.confusionValueSlider, 6, 1, 1, 1)
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.gridLayout.addWidget(self.label_2, 6, 2, 1, 1)
        self.confusionTargetLabel = QtGui.QLabel(self.centralwidget)
        self.confusionTargetLabel.setText(_fromUtf8(""))
        self.confusionTargetLabel.setObjectName(_fromUtf8("confusionTargetLabel"))
        self.gridLayout.addWidget(self.confusionTargetLabel, 1, 1, 1, 1)
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout.addWidget(self.label, 6, 0, 1, 1)
        self.horizontalSlider_2 = QtGui.QSlider(self.centralwidget)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName(_fromUtf8("horizontalSlider_2"))
        self.gridLayout.addWidget(self.horizontalSlider_2, 8, 1, 1, 1)
        self.label_11 = QtGui.QLabel(self.centralwidget)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.gridLayout.addWidget(self.label_11, 0, 0, 1, 1)
        self.label_4 = QtGui.QLabel(self.centralwidget)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.gridLayout.addWidget(self.label_4, 2, 0, 1, 1)
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.gridLayout.addWidget(self.label_5, 4, 0, 1, 1)
        self.label_8 = QtGui.QLabel(self.centralwidget)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.gridLayout.addWidget(self.label_8, 8, 2, 1, 1)
        self.confusionTargetSlider = QtGui.QSlider(self.centralwidget)
        self.confusionTargetSlider.setOrientation(QtCore.Qt.Horizontal)
        self.confusionTargetSlider.setObjectName(_fromUtf8("confusionTargetSlider"))
        self.gridLayout.addWidget(self.confusionTargetSlider, 4, 1, 1, 1)
        self.label_6 = QtGui.QLabel(self.centralwidget)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.gridLayout.addWidget(self.label_6, 4, 2, 1, 1)
        self.label_9 = QtGui.QLabel(self.centralwidget)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout.addWidget(self.label_9, 7, 1, 1, 1)
        self.label_7 = QtGui.QLabel(self.centralwidget)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.gridLayout.addWidget(self.label_7, 8, 0, 1, 1)
        self.confusionVelocityLabel = QtGui.QLabel(self.centralwidget)
        self.confusionVelocityLabel.setText(_fromUtf8(""))
        self.confusionVelocityLabel.setObjectName(_fromUtf8("confusionVelocityLabel"))
        self.gridLayout.addWidget(self.confusionVelocityLabel, 2, 1, 1, 1)
        self.label_3 = QtGui.QLabel(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignHCenter)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.gridLayout.addWidget(self.label_3, 5, 1, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 504, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Face Animation", None))
        self.label_13.setText(_translate("MainWindow", "Current Target:", None))
        self.label_10.setText(_translate("MainWindow", "Confusion Target", None))
        self.label_2.setText(_translate("MainWindow", "More Confused", None))
        self.label.setText(_translate("MainWindow", "Less Confused", None))
        self.label_11.setText(_translate("MainWindow", "Current Value:", None))
        self.label_4.setText(_translate("MainWindow", "Current Velocity:", None))
        self.label_5.setText(_translate("MainWindow", "Less Confused", None))
        self.label_8.setText(_translate("MainWindow", "Faster", None))
        self.label_6.setText(_translate("MainWindow", "More Confused", None))
        self.label_9.setText(_translate("MainWindow", "Velocity (FPS)", None))
        self.label_7.setText(_translate("MainWindow", "Slower", None))
        self.label_3.setText(_translate("MainWindow", "Confusion Value", None))

