#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4 import QtCore, QtGui
import rospy
from std_msgs.msg import Empty
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged

bateria = 0

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


class Ui_Form(object):
    def setupUi(self, Form):
        global takeoff_pub, land_pub
        global bateria
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(400, 300)
        self.BTN_TakeOff = QtGui.QPushButton(Form)
        self.BTN_TakeOff.setGeometry(QtCore.QRect(130, 160, 85, 27))
        self.BTN_TakeOff.setObjectName(_fromUtf8("BTN_TakeOff"))
        self.BTN_Land = QtGui.QPushButton(Form)
        self.BTN_Land.setGeometry(QtCore.QRect(10, 160, 85, 27))
        self.BTN_Land.setObjectName(_fromUtf8("BTN_Land"))
        self.PBAR_Battery = QtGui.QProgressBar(Form)
        self.PBAR_Battery.setGeometry(QtCore.QRect(10, 30, 118, 23))
        self.PBAR_Battery.setProperty("value", bateria)
        self.PBAR_Battery.setObjectName(_fromUtf8("PBAR_Battery"))

        self.retranslateUi(Form)
        QtCore.QObject.connect(self.BTN_Land, QtCore.SIGNAL(_fromUtf8("clicked()")), self.clickland)
        QtCore.QObject.connect(self.BTN_TakeOff, QtCore.SIGNAL(_fromUtf8("clicked()")), self.clicktakeoff)
        QtCore.QMetaObject.connectSlotsByName(Form)

        takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)

        rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged, self.receive_beteria)
        rospy.init_node('GUI', anonymous=True)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "dotMEX Drones", None))
        self.BTN_TakeOff.setText(_translate("Form", "Take Off", None))
        self.BTN_Land.setText(_translate("Form", "Land", None))
    
    def clickland(self):
        print "Land"
        land_pub.publish(Empty())
    
    def clicktakeoff(self):
        print "Take Off"
        takeoff_pub.publish(Empty())

    def receive_beteria(self, data):
        bateria = data.percent



if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    print str(bateria)
    Form.show()
    sys.exit(app.exec_())

