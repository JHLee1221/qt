# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'turtle.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.setWindowModality(QtCore.Qt.WindowModal)
        Form.resize(600, 400)
        self.horizontalLayoutWidget = QtWidgets.QWidget(Form)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(20, 300, 291, 81))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.btn_run = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.btn_run.setObjectName("btn_run")
        self.horizontalLayout.addWidget(self.btn_run)
        self.btn_exit = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.btn_exit.setObjectName("btn_exit")
        self.horizontalLayout.addWidget(self.btn_exit)
        self.gridLayoutWidget = QtWidgets.QWidget(Form)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(330, 300, 254, 89))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.btn_stop = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.btn_stop.setObjectName("btn_stop")
        self.gridLayout.addWidget(self.btn_stop, 1, 1, 1, 1)
        self.btn_left = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.btn_left.setObjectName("btn_left")
        self.gridLayout.addWidget(self.btn_left, 1, 0, 1, 1)
        self.btn_up = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.btn_up.setObjectName("btn_up")
        self.gridLayout.addWidget(self.btn_up, 0, 1, 1, 1)
        self.btn_right = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.btn_right.setObjectName("btn_right")
        self.gridLayout.addWidget(self.btn_right, 1, 2, 1, 1)
        self.btn_down = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.btn_down.setObjectName("btn_down")
        self.gridLayout.addWidget(self.btn_down, 2, 1, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.btn_run.setText(_translate("Form", "run"))
        self.btn_exit.setText(_translate("Form", "exit"))
        self.btn_stop.setText(_translate("Form", "stop"))
        self.btn_left.setText(_translate("Form", "left"))
        self.btn_up.setText(_translate("Form", "up"))
        self.btn_right.setText(_translate("Form", "right"))
        self.btn_down.setText(_translate("Form", "down"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

