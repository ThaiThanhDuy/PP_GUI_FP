# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'from_nhaptenban.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form_nhaptenban(object):
    def setupUi(self, Form_nhaptenban):
        Form_nhaptenban.setObjectName("Form_nhaptenban")
        Form_nhaptenban.resize(750, 550)
        Form_nhaptenban.setStyleSheet("")
        self.frame = QtWidgets.QFrame(Form_nhaptenban)
        self.frame.setGeometry(QtCore.QRect(0, 0, 750, 550))
        self.frame.setMinimumSize(QtCore.QSize(750, 550))
        self.frame.setMaximumSize(QtCore.QSize(750, 550))
        self.frame.setStyleSheet("background-color: rgb(230, 230, 230);\n"
"border-radius: 35px;\n"
"box-shadow: 5px 5px 10px rgba(0, 0, 0, 0.5);")
        self.frame.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.lineEdit_nhaptenban = QtWidgets.QLineEdit(self.frame)
        self.lineEdit_nhaptenban.setGeometry(QtCore.QRect(130, 260, 500, 100))
        self.lineEdit_nhaptenban.setMinimumSize(QtCore.QSize(500, 100))
        self.lineEdit_nhaptenban.setMaximumSize(QtCore.QSize(500, 100))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.lineEdit_nhaptenban.setFont(font)
        self.lineEdit_nhaptenban.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border-radius:20px;")
        self.lineEdit_nhaptenban.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit_nhaptenban.setObjectName("lineEdit_nhaptenban")
        self.Button_xacnhan_capnhat = QtWidgets.QPushButton(self.frame)
        self.Button_xacnhan_capnhat.setGeometry(QtCore.QRect(450, 400, 241, 111))
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(18)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(9)
        self.Button_xacnhan_capnhat.setFont(font)
        self.Button_xacnhan_capnhat.setStyleSheet("QPushButton#Button_xacnhan_capnhat\n"
"{\n"
"    border-radius: 20px;\n"
"    font: 75 18pt \"MS Shell Dlg 2\";\n"
"    border: 1px solid rgb(230, 230, 230);\n"
"    background-color: rgb(250, 250, 250);\n"
"/*border: 3px solid rgb(0, 0, 255);*/\n"
"\n"
"}\n"
"QPushButton#Button_xacnhan_capnhat:hover{\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(58, 192, 255, 219), stop:1 rgba(58, 192, 255, 226));\n"
"\n"
"}\n"
"QPushButton#Button_xacnhan_capnhat:pressed{\n"
"padding-left :2px;\n"
"padding-top :2px;\n"
"\n"
"    background-color: rgb(58, 192, 240);\n"
"}\n"
"QPushButton#Button_xacnhan_capnhat:checked {\n"
"background-color: rgb(58, 192, 255);\n"
"\n"
"}\n"
"")
        self.Button_xacnhan_capnhat.setObjectName("Button_xacnhan_capnhat")
        self.Button_docksac = QtWidgets.QPushButton(self.frame)
        self.Button_docksac.setGeometry(QtCore.QRect(80, 130, 100, 81))
        self.Button_docksac.setMinimumSize(QtCore.QSize(100, 60))
        self.Button_docksac.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.Button_docksac.setFont(font)
        self.Button_docksac.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.Button_docksac.setStyleSheet("QPushButton#Button_docksac\n"
"{\n"
"    border-radius: 20px;\n"
"    border: 1px solid rgb(230, 230, 230);\n"
"    background-color: rgb(250, 250, 250);\n"
"/*border: 3px solid rgb(0, 0, 255);*/\n"
"\n"
"\n"
"\n"
"}\n"
"QPushButton#Button_docksac:hover{\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(58, 192, 255, 219), stop:1 rgba(58, 192, 255, 226));\n"
"\n"
"}\n"
"QPushButton#Button_docksac:pressed{\n"
"padding-left :2px;\n"
"padding-top :2px;\n"
"\n"
"    background-color: rgb(58, 192, 240);\n"
"}\n"
"QPushButton#Button_docksac:checked {\n"
"background-color: rgb(58, 192, 255);\n"
"\n"
"}\n"
"")
        self.Button_docksac.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("wireless-charging.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.Button_docksac.setIcon(icon)
        self.Button_docksac.setIconSize(QtCore.QSize(50, 40))
        self.Button_docksac.setObjectName("Button_docksac")
        self.Button_kvchow = QtWidgets.QPushButton(self.frame)
        self.Button_kvchow.setGeometry(QtCore.QRect(250, 130, 100, 81))
        self.Button_kvchow.setMinimumSize(QtCore.QSize(100, 60))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.Button_kvchow.setFont(font)
        self.Button_kvchow.setStyleSheet("QPushButton#Button_kvchow\n"
"{\n"
"    border-radius: 20px;\n"
"    \n"
"    border: 1px solid rgb(230, 230, 230);\n"
"    background-color: rgb(250, 250, 250);\n"
"/*border: 3px solid rgb(0, 0, 255);*/\n"
"\n"
"}\n"
"QPushButton#Button_kvchow:hover{\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(58, 192, 255, 219), stop:1 rgba(58, 192, 255, 226));\n"
"\n"
"}\n"
"QPushButton#Button_kvchow:pressed{\n"
"padding-left :2px;\n"
"padding-top :2px;\n"
"\n"
"    background-color: rgb(58, 192, 240);\n"
"}\n"
"QPushButton#Button_kvchow:checked {\n"
"background-color: rgb(58, 192, 255);\n"
"\n"
"}\n"
"")
        self.Button_kvchow.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("../images/clock1.jpg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.Button_kvchow.setIcon(icon1)
        self.Button_kvchow.setIconSize(QtCore.QSize(50, 50))
        self.Button_kvchow.setObjectName("Button_kvchow")
        self.Button_ban = QtWidgets.QPushButton(self.frame)
        self.Button_ban.setGeometry(QtCore.QRect(420, 130, 100, 81))
        self.Button_ban.setMinimumSize(QtCore.QSize(100, 60))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.Button_ban.setFont(font)
        self.Button_ban.setStyleSheet("QPushButton#Button_ban\n"
"{\n"
"    border-radius: 20px;\n"
"    \n"
"    border: 1px solid rgb(230, 230, 230);\n"
"    background-color: rgb(250, 250, 250);\n"
"/*border: 3px solid rgb(0, 0, 255);*/\n"
"\n"
"}\n"
"QPushButton#Button_ban:hover{\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(58, 192, 255, 219), stop:1 rgba(58, 192, 255, 226));\n"
"\n"
"}\n"
"QPushButton#Button_ban:pressed{\n"
"padding-left :2px;\n"
"padding-top :2px;\n"
"\n"
"    background-color: rgb(58, 192, 240);\n"
"}\n"
"QPushButton#Button_ban:checked {\n"
"background-color: rgb(58, 192, 255);\n"
"\n"
"}\n"
"")
        self.Button_ban.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("door.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.Button_ban.setIcon(icon2)
        self.Button_ban.setIconSize(QtCore.QSize(50, 50))
        self.Button_ban.setObjectName("Button_ban")
        self.Button_khacc = QtWidgets.QPushButton(self.frame)
        self.Button_khacc.setGeometry(QtCore.QRect(590, 130, 100, 81))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.Button_khacc.setFont(font)
        self.Button_khacc.setStyleSheet("QPushButton#Button_khacc\n"
"{\n"
"    border-radius: 20px;\n"
"    \n"
"    border: 1px solid rgb(230, 230, 230);\n"
"    background-color: rgb(250, 250, 250);\n"
"/*border: 3px solid rgb(0, 0, 255);*/\n"
"\n"
"}\n"
"QPushButton#Button_khacc:hover{\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(58, 192, 255, 219), stop:1 rgba(58, 192, 255, 226));\n"
"\n"
"}\n"
"QPushButton#Button_khacc:pressed{\n"
"padding-left :2px;\n"
"padding-top :2px;\n"
"\n"
"    background-color: rgb(58, 192, 240);\n"
"}\n"
"QPushButton#Button_khacc:checked {\n"
"background-color: rgb(58, 192, 255);\n"
"\n"
"}\n"
"")
        self.Button_khacc.setText("")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("kvkhac.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.Button_khacc.setIcon(icon3)
        self.Button_khacc.setIconSize(QtCore.QSize(50, 50))
        self.Button_khacc.setObjectName("Button_khacc")
        self.Button_huy_capnhat = QtWidgets.QPushButton(self.frame)
        self.Button_huy_capnhat.setGeometry(QtCore.QRect(70, 400, 241, 111))
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(18)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(9)
        self.Button_huy_capnhat.setFont(font)
        self.Button_huy_capnhat.setStyleSheet("QPushButton#Button_huy_capnhat\n"
"{\n"
"    border-radius: 20px;\n"
"    font: 75 18pt \"MS Shell Dlg 2\";\n"
"    border: 1px solid rgb(230, 230, 230);\n"
"    background-color: rgb(250, 250, 250);\n"
"/*border: 3px solid rgb(0, 0, 255);*/\n"
"\n"
"}\n"
"QPushButton#Button_huy_capnhat:hover{\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(58, 192, 255, 219), stop:1 rgba(58, 192, 255, 226));\n"
"\n"
"}\n"
"QPushButton#Button_huy_capnhat:pressed{\n"
"padding-left :2px;\n"
"padding-top :2px;\n"
"\n"
"    background-color: rgb(58, 192, 240);\n"
"}\n"
"QPushButton#Button_huy_capnhat:checked {\n"
"background-color: rgb(58, 192, 255);\n"
"\n"
"}\n"
"")
        self.Button_huy_capnhat.setObjectName("Button_huy_capnhat")
        self.label = QtWidgets.QLabel(self.frame)
        self.label.setGeometry(QtCore.QRect(130, 40, 501, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.label.setFont(font)
        self.label.setStyleSheet("background-color: rgb(255, 255, 255);\n"
" border-radius: 20px;")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")

        self.retranslateUi(Form_nhaptenban)
        QtCore.QMetaObject.connectSlotsByName(Form_nhaptenban)

    def retranslateUi(self, Form_nhaptenban):
        _translate = QtCore.QCoreApplication.translate
        Form_nhaptenban.setWindowTitle(_translate("Form_nhaptenban", "Form"))
        self.lineEdit_nhaptenban.setPlaceholderText(_translate("Form_nhaptenban", "   Nhập số phòng 1,2,3..."))
        self.Button_xacnhan_capnhat.setText(_translate("Form_nhaptenban", "CẬP NHẬT"))
        self.Button_huy_capnhat.setText(_translate("Form_nhaptenban", "HỦY"))
        self.label.setText(_translate("Form_nhaptenban", "Chọn các chế độ trước khi cập nhật"))
