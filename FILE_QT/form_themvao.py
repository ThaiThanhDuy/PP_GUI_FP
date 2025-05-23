# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'form_themvao.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form_them(object):
    def setupUi(self, Form_them):
        Form_them.setObjectName("Form_them")
        Form_them.resize(550, 450)
        Form_them.setStyleSheet("")
        self.frames = QtWidgets.QFrame(Form_them)
        self.frames.setGeometry(QtCore.QRect(0, 0, 550, 450))
        self.frames.setStyleSheet("background-color: rgb(230, 230, 230);\n"
"border-radius: 35px;")
        self.frames.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frames.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frames.setObjectName("frames")
        self.listWidget_themvao = QtWidgets.QListWidget(self.frames)
        self.listWidget_themvao.setGeometry(QtCore.QRect(70, 90, 400, 200))
        self.listWidget_themvao.setMinimumSize(QtCore.QSize(400, 200))
        self.listWidget_themvao.setMaximumSize(QtCore.QSize(400, 200))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.listWidget_themvao.setFont(font)
        self.listWidget_themvao.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"  border-radius: 14px;")
        self.listWidget_themvao.setObjectName("listWidget_themvao")
        self.Button_huythem = QtWidgets.QPushButton(self.frames)
        self.Button_huythem.setGeometry(QtCore.QRect(290, 320, 180, 100))
        self.Button_huythem.setMinimumSize(QtCore.QSize(180, 100))
        self.Button_huythem.setMaximumSize(QtCore.QSize(180, 100))
        self.Button_huythem.setStyleSheet("QPushButton#Button_huythem{\n"
"    border-radius: 20px;\n"
"    font: 75 14pt \"MS Shell Dlg 2\";\n"
"    border: 1px solid rgb(230, 230, 230);\n"
"\n"
"    background-color: rgb(255, 255, 255);\n"
"\n"
"}\n"
"QPushButton#Button_huythem:hover{\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(58, 192, 255, 219), stop:1 rgba(58, 192, 255, 226));\n"
"\n"
"}\n"
"QPushButton#Button_huythem:pressed{\n"
"padding-left :2px;\n"
"padding-top :2px;\n"
"\n"
"    background-color: rgb(58, 192, 240);\n"
"}\n"
"QPushButton#Button_huythem:checked {\n"
"background-color: rgb(58, 192, 255);\n"
"\n"
"}\n"
"")
        self.Button_huythem.setObjectName("Button_huythem")
        self.Button_them = QtWidgets.QPushButton(self.frames)
        self.Button_them.setGeometry(QtCore.QRect(70, 320, 180, 100))
        self.Button_them.setMinimumSize(QtCore.QSize(180, 100))
        self.Button_them.setMaximumSize(QtCore.QSize(180, 100))
        self.Button_them.setStyleSheet("QPushButton#Button_them{\n"
"    border-radius: 20px;\n"
"    font: 75 14pt \"MS Shell Dlg 2\";\n"
"    border: 1px solid rgb(230, 230, 230);\n"
"\n"
"    background-color: rgb(255, 255, 255);\n"
"\n"
"}\n"
"QPushButton#Button_them:hover{\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(58, 192, 255, 219), stop:1 rgba(58, 192, 255, 226));\n"
"\n"
"}\n"
"QPushButton#Button_them:pressed{\n"
"padding-left :2px;\n"
"padding-top :2px;\n"
"\n"
"    background-color: rgb(58, 192, 240);\n"
"}\n"
"QPushButton#Button_them:checked {\n"
"background-color: rgb(58, 192, 255);\n"
"\n"
"}\n"
"")
        self.Button_them.setObjectName("Button_them")
        self.label_themvao = QtWidgets.QLabel(self.frames)
        self.label_themvao.setGeometry(QtCore.QRect(30, 40, 500, 30))
        self.label_themvao.setMinimumSize(QtCore.QSize(500, 30))
        self.label_themvao.setMaximumSize(QtCore.QSize(5000, 30))
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(14)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label_themvao.setFont(font)
        self.label_themvao.setStyleSheet("font: 14pt \"MS Shell Dlg 2\";")
        self.label_themvao.setAlignment(QtCore.Qt.AlignCenter)
        self.label_themvao.setObjectName("label_themvao")

        self.retranslateUi(Form_them)
        QtCore.QMetaObject.connectSlotsByName(Form_them)

    def retranslateUi(self, Form_them):
        _translate = QtCore.QCoreApplication.translate
        Form_them.setWindowTitle(_translate("Form_them", "Form"))
        self.Button_huythem.setText(_translate("Form_them", "HỦY"))
        self.Button_them.setText(_translate("Form_them", "THÊM"))
        self.label_themvao.setText(_translate("Form_them", "DANH SÁCH CÁC PHÒNG ĐÃ XÓA TRƯỚC ĐÓ"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form_them = QtWidgets.QWidget()
    ui = Ui_Form_them()
    ui.setupUi(Form_them)
    Form_them.show()
    sys.exit(app.exec_())
