# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'from_error_dinhvi.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets
import Utilities as Uti

class Ui_Form_error_dinhvi(object):
    def setupUi(self, Form_error_dinhvi):
        Form_error_dinhvi.setObjectName("Form_error_dinhvi")
        Form_error_dinhvi.resize(550, 250)
        Form_error_dinhvi.setStyleSheet("")
        self.frames = QtWidgets.QFrame(Form_error_dinhvi)
        self.frames.setGeometry(QtCore.QRect(5, 5, 540, 240))
        self.frames.setStyleSheet("background-color: rgb(255,255,255);\n"
"border-radius: 25px;")
        self.frames.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frames.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frames.setObjectName("frames")
        self.label_themvao = QtWidgets.QLabel(self.frames)
        self.label_themvao.setGeometry(QtCore.QRect(10, 10, 501, 60))
        self.label_themvao.setMinimumSize(QtCore.QSize(500, 30))
        self.label_themvao.setMaximumSize(QtCore.QSize(5000, 60))
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(16)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label_themvao.setFont(font)
        self.label_themvao.setStyleSheet("font: 16pt \"MS Shell Dlg 2\"")
        self.label_themvao.setAlignment(QtCore.Qt.AlignCenter)
        self.label_themvao.setObjectName("label_themvao")
        self.pushButton = QtWidgets.QPushButton(self.frames)
        self.pushButton.setGeometry(QtCore.QRect(180, 80, 190, 100))
        self.pushButton.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(Uti.image_path("error.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton.setIcon(icon)
        self.pushButton.setIconSize(QtCore.QSize(80, 80))
        self.pushButton.setObjectName("pushButton")
        self.btn_error_gps = QtWidgets.QPushButton(self.frames)
        self.btn_error_gps.setGeometry(QtCore.QRect(420, 180, 110, 50))
        self.btn_error_gps.setMinimumSize(QtCore.QSize(100, 50))
        self.btn_error_gps.setMaximumSize(QtCore.QSize(150, 50))
        self.btn_error_gps.setStyleSheet("QPushButton#btn_error_gps{\n"
"     border-radius: 10px;\n"
"    font: 75 16pt \"MS Shell Dlg 2\";\n"
"    border: 1px solid rgb(230, 230, 230);\n"
"    background-color: rgb(230, 230, 230);\n"
"\n"
"}\n"
"\n"
"QPushButton#btn_error_gps:pressed{\n"
"padding-left :2px;\n"
"padding-top :2px;\n"
"\n"
"    background-color: rgb(58, 192, 240);\n"
"}\n"
"QPushButton#btn_error_gps:checked {\n"
"background-color: rgb(58, 192, 255);\n"
"\n"
"}\n"
"")
        self.btn_error_gps.setIconSize(QtCore.QSize(80, 80))
        self.btn_error_gps.setObjectName("btn_error_gps")

        self.retranslateUi(Form_error_dinhvi)
        QtCore.QMetaObject.connectSlotsByName(Form_error_dinhvi)

    def retranslateUi(self, Form_error_dinhvi):
        _translate = QtCore.QCoreApplication.translate
        Form_error_dinhvi.setWindowTitle(_translate("Form_error_dinhvi", "Form"))
        self.label_themvao.setText(_translate("Form_error_dinhvi", "Quá trình định vị bị lỗi, vui lòng thử lại."))
        self.btn_error_gps.setText(_translate("Form_error_dinhvi", "Xác nhận"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form_error_dinhvi = QtWidgets.QWidget()
    ui = Ui_Form_error_dinhvi()
    ui.setupUi(Form_error_dinhvi)
    Form_error_dinhvi.show()
    sys.exit(app.exec_())
