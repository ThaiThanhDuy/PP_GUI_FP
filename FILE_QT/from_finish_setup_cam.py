# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'from_finish_setup_cam.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets
import Utilities as Uti

class Ui_Form_finish_setup_cam(object):
    def setupUi(self, Form_finish_setup_cam):
        Form_finish_setup_cam.setObjectName("Form_finish_setup_cam")
        Form_finish_setup_cam.resize(550, 250)
        Form_finish_setup_cam.setStyleSheet("")
        self.frames = QtWidgets.QFrame(Form_finish_setup_cam)
        self.frames.setGeometry(QtCore.QRect(5, 5, 540, 240))
        self.frames.setStyleSheet("background-color: rgb(255,255,255);\n"
"border-radius: 25px;")
        self.frames.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frames.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frames.setObjectName("frames")
        self.label_themvao = QtWidgets.QLabel(self.frames)
        self.label_themvao.setGeometry(QtCore.QRect(5, 10, 531, 60))
        self.label_themvao.setMinimumSize(QtCore.QSize(500, 30))
        self.label_themvao.setMaximumSize(QtCore.QSize(5000, 60))
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(16)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label_themvao.setFont(font)
        self.label_themvao.setStyleSheet("font: 16pt \"MS Shell Dlg 2\";\n"
"border-radius: 20px;")
        self.label_themvao.setAlignment(QtCore.Qt.AlignCenter)
        self.label_themvao.setObjectName("label_themvao")
        self.pushButton = QtWidgets.QPushButton(self.frames)
        self.pushButton.setGeometry(QtCore.QRect(175, 80, 191, 101))
        self.pushButton.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(Uti.image_path("check.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton.setIcon(icon)
        self.pushButton.setIconSize(QtCore.QSize(80, 80))
        self.pushButton.setObjectName("pushButton")
        self.btn_finish_setup_cam = QtWidgets.QPushButton(self.frames)
        self.btn_finish_setup_cam.setGeometry(QtCore.QRect(420, 180, 110, 50))
        self.btn_finish_setup_cam.setMinimumSize(QtCore.QSize(100, 50))
        self.btn_finish_setup_cam.setMaximumSize(QtCore.QSize(150, 50))
        self.btn_finish_setup_cam.setStyleSheet("QPushButton#btn_finish_setup_cam{\n"
        "     border-radius: 10px;\n"
        "    font: 75 16pt \"MS Shell Dlg 2\";\n"
        "    border: 1px solid rgb(230, 230, 230);\n"
        "    background-color: rgb(230, 230, 230);\n"
        "\n"
        "}\n"
        "\n"
        "QPushButton#btn_finish_setup_cam:pressed{\n"
        "padding-left :2px;\n"
        "padding-top :2px;\n"
        "\n"
        "    background-color: rgb(58, 192, 240);\n"
        "}\n"
        "QPushButton#btn_finish_setup_cam:checked {\n"
        "background-color: rgb(58, 192, 255);\n"
        "\n"
        "}\n"
        "")
        self.btn_finish_setup_cam.setIconSize(QtCore.QSize(80, 80))
        self.btn_finish_setup_cam.setObjectName("btn_finish_setup_cam")

        self.retranslateUi(Form_finish_setup_cam)
        QtCore.QMetaObject.connectSlotsByName(Form_finish_setup_cam)

    def retranslateUi(self, Form_finish_setup_cam):
        _translate = QtCore.QCoreApplication.translate
        Form_finish_setup_cam.setWindowTitle(_translate("Form_finish_setup_cam", "Form"))
        self.label_themvao.setText(_translate("Form_finish_setup_cam", "Hoàn thành quá trình setup camera"))
        self.btn_finish_setup_cam.setText(_translate("Form_finish_setup_cam", "Xác nhận"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form_finish_setup_cam = QtWidgets.QWidget()
    ui = Ui_Form_finish_setup_cam()
    ui.setupUi(Form_finish_setup_cam)
    Form_finish_setup_cam.show()
    sys.exit(app.exec_())

