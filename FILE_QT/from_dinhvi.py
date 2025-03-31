# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'from_dinhvi.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets
import Utilities as Uti

class Ui_Form_dang_dinhvi(object):
    def setupUi(self, Form_dang_dinhvi):
        Form_dang_dinhvi.setObjectName("Form_dang_dinhvi")
        Form_dang_dinhvi.resize(550, 250)
        Form_dang_dinhvi.setStyleSheet("")
        self.frames = QtWidgets.QFrame(Form_dang_dinhvi)
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
        self.pushButton.setGeometry(QtCore.QRect(180, 80, 191, 101))
        self.pushButton.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(Uti.image_path("warning_2.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton.setIcon(icon)
        self.pushButton.setIconSize(QtCore.QSize(80, 80))
        self.pushButton.setObjectName("pushButton")

        self.retranslateUi(Form_dang_dinhvi)
        QtCore.QMetaObject.connectSlotsByName(Form_dang_dinhvi)

    def retranslateUi(self, Form_dang_dinhvi):
        _translate = QtCore.QCoreApplication.translate
        Form_dang_dinhvi.setWindowTitle(_translate("Form_dang_dinhvi", "Form"))
        self.label_themvao.setText(_translate("Form_dang_dinhvi", "Đang trong quá trình định vị.\n"
" Vui lòng giữ khoảng cách an toàn."))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form_dang_dinhvi = QtWidgets.QWidget()
    ui = Ui_Form_dang_dinhvi()
    ui.setupUi(Form_dang_dinhvi)
    Form_dang_dinhvi.show()
    sys.exit(app.exec_())
