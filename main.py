#Library python

import sys

# File Python 

# Library GUI
from PyQt5.QtWidgets import QApplication,QMainWindow
from FILE_QT.QT_main import Ui_MainWindow 
# build QT - pyuic5 QT_main.ui -o QT_main.py

class MainApp(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self) 
        self.setupSingal()
    
    # Button event 
    def setupSingal(self):
        self.bt_thoat.clicked.connect(self.close) 
        self.bt_thu_phong.clicked.connect(self.toggleFullScreen)
        self.bt_thu_nho.clicked.connect(self.showMinimized)

        ## Back button
        self.bt_back_setup.clicked.connect(self.back_setup)
        self.bt_back_dichuyen_robot.clicked.connect(self.back_setup)
        self.bt_back_setup_ts_robot.clicked.connect(self.back_setup)
        self.bt_back_dan_duong.clicked.connect(self.back_setup)

        ## diem danh button
        self.bt_diem_danh.clicked.connect(self.show_diem_danh)
        self.bt_back_diem_danh.clicked.connect(self.back_diem_danh)

        #Setup button
        self.bt_setup.clicked.connect(self.show_page_setup)
        self.bt_setup_robot.clicked.connect(self.show_setup_ts_robot)
        self.bt_di_chuyen.clicked.connect(self.show_dichuyen_robot)


        #Ros button
        self.bt_dan_duong.clicked.connect(self.show_dan_duong)
        self.btn_batdau_danduong.clicked.connect(self.show_robot_dichuyen)

    # Funtion Page
    
    
    # Page Setup

    def back_setup(self):
        self.stackedWidget.setCurrentWidget(self.page_main)

    def show_setup_ts_robot(self):
        self.stackedWidget.setCurrentWidget(self.page_setup_ts_robot)
    def show_dichuyen_robot(self):
        self.stackedWidget.setCurrentWidget(self.page_dichuyen_robot)
    #Page ros
    def show_dan_duong(self):
        self.stackedWidget.setCurrentWidget(self.page_dan_duong)
    def show_page_setup(self):
        self.stackedWidget.setCurrentWidget(self.page_setup)
    def show_robot_dichuyen(self):
        self.stackedWidget.setCurrentWidget(self.page_robot_dichuyen)
    # Page diem danh
    def show_diem_danh(self):
        self.stackedWidget.setCurrentWidget(self.page_diem_danh)
    def back_diem_danh(self):
        self.stackedWidget.setCurrentWidget(self.page_main)

    # Funtion event 
    def toggleFullScreen(self):
        """ Bật hoặc tắt chế độ toàn màn hình """
        if self.isFullScreen():
            self.showNormal()  # Thoát toàn màn hình
        else:
            self.showFullScreen()  # Bật toàn màn hình

    #Ros2

    
    
    
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())