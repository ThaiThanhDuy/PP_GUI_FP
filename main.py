#!/home/robot/robot_lib/bin/python3
#Library python
import os
import traceback
import tensorflow as tf


import numpy as np
import serial
import cv2
import collections
import threading
import time
import sys
import keyyyyy
import threading
import yaml
import numpy as np
import math
from PIL import Image
import keyboard
# import rospy
import os
import pygame
import subprocess
from gtts import gTTS
from datetime import datetime
import cv2
from pydub import AudioSegment
import unicodedata
import unidecode

from datetime import datetime, timedelta
from reportlab.lib.pagesizes import letter
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle
from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from openpyxl.styles import Alignment, Border, Side
from openpyxl.utils import get_column_letter
from openpyxl.styles import Alignment
from fpdf import FPDF
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, Paragraph
from reportlab.lib.styles import getSampleStyleSheet
import re
import mysql.connector
# File Python 
import Utilities as Uti
import RobotConfig as RobConf

#UI
import keyyyyy
from FILE_QT.QT_main import Ui_MainWindow 
from status_popup import StatusPopup, NamedMap
from FILE_QT.from_nhapphong import Ui_Form_nhaptenban
from FILE_QT.form_themvao import Ui_Form_them
from FILE_QT.form_map import Ui_Form_map

from FILE_QT.from_finish_setup_cam import Ui_Form_finish_setup_cam

from FILE_QT.from_dang_nd_cam import Ui_Form_dangnhap_cam
from FILE_QT.from_error_cam import Ui_Form_error_cam
from FILE_QT.from_finish_cam import Ui_Form_finish_cam

from FILE_QT.from_setup_cam import Ui_Form_dang_setup_cam
from FILE_QT.from_error_setup_cam import Ui_Form_error_setup_cam
from FILE_QT.from_finish_setup_cam import Ui_Form_finish_setup_cam

from FILE_QT.from_dinhvi import Ui_Form_dang_dinhvi
from FILE_QT.from_error_dinhvi import Ui_Form_error_dinhvi
from FILE_QT.from_finish_dinhvi import Ui_Form_finish_dinhvi
# FILE CUSTOM
from navigation_thread import NavigationThread
from arduino_connection import ReadArduinoPin, arduino
from ros2_handle import ROS2Handle
import ros2_handle as ros2_handle
import SQL as sql
from ModuleSetupDinhVi import SetupDinhVi
# Library GUI
from PyQt5 import QtCore, QtGui

from PyQt5.QtCore import *
from PyQt5.QtCore import QEvent, Qt, QThread, pyqtSignal

from PyQt5.QtGui import *
from PyQt5.QtGui import QIcon

from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication,QMainWindow,QTableWidgetItem, QCheckBox, QMessageBox,QLineEdit, QVBoxLayout,QWidget

from PyQt5.QtGui import *

# build QT - pyuic5 QT_main.ui -o QT_main.py
# fix duong dan - ../ROBOT_HD/
def xuly_cham_ngoai(): # ham xu ly khi cham ra ngoai ban phim se tat
    try:
        if not hasattr(keyyyyy.Registe, "virtual_keyboard") or not keyyyyy.Registe.virtual_keyboard.isVisible():
            keyyyyy.Registe.virtual_keyboard.close()
        else:
            keyyyyy.Registe.virtual_keyboard.close()
    except:
        print("failed to open virtual keyboard ,already exist ")
class MainApp(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.navigation_thread = NavigationThread()
        self.setupUi(self) 
        self.ros2_handle = ROS2Handle()
        self.status_popup = StatusPopup()
        self.initialize_ros2_manager()
        
        
        self.css_ham = """
                               border-radius: 20px;
                               background-color: rgba(58, 192, 255, 255);
                               border: 3px solid rgba(58, 192, 255, 255);
                               font: 20pt "MS Shell Dlg 2";
                                               """
        self.css_unham = """
                               border-radius: 20px;
        	                   border: 1px solid rgb(230, 230, 230);
        	                   background-color: rgb(250, 250, 250);
                                               """
        self.username_global = RobConf.USERNAME
        self.password_global = RobConf.PASSWORD #caidatrobot
        self.VX_MANUAL_MAX = RobConf.VX_MANUAL_MAX
        self.VW_MANUAL_MAX = RobConf.VW_MANUAL_MAX
        self.VX_AUTO_MAX = RobConf.VX_AUTO_MAX
        self.VW_AUTO_MAX = RobConf.VW_AUTO_MAX
        self.CHIEU_CAO_LED = RobConf.chieu_cao_led
        self.DUNG_SAI_DIEM_DEN_VITRI = RobConf.DUNG_SAI_DIEM_DEN_VITRI
        self.DUNG_SAI_DIEM_DEN_GOC = RobConf.DUNG_SAI_DIEM_DEN_GOC
        self.MUC_DIEN_AP_SAC_PIN = RobConf.MUC_DIEN_AP_SAC
        self.port_headcamera = RobConf.headCamera
        self.port_backcamera = RobConf.chargCame
        self.port_front_R = RobConf.faceCame1
        self.port_front_L = RobConf.faceCam2
        self.cap_qr = None
        self.capC = None
        
        self.selected_item_infor = None
        self.selected_tram_them = None
        self.selected_kvcho_them = None
        self.selected_kvcho = None
        self.selected_tram = None
        self.selected_them = None
        self.trangthai_kv = None
        self.setupSingal()
        self.soban = {
            "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"}
        self.control_robot()
        self.control_manual()
        self.control_auto()
        self.x = ''
        self.y = ''
        self.z = ''
        self.w = ''
        self.id_voice = 0
        self.hien_thi_nut_theo_id()
        self.item_states = {i: False for i in range(1, 201)}
        # Navigate thread 
        
        self.dinhvi_dauhanhtrinh = False
        self.goal_ids = []
        self.btn_xac_nhan.setEnabled(False)

        # PIN
        self.on_off_sac_tu_dong=False
        self.read_arduino = ReadArduinoPin()
        self.read_arduino.doc_pin.connect(self.trangthai_pin)
     
        self.sac_pin_tu_dong =False
        self.arduino = arduino()
        
        # Kiemtra robot co dang du chuyen ko 
        if(self.navigation_thread.robot_dang_di_chuyen==True):
            self.bt_mode_auto.setEnabled(False)
        else:
            self.bt_mode_auto.setEnabled(True)

        self.btn_batdau_danduong.setEnabled(False)
        self.btn_xac_nhan.pressed.connect(self.button_xn_pressed)
        self.btn_xn_enable = self.btn_xac_nhan.isEnabled()

        # DINH VI
    
        values = [200,100, 123, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.id_vitri.addItems([str(i) for i in values])
        self.setup_dv  = SetupDinhVi()
        self.DINH_VI_BTN()

    def xu_ly_nhap_banphim(self, event):
        print("da nhap")
        try:
            if not hasattr(keyyyyy.Registe, "virtual_keyboard") or not keyyyyy.Registe.virtual_keyboard.isVisible():
                keyyyyy.Registe.virtual_keyboard = keyyyyy.VKBD(self.uic3.lineEdit_nhaptenban)
                keyyyyy.Registe.virtual_keyboard.show()
        
            else:
                #keyyyyy.Registe.virtual_keyboard.activateWindow()
                keyyyyy.Registe.virtual_keyboard.hide()
        except:
            print("failed to open virtual keyboard ,already exist ")
################### Button event ##################
    def setupSingal(self):
    
        
        self.bt_thoat.clicked.connect(self.close) 
        self.bt_thu_phong.clicked.connect(self.toggleFullScreen)
        self.bt_thu_nho.clicked.connect(self.showMinimized)
        self.bt_thoat.clicked.connect(self.ros2_handle.stop) 
        ## Back button
        self.bt_back_setup.clicked.connect(self.back_setup)
        self.bt_back_dichuyen_robot.clicked.connect(self.back_setup)
        self.bt_back_setup_ts_robot.clicked.connect(self.back_setup)
        self.bt_back_dan_duong.clicked.connect(self.back_setup)
        self.bt_back_setup_3.clicked.connect(self.back_setup)
        self.bt_back_setup_2.clicked.connect(self.back_setup)
        ## diem danh button
        
       

        #Setup button
        self.bt_setup.clicked.connect(self.show_page_setup)
        self.bt_setup_robot.clicked.connect(self.show_setup_ts_robot)
        self.bt_di_chuyen.clicked.connect(self.show_dichuyen_robot)
        self.btn_save_sql.clicked.connect(self.fcn_save_data_sql)
        self.btn_load_sql.clicked.connect(self.show_thong_so_robot)
        self.btn_resetall_sql.clicked.connect(self.fcn_resetall_sql)
        self.btn_sac_auto.clicked.connect(self.che_do_sac)
        
        self.bt_setup_dinh_vi.clicked.connect(self.show_page_setup_dinhvi)
        #Camera
        self.bt_setup_cam.clicked.connect(self.show_page_setup_cam)
        self.bt_check_cam.clicked.connect(self.dang_nd_cam)
        self.btn_start_setupcamera.clicked.connect(self.fcn_start_setup_camera)
        self.btn_save_id_camera.clicked.connect(self.fcn_save_id_camera)
        port_hC = [RobConf.headCamera, 0,1, 2, 3, 4, 5, 6]
        port_bC = [RobConf.chargCame, 0, 1, 2, 3, 4, 5, 6]
        port_flC = [RobConf.faceCam2, 0, 1, 2, 3, 4, 5, 6]
        port_frC = [RobConf.faceCame1, 0, 1, 2, 3, 4, 5, 6]
        self.combobox_headcame.addItems([str(i) for i in port_hC])
        self.combobox_back.addItems([str(i) for i in port_bC])
        self.combobox_front_1.addItems([str(i) for i in port_flC])
        self.combobox_front_2.addItems([str(i) for i in port_frC])
        #Ros button
        self.bt_back_mode.clicked.connect(self.back_setup)
        self.bt_dan_duong.clicked.connect(self.show_page_mode)
        self.bt_mode_manual.clicked.connect(self.show_dichuyen_robot)
        self.bt_mode_auto.clicked.connect(self.show_dan_duong)
        #ROBOT RUN AUTO
        self.btn_batdau_danduong.clicked.connect(self.show_robot_dichuyen)
        self.btn_xac_nhan.clicked.connect(self.man_hinh_dan_duong)
        #ROBOT RUN MANUAL
        self.listWidget_dsban.itemClicked.connect(self.item_xoa_clicked)
       

        #Slice 

        self.Slider_vantoc.setMaximum(1000)
        self.Slider_vantoc.setMinimum(0)
        self.Slider_vantoc.setValue(0)
        self.Slider_vantoc.setSingleStep(1) 
        self.Slider_vantoc.valueChanged.connect(self.thaydoivantoc)
        self.label_tocdo.setText(str(0)+"%")

    def control_manual(self):
        self.map_scrollArea.ensureVisible((2048-660) // 2, (2048-660) // 2)
        self.mapping_button.clicked.connect(self.mapping)
        self.stop_mapping_button.clicked.connect(self.stop_mapping) 
        self.save_map_button.clicked.connect(self.save_map)
        self.load_map_button.clicked.connect(self.load_map)

        self.Button_capnhat.clicked.connect(self.manhinh_nhap_soban)
        self.Button_lammoi.clicked.connect(self.xoatoanbo_mysql)
        self.Button_add.clicked.connect(self.manhinh_them)
        self.Button_xoa.clicked.connect(self.xoa_tungthanhphan)
        self.Button_toado.clicked.connect(self.save_map)

    def control_auto(self):
        self.btn_phong1.clicked.connect(lambda: self.toggle_item(1))
        self.btn_phong2.clicked.connect(lambda: self.toggle_item(2))
        self.btn_phong3.clicked.connect(lambda: self.toggle_item(3))
        self.btn_phong4.clicked.connect(lambda: self.toggle_item(4))
        self.btn_phong5.clicked.connect(lambda: self.toggle_item(5))
        self.btn_phong6.clicked.connect(lambda: self.toggle_item(6))
        self.btn_phong7.clicked.connect(lambda: self.toggle_item(7))
        self.btn_phong8.clicked.connect(lambda: self.toggle_item(8))
        self.btn_phong9.clicked.connect(lambda: self.toggle_item(9))
        self.btn_phong10.clicked.connect(lambda: self.toggle_item(10))

        self.btn_tram_sac.clicked.connect(lambda: self.toggle_item(100))
        self.btn_khu_vuc_cho.clicked.connect(lambda: self.toggle_item(200))
        self.checkBox_kvc.stateChanged.connect(lambda: self.toggle_item(200))

        if (sql.kiem_tra_id_ton_tai(200)==True):
            self.checkBox_kvc.setEnabled(True)
        else:
             self.checkBox_kvc.setEnabled(False)

    
        self.btn_ban_do.clicked.connect(self.show_map)
        self.btn_batdau_danduong.clicked.connect(self.dan_duong)
    def DINH_VI_BTN(self):
        self.btn_enable.clicked.connect(self.fcn_enable_type)
        self.btn_toado_td.clicked.connect(self.fcn_nhap_toado_td)
        self.btn_save_data.clicked.connect(self.fcn_save_vitri)
        self.btn_load_data.clicked.connect(self.fcn_load_data)
        self.btn_delete_data.clicked.connect(self.fcn_delete_data) 
        self.btn_test.clicked.connect(self.fcn_test_data)
        self.btn_dinh_vi.clicked.connect(self.dinh_vi_fcn_1)
    # Funtion Page
################### Page Setup ##################

    def back_setup(self):
        self.stackedWidget.setCurrentWidget(self.page_main)

    def show_setup_ts_robot(self):
        self.stackedWidget.setCurrentWidget(self.page_setup_ts_robot)
    def show_dichuyen_robot(self):
        self.stackedWidget.setCurrentWidget(self.page_dichuyen_robot)
    def show_page_setup_cam(self):
        self.stackedWidget.setCurrentWidget(self.page_setup_cam)
    def show_page_setup_dinhvi(self):
        self.stackedWidget.setCurrentWidget(self.page_setup_dinhvi)
    #Page ros
    def show_page_mode(self):
        self.stackedWidget.setCurrentWidget(self.page_mode)
    def show_dan_duong(self):
        self.stackedWidget.setCurrentWidget(self.page_dan_duong)
        
    def show_page_setup(self):
        self.stackedWidget.setCurrentWidget(self.page_setup)
        self.show_thong_so_robot()
    def show_robot_dichuyen(self):
        self.stackedWidget.setCurrentWidget(self.page_robot_dichuyen)
    def dan_duong(self):
        self.stackedWidget.setCurrentWidget(self.page_robot_dichuyen)
        self.man_hinh_dan_duong()
    # Funtion event 
    def toggleFullScreen(self):
        """ Bật hoặc tắt chế độ toàn màn hình """
        if self.isFullScreen():
            self.showNormal()  # Thoát toàn màn hình
        else:
            self.showFullScreen()  # Bật toàn màn hình
    def display_thanhcong(self, text):
        msg = QMessageBox(self)
        msg.setIconPixmap(QIcon(Uti.image_path("check.png")).pixmap(40, 40))  # Đặt biểu tượng tùy chỉnh
        msg.setWindowTitle("Successful")  # Đặt tiêu đề cho cửa sổ
        msg.setText(text)
        msg.resize(650, 450)
        msg.setStyleSheet("""
        QMessageBox {
           
            border-radius: 5px; /* Optional: Add rounded corners */
        }
        QMessageBox QLabel {
            qproperty-alignment: AlignCenter;
        }
         """)
        msg.exec_()
    def display_thatbai(self, text):
        msg = QMessageBox(self)
        msg.setIconPixmap(QIcon(Uti.image_path("error.png")).pixmap(40, 40))  # Đặt biểu tượng tùy chỉnh
        msg.setWindowTitle("Error")  # Đặt tiêu đề cho cửa sổ
        msg.setText(text)
        msg.setStyleSheet("""
        QMessageBox {
           
            border-radius: 5px; /* Optional: Add rounded corners */
        }
        QMessageBox QLabel {
            qproperty-alignment: AlignCenter;
        }
         """)
        msg.resize(650, 450)
        msg.exec_()
################## Setup ##################
################## CAI DAT THONG SO ROBOT ##################
    def show_thong_so_robot(self):
       
        self.textbox_vtt.setPlainText(str(RobConf.VX_AUTO_MAX))
        self.textbox_vtt_2.setPlainText(str(RobConf.VX_MANUAL_MAX))
        self.textbox_vtx.setPlainText(str(RobConf.VW_AUTO_MAX))
        self.textbox_vtx_2.setPlainText(str(RobConf.VW_MANUAL_MAX))
        self.textbox_sskc.setPlainText(str(RobConf.DUNG_SAI_DIEM_DEN_VITRI))
        self.textbox_ssgx.setPlainText(str(RobConf.DUNG_SAI_DIEM_DEN_GOC))
        self.textbox_ccdl.setPlainText(str(RobConf.chieu_cao_led))
        self.textbox_pin_sac_auto.setPlainText(str(RobConf.MUC_DIEN_AP_SAC))
        self.textbox_delay_time.setPlainText(str(RobConf.DELAY_TIME))

    def validate_input(self, value, min_value, max_value, error_message):
        if value is None:  # Kiểm tra xem giá trị có phải là None không
            raise ValueError(f"{error_message} not empty")
        if not isinstance(value, (int, float)):  # Kiểm tra xem có phải số hay không
            raise ValueError(f"{error_message} is a number")
        if value < min_value or value > max_value:
            raise ValueError(f"{error_message} (giá trị hợp lệ: {min_value} đến {max_value})")

    def fcn_save_data_sql(self):
        try:
        # Kiểm tra từng giá trị từ textbox
            vxt_auto_max_text = self.textbox_vtt.toPlainText()
            self.VX_AUTO_MAX = float(vxt_auto_max_text) if vxt_auto_max_text else None
            self.validate_input(self.VX_AUTO_MAX, RobConf.AUTO_VX_MIN, RobConf.AUTO_VX_MAX, "[Auto] vận tốc thẳng.")

            vw_auto_max_text = self.textbox_vtx.toPlainText()
            self.VW_AUTO_MAX = float(vw_auto_max_text) if vw_auto_max_text else None
            self.validate_input(self.VW_AUTO_MAX, RobConf.AUTO_VW_MIN, RobConf.AUTO_VW_MAX, "[Auto] vận tốc xoay.")

            vx_manual_max_text = self.textbox_vtt_2.toPlainText()
            self.VX_MANUAL_MAX = float(vx_manual_max_text) if vx_manual_max_text else None
            self.validate_input(self.VX_MANUAL_MAX, RobConf.MANUAL_VX_MIN, RobConf.MANUAL_VX_MAX, "[Manual] vận tốc thẳng.")

            vw_manual_max_text = self.textbox_vtx_2.toPlainText()
            self.VW_MANUAL_MAX = float(vw_manual_max_text) if vw_manual_max_text else None
            self.validate_input(self.VW_MANUAL_MAX, RobConf.MANUAL_VW_MIN, RobConf.MANUAL_VW_MAX, "[Manual] vận tốc xoay.")

            dung_sai_diem_den_vitri_text = self.textbox_sskc.toPlainText()
            self.DUNG_SAI_DIEM_DEN_VITRI = float(dung_sai_diem_den_vitri_text) if dung_sai_diem_den_vitri_text else None
            self.validate_input(self.DUNG_SAI_DIEM_DEN_VITRI, 0, RobConf.DUNG_SAI_KC_MAX, "Sai số khoảng cách.")

            dung_sai_diem_den_goc_text = self.textbox_ssgx.toPlainText()
            self.DUNG_SAI_DIEM_DEN_GOC = float(dung_sai_diem_den_goc_text) if dung_sai_diem_den_goc_text else None
            self.validate_input(self.DUNG_SAI_DIEM_DEN_GOC, 0, RobConf.DUNG_SAI_GOC_MAX, "Sai số góc xoay.")

            chieu_cao_led_text = self.textbox_ccdl.toPlainText()
            self.CHIEU_CAO_LED = float(chieu_cao_led_text) if chieu_cao_led_text else None
            self.validate_input(self.CHIEU_CAO_LED, RobConf.CHIEU_CAO_LED_MIN, RobConf.CHIEU_CAO_LED_MAX, "Chiều cao đèn LED.")

            muc_dien_ap_sac_pin_text = self.textbox_pin_sac_auto.toPlainText()
            self.MUC_DIEN_AP_SAC_PIN = float(muc_dien_ap_sac_pin_text) if muc_dien_ap_sac_pin_text else None
            self.validate_input(self.MUC_DIEN_AP_SAC_PIN, RobConf.MUC_DIEN_AP_SAC_MIN, RobConf.MUC_DIEN_AP_SAC_MAX, "Mức pin sạc tự động.")
          
            delay_time = self.textbox_delay_time.toPlainText()
            try:
                self.ros2_handle.goal_publisher.delay_time = int(delay_time)
            except ValueError:
                self.display_thatbai("Error: "+str(ValueError))
                return

            success = sql.setupRobot_sql_save(
            self.VX_AUTO_MAX,
            self.VW_AUTO_MAX,
            self.VX_MANUAL_MAX,
            self.VW_MANUAL_MAX,
            self.DUNG_SAI_DIEM_DEN_VITRI,
            self.DUNG_SAI_DIEM_DEN_GOC,
            self.CHIEU_CAO_LED,
            self.username_global,
            self.password_global,
            self.MUC_DIEN_AP_SAC_PIN,
            self.port_backcamera,
            self.port_headcamera,
            self.port_front_L,
            self.port_front_R,
        )
            if success:
                self.display_thanhcong("Dữ Liệu Đã Lưu Thành Công")
      
        except Exception as e:
                self.display_thatbai("Error: "+str(e))
                return

    def fcn_resetall_sql(self):
        # Tạo hộp thoại xác nhận
        reply = QMessageBox.question(
            self,
            "Xác Nhận",
            "Bạn có chắc chắn muốn reset dữ liệu? Điều này sẽ mất hết dữ liệu hiện tại.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        # Kiểm tra phản hồi của người dùng
        if reply == QMessageBox.Yes:
            # Nếu người dùng chọn Yes, thực hiện reset
            self.VX_AUTO_MAX = RobConf.AUTO_VX_DF
            self.VW_AUTO_MAX = RobConf.AUTO_VW_DF 
            self.VX_MANUAL_MAX = RobConf.MANUAL_VX_DF 
            self.VW_MANUAL_MAX = RobConf.MANUAL_VW_DF 
            self.DUNG_SAI_DIEM_DEN_VITRI = RobConf.DUNG_SAI_KC_DF 
            self.DUNG_SAI_DIEM_DEN_GOC = RobConf.DUNG_SAI_GOC_DF
            self.CHIEU_CAO_LED = RobConf.CHIEU_CAO_LED_DF
            self.MUC_DIEN_AP_SAC_PIN = RobConf.MUC_DIEN_AP_DF 
            
            sql.setupRobot_sql_save(
                self.VX_AUTO_MAX,
                self.VW_AUTO_MAX,
                self.VX_MANUAL_MAX,
                self.VW_MANUAL_MAX,
                self.DUNG_SAI_DIEM_DEN_VITRI,
                self.DUNG_SAI_DIEM_DEN_GOC,
                self.CHIEU_CAO_LED,
                self.username_global,
                self.password_global,
                self.MUC_DIEN_AP_SAC_PIN,
                self.port_backcamera,
                self.port_headcamera,
                self.port_front_L,
                self.port_front_R,)

            # Hiển thị thông báo thành công
            self.display_thanhcong("Dữ Liệu Đã Được Reset")
################## CAMERA ##################
    def capture_images_from_cameras(self, output_folder=Uti.image_path("image_setup_camera"), threshold=10, max_attempts=30):
        # Tạo thư mục lưu ảnh nếu chưa có
        camera_ids = [0, 2, 4, 6]

        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        # Duyệt qua các ID camera và kiểm tra camera nào mở được
        for idx, cam_id in enumerate(camera_ids):
            cap = cv2.VideoCapture(cam_id)
            
            if cap.isOpened():
                print(f"Camera {cam_id} mở được.")
                
                attempts = 0
                frame = None
                
                while attempts < max_attempts:
                    # Đọc khung hình từ camera
                    ret, frame = cap.read()

                    if not ret:
                        print(f"[capture_image] Không thể đọc khung hình từ camera {cam_id}.")
                        break

                    # Tính độ sáng trung bình của khung hình (tính trên kênh độ xám)
                    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    brightness = np.mean(gray_frame)
                    print(f"[capture_image] Độ sáng khung hình từ camera {cam_id}: {brightness}")

                    # Kiểm tra nếu độ sáng lớn hơn ngưỡng (threshold), lưu ảnh và thoát
                    if brightness > threshold:
                        file_name = f"camera_{cam_id}.jpg"
                        file_path = os.path.join(output_folder, file_name)
                        cv2.imwrite(file_path, frame)
                        print(f"Đã chụp và lưu ảnh từ camera {cam_id} vào {file_path}")
                        print("DA VO DE GHI ANH LEN TABLE")
                        # Hiển thị khung hình lên QLabel tương ứng
                        break
                    else:
                        print(f"[capture_image] Khung hình từ camera {cam_id} quá tối, chụp lại...")
                        attempts += 1
                
                if attempts == max_attempts:
                    print(f"[capture_image] Đã đạt đến số lần chụp tối đa từ camera {cam_id} mà vẫn chưa có khung hình sáng.")
                
            else:
                print(f"Camera {cam_id} không mở được.")
            
            # Giải phóng camera sau khi sử dụng
            cap.release()
    def get_camera_id_from_filename(self, filename):
        # Sử dụng regex để tìm số trong tên file
        match = re.search(r'camera_(\d+)', filename)
        if match:
            return int(match.group(1))
        return None
    def read_images_from_folder(self, folder_path):
        images = []
        # Lặp qua tất cả các file trong folder
        for filename in os.listdir(folder_path):
            # Kiểm tra nếu file có định dạng hình ảnh
            if filename.endswith(('.jpg', '.jpeg', '.png')):
                image_path = os.path.join(folder_path, filename)
                camera_id = self.get_camera_id_from_filename(filename)
                if camera_id is not None:
                    images.append((camera_id, image_path))
        print(images)
        return images
    def display_image_on_table(self, port, image_path, lable, table):
        # Tải ảnh từ đường dẫn
        lable.setText(f"Port: {port}")
        pixmap = QtGui.QPixmap(image_path)
        
        # Resize QPixmap theo kích thước mong muốn (400x400)
        resized_pixmap = pixmap.scaled(400, 400, QtCore.Qt.KeepAspectRatio)
        
        # Đặt QPixmap vào QLabel
        table.setPixmap(resized_pixmap)

    def fcn_start_setup_camera(self):
            #self.display_thanhcong("Đang trong quá trình setup Camera")
            Uti.RobotSpeakWithPath('voice_hmi_new/setup_cam.wav')
            self.capture_images_from_cameras()
            folder_image = Uti.image_path("image_setup_camera")
            image_files = self.read_images_from_folder(folder_image)
            idx = 0
            lable_port =  [
                self.lable_id_0,
                self.lable_id_2,
                self.lable_id_4,
                self.lable_id_6
            ]

            table_port =  [
                self.chua_camera_0,
                self.chua_camera_2,
                self.chua_camera_4,
                self.chua_camera_6
            ]

            for camera_id, image_path in image_files:
                print(f"Camera ID: {camera_id}, Image Path: {image_path}")
                self.display_image_on_table(camera_id,image_path, lable_port[idx], table_port[idx])
                idx += 1
            self.finish_setup_cam()
    
    def fcn_save_id_camera(self):

        try:
            self.port_headcamera = int(self.combobox_headcame.currentText())
            self.port_backcamera = int(self.combobox_back.currentText())
            self.port_front_L = int(self.combobox_front_1.currentText())
            self.port_front_R = int(self.combobox_front_2.currentText())

        except ValueError:
            return None, None, None, None  # Trả về None nếu có lỗi
        
        
        sql.setupRobot_sql_save(
            self.VX_AUTO_MAX,
            self.VW_AUTO_MAX,
            self.VX_MANUAL_MAX,
            self.VW_MANUAL_MAX,
            self.DUNG_SAI_DIEM_DEN_VITRI,
            self.DUNG_SAI_DIEM_DEN_GOC,
            self.CHIEU_CAO_LED,
            self.username_global,
            self.password_global,
            self.MUC_DIEN_AP_SAC_PIN,
            self.port_backcamera,
            self.port_headcamera,
            self.port_front_L,
            self.port_front_R,
        )
    #========================== cac ham GUI cua nut nhan SETUP CAM ===============================
    def finish_setup_cam(self):
        self.from_finish_setup_cam = QMainWindow()
        self.uic14 = Ui_Form_finish_setup_cam()
        self.uic14.setupUi(self.from_finish_setup_cam)
        self.from_finish_setup_cam.setGeometry(680, 240, 550, 250)
        self.from_finish_setup_cam.setStyleSheet("background-color:rgb(255, 255, 255);")
        self.from_finish_setup_cam.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        
        self.from_finish_setup_cam.show()
        self.uic14.btn_finish_setup_cam.clicked.connect(self.close_finish_setup_cam)

    def close_finish_setup_cam(self):
        self.from_finish_setup_cam.close()

     #========================== cac ham GUI  cua nut nhan CHECK CAM ===============================
    def dang_nd_cam(self):
        self.from_dang_nd_cam = QMainWindow()
        self.uic10 = Ui_Form_dangnhap_cam()
        self.uic10.setupUi(self.from_dang_nd_cam)
        self.from_dang_nd_cam.setGeometry(680, 240, 550, 250)
        self.from_dang_nd_cam.setStyleSheet("background-color:rgb(255, 255, 255);")
        self.from_dang_nd_cam.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.from_dang_nd_cam.show()
        # self.uic10.btn_dang_nd_cam.clicked.connect(self.close_dang_nd_cam)
        self.uic10.btn_dang_nd_cam.clicked.connect(self.close_dang_nd_cam)
    def close_dang_nd_cam(self):
        # self.dinh_vi()
        self.from_dang_nd_cam.close()
        Uti.RobotSpeakWithPath('voice_hmi_new/check_cam.wav')
        self.check_cam()
    def finish_cam(self):
        self.from_finish_cam = QMainWindow()
        self.uic12= Ui_Form_finish_cam()
        self.uic12.setupUi(self.from_finish_cam)
        self.from_finish_cam.setGeometry(680, 240, 550, 250)
        self.from_finish_cam.setStyleSheet("background-color:rgb(255, 255, 255);")
        self.from_finish_cam.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.from_finish_cam.show()
        self.uic12.btn_xacnhan_hoanthanh_cam.clicked.connect(self.close_finish_cam)
    def close_finish_cam(self):
        self.from_finish_cam.close()
    def error_cam(self):
        self.from_error_cam = QMainWindow()
        self.uic11 = Ui_Form_error_cam()
        self.uic11.setupUi(self.from_error_cam)
        self.from_error_cam.setGeometry(680, 240, 550, 250)
        self.from_error_cam.setStyleSheet("background-color:rgb(255, 255, 255);")
        self.from_error_cam.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.from_error_cam.show()
        self.uic11.btn_error_cam.clicked.connect(self.close_err_cam)
    def close_err_cam(self):
        self.from_error_cam.close()
    #======================= ham cua nut nhan CHECK CAM ======================
    def check_cam(self):
        global headCamera
        # self.dang_nd_cam()

        brightness_value = 100

        print('------------ Bắt đầu tìm tâm hình tròn -----------')
        # print("sanggggg: ", brightness_value)

        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            print("Không thể mở camera.")
            return
        
        cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness_value / 1)
        start_time = time.time()
        max_radius = 0
        circle_count = 0  # Đếm số lượng hình tròn
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Không thể đọc khung hình từ camera.")
                break
            frame = cv2.resize(frame, (1000, 600))  
            blurred_frame = cv2.GaussianBlur(frame, (25, 25), 0) 
            hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
            color_ranges = {
                'yellow': ([20, 100, 100], [36, 255, 255]) 
                # 'green': ([36, 25, 25], [86, 255, 255])
            }
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            combined_mask = np.zeros(gray_frame.shape[:2], dtype=np.uint8)
            centers = {}

            found_circle = False
            circle_count = 0  # Đếm số lượng hình tròn

            for color_name, (lower, upper) in color_ranges.items():
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                combined_mask = cv2.bitwise_or(combined_mask, mask)
                contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    ((x, y), radius) = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    if radius > max_radius:
                        max_radius = radius
                    cv2.circle(frame, center, radius, (0, 255, 0), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    text = f"{radius}px" 
                    cv2.putText(frame, text, (center[0] - 10, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    found_circle = True
                    circle_count += 1  # Tăng đếm số lượng hình tròn

                # cv2.imshow('Webcam', frame)

            

            if not found_circle:
                brightness_value += 5
                if brightness_value > 255:
                    brightness_value = 150
                    print("Không tìm thấy hình tròn sau khi điều chỉnh độ sáng.")
                    break
                cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness_value / 1)
                time.sleep(0.5)

         #   if (cv2.waitKey(1) & 0xFF == ord('q')) or (time.time() - start_time > 10):
              #  brightness_value = 100
             #   break

        cap.release()
        # cv2.destroyAllWindows()

        print(f"Bán kính lớn nhất được phát hiện: {max_radius} pixel")


        #---- giao diện thông báo ------
        if circle_count == 1:
            self.finish_cam()
            Uti.RobotSpeakWithPath('voice_hmi_new/finish_check_cam.wav')
        else:
            print(f"Phát hiện được {circle_count} hình tròn")
            self.error_cam()
    def check_and_opencamera(self):
        # Mở camera nếu chưa mở
        if self.cap_qr is None or not self.cap_qr.isOpened():
            print("[check_and_opencamera] dang mo camera F")
            try:
                self.cap_qr = cv2.VideoCapture(RobConf.faceCame1)
            except Exception as e:
                print(e)

        if self.capC is None or not self.capC.isOpened():
            print("[check_and_opencamera] dang mo camera C")
            try:
                self.capC = cv2.VideoCapture(RobConf.headCamera)
            except Exception as e:
                print(e)
################## ROS ##################
################## MANUAL ##################
    def thaydoivantoc(self):
        slider_value = self.Slider_vantoc.value()
        # lấy giá trị thanh Slider
        tocdo = slider_value / 1000.0
        self.ros2_handle.cmd_vel_publisher.vx = tocdo
        self.ros2_handle.cmd_vel_publisher.vw = tocdo*1.0
        tocdo_phantram = tocdo*100
        self.label_tocdo.setText(str(round(tocdo_phantram,1))+"%")
    def initialize_ros2_manager(self, ):
        self.ros2_handle.start()
        self.ros2_handle.progress_status.connect(self.status_popup.update_status)
        self.ros2_handle.goal_publisher.reached_goal.connect(self.reached_goal)
   
    def mapping(self, ):
        self.mapping_button.setEnabled(False)
        self.ros2_handle.stop_navigation()
        self.ros2_handle.start_mapping()
        self.stop_mapping_button.setEnabled(True)
        self.ros2_handle.map_viewer_subcriber.mapping_status.connect(self.update_map)
    def update_map(self, flag):
        if flag == True:
            self.map_label.setPixmap(self.ros2_handle.map_viewer_subcriber.map_pixmap)
    def stop_mapping(self, ):
        self.stop_mapping_button.setEnabled(False)
        self.ros2_handle.stop_mapping()
        self.ros2_handle.start_navigation()
        self.mapping_button.setEnabled(True)
    def save_map(self, ):
        if self.ros2_handle.map_viewer_subcriber.is_running:
            named_map_popup = NamedMap(RobConf.MAP_FOLDER, self.ros2_handle.map_viewer_subcriber.default_map_name)
            map_name = None
            if named_map_popup.exec_() == QDialog.Accepted:
                map_name = named_map_popup.get_selected_map_name()

                if not map_name:
                    QMessageBox.warning(self, "Invalid Input", "Please enter a valid map name.")
                    return

                file_name = os.path.join(RobConf.MAP_FOLDER, map_name + ".yaml")

                if os.path.exists(file_name):
                    reply = QMessageBox.question(
                        self,
                        'Overwrite File',
                        f'The file "{map_name}" already exists.\nDo you want to overwrite it?',
                        QMessageBox.Yes | QMessageBox.No,
                        QMessageBox.No
                    )
                    if reply == QMessageBox.No:
                        return
                if named_map_popup.is_set_as_default():
                    self.ros2_handle.map_viewer_subcriber.save_map(map_name=map_name)
                else:
                    self.ros2_handle.map_viewer_subcriber.save_map(map_name=map_name, is_set_to_default=False)
    def load_map(self, ):
        self.ros2_handle.map_viewer_subcriber.load_map()
        self.map_label.setPixmap(self.ros2_handle.map_viewer_subcriber.map_pixmap)
        self.ros2_handle.map_viewer_subcriber.mapping_status.connect(self.update_map)
    def show_map(self):
        self.manhinh_map()
    def control_robot(self):
        self.bt_UP.pressed.connect(self.ros2_handle.cmd_vel_publisher.forward)
        self.bt_UP.released.connect(self.ros2_handle.cmd_vel_publisher.stop_movement)
        self.bt_DOWN.pressed.connect(self.ros2_handle.cmd_vel_publisher.backward)
        self.bt_DOWN.released.connect(self.ros2_handle.cmd_vel_publisher.stop_movement)
        self.bt_RIGHT.pressed.connect(self.ros2_handle.cmd_vel_publisher.right)
        self.bt_RIGHT.released.connect(self.ros2_handle.cmd_vel_publisher.stop_movement)
        self.bt_LEFT.pressed.connect(self.ros2_handle.cmd_vel_publisher.left)
        self.bt_LEFT.released.connect(self.ros2_handle.cmd_vel_publisher.stop_movement)
    def manhinh_nhap_soban(self):  # dùng cho nút cập nhật số bàn
        self.from_nhap = QMainWindow()
        self.uic3 = Ui_Form_nhaptenban()
        self.uic3.setupUi(self.from_nhap)

        # self.from_nhap.setGeometry(160, 140, 750, 550)
        self.from_nhap.setGeometry(600, 240, 750, 550)
        self.from_nhap.setStyleSheet("background-color:rgb(255, 255, 255);")
        self.from_nhap.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.from_nhap.show()
     
        self.uic3.Button_huy_capnhat.clicked.connect(self.tatmanhinh_capnhat)

        self.uic3.lineEdit_nhaptenban.mousePressEvent = self.xu_ly_nhap_banphim
 
        # sau khi nhấn enter hoặc nút xác nhận
        self.uic3.lineEdit_nhaptenban.returnPressed.connect(self.close_manhinh_nhapban)
        self.uic3.Button_xacnhan_capnhat.clicked.connect(self.close_manhinh_nhapban)
        # =====================#
        self.uic3.Button_docksac.clicked.connect(self.ham_dock)
        self.uic3.Button_kvchow.clicked.connect(self.ham_kvcho)
        self.uic3.Button_ban.clicked.connect(self.ham_toado)
        self.uic3.Button_khacc.clicked.connect(self.ham_tg)
        self.uic3.lineEdit_nhaptenban.setEnabled(False)

        # --------------------- phat am thanh -------------------------
        Uti.RobotSpeakWithPath('voice_hmi_new/capnhat.wav')
    def ham_dock(self):
       # print("ham_dock")
        self.uic3.lineEdit_nhaptenban.setText("Trạm sạc")
        self.trangthai_kv = 1
        self.uic3.lineEdit_nhaptenban.setEnabled(False)
        self.uic3.Button_docksac.setStyleSheet(self.css_ham)
        # =============3 nút còn lại ko sáng
        self.uic3.Button_ban.setStyleSheet(self.css_unham)
        self.uic3.Button_kvchow.setStyleSheet(self.css_unham)
        self.uic3.Button_khacc.setStyleSheet(self.css_unham)
    def ham_tg(self):
       # print("ham_tg")
        self.uic3.lineEdit_nhaptenban.setText("Vị trí trung gian")
        self.uic3.lineEdit_nhaptenban.setEnabled(False)
        self.trangthai_kv = 4
    def ham_kvcho(self):
        #print("ham_kvchow")
        self.uic3.lineEdit_nhaptenban.setText("Khu vực chờ")
        self.trangthai_kv = 2
        self.uic3.lineEdit_nhaptenban.setEnabled(False)
        self.uic3.Button_kvchow.setStyleSheet(self.css_ham)
        # =============3 nút còn lại ko sáng
        self.uic3.Button_ban.setStyleSheet(self.css_unham)
        self.uic3.Button_docksac.setStyleSheet(self.css_unham)
        self.uic3.Button_khacc.setStyleSheet(self.css_unham)
    def ham_toado(self):
        #print("ham_ban")
        self.uic3.lineEdit_nhaptenban.setEnabled(True)
        self.uic3.lineEdit_nhaptenban.clear()
        self.trangthai_kv = 3
        self.uic3.Button_ban.setStyleSheet(self.css_ham)
        # =============3 nút còn lại ko sáng
        self.uic3.Button_docksac.setStyleSheet(self.css_unham)
        self.uic3.Button_kvchow.setStyleSheet(self.css_unham)
        self.uic3.Button_khacc.setStyleSheet(self.css_unham)
    def ham_khacc(self):
        print("ham_khacc")
   
    def tatmanhinh_capnhat(self):
        self.from_nhap.close()
    def close_manhinh_nhapban(self):
        xuly_cham_ngoai()
        print('///////////////////////////////////////////////////////')
        # nhập giá trị từ 1 đến 10 đã khai báo sẵn, giá trị nhập từ máy tính là str
        # ======= phần này của Trạm sạc
        if self.trangthai_kv == 1:
            self.capnhattoado(100)
            self.listWidget_dsban.addItem("Trạm sạc")
            #button_name_kv = f"Button_tramsac_2"
            #button = getattr(self, button_name_kv)
            #button.setText('Về khu vực chờ')
            #button.show()

            self.from_nhap.close()
            # ====================Kiểm tra sự trùng lặp trên listWidget_dsban=============#
            items = [self.listWidget_dsban.item(index).text() for index in
                     range(self.listWidget_dsban.count())]
            # Sử dụng tập hợp (set) để loại bỏ các phần tử trùng lặp
            unique_items = set(items)
            ##print("hahah", unique_items)
            # Xóa tất cả các phần tử hiện tại trong listWidget_nhaptenban
            self.listWidget_dsban.clear()
            # Thêm lại các phần tử duy nhất từ tập hợp vào listWidget_nhaptenban
            for item_1 in unique_items:
                self.listWidget_dsban.addItem(item_1)
            self.sap_xep_stt_ban()
            # ======= phần này của Khu vực chờ
        elif self.trangthai_kv == 2:
            self.capnhattoado(200)
            self.listWidget_dsban.addItem("Khu vực chờ")
            #button_name_kv = f"Button_khuvuccho_2"
            #button = getattr(self, button_name_kv)
            # button.show()
            #button.hide()

            self.from_nhap.close()
            # ====================Kiểm tra sự trùng lặp trên listWidget_dsban=============#
            items = [self.listWidget_dsban.item(index).text() for index in
                     range(self.listWidget_dsban.count())]
            # Sử dụng tập hợp (set) để loại bỏ các phần tử trùng lặp
            unique_items = set(items)
            # print("hahah", unique_items)
            # Xóa tất cả các phần tử hiện tại trong listWidget_nhaptenban
            self.listWidget_dsban.clear()
            # Thêm lại các phần tử duy nhất từ tập hợp vào listWidget_nhaptenban
            for item_1 in unique_items:
                self.listWidget_dsban.addItem(item_1)
            self.sap_xep_stt_ban()

            # print(text_nhap)  # in ra coi thử
            # ======= phần này của 10 nút bàn
        elif self.trangthai_kv == 3:  # kiểm tra nếu nhập đúng từ 1 đến 10
            text_nhap = self.uic3.lineEdit_nhaptenban.text()
            if text_nhap in self.soban:

                # gọi hàm cập nhật tọa độ và truyền số nhập được vào hàm
                self.capnhattoado(text_nhap)
                # print("nhập số bàn thành công")
                self.listWidget_dsban.addItem("Phòng {0}".format(text_nhap))
                # truyền dữ liệu vào text và hiển thị nút Button_1_2 đến Button_10_2
                #button_name = f"Button_{int(text_nhap)}_2"
                #button = getattr(self, button_name)
                #button.show()
                # đóng from_nhập lại
                self.from_nhap.close()
                # ====================Kiểm tra sự trùng lặp trên listWidget_dsban=============#
                items = [self.listWidget_dsban.item(index).text() for index in
                         range(self.listWidget_dsban.count())]
                # Sử dụng tập hợp (set) để loại bỏ các phần tử trùng lặp
                unique_items = set(items)
                # print("hahah", unique_items)
                # Xóa tất cả các phần tử hiện tại trong listWidget_nhaptenban
                self.listWidget_dsban.clear()
                # Thêm lại các phần tử duy nhất từ tập hợp vào listWidget_nhaptenban
                for item_1 in unique_items:
                    self.listWidget_dsban.addItem(item_1)
                self.sap_xep_stt_ban()
                self.uic3.lineEdit_nhaptenban.clear()
            else:  # thông báo nhập sai
                QMessageBox.information(self, "Chào bạn", "Mời nhập lại")
                self.uic3.lineEdit_nhaptenban.clear()
        elif self.trangthai_kv == 4:
            self.idban = 123
            print("idban   cap nhat:                    ", self.idban)
            self.capnhattoado(100) # CAP NHAT TOA DO CHAM DOCK SAC
            self.capnhattoado(123)  # CAP NHAT VI TRI TRUOC DOCK SAC
            self.from_nhap.close()

    # ============ sắp xếp thứ tự bàn hien thi tren list ==============#
    def sap_xep_stt_ban(self):
        items = [self.listWidget_dsban.item(index).text() for index in
                 range(self.listWidget_dsban.count())]

        def custom_sort_key(item):
            if item == "Trạm sạc" or item == "Khu vực chờ":
                return (1, item)
            elif item.startswith("Phòng "):
                return (2, int(item.split("Phòng ")[-1]))
            else:
                return (3, item)

        sorted_items = sorted(items, key=custom_sort_key)
        self.listWidget_dsban.clear()
        for item in sorted_items:
            self.listWidget_dsban.addItem(item)
    
    #=================== ham xu ly tinh goc yaw rad ===================
    def calcular_yaw_rad(self, z, w):
        x = 0
        y = 0
        psi = x * x + y * y + z * z + w
        eps1 = x
        eps2 = y
        eps3 = z
        eps4 = w
        r11 = 1 - 2 * eps2 * eps2 - 2 * eps3 * eps3
        r21 = 2 * (eps1 * eps2 + eps3 * eps4)
        yaw = math.atan2(r21, r11)
        #yaw = yaw * 180 / math.pi
        #print("yaw: ", yaw)
        return yaw
    def capnhattoado(self, id):  # gửi dữ liệu lên sql\
        # Hiện thông báo
        # print("đã kích hoạt")
        # Kiểm tra xem idban đã tồn tại trong cơ sở dữ liệu hay không
        try:
            mydb = mysql.connector.connect(
      
                user="robot",
                password="12345678",
                host="127.0.0.1",
                database="sql_rsr"
            )
            mycursor = mydb.cursor()
        except mysql.connector.Error as err:
            print(f"Lỗi kết nối cơ sở dữ liệu: {err}")
            return None  # Trả về None nếu kết nối thất bại
        check_query = "SELECT COUNT(*) FROM toado WHERE `IDban` = %s;"
        mycursor = mydb.cursor()
        mycursor.execute(check_query, (id,))
    # lệnh truy vấn SQL và gán cho biến result
        result = mycursor.fetchone()
        if result and result[0] > 0:
            # Nếu có idban đã tồn tại, thực hiện lệnh DELETE để xóa dữ liệu có idban tương ứng
            delete_query = "DELETE FROM toado WHERE `IDban` = %s;"
            # delete_query = "DELETE FROM toa_do ;" # xóa toàn bộ dữ liệu trên table có tên toa_do
            mycursor.execute(delete_query, (id,))
            # mycursor.execute(delete_query)
            mydb.commit()
            # print(f"Đã xóa các dữ liệu có IDban = {id} trùng lặp")
            # =======gửi tọa độ đã đặt sẵn vào biến data_to_insert
        self.x, self.y, self.z, self.w = self.ros2_handle.pose_listener.data_odom
       # print(self.x)
       # print(self.y)
       # print(self.z)
        #print(self.w)
      
        if(id==RobConf.TruocDockSacID): # lay vi tri trc dock sac
            yaw =self.calcular_yaw_rad(self.z, self.w)
            l = RobConf.KHOANG_CACH_VE_TRUOC_DOCKSAC
            self.x = self.x + l*math.cos(yaw)
            self.y = self.y + l*math.sin(yaw)
        # if id == 100:
        #     data_to_insert = (self.x+0.3, self.y+0.35, self.z, self.w, id, 1)
        # else:
        data_to_insert = (self.x, self.y, self.z, self.w, id, 1)
        insert_query = "INSERT INTO toado (`toadoX`, `toadoY`, `toadoZ`, `toadoW`, `IDban`, `boole`) VALUES (%s, %s, %s, %s, %s, %s);"
        mycursor = mydb.cursor()
        mycursor.execute(insert_query, data_to_insert)
        mydb.commit()

        if id == RobConf.HOME_ID:
            with open('txt/KHUVUCCHO_data.txt', 'w') as file:
                file.write(f'x: {self.x}, y: {self.y}, z: {self.z}, w: {self.w}\n')
        # print("cập nhật hoàn tất")
    def sap_xep_stt_ban_uic4(self):
            items = [self.uic4.listWidget_themvao.item(index).text() for index in
                 range(self.uic4.listWidget_themvao.count())]
    def xoatoanbo_mysql(self):
        # ===============hiển thị cảnh báo xóa===========#
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setWindowTitle("Cảnh báo")
        msg_box.setText("Nếu xóa bạn phải cập nhật lại từ đầu?")
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        msg_box.setDefaultButton(QMessageBox.StandardButton.No)
        resultb = msg_box.exec()
        # ===============kiểm tra hiển thị====================#
        # kiểm tra có dữ liệu chưa
        if resultb == QMessageBox.StandardButton.Yes:
            mydb = mysql.connector.connect(
      
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
            )
            mycursor = mydb.cursor()
            # Sử dụng câu lệnh TRUNCATE để xóa toàn bộ dữ liệu từ bảng
            truncate_query = "TRUNCATE TABLE toado"
            mycursor.execute(truncate_query)
            # xóa list
            self.listWidget_dsban.clear()
            # ẩn toàn bộ 10 nút
           # for i in range(1, 10):
               # button_name = f"Button_{i}_2"
               # button = getattr(self.sub_win1.uic, button_name)
                #button.hide()
    def custom_sort_key(item):
            if item == "Trạm sạc" or item == "Khu vực chờ":
                return (1, item)
            elif item.startswith("Phòng "):
                return (2, int(item.split("Phòng ")[-1]))
            else:
                return (3, item)

            sorted_items = sorted(items, key=custom_sort_key)
            self.uic4.listWidget_themvao.clear()
            for item in sorted_items:
                self.uic4.listWidget_themvao.addItem(item)
    
    #=================================================== GIAO DIEN NUT THEM =============================================================
    def manhinh_them(self):  # dùng cho nút thêm bàn sau khi xóa
        self.from_them = QMainWindow()
        self.uic4 = Ui_Form_them()
        self.uic4.setupUi(self.from_them)
        # tắt viền màn hình thêm
        self.from_them.setGeometry(630, 240, 550, 450)
        self.from_them.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.from_them.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.bien_close_them = True

        self.from_them.show()
        # Form_them.setWindowFlag(QtCore.Qt.FramelessWindowHint)
        # Form_them.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.uic4.Button_huythem.clicked.connect(self.tat_Form_them)
        # keyyyyy.Registe(self.uic4.lineEdit_nhaptenban_add)
        self.uic4.Button_them.clicked.connect(self.close_manhinh_them)
        self.uic4.Button_them.setEnabled(False)
        self.uic4.listWidget_themvao.itemClicked.connect(self.item_them_clicked)
        self.addtu_uic2_uic4()
        self.kiem_tra_listWidget_themvao()

        # --------------------- phat am thanh -------------------------
        Uti.RobotSpeakWithPath('voice_hmi_new/them.wav')
    def tat_Form_them(self):
        self.from_them.close()
    def kiem_tra_listWidget_themvao(self):  # ======== Kiểm tra để set nút Thêm
        if self.uic4.listWidget_themvao.count() > 0:
            self.uic4.Button_them.setEnabled(True)
        else:
            self.uic4.Button_them.setEnabled(False)
    def close_manhinh_them(self):
        db = mysql.connector.connect(
      
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
            )
        # Phần này của 10 nút 
        if self.selected_them is not None:
            # Xóa phần tử đã chọn từ listWidget_dsban
            self.uic4.listWidget_themvao.takeItem(self.uic4.listWidget_themvao.row(self.selected_them))
            # Ẩn nút Button tương ứng với phần tử đã xóa
            hang_delete_uic4 = int(self.extract_number(self.selected_number1))
            print("kiểu của hàng delete là :", hang_delete_uic4)
            if hang_delete_uic4:
                data4 = (1, hang_delete_uic4)
                print("đã set 1")
                # =======gửi dữ liệu của boole = 1 lên nút cần xóa======#
                insert_query4 = "UPDATE toado SET boole = %s WHERE IDban = %s;"
                mycursor = db.cursor()
                mycursor.execute(insert_query4, data4)
                db.commit()
                # cập nhật hoàn thành============#
                print(hang_delete_uic4)
                # Thêm các mục từ kết quả truy vấn vào listWidget_dsban
                ban = hang_delete_uic4
                #ban_int = int(ban)+310
                #ban_str = str(ban_int)
                self.listWidget_dsban.addItem(f"Phòng {ban}")
           #     button_name = f"Button_{ban}_2"
           #     button = getattr(self.sub_win1.uic, button_name)
            #    button.show()
            self.selected_them = None
            # ============= Phần này của Tram sạc
        elif self.selected_tram_them is not None:
            data4 = (1, 100)
            # =======gửi dữ liệu của boole = 1 lên nút cần xóa======#
            insert_query4 = "UPDATE toado SET boole = %s WHERE IDban = %s;"
            mycursor = db.cursor()
            mycursor.execute(insert_query4, data4)
            db.commit()
            # cập nhật hoàn thành============#
            self.listWidget_dsban.addItem("Trạm sạc")
            #button_name = f"Button_tramsac_2"
            #button = getattr(self.sub_win1.uic, button_name)
            #button.setText('Về khu vực chờ')
            #button.show()
            self.selected_tram_them = None
            # ============= Phần này của Khu vực chờ
        elif self.selected_kvcho_them is not None:
            data4 = (1, 200)
            # =======gửi dữ liệu của boole = 1 lên nút cần xóa======#
            insert_query4 = "UPDATE toado SET boole = %s WHERE IDban = %s;"
            mycursor = db.cursor()
            mycursor.execute(insert_query4, data4)
            db.commit()
            # cập nhật hoàn thành============#
            self.listWidget_dsban.addItem("Khu vực chờ")
           # button_name = f"Button_tramsac_2"
           ## button = getattr(self.sub_win1.uic, button_name)
           # button.setText('Về khu vực chờ')
           # button.show()
            self.selected_kvcho_them = None
        self.sap_xep_stt_ban()
        self.from_them.close()
     # ==============Khi mà đã click vào thành phần trên listWidget_dsban==========#
    def item_xoa_clicked(self, item):

        # Lưu trữ tham chiếu đến phần tử đã chọn
        if item.text() == "Trạm sạc":
            self.selected_tram = item
            self.selected_item_infor = None
            self.selected_kvcho = None
        elif item.text() == "Khu vực chờ":
            self.selected_kvcho = item
            self.selected_tram = None
            self.selected_item_infor = None
        else:
            self.selected_item_infor = item
            self.selected_number = item.text()
            print(self.selected_item_infor)
            self.selected_kvcho = None
            self.selected_tram = None
    def item_them_clicked(self, item1):
        self.uic4.Button_them.setEnabled(True)
        if item1.text() == "Trạm sạc":
            self.selected_tram_them = item1
            self.selected_kvcho_them = None
            self.selected_them = None
        elif item1.text() == "Khu vực chờ":
            self.selected_kvcho_them = item1
            self.selected_tram_them = None
            self.selected_them = None
        else:
            self.selected_them = item1
            self.selected_number1 = item1.text()
            print(self.selected_them)
            self.selected_tram_them = None
            self.selected_kvcho_them = None
    def addtu_uic2_uic4(self):
        # ================lấy dữ liệu có sẵn trước đó trên mysql=================#
        db = mysql.connector.connect(

            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
            )
        select_query = "SELECT DISTINCT IDban FROM toado WHERE boole = 0"
        mycursor = db.cursor()
        mycursor.execute(select_query)
        result_set_0 = mycursor.fetchall()
        if result_set_0:
            # Xóa tất cả các phần tử hiện tại trong listWidget_dsban
            self.uic4.listWidget_themvao.clear()
            # Thêm các mục từ kết quả truy vấn vào listWidget_dsban
            for row in result_set_0:
                ban = row[0]
                if ban == '100':
                    self.uic4.listWidget_themvao.addItem("Trạm sạc")
                elif ban == '200':
                    self.uic4.listWidget_themvao.addItem("Khu vực chờ")
                elif ban != "200" and ban != "100":
                    #ban_int = int(ban) + 310
                   # ban_str = str(ban_int)
                    self.uic4.listWidget_themvao.addItem(f"Phòng {ban}")
            self.sap_xep_stt_ban_uic4()
    def extract_number(self, text):
        # Trích xuất số từ văn bản và trả về số dưới dạng chuỗi
        number = ''.join(filter(str.isdigit, text))
        return number
    def xoa_tungthanhphan(self):
        print('da bam nut xoa')
        mydb = mysql.connector.connect(
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
            )
        if self.selected_item_infor is not None:
            # Xóa phần tử đã chọn từ listWidget_dsban
            self.listWidget_dsban.takeItem(self.listWidget_dsban.row(self.selected_item_infor))
            # Ẩn nút Button tương ứng với phần tử đã xóa
            hang_delete = int(self.extract_number(self.selected_number))
            print("kiểu của hàng delete là :", hang_delete)
            if hang_delete:
                data2 = (0, hang_delete)
                # =======gửi dữ liệu của boole = 0 lên nút cần xóa======#
                insert_query2 = "UPDATE toado SET boole = %s WHERE IDban = %s;"
                mycursor = mydb.cursor()
                mycursor.execute(insert_query2, data2)
                mydb.commit()  # cập nhật hoàn thành
                print("đã xóa nút")
                # =================Ẩn nút đã xóa================#
                #button_name = f"Button_{hang_delete}_2"
                #button = getattr(self.sub_win1.uic, button_name)
                #button.hide()

        if self.selected_kvcho is not None:
            print("lll")
            self.listWidget_dsban.takeItem(self.listWidget_dsban.row(self.selected_kvcho))
            data2 = (0, 200)
            # =======gửi dữ liệu của boole = 0 lên nút cần xóa======#
            insert_query2 = "UPDATE toado SET boole = %s WHERE IDban = %s;"
            mycursor = mydb.cursor()
            mycursor.execute(insert_query2, data2)
            mydb.commit()  # cập nhật hoàn thành
            print("đã xóa nút")
            # =================Ẩn nút đã xóa================#
           # button_name = f"Button_khuvuccho_2"
           # button = getattr(self.sub_win1.uic, button_name)
         #   button.hide()

        if self.selected_tram is not None:
            self.listWidget_dsban.takeItem(self.listWidget_dsban.row(self.selected_tram))
            data2 = (0, 100)
            # =======gửi dữ liệu của boole = 0 lên nút cần xóa======#
            insert_query2 = "UPDATE toado SET boole = %s WHERE IDban = %s;"
            mycursor = mydb.cursor()
            mycursor.execute(insert_query2, data2)
            mydb.commit()  # cập nhật hoàn thành
            print("đã xóa nút")
            # =================Ẩn nút đã xóa================#
           # button_name = f"Button_tramsac_2"
          #  button = getattr(self.sub_win1.uic, button_name)
          #  button.setText('Về khu vực chờ')
           # button.hide()
        map_folder_path = RobConf.MAP_FOLDER
    def savemap(self):
        os.system(
            f'gnome-terminal -- bash -c "cd {map_folder_path} && rosrun map_server map_saver -f mymap2"')
        # time.sleep(3)
        print("1234556789")
################## GIAO DIEN THEM PHONG DE CHAY ##################
    def hien_thi_nut_theo_id(self):
        nut_dict = {
            100: self.btn_tram_sac,
            200: self.btn_khu_vuc_cho,
            1: self.btn_phong1,
            2: self.btn_phong2,
            3: self.btn_phong3,
            4: self.btn_phong4,
            5: self.btn_phong5,
            6: self.btn_phong6,
            7: self.btn_phong7,
            8: self.btn_phong8,
            9: self.btn_phong9,
            10: self.btn_phong10,
            
            # Thêm các nút khác nếu cần
        }

        for id_can_kiem_tra, nut in nut_dict.items():
            if sql.kiem_tra_id_ton_tai(id_can_kiem_tra):
                nut.show()
            else:
                nut.hide()
    def toggle_item(self, id):
        
        # kiem tra nut nhan 
        if(id==100):
            item_text = f"Trạm sạc" 
        elif(id==200):
            item_text = f"Khu vực chờ"
        else:
            item_text = f"Phòng {id}"
            
        if not self.item_states[id]:
            self.listWidget_ds.addItem(item_text)
            self.item_states[id] = True
        else:
            for i in range(self.listWidget_ds.count()):
                if self.listWidget_ds.item(i).text() == item_text:
                    self.listWidget_ds.takeItem(i)
                    self.item_states[id] = False
                    break
        self.kiem_tra_du_dieu_khien_chay()
        self.sort_items() # Add this line to sort items after each toggle
        

    def sort_items(self):
        # Sap sep trinh tu di tram sac, 
        items = []
        for i in range(self.listWidget_ds.count()):
            items.append(self.listWidget_ds.item(i).text())
        

        def sort_key(item):
            if item == "Trạm sạc":
                return 0  # Đặt "Trạm sạc" lên đầu
            elif item == "Khu vực chờ":
                return 1  # Đặt "Khu vực chờ" lên sau "Trạm sạc"
            else:
                try:
                    return int(item.split()[1]) # Sắp xếp phòng theo số thứ tự
                except (ValueError, IndexError):
                    return float('inf')  # Đặt các mục không phù hợp xuống cuối

        items.sort(key=sort_key)
        
        self.listWidget_ds.clear()
        for item in items:
            self.listWidget_ds.addItem(item)
        

    ################## SHOW MAP  ##################
    def show_map(self):
        self.mymap = QMainWindow()
        self.uic5 = Ui_Form_map()
        self.uic5.setupUi(self.mymap)

        self.mymap.setGeometry(59, 175, 1525, 804)
        self.mymap.setStyleSheet("background-color:rgb(230, 230, 230);")
        self.mymap.setWindowFlag(QtCore.Qt.FramelessWindowHint)

        self.mymap.show()
        print("đã hiển thị Map")
        self.uic5.Button_thoat.clicked.connect(self.tat_map)
       # self.sub_win1.uic.page.mousePressEvent = self.xu_ly_close_map
      #  self.sub_win1.uic.label_3.mousePressEvent = self.xu_ly_close_map
        # -------------------- Phần thêm thông tin yaml --------------------------
        # Đường dẫn đến tệp YAML
        yaml_file_path = RobConf.Path_yaml
        # Đọc tệp YAML
        with open(yaml_file_path, "r") as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
        # Trích xuất giá trị 'origin' từ dữ liệu YAML
        origin_value = yaml_data.get("origin")
        for pt in origin_value:
            self.x_home = origin_value[0]
            self.y_home = origin_value[1]
        print("Origin element:", self.x_home, self.y_home)

        # ====================Phần thêm ảnh map
        # tạo QGraphicsView
        self.view = QGraphicsView()
        self.scene = QGraphicsScene()
        self.view.setScene(self.scene)

        # tạo QVBoxLayout cho page
        self.uic5.page.setLayout(QVBoxLayout())
        self.uic5.page.layout().addWidget(self.view)

        # Tạo QLabel chứa hình ảnh
        self.image_label = QLabel()
        self.pixmap = QPixmap(RobConf.Map_Lidar)  # Đặt đường dẫn của hình ảnh
        self.image_label.setPixmap(self.pixmap)
        # self.image_label.setGeometry(1000, 10, 100, 2)
        # self.image_label.setScaledContents(True)
        # Thêm hình ảnh vào QGraphicsScene bằng cách sử dụng QGraphicsPixmapItem
        self.image_item = QGraphicsPixmapItem(self.pixmap)

        self.ti_le = 600 / self.pixmap.width()
        # self.ti_le = 1
        self.image_item.setScale(self.ti_le)
        self.scene.addItem(self.image_item)

        print(self.scene.width())
        print(self.scene.height())
        # Đặt giá trị ban đầu cho slider để ảnh chưa bị biến dạn
        self.uic5.slider_1_map.setMinimum(40)
        self.uic5.slider_1_map.setMaximum(150)
        self.uic5.slider_1_map.setValue(80)

        self.uic5.slider_1_map.valueChanged.connect(self.scale_elements)
        # Tạo QLabel để hiển thị thông tin kích thước
        self.info_label = QLabel(self)
        self.uic5.page.layout().addWidget(self.info_label)
        self.info_label.setText(f'Kích thước (px): {int(self.scene.width())}, {int(self.scene.height())} ')
        print(self.image_label.geometry())
        # gọi hàm tạo nút
        self.create_label_map()
        # Tạo click chuột
        self.clicked_position = None
        self.scale_factor = 1
        self.uic5.page.mousePressEvent = self.mouse_click_event
        # self.view.setGeometry(11, 1, 1876, 934)
        print(self.view.geometry())
    def tat_map(self):
        self.mymap.close()
    def mouse_click_event(self, event):
        # Lấy tọa độ x, y khi click chuột trên QGraphicsView
        x = event.x() * self.scale_factor
        y = event.y() * self.scale_factor
        self.clicked_position = (float(x), float(y))
        # Lấy 3 số sau dấu chấm của x và y
        x_str = "{:.5f}".format(x)
        y_str = "{:.5f}".format(y)
        # In ra tọa độ (x, y) trên hình ảnh
        print("(x, y):", x_str, y_str)
    def scale_elements(self):
        self.scale_factor = self.uic5.slider_1_map.value() / 80
        # Thay đổi tỷ lệ của QGraphicsView, chính là hiệu ứng zoom
        self.view.resetTransform()
        self.view.scale(self.scale_factor, self.scale_factor)
        # Lấy kích thước hiện tại của hình ảnh
        self.image_size1 = self.scale_factor * self.scene.width()
        self.image_size2 = self.scale_factor * self.scene.height()
        # Cập nhật QLabel hiển thị thông tin
        self.info_label.setText(f'Kích thước, (x, y): {int(self.image_size1)}, {int(self.image_size2)}')
    def create_label_map(self):
        db = mysql.connector.connect(

            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
            )
        css3 = """
                    background-color: transparent;
                    border-radius: 5px;
                    border: 1px solid rgb(255, 0, 0);
                    font: 6pt "MS Shell Dlg 2";
                    """
        csshome = """
                    background-color: transparent;
                    /*border: 2px solid rgb(255, 255, 0);*/

                    """
        css_dock = """
                    background-color: transparent;


                    """

        select_query_xy = "SELECT DISTINCT toadoX, toadoY, IDban FROM toado WHERE boole = 1"
        mycursor = db.cursor()
        mycursor.execute(select_query_xy)
        result_xy = mycursor.fetchall()
        if result_xy:
            for row in result_xy:
                x_tc = float(row[0])
                y_tc = float(row[1])
                idbann = float(row[2])
                print('kich thước pixmap w, h ', self.pixmap.width(), self.pixmap.height())

                if idbann == 100:
                    # label = QLabel("Trạm")
                    x_pic_dock = (x_tc + abs(self.x_home)) * self.ti_le
                    y_pic_dock = (self.pixmap.height() * 0.05 - (y_tc + abs(self.y_home))) * self.ti_le
                    # sửa đổi
                    label = QLabel()
                    label_name = "tramsac"
                    pixmap_tram = QPixmap(Uti.image_path('dock_position.jpg'))
                    label.setScaledContents(True)
                    label.setPixmap(pixmap_tram)
                    setattr(self, label_name, label)
                    x = int(float(x_pic_dock) / 0.05)
                    y = int(float(y_pic_dock) / 0.05)
                    label.setGeometry(x, y, 12, 12)
                    #label.setGeometry(int(x_pic_dock) / 0.05, int(y_pic_dock) / 0.05, 12, 12)
                    label.setStyleSheet(css_dock)
                    label.setAlignment(Qt.AlignCenter)
                    self.view.scene().addWidget(label)
                elif idbann == 200:
                    # label = QLabel("Chờ")
                    # tương đương với điểm home
                    # x_pic_home = self.x_home * self.ti_le / 0.05
                    # y_pic_home = self.scene.height() - (self.y_home * self.ti_le / 0.05)

                    x_pic_khuvuccho = (x_tc + abs(self.x_home)) * self.ti_le
                    y_pic_khuvuccho = (self.pixmap.height() * 0.05 - (y_tc + abs(self.y_home))) * self.ti_le

                    # sửa đổi
                    label = QLabel()
                    label_name = "khuvuccho"
                    pixmap_cho = QPixmap('images/airport.png')
                    label.setScaledContents(True)
                    label.setPixmap(pixmap_cho)

                    setattr(self, label_name, label)
                    label.setGeometry(int(x_pic_khuvuccho / 0.05), int(y_pic_khuvuccho / 0.05), 15, 15)
                    label.setStyleSheet(csshome)
                    label.setAlignment(Qt.AlignCenter)
                    self.view.scene().addWidget(label)
                elif idbann != 200 and idbann != 100:
                    label = QLabel(f"{int(idbann)}")

                    x_pic_ban = (x_tc + abs(self.x_home)) * self.ti_le
                    y_pic_ban = (self.pixmap.height() * 0.05 - (y_tc + abs(self.y_home))) * self.ti_le

                    label_name = f"{int(idbann)}"
                    setattr(self, label_name, label)
                    label.setGeometry(int(x_pic_ban / 0.05), int(y_pic_ban / 0.05), 10, 10)
                    label.setStyleSheet(css3)
                    label.setAlignment(Qt.AlignCenter)
                    self.view.scene().addWidget(label)

        # --------------------------------- vi tri ROBOT ----------------------------
        self.x, self.y, self.z, self.w = self.ros2_handle.odom_listener.data_odom
        print("zxxxxx: ")
        print(self.x)
        x_pic_robot = (float(self.x) + abs(self.x_home)) * self.ti_le
        y_pic_robot = (self.pixmap.height() * 0.05 - (float(self.y) + abs(self.y_home))) * self.ti_le

        # sửa đổi
        label = QLabel()
        label_name = "robot"
        pixmap_cho = QPixmap(Uti.image_path('icon_robot_map.jpg'))
        label.setScaledContents(True)
        label.setPixmap(pixmap_cho)

        setattr(self, label_name, label)
        label.setGeometry(int(x_pic_robot / 0.05), int(y_pic_robot / 0.05), 15, 15)
        label.setStyleSheet(csshome)
        label.setAlignment(Qt.AlignCenter)
        self.view.scene().addWidget(label)

        # --------------------------------- vi tri HOME --------------------------
        x_pic_home = (0 + abs(self.x_home)) * self.ti_le
        y_pic_home = (self.pixmap.height() * 0.05 - (0 + abs(self.y_home))) * self.ti_le
        print('x: ', x_pic_home)
        print('y: ', y_pic_home)

        # sửa đổi
        label = QLabel()
        label_name = "home_00"
        pixmap_cho = QPixmap(Uti.image_path('home_position.jpg'))
        label.setScaledContents(True)
        label.setPixmap(pixmap_cho)

        setattr(self, label_name, label)
        label.setGeometry(int(x_pic_home / 0.05), int(y_pic_home / 0.05), 15, 15)
        label.setStyleSheet(csshome)
        label.setAlignment(Qt.AlignCenter)
        self.view.scene().addWidget(label)

################## PIN ##################

    def trangthai_pin(self, status):
        if status == "A":
            print("da nhan A")
        else:
          #  if self.on_off_sac_tu_dong== True:
                if not self.navigation_thread.robot_dang_di_chuyen:
                    print("--------------------gia tri dien ap----------------------: ", status)
                    if status != "B":
                        try:
                            parts = status.split()  # Tách chuỗi thành danh sách các phần tử
                            value = float(parts[0])  # Lấy phần tử đầu tiên và chuyển đổi
                            if value < 10.6:
                                print("Giá trị điện áp nhỏ hơn 10.8 :", value)
                             
                                print("Thiếu điện")
                            #    Uti.RobotSpeakWithPath('voice_hmi_new/thongbao_pin_yeu.wav')
                                time.sleep(2)
                                self.navigation_thread.du_pin = False
                                self.read_arduino.doc_pin.disconnect(self.trangthai_pin)
                                self.read_arduino.stop()
                           #     Uti.RobotSpeakWithPath('voice_hmi_new/batdau_quatrinh_sac.wav')
                                time.sleep(2)
                             #   self.Auto_charing()
                            elif value > 10.6:
                                print("Giá trị điện áp lớn hơn 10.8 :", value)
                                print("Đủ điện")
                                time.sleep(2)
                            #    Uti.RobotSpeakWithPath('voice_hmi_new/thongbao_pin_day.wav')
                                self.navigation_thread.du_pin = True
                                self.read_arduino.doc_pin.disconnect(self.trangthai_pin)
                                self.read_arduino.stop()
                             #   self.ve_home_trong_chu_trinh()
                            else:
                                print("Giá trị điện áp bình thường :", value)
                                self.read_arduino.doc_pin.disconnect(self.trangthai_pin)
                                self.read_arduino.stop()
                        except (ValueError, IndexError): #Thêm IndexError đề phòng trường hợp parts không có index 0
                            print(f"Không thể chuyển đổi '{status}' thành số thực.")
                    else:
                        print("Status la B")
     #======================= ham on off che do sac tu dong khi het pin =======================
    def che_do_sac(self):
        self.on_off_sac_tu_dong = not self.on_off_sac_tu_dong
        if self.on_off_sac_tu_dong:
            self.btn_sac_auto.setStyleSheet("background-color: rgb(0,255,0);border-radius:20px;")
            Uti.RobotSpeakWithPath('voice_hmi_new/bat_chedosac.wav')
             # time.sleep(1)
           
        else:
            self.btn_sac_auto.setStyleSheet("background-color: rgb(255,255,255);border-radius:20px;")
            Uti.RobotSpeakWithPath('voice_hmi_new/tat_chedosac.wav')
            #time.sleep(1)
        print("---- dao trang thai che do sac -----") 
        
        self.ham_chay_sac_tu_dong()
    def ham_chay_sac_tu_dong(self):
        if(self.on_off_sac_tu_dong == True):
              self.read_arduino.start()
           #   self.read_arduino.doc_pin.connect(self.trangthai_pin)
              if not self.navigation_thread.robot_dang_di_chuyen:     
                if(self.navigation_thread.du_pin == False):
                    Uti.RobotSpeakWithPath('voice_hmi_new/thongbao_pin_yeu.wav')
                    self.read_arduino.stop()

                    print("chay ve tram sac")
                    self.sac_pin_tu_dong = True
                    self.stackedWidget.setCurrentWidget(self.page_robot_dichuyen)
                    self.man_hinh_dan_duong()

                elif(self.navigation_thread.du_pin == True): 
                    Uti.RobotSpeakWithPath('voice_hmi_new/thongbao_pin_day.wav')
                    self.read_arduino.stop()
                    print("chay ve home")
                    self.sac_pin_tu_dong = False
                    self.stackedWidget.setCurrentWidget(self.page_robot_dichuyen)
                    self.man_hinh_dan_duong()
################## Dinh vi C  ##################              
 # SETUP DINH VI 
 ## KIEM TRA NUT NHAN PRESS
    def enable_editing(self, enable):
        xuly_cham_ngoai()
        # Kích hoạt hoặc vô hiệu hóa các QLineEdit
        self.toado_x.setEnabled(enable)
        self.toado_y.setEnabled(enable)
        self.toado_z.setEnabled(enable)
     
        # Update visual state to match the enabled/disabled state
        if enable:
            # Optional: Change background color when enabled
            self.toado_x.setStyleSheet("background-color: white;")
            self.toado_y.setStyleSheet("background-color: white;")
            self.toado_z.setStyleSheet("background-color: white;")
     
     
        else:
            # Optional: Change background color when disabled
            self.toado_x.setStyleSheet("background-color: lightgray;")
            self.toado_y.setStyleSheet("background-color: lightgray;")
            self.toado_z.setStyleSheet("background-color: lightgray;")
 ## HAM NUT NHAN CHO PHÉP NHAP TOA DO X Y Z
    def fcn_enable_type(self):
        # Kiểm tra trạng thái hiện tại của các QLineEdit
        current_state = self.toado_x.isEnabled()
        # Chuyển trạng thái của các QLineEdit
        self.enable_editing(not current_state)
         # Update button text to reflect current state
        if not current_state:  
          
            print("Cho Phép Nhập")
            self.btn_toado_td.setEnabled(True)
            xuly_cham_ngoai()
            self.toado_x.mousePressEvent = self.xu_ly_nhap_banphim
            self.toado_y.mousePressEvent = self.xu_ly_nhap_banphim
            self.toado_z.mousePressEvent = self.xu_ly_nhap_banphim
        else:  
            print("Khóa Nhập")
            self.btn_toado_td.setEnabled(False)
    def fcn_nhap_toado_td(self):
      # self.toado_x.setText() = self.ros2_handle.odom_listener.data_odom[0]
    #   self.toado_y = self.ros2_handle.odom_listener.data_odom[1]
      # self.toado_z = (self.ros2_handle.odom_listener.data_odom[3]*180.0)/math.pi
       self.toado_x.setText(str(round(self.ros2_handle.odom_listener.data_odom[0], 3)))
       self.toado_y.setText(str(round(self.ros2_handle.odom_listener.data_odom[1], 3)))
       self.toado_z.setText(str(round(((self.ros2_handle.odom_listener.data_odom[3]*180.0)/math.pi),3)))
       print("Cap nhat toa do dinh vi")
       print("X:"+str(self.ros2_handle.odom_listener.data_odom[0]))
       print("Y:"+str(self.ros2_handle.odom_listener.data_odom[1]))
       print("Z:"+str((self.ros2_handle.odom_listener.data_odom[3]*180.0)/math.pi))
     # HAM DUNG DE SAVE VI TRI SAU KHI NHAP
    def fcn_save_vitri(self):
        try:
           
            toa_do_x = float(self.toado_x.text())
          #  toa_do_x = float(self.toado_x)
        except ValueError:
            self.terminal_2.setPlainText("Error: toado_x is not a valid number")
            return

        try:
            toa_do_y = float(self.toado_y.text())
           # toa_do_y = float(self.toado_y)
        except ValueError:
            self.terminal_2.setPlainText("Error: toado_y is not a valid number")
            return

        try:
            toa_do_z = float(self.toado_z.text())
           # toa_do_z = float(self.toado_z)
        except ValueError:
            self.terminal_2.setPlainText("Error: toado_z is not a valid number")
            return
        try:
            id_vi_tri = float(self.id_vitri.currentText())
        except ValueError:
            self.terminal_2.setPlainText("Error: id_vitri is not a valid number")
            return
       
        self.setup_dv.add_data(toa_do_x, toa_do_y, toa_do_z, id_vi_tri, self.capC, self.cap_qr)

       # self.speak_test("Vị trí đã được lưu")
        self.check_and_opencamera()
        self.fcn_load_data()
        self.terminal_2.setPlainText("Data saved successfully")

    # HAM DUNG DE SET CHIEU RONG CUA COT
    def set_column_widths(self):
        self.tableWidget.setColumnWidth(0, 85)  # Cột checkbox
        self.tableWidget.setColumnWidth(1, 180)  # Cột Image Name
        self.tableWidget.setColumnWidth(2, 100)  # Cột X
        self.tableWidget.setColumnWidth(3, 100)  # Cột Y
        self.tableWidget.setColumnWidth(4, 100)  # Cột Z
        self.tableWidget.setColumnWidth(5, 100)  # Cột ID
    # SET CHIEU CAO CUA HANG
    def set_row_heights(self):
        for row in range(self.tableWidget.rowCount()):
            self.tableWidget.setRowHeight(row, 30)  # Chiều cao mỗi hàng là 30 pixel

    # HAM LOAD DATA LEN BANG
    def fcn_load_data(self):
        self.setup_dv.array_storted_data()
        data = self.setup_dv.load_images()
        num_rows = len(data)
        num_columns = 6  # 5 dữ liệu và 1 checkbox
        
        self.tableWidget.setRowCount(num_rows)
        self.tableWidget.setColumnCount(num_columns)  
        self.tableWidget.setHorizontalHeaderLabels(['Select', 'Image Name', 'X', 'Y', 'Z', 'ID'])

        for row_idx, row_data in enumerate(data):
            # Tạo checkbox cho từng hàng và căn giữa
            checkbox = QCheckBox()
            checkbox_widget = QWidget()
            layout = QHBoxLayout(checkbox_widget)
            layout.addWidget(checkbox)
            layout.setAlignment(Qt.AlignCenter)
            layout.setContentsMargins(0, 0, 0, 0)
            checkbox_widget.setLayout(layout)

            # Kết nối tín hiệu checkbox với hàm xử lý
            checkbox.stateChanged.connect(self.get_selected_rows)

            # Thêm widget checkbox vào bảng
            self.tableWidget.setCellWidget(row_idx, 0, checkbox_widget)

            # Thêm dữ liệu vào các cột còn lại và căn giữa
            for col_idx, item in enumerate(row_data):
                item_widget = QTableWidgetItem(str(item))
                item_widget.setTextAlignment(Qt.AlignCenter)
                self.tableWidget.setItem(row_idx, col_idx + 1, item_widget)

        self.set_column_widths()
        self.set_row_heights()

        self.terminal_2.setPlainText("Data loaded successfully")

    # HAM CHON CHECKBOX

    def get_selected_rows(self):
        self.selected_rows = []
        num_rows = self.tableWidget.rowCount()
        for row in range(num_rows):
            # Lấy widget checkbox từ ô
            checkbox_widget = self.tableWidget.cellWidget(row, 0)
            if checkbox_widget and isinstance(checkbox_widget, QWidget):
                checkbox = checkbox_widget.findChild(QCheckBox)
                if checkbox and checkbox.isChecked():
                    # Thêm chỉ số hàng vào mảng, +1 để chỉ số bắt đầu từ 1
                    self.selected_rows.append(row + 1)
        
        print("Selected rows:", self.selected_rows)
        return self.selected_rows
    # HAM XOA DU LIEU
    def fcn_delete_data(self):

        for i in range(len(self.selected_rows)):
            self.setup_dv.delete_data(self.selected_rows[i])

        self.fcn_load_data()
        self.terminal.clear()
        self.selected_rows = []
     # HAM TEST

    def fcn_test_data(self):
        start_time = time.time()
        self.terminal_2.setPlainText("Testing ...")
        #self.speak_test("Đang kiểm tra vị trí")
        
        #rdeg, pdeg, yawdeg = Uti.quaternion_to_euler(self.ros2_handle.odom_listener.data_odom[0],self.ros2_handle.odom_listener.data_odom[1], self.ros2_handle.odom_listener.data_odom[2], self.ros2_handle.odom_listener.data_odom[3])
       # roll, pitch, yawdeg = self.euler_from_quaternion([0,0, self.ros2_handle.odom_listener.data_odom[2], self.ros2_handle.odom_listener.data_odom[3]])
        #print("R"+str(roll))
        #print("P"+str(pitch))
        #print("Y"+str(yawdeg))
        #yawdeg = yawdeg*180/math.pi
        yawdeg  =round((self.ros2_handle.odom_listener.data_odom[3]*180.0)/math.pi,3)
        print(yawdeg)
        result  = self.setup_dv.test_image(200, yawdeg, self.capC,  self.cap_qr)
        result_text = ""
        for idx, (coords, similarity) in enumerate(result[:1]):
            result_text += f"({coords[0]}, {coords[1]}, {coords[2]}, {coords[3]}) "
            result_text += f"{similarity:.6f} \n"

        print(result_text)
           
        end_time = time.time() - start_time
        print("THOI GIAN THUC THI", end_time)
        self.terminal.setPlainText(result_text)
        #self.sub_win1.uic.terminal_2.setPlainText("complete the test")
        #self.sub_win1.uic.terminal_2.setPlainText("time: "+str(end_time)+ " yaw: "+str(yawdeg))
        #self.sub_win1.uic.terminal_2.setPlainText(str(yawdeg))

        #self.cap_qr = cv2.VideoCapture(RobConf.faceCame1)
        #self.capC = cv2.VideoCapture(RobConf.headCamera)
        self.check_and_opencamera()

        self.terminal_2.setPlainText("time: "+str(round(end_time,3))+"s"+ " - yaw: "+str(yawdeg))


 # Chay DINH VI
    def dongThongbaoKTSetup(self, flag):
            print("done file:   ",flag)
            if flag == 1:
                self.from_setup_cam.close()
                Uti.RobotSpeakWithPath('voice_hmi_new/finish_setup_cam.wav')
                self.finish_setup_cam()
            elif flag == 2:
                print("=============DONE DINH VI ==============")
            #   self.from_dang_dinhvi.close()
                Uti.RobotSpeakWithPath('voice_hmi_new/finish_dinhvi.wav')
                self.finish_dinhvi()
            elif flag == 10:
                self.error_setup_cam()
            else:
                pass
    #======================== ham cua nut nhan DINH VI =======================
     #========================== cac ham GUI nut nhan DINH VI ========================================
    def dang_dinhvi(self):
        self.from_dang_dinhvi = QMainWindow()
        self.uic15 = Ui_Form_dang_dinhvi()
        self.uic15.setupUi(self.from_dang_dinhvi)
        self.from_dang_dinhvi.setGeometry(680, 240, 550, 250)
        self.from_dang_dinhvi.setStyleSheet("background-color:rgb(255, 255, 255);")
        self.from_dang_dinhvi.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.from_dang_dinhvi.show()

    def error_dinhvi(self):
        self.from_error_dinhvi = QMainWindow()
        self.uic16 = Ui_Form_error_dinhvi()
        self.uic16.setupUi(self.from_error_dinhvi)
        self.from_error_dinhvi.setGeometry(680, 240, 550, 250)
        self.from_error_dinhvi.setStyleSheet("background-color:rgb(255, 255, 255);")
        self.from_error_dinhvi.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.from_error_dinhvi.show()
        self.uic16.btn_error_gps.clicked.connect(self.close_error_dinhvi)

    def close_error_dinhvi(self):
        self.from_error_dinhvi.close()

    def finish_dinhvi(self):
        self.from_finish_dinhvi = QMainWindow()
        self.uic17 = Ui_Form_finish_dinhvi()
        self.uic17.setupUi(self.from_finish_dinhvi)
        self.from_finish_dinhvi.setGeometry(680, 240, 550, 250)
        self.from_finish_dinhvi.setStyleSheet("background-color:rgb(255, 255, 255);")
        self.from_finish_dinhvi.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.from_finish_dinhvi.show()
        self.uic17.btn_finish_gps.clicked.connect(self.close_finish_dinhvi)

    def close_finish_dinhvi(self):
        self.from_finish_dinhvi.close()

    #========================== cac ham xu ly cua nut nhan DINH VI ========================================
    def dinh_vi_fcn_1(self): 
            self.check_and_opencamera()       
            Uti.RobotSpeakWithPath('voice_hmi_new/dinhvi.wav')
            print('[dinh_vi_fcn_1]Dang dinh vi ')

          #  if self.dinh_vi_fcn():
           #     self.dongThongbaoKTSetup(2)
            #    self.from_dang_dinhvi.close()

            #else:
             #   print('[dinh_vi_fcn_1]Dinh vi that bai')
              #  Uti.RobotSpeakWithPath('voice_hmi_new/dinh_vi_that_bai.mp3')
      

################## Navigate thread  ##################
    def kiem_tra_du_dieu_khien_chay(self):
        self.read_arduino.start()
        self.read_arduino.doc_pin.connect(self.trangthai_pin)
        ## Kiem tra dieu khien pin
    #    print("du Pin")
   
        ## Kiem tra da nhap ten phong
        if( self.navigation_thread.du_pin==True):
            self.btn_batdau_danduong.setEnabled(True)
            self.read_arduino.stop()
            print("du Pin")
        else:
            self.btn_batdau_danduong.setEnabled(False)
            Uti.RobotSpeakWithPath('voice_hmi_new/vuilongsacpin.wav')
            self.read_arduino.stop()
            print("Thieu Pin")

    
    def button_xn_pressed(self):
           if(self.id_voice==200 or self.id_voice==123):
            self.stackedWidget.setCurrentWidget(self.page_main)
           else:
            pass

    def reached_goal(self, flag):
    
            if flag:
                self.btn_xac_nhan.setEnabled(True)
                if self.id_voice == RobConf.HOME_ID:
                    Uti.RobotSpeakWithPath('new_voice/new_hoan_thanh.wav')
                    
                else:
                    Uti.RobotSpeakWithPath('voice_hmi_new/new_xac_nhan.wav')
                    self.listWidget_ds.takeItem(0)
            else: 
                print("fail go")
                self.man_hinh_dan_duong()
  
    def ve_home_trong_chu_trinh(self):
        # B1 chay ve home
        x_goal, y_goal, z_goal, w_goal, idout = sql.doc_du_lieu_toado_robot(200)

        time.sleep(1)
        print('Go HOME')
        Uti.RobotSpeakWithPath('voice_hmi_new/ve_home.wav')
        self.navigation_thread.clear_done_navigation_status() #reset bien done_navigation

        self.navigation_thread.id =  idout#300

        self.navigation_thread.quatrinh_move = True

        self.ros2_handle.goal_publisher.send_goal(x_goal, y_goal, z_goal, w_goal)
        # B2 dinh vi 
        print("Robot dang dinh vi")

        print("da dinh vi xong")

        self.listWidget_ds.clear()
         # Đặt lại tất cả các trạng thái item về False
        for id_item in self.item_states:
            self.item_states[id_item] = False
            
        self.checkBox_kvc.setCheckState(Qt.Unchecked)

       
    def ve_dock_sac(self):

        x_goal, y_goal, z_goal, w_goal, idout = sql.doc_du_lieu_toado_robot(123)

        time.sleep(1)
        print('Go DOCK')
        Uti.RobotSpeakWithPath('voice_hmi_new/batdau_quatrinh_sac.wav')
        self.navigation_thread.clear_done_navigation_status() #reset bien done_navigation

        self.navigation_thread.id =  idout#300

        self.navigation_thread.quatrinh_move = True

        self.ros2_handle.goal_publisher.send_goal(x_goal, y_goal, z_goal, w_goal)
        Uti.RobotSpeakWithPath('voice_hmi_new/toidangdisac.wav')
        self.listWidget_ds.clear()
         # Đặt lại tất cả các trạng thái item về False
        for id_item in self.item_states:
            self.item_states[id_item] = False

    def auto_clear(self,id):
            x_goal, y_goal, z_goal, w_goal, idout = sql.doc_du_lieu_toado_robot(id)

            time.sleep(1)
           
            self.navigation_thread.clear_done_navigation_status() #reset bien done_navigation

            self.navigation_thread.id =  idout#300

            self.navigation_thread.quatrinh_move = True

            self.ros2_handle.goal_publisher.send_goal(x_goal, y_goal, z_goal, w_goal)

            # B4 Xoa phan tu dau tien 
            
            
            
    def man_hinh_dan_duong(self):
           
      #  if not self.arduino.batdau_sac:

            self.btn_xac_nhan.setEnabled(False)
            goals = [self.listWidget_ds.item(index).text() for index in range(self.listWidget_ds.count())]
            print('Goals: ', goals)
            self.goal_ids = []
            for goal in goals:
                if 'Phòng' in goal:
                    self.goal_ids.append(int(goal.split()[-1]))
                elif 'Khu vực chờ' in goal:
                    self.goal_ids.append(RobConf.HOME_ID)
                elif 'Trạm sạc' in goal:
                    self.goal_ids.append(RobConf.TruocDockSacID)
            print('Goal ids: ', self.goal_ids)
            if goals:  # kiểm tra danh sách có phần tử nào ko
                first_item = goals[0]
                self.label_tt.setText(first_item)  # label có ô vuông
            if not self.goal_ids:
                if(self.sac_pin_tu_dong==True):
                    self.label_tt.setText("Trạm sạc")
                    self.id_voice = 123
                    self.ve_dock_sac()

                else:
                    self.label_tt.setText("Khu vực chờ")
                    self.id_voice = 200
                    self.ve_home_trong_chu_trinh()  

            else:    
                Uti.RobotSpeakWithPath('voice_hmi_new/new_nhuong_duong.wav')
                
                if (self.goal_ids[0]==200):
                    self.id_voiced = 200
                    self.ve_home_trong_chu_trinh()
           
                elif(self.goal_ids[0]==123):
                    self.id_voiced = 123
                    self.ve_dock_sac()
                else:
                    self.id_voice = 0
                    self.auto_clear(self.goal_ids[0])
          
      #  else:
          #  Uti.RobotSpeakWithPath('voice_hmi_new/toidangdisac.wav')
          #  print('toi dang di sac')
            

                 
    
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()

    window.show()
    #window.showFullScreen()
    sys.exit(app.exec_())