#!/home/robot/robot_lib/bin/python3
#Library python
import os
import traceback
import tensorflow as tf
from openpyxl import Workbook
from datetime import datetime

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
from pyzbar.pyzbar import decode
import platform
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
import paho.mqtt.client as mqtt

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
# TRA CUU
import pandas as pd
import pyttsx3
import textwrap
import openpyxl
import google.generativeai as genai
from gtts import gTTS 
import speech_recognition as sr
from playsound3 import playsound  # Thay đổi import ở đây
#UI
import keyyyyy
from FILE_QT.QT_main import Ui_MainWindow 
from FILE_QT.login import Ui_MainWindow_login
from FILE_QT.ProgressBar_Bibot import Ui_MainWindow_bar
from FILE_QT.form_cho_giay_lat import Ui_Form_cho_giay_lat
from status_popup import StatusPopup, NamedMap
from FILE_QT.from_nhapphong import Ui_Form_nhaptenban
from FILE_QT.form_themvao import Ui_Form_them
from FILE_QT.form_map import Ui_Form_map
from FILE_QT.QT_nhapten_ID import Ui_Form_nhaptenID

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
from FILE_QT.from_xn_chuphinh import Ui_Form_xn_chupanh
from FILE_QT.from_tinh_trang_pin import Ui_Form_status_pin
from FILE_QT.from_trang_toa_do import Ui_Form_show_toado

from FILE_QT.from_trang_ID import Ui_Form_show_ID
from FILE_QT.form_dv import Ui_Form_ui_dv

from FILE_QT.page_waiting import Ui_UI_waiting

# FILE CUSTOM
from navigation_thread import NavigationThread
from arduino_connection import ReadArduinoPin, arduino
from ros2_handle import ROS2Handle
import ros2_handle as ros2_handle
from ros2_custom_nodes import GoalPosePublisher
import SQL as sql
from ModuleSetupDinhVi import SetupDinhVi
# Library GUI
from PyQt5 import QtCore, QtGui

from PyQt5.QtCore import *
from PyQt5.QtCore import QEvent, Qt, QThread, pyqtSignal,QTimer

from PyQt5.QtGui import *
from PyQt5.QtGui import QIcon

from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication,QMainWindow,QTableWidgetItem, QCheckBox, QMessageBox,QLineEdit, QVBoxLayout,QWidget

from PyQt5.QtGui import *
from PyQt5.QtGui import QMovie

from PyQt5.QtMultimedia import QAbstractVideoBuffer, QVideoFrame
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimedia import QAbstractVideoSurface, QVideoSurfaceFormat
from PyQt5.QtCore import QUrl, QRect
from PyQt5.QtGui import QPainter

import vlc
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
def phat_tieng_viet(text, out_file="out_file.mp3"):
        try:
            text = text.strip() # Ensures 'text' is a string and cleans whitespace
            
        #    print(f"du lieu:{text}")
          #  print(f"kieu:{type(text)}")
            if not text:
                print("Không có nội dung để phát.")
                return

            if pygame.mixer.get_init():
                if pygame.mixer.music.get_busy():
                    pygame.mixer.music.stop()
                pygame.mixer.quit()

            if os.path.exists(out_file):
                os.remove(out_file)

            # The core gTTS call for Vietnamese
            tts = gTTS(text=text, lang='vi')
            tts.save(out_file)
            print(f"Đã lưu file mp3: {out_file}")

            pygame.mixer.init()
            pygame.mixer.music.load(out_file)
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)

            pygame.mixer.quit()
            os.remove(out_file)

        except Exception as e:
            print(f"Lỗi phát âm: {e}")
class LoginScreen(QMainWindow):
    def __init__(self, loading_screen):
        super().__init__()
        self.loading_screen = loading_screen  # Lưu tham chiếu đến LoadingScreen
        self.ui_login = Ui_MainWindow_login()  # Khởi tạo giao diện login
        self.ui_login.setupUi(self)

        if hasattr(self.ui_login, 'btn_dn_login'):
            self.ui_login.btn_dn_login.clicked.connect(self.handle_login)
        else:
            print("Lỗi: Không tìm thấy nút btn_dn_login trong giao diện LoginScreen!")

        self.ui_login.Button_tat_login.clicked.connect(self.close_login)
        # self.ui_login.Button_tat_login.clicked.connect(self.ros2_handle.stop)
        self.ui_login.lineEdit_dangnhap.setText(RobConf.USERNAME_DF)
        self.ui_login.lineEdit_matkhau.setText(RobConf.PASSWORD_DF)
        self.ui_login.Button_tat_login.setEnabled(False)
        self.ui_login.Button_quenmk.clicked.connect(
            lambda: QMessageBox.information(self, "Chào bạn", "Liên hệ qua sđt 0942385474"))
        self.ui_login.lineEdit_matkhau.mousePressEvent = self.xu_ly_mk
        self.ui_login.lineEdit_dangnhap.mousePressEvent = self.xu_ly_dn
        self.ui_login.label_222.mousePressEvent = self.xu_ly_close

    def xu_ly_close(self, event):
        print("da an")
        self.check_acc()
        # xuly_cham_ngoai()  # Chú thích hoặc định nghĩa hàm này nếu cần

    def xu_ly_dn(self, event):
        print("dang nhap")
        self.ui_login.Button_tat_login.setEnabled(False)
        try:
            if not hasattr(keyyyyy.Registe, "virtual_keyboard") or not keyyyyy.Registe.virtual_keyboard.isVisible():
                keyyyyy.Registe.virtual_keyboard = keyyyyy.VKBD(self.ui_login.lineEdit_dangnhap)
                keyyyyy.Registe.virtual_keyboard.show()
            else:
                # keyyyyy.Registe.virtual_keyboard.activateWindow()
                keyyyyy.Registe.virtual_keyboard.hide()
        except Exception as e:
            print(f"failed to open virtual keyboard ,already exist or error: {e}")

    def xu_ly_mk(self, event):
        print("mat khau")
        try:
            if not hasattr(keyyyyy.Registe, "virtual_keyboard") or not keyyyyy.Registe.virtual_keyboard.isVisible():
                keyyyyy.Registe.virtual_keyboard = keyyyyy.VKBD(self.ui_login.lineEdit_matkhau)
                keyyyyy.Registe.virtual_keyboard.show()
            else:
                # keyyyyy.Registe.virtual_keyboard.activateWindow()
                keyyyyy.Registe.virtual_keyboard.hide()
        except Exception as e:
            print(f"failed to open virtual keyboard ,already exist or error: {e}")

    def check_acc(self):
        tendangnhap = self.ui_login.lineEdit_dangnhap.text()
        password = self.ui_login.lineEdit_matkhau.text()

        if (tendangnhap == RobConf.USERNAME) and (password == RobConf.PASSWORD):
            self.ui_login.Button_tat_login.setEnabled(True)
        else:
            self.ui_login.Button_tat_login.setEnabled(False)
            Uti.RobotSpeakWithPath('voice_hmi_new/dangnhapsai.wav')
            QMessageBox.information(self, "Chào bạn", "Tài khoản hoặc mật khẩu không đúng")
            self.ui_login.lineEdit_dangnhap.clear()
            self.ui_login.lineEdit_matkhau.clear()

    def show_login(self):
        self.showFullScreen()
        Uti.RobotSpeakWithPath('voice_hmi_new/dang_nhap.wav')

    def close_login(self):
        self.close()

    def handle_login(self):
        print("Đã nhấn nút đăng nhập!")
        tendangnhap = self.ui_login.lineEdit_dangnhap.text()
        password = self.ui_login.lineEdit_matkhau.text()

        if (tendangnhap == RobConf.USERNAME) and (password == RobConf.PASSWORD):
            self.ui_login.Button_tat_login.setEnabled(True)
            print("Đăng nhập thành công!")
            self.close_login()
            self.loading_screen.show_loading()
        else:
            self.ui_login.Button_tat_login.setEnabled(False)
            Uti.RobotSpeakWithPath('voice_hmi_new/dangnhapsai.wav')
            QMessageBox.information(self, "Chào bạn", "Tài khoản hoặc mật khẩu không đúng")
            self.ui_login.lineEdit_dangnhap.clear()
            self.ui_login.lineEdit_matkhau.clear()
class LoadingScreen(QMainWindow):
    def __init__(self, main_app):
        super().__init__()
        self.main_app = main_app
        self.ui_bar = Ui_MainWindow_bar()
        self.ui_bar.setupUi(self)
        self.progress_bar = self.ui_bar.progressBar_1
        self.a = 0
        self.running = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.process)

    def show_loading(self):
        self.progress_bar.setValue(0)
        self.a = 0
        self.running = True
        self.showFullScreen()
        Uti.RobotSpeakWithPath('voice_hmi_new/new_doi.wav')
        self.timer.start(50)

    def close_loading(self):
        self.running = False
        self.timer.stop()
        self.close()
        self.main_app.showFullScreen()
        Uti.RobotSpeakWithPath('voice_hmi_new/new_tinh_nang.wav')
    def process(self):
        if not self.running:
            return
        self.a += 1
        self.progress_bar.setValue(self.a)
        if self.progress_bar.value() == self.progress_bar.maximum():
            self.close_loading()

class MainApp(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super().__init__()
      # loading_screen = LoadingScreen(self)
       #loading_screen.show_loading()
      #
        self.setupUi(self) 
        self.navigation_thread = NavigationThread()
        self.ros2_handle = ROS2Handle()
        self.status_popup = StatusPopup()
        self.initialize_ros2_manager()

        #Process bar vs login 
        
        self.a = 0
        self.running = True
        
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
        self.value_pin =0.0
        self.phantram_pin = 0
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
        values_yaw = [0,45, 90, 135, 180, 225, 270, 315]
        self.id_vitri.addItems([str(i) for i in values])
        self.toado_z.addItems([str(i) for i in values_yaw])
        self.setup_dv  = SetupDinhVi()
        self.DINH_VI_BTN()

        # DIEM DANH
        self.CHECK_LIST_BTN()
        self.checkbox_checkin_2.stateChanged.connect(self.check_checkbox_state)
        self.checkbox_checkout_2.stateChanged.connect(self.check_checkbox_state)

        #Tra cuu
        GOOGLE_API_KEY = "AIzaSyCxp0wD3-6nZOKaRn_WUkvzwlHOKfw-hJw"
        if not GOOGLE_API_KEY:
            print("Lỗi: Vui lòng thiết lập API Key.")
            exit()
        genai.configure(api_key=GOOGLE_API_KEY)
        self.model = genai.GenerativeModel('gemini-2.5-flash-preview-04-17-thinking')
        self.tc_nha_truong_2.clear()  # Reset danh sách câu hỏi
        self.tra_loi_tc_nha_truong_4.clear()  # Reset chỗ hiển thị câu trả lời
     

        self.bt_tc_ai.clicked.connect(self.tra_cuu_ai)
        self.bt_kt_ai.clicked.connect(self.home_nha_truong)
        self.cau_hoi_ai.clear()
        self.tra_loi_ai.clear()
        self.bt_stop_ai.clicked.connect(self.reload_ai)
        self.bt_stop_ai.setEnabled(False)   
        self.tra_loi_ai.setWordWrap(True)  # Tự động ngắt dòng
        self.tra_loi_ai.setAlignment(Qt.AlignCenter)
        self.tt_ai = True
        self.gif_process = QMovie("../ROBOT_HD/Gif/writing-loading.gif")
        self.gif_talking = QMovie("../ROBOT_HD/Gif/sound.gif")
        self.label_gif.setMovie(self.gif_process)
        self.gif_process.start()
        #gan su kien 
        self.TRA_CUU_BTN()
        # trang thai ai 
        self.ai_dang_chay = False
        self.di_dendiem_DV = False

        self.speech_thread = None
       
        #Mobile
        self.broker_url = ""
        self.port = 0
        self.topic_subscribe_command = ""
        self.topic_publish_camera = ""
        self.username = ""
        self.password = ""
        self.client = None
        self.btn_bat_ketnoi_mobile.clicked.connect(self.start_mobile)
        self.btn_tat_ketnoi_mobile.clicked.connect(self.stop_mobile)
        self.btn_wait.clicked.connect(self.show_page_waiting)
        self.btn_toggle_camera_mobile.clicked.connect(self.toggle_camera_mobile)
        self.btn_toggle_camera_mobile.setEnabled(False)


        # Waiting screen 
        self.instance = None
        self.player = None
        self.page_waiting = None
        self.initVLC()  # Initialize VLC instance
        self.inactivity_timer = QTimer()
        self.inactivity_timer.setInterval(24000000)  # 5 giây
        self.inactivity_timer.setSingleShot(True)  # Bắn 1 lần rồi dừng
        self.inactivity_timer.timeout.connect(self.show_page_waiting)
        self.inactivity_timer.start()

        # Gắn event filter toàn cục
        QApplication.instance().installEventFilter(self)


       
    def show_main(self):
        self.show() # Hàm này được gọi từ LoadingScreen sau khi đóng
        
    def manhinh_chogiaylat(self):
        self.form_chogiaylat = QMainWindow()
        self.form_chogiaylat.showFullScreen()
        self.uic6 = Ui_Form_cho_giay_lat()
        # self.form_chogiaylat.setGeometry(500, 200, 600, 550)

        self.uic6.setupUi(self.form_chogiaylat)
        self.form_chogiaylat.show()

        # đặt giá trị cho thanh Bar
        self.uic6.progressBar_2.setValue(0)
        # gọi hàm program
        self.program_chogiaylat()
        self.b = 0
        self.running_s = True

    def program_chogiaylat(self):
        timer = QTimer(self)
        timer.timeout.connect(self.process_chogiaylat)
        # thời gian chạy hết thanh Bar là 10 mili giây
        timer.start(10)

    def process_chogiaylat(self):
        if not self.running_s:  # kiểm tra nếu biến running là False
            return  # thoát khỏi hàm process
        self.b += 1
        self.uic6.progressBar_2.setValue(self.b)
        # Kiểm tra hoàn thành (đạt 100%)
        if self.uic6.progressBar_2.value() == self.uic6.progressBar_2.maximum():
            # Hiển thị màn hình đăng nhập
            print("đã load xong")
            self.form_chogiaylat.close()
            # đưa biến running False
            self.running_s = False
#            self.hien_thi_hinh_anh()
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
        self.check_pin.clicked.connect(self.show_data_pin)
        self.bt_stop.clicked.connect(self.ros2_handle.cmd_vel_publisher.stop_movement)
        ## Back button
        self.bt_back_setup.clicked.connect(self.back_setup)
        self.bt_back_dichuyen_robot.clicked.connect(self.back_setup)
        self.bt_back_setup_ts_robot.clicked.connect(self.back_setup)
        self.bt_back_dan_duong.clicked.connect(self.back_setup)
        self.bt_back_setup_3.clicked.connect(self.back_setup)
        self.bt_back_setup_4.clicked.connect(self.back_setup)
        self.bt_back_setup_2.clicked.connect(self.back_setup)
        self.bt_back_dinhvi.clicked.connect(self.back_setup)
        self.bt_back_setup_5.clicked.connect(self.back_setup)
        self.bt_back_wifi.clicked.connect(self.back_setup)
        self.bt_back_setup_ts_robot_2.clicked.connect(self.back_setup)
       
        self.bt_back_tc_nha_truong_4.clicked.connect(self.back_setup)
        self.bt_back_mobile.clicked.connect(self.back_setup)
        #Setup button
        self.bt_setup.clicked.connect(self.show_page_setup)
        self.bt_setup_robot.clicked.connect(self.show_setup_ts_robot)
        self.bt_danhsach.clicked.connect(self.show_danhsach)
        self.btn_save_sql.clicked.connect(self.fcn_save_data_sql)
        self.btn_load_sql.clicked.connect(self.show_thong_so_robot)
        self.btn_resetall_sql.clicked.connect(self.fcn_resetall_sql)
        
        self.btn_save_sql_2.clicked.connect(self.fcn_save_data_sql)
        self.btn_load_sql_2.clicked.connect(self.show_thong_so_robot)
        self.btn_resetall_sql_2.clicked.connect(self.fcn_resetall_sql)
        
        self.bt_setup_dinh_vi.clicked.connect(self.show_page_setup_dinhvi)
        self.bt_wifi.clicked.connect(self.show_page_wifi)
        self.btn_kt_wifi.clicked.connect(self.kiem_tra_wifi)
    
        self.bt_set_home.clicked.connect(self.show_dichuyen_robot)
        self.btn_reset_pw.clicked.connect(self.fcn_reset_password)
        self.btn_save_pw.clicked.connect(self.fcn_save_password)

        self.bt_setup_tk.clicked.connect(self.show_page_setup_tk)
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
    
        self.Button_toado.clicked.connect(self.show_toado)
        self.mobile_button.clicked.connect(self.show_mobile)

        self.setName_button.clicked.connect(self.manhinh_nhap_ID)
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

        self.btn_ve_home_khancap.clicked.connect(self.ve_home_khan_cap)

        self.btn_them_bot.clicked.connect(self.xoaDS)
        self.btn_thu_tu.clicked.connect(self.sort_items)
    
    def DINH_VI_BTN(self):
        self.ht_chutrinh_dv = True
        self.btn_enable.clicked.connect(self.fcn_enable_type)
        self.btn_toado_td.clicked.connect(self.fcn_nhap_toado_td)
        self.btn_save_data.clicked.connect(self.fcn_save_vitri)
        self.btn_load_data.clicked.connect(self.fcn_load_data)
        self.btn_delete_data.clicked.connect(self.fcn_delete_data) 
        self.btn_delete_data_all.clicked.connect(self.fcn_delete_data_all) 
        self.btn_test.clicked.connect(self.fcn_test_data)
        self.btn_dinh_vi.clicked.connect(self.show_page_dinhvi)
        self.btn_bd_dinhvi.clicked.connect(self.dinh_vi_fcn_1)

    def CHECK_LIST_BTN(self):
        
        self.cap = None
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.chua_camera.setScaledContents(True)
        self.chua_camera_3.setText("Chưa quét được mã QR")
        self.btn_opencamera_2.setEnabled(False)
        self.camera_running = False
        self.qr_data_buffer = None  # Biến để lưu trữ dữ liệu QR đã quét
        self.last_qr_text = "Không tìm thấy mã QR" # Lưu trữ text QR cuối cùn
        self.chup_hinh = None
        self.ui20 = None
        self.image_save_path = None # Lưu đường dẫn ảnh sau khi chụp
        self.checkin_active = False
        self.checkout_active = False
        self.is_checkin_processed = False
        self.is_checkout_processed = False
        self.bt_diem_danh.clicked.connect(self.show_check_list)

        self.btn_opencamera_2.clicked.connect(self.toggle_camera)
        self.btn_load_ds.clicked.connect(self.fcn_load_list)
        self.btn_detele_all_ds.clicked.connect(self.fcn_delete_list_all)
        self.btn_export.clicked.connect(self.export_to_excel)
        self.btn_diemdanh_2.clicked.connect(self.process_and_add_qr_data)
        self.checkbox_checkin_2.toggled.connect(self.uncheck_checkout)
        self.checkbox_checkout_2.toggled.connect(self.uncheck_checkin)
        self.checkbox_checkin_2.toggled.connect(self.update_checkin_state)
        self.checkbox_checkout_2.toggled.connect(self.update_checkout_state)
        self.btn_showlist_2.clicked.connect(self.fcn_load_list_1)
    
    def TRA_CUU_BTN(self):
        self.bt_tra_cuu.clicked.connect(self.show_page_tracuu)
        self.tc_nha_truong_2.itemClicked.connect(self.on_question_clicked)
        self.bt_tc_cap_nhat_4.clicked.connect(self.load_excel)
     
    ### Page waiting


        
    def initVLC(self):
        """Initialize the VLC instance."""
        if self.instance is None:
            try:
                self.instance = vlc.Instance("--no-xlib")  #if this causes error, remove it
            except Exception as e:
                print(f"Error initializing VLC: {e}")
                self.instance = vlc.Instance() #use this instead

    def UI_page_waiting(self):
        self.page_waiting = QMainWindow()
        self.ui_waiting = Ui_UI_waiting()
        self.ui_waiting.setupUi(self.page_waiting)

        video_container = self.ui_waiting.videoContainer
        if not video_container:
            print("Error: videoContainer widget not found in UI.")
            return

        self.win_id = video_container.winId() #store win_id

        video_path = "video/Robot.mp4"
        if not os.path.exists(video_path):
            print(f"Error: Video file not found at {video_path}")
            return

        # Create VLC instance if needed
        if self.instance is None:
            self.instance = vlc.Instance()

        # Use MediaListPlayer for looping
        self.media_list_player = self.instance.media_list_player_new()
        self.media_player = self.instance.media_player_new()
        self.media_list_player.set_media_player(self.media_player)

        # Set output window
        if sys.platform.startswith('linux'):
            self.media_player.set_xwindow(int(self.win_id))
        elif sys.platform == 'win32':
            self.media_player.set_hwnd(int(self.win_id))
        elif sys.platform == 'darwin':
            self.media_player.set_nsobject(int(self.win_id))

        # Set media list with loop
        media_list = self.instance.media_list_new([video_path])
        self.media_list_player.set_media_list(media_list)
        self.media_list_player.set_playback_mode(vlc.PlaybackMode.loop)

        # Connect events to close window.  Use the video container.
        video_container.installEventFilter(self) # Install event filter
        self.media_list_player.play()
        self.page_waiting.showFullScreen()

    def show_page_waiting(self):
        if (self.stackedWidget.currentWidget() == self.page_ai) or (self.stackedWidget.currentWidget() == self.page_robot_dichuyen) or (self.stackedWidget.currentWidget() == self.page_check_list):
            print("Đang ở trang excution không vào trang chờ.")
            return
        self.UI_page_waiting()

    def close_page_waiting(self):
        if self.page_waiting:
            self.page_waiting.close()
            self.page_waiting = None # prevent error
            if self.media_player:
                self.media_player.stop() #stop the player
                self.media_player.release()
                self.media_player = None
            if self.media_list_player:
                self.media_list_player.release()
                self.media_list_player = None

    def close_page_waiting_event(self, event): # Removed event argument.  Not used
        self.close_page_waiting()

    def eventFilter(self, obj, event):
        if event.type() in [
            QEvent.MouseButtonPress, QEvent.MouseMove, QEvent.KeyPress,
            QEvent.TouchBegin, QEvent.TouchUpdate, QEvent.TabletPress, QEvent.TabletMove
        ]:
            # Nếu trang chờ đang mở → đóng và chặn sự kiện (không cho lan tiếp)
            if self.page_waiting is not None:
                self.close_page_waiting()
                self.inactivity_timer.start()
                return True

            # Trang chờ chưa mở → reset timer nhưng cho sự kiện lan tiếp (để nút nhận được)
            self.inactivity_timer.start()
            return False

        return super().eventFilter(obj, event)

################### Page Setup ##################

    def back_setup(self):
        self.stackedWidget.setCurrentWidget(self.page_main)
      #  Uti.RobotSpeakWithPath('voice_hmi_new/new_tinh_nang.wav')
    def Go_to_main(self):
        self.tabWidget_main = self.findChild(QTabWidget, "tab_main")
        Uti.RobotSpeakWithPath('voice_hmi_new/new_tinh_nang.wav')
    def show_setup_ts_robot(self):
        self.stackedWidget.setCurrentWidget(self.page_setup_ts_robot)
    def show_dichuyen_robot(self):
        self.stackedWidget.setCurrentWidget(self.page_dichuyen_robot)
    def show_page_setup_cam(self):
        self.stackedWidget.setCurrentWidget(self.page_setup_cam)
        Uti.RobotSpeakWithPath('voice_hmi_new/setup_cam.wav')
    def show_page_setup_dinhvi(self):
        self.stackedWidget.setCurrentWidget(self.page_setup_dinhvi)
    def show_page_setup_tk(self):
        self.stackedWidget.setCurrentWidget(self.page_setup_tk)
    #Page ros
    def show_page_mode(self):
        self.stackedWidget.setCurrentWidget(self.page_mode)
        Uti.RobotSpeakWithPath('voice_hmi_new/new_che_do_di_chuyen.wav')
    def show_dan_duong(self):
        self.stackedWidget.setCurrentWidget(self.page_dan_duong)
        Uti.RobotSpeakWithPath('voice_hmi_new/new_vi_tri_di_chuyen.wav')
        self.cap_nhat_ten_nut_theo_db()
        self.fcn_load_list_0()
    
    def show_page_setup(self):
        self.stackedWidget.setCurrentWidget(self.page_setup)
        self.show_thong_so_robot()
    def show_robot_dichuyen(self):
        self.stackedWidget.setCurrentWidget(self.page_robot_dichuyen)
    def dan_duong(self):
        self.stackedWidget.setCurrentWidget(self.page_robot_dichuyen)
        self.man_hinh_dan_duong()
    def show_page_dinhvi(self):
        
        self.stackedWidget.setCurrentWidget(self.page_dinhvi)
    def show_check_list(self):
        self.stackedWidget.setCurrentWidget(self.page_check_list)
       
    def show_danhsach(self):
        self.stackedWidget.setCurrentWidget(self.page_danhsach)

    def show_page_wifi(self):
        self.stackedWidget.setCurrentWidget(self.page_wifi)
    def show_page_tracuu(self):
        self.stackedWidget.setCurrentWidget(self.page_tra_cuu)
    def show_mobile(self):
        self.stackedWidget.setCurrentWidget(self.page_mobile)
        
    # Funtion event 

    def toggleFullScreen(self):
        """ Bật hoặc tắt chế độ toàn màn hình """
        if self.isFullScreen():
            self.showNormal()  # Thoát toàn màn hình
        else:
            self.showFullScreen()  # Bật toàn màn hình
    def show_data_pin(self):
        self.UI_xn_pin()

    def UI_xn_pin(self):
        self.tinhtrang_pin = QMainWindow()
        self.ui21 = Ui_Form_status_pin()
        self.ui21.setupUi(self.tinhtrang_pin)
        self.ui21.btn_xn_pin.clicked.connect(self.close_ui_pin)
        self.ui21.btn_sac_auto.clicked.connect(self.che_do_sac)
        self.ui21.btn_sac_auto.clicked.connect(self.close_ui_pin)
        self.tinhtrang_pin.show()
        self.show_pin()
    def close_ui_pin(self):
        self.tinhtrang_pin.close()
    def show_pin(self):
        self.read_arduino.start()
        self.read_arduino.doc_pin.connect(self.trangthai_pin)


        print(f"{self.value_pin}")
        if self.value_pin  < 10.6:
            self.ui21.label_thongbao_pin.setText("Pin yếu vui lòng sạc robot")
            self.ui21.btn_xn_pin.setIcon(QIcon("/home/robot/ROBOT_HD/images/low-battery_12193220.png"))
        else :
            self.ui21.label_thongbao_pin.setText("Pin đủ cho robot")
            self.ui21.btn_xn_pin.setIcon(QIcon("/home/robot/ROBOT_HD/images/full-battery_12193313.png"))
        self.phantram_pin = max(0, min(100, int((self.value_pin - 10.3) / 2.0 * 100)))

        self.ui21.label_phantram_pin.setText(f"Percent:{self.phantram_pin}% \nVoltage:{self.value_pin}")

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
    def kiem_tra_wifi(self):
        """Kiểm tra trạng thái kết nối WiFi và tên mạng (SSID)."""

        os_name = platform.system()

        if os_name == "Windows":
            try:
                output = subprocess.check_output(["netsh", "wlan", "show", "interfaces"], text=True, encoding='utf-8')
                for line in output.splitlines():
                    if "State" in line and "Connected" in line:
                        print("Trạng thái kết nối: Đang kết nối")
                        for sub_line in output.splitlines():
                            if "SSID" in sub_line:
                                ssid = sub_line.split(":")[1].strip()
                                print(f"Tên WiFi: {ssid}")
                                return
                print("Trạng thái kết nối: Không kết nối")
            except subprocess.CalledProcessError:
                print("Lỗi khi kiểm tra thông tin WiFi (Windows).")
                print("Đảm bảo bạn chạy script với quyền quản trị viên.")
            except FileNotFoundError:
                print("Lệnh 'netsh' không tìm thấy (Windows).")

        elif os_name == "Linux":
            try:
                output = subprocess.check_output(["iwgetid", "-r"], text=True, encoding='utf-8').strip()
                if output:
                    print("Trạng thái kết nối: Đang kết nối")
                    print(f"Tên WiFi: {output}")
                    self.label_status_2.setText(f"Trạng thái kết nối: Đang kết nối\nTên WiFi: {output}")
                else:
                    print("Trạng thái kết nối: Không kết nối")
                    self.label_status_2.setText("Trạng thái kết nối: Không kết nối")
            except subprocess.CalledProcessError:
                print("Trạng thái kết nối: Không kết nối")
                self.label_status_2.setText("Trạng thái kết nối: Không kết nối")
            except FileNotFoundError:
                print("Lệnh 'iwgetid' không tìm thấy (Linux).")
                print("Hãy đảm bảo bạn đã cài đặt 'wireless-tools'.")
                self.label_status_2.setText("Lệnh 'iwgetid' không tìm thấy (Linux). \n Hãy đảm bảo bạn đã cài đặt 'wireless-tools'. ")

            # Thử cách khác cho Linux (có thể cần điều chỉnh interface)
           
           # try:
            #    output = subprocess.check_output(["nmcli", "c", "show", "--active"], text=True, encoding='utf-8')
             #   lines = output.splitlines()
              #  if len(lines) > 1:
               #     ssid_line = lines[1].split()[0]
                #    if ssid_line != "--":
                #        print("Trạng thái kết nối: Đang kết nối")
                #        print(f"Tên WiFi: {ssid_line}")
                #        return
              #  print("Trạng thái kết nối: Không kết nối")
            #except subprocess.CalledProcessError:
             #   print("Lỗi khi kiểm tra thông tin WiFi bằng 'nmcli' (Linux).")
            #except FileNotFoundError:
             #   print("Lệnh 'nmcli' không tìm thấy (Linux).")
              #  print("Hãy đảm bảo bạn đã cài đặt 'network-manager'.")

        elif os_name == "Darwin": # macOS
            try:
                output = subprocess.check_output(["/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport", "-I"], text=True, encoding='utf-8')
                ssid = None
                state = "Không kết nối"
                for line in output.splitlines():
                    if "SSID:" in line:
                        ssid = line.split(":")[1].strip()
                        state = "Đang kết nối"
                        break
                print(f"Trạng thái kết nối: {state}")
                if ssid:
                    print(f"Tên WiFi: {ssid}")
            except subprocess.CalledProcessError:
                print("Lỗi khi kiểm tra thông tin WiFi (macOS).")
            except FileNotFoundError:
                print("Lệnh 'airport' không tìm thấy (macOS).")

        else:
            print(f"Hệ điều hành '{os_name}' không được hỗ trợ.")

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

        self.textbox_confirm_pw.setEnabled(False)
        self.textbox_password.setEnabled(False)
        self.textbox_username.setEnabled(False)
        self.textbox_username.setStyleSheet("background-color: rgb(235, 235, 235);")  # Màu xám
        self.textbox_confirm_pw.setStyleSheet("background-color: rgb(235, 235, 235);")  #Màu xám
        self.textbox_password.setStyleSheet("background-color: rgb(235, 235, 235);")  #caidatrobot

    def fcn_reset_password(self):
        # Define the correct password

        correct_password = self.password_global
        max_attempts = 3
        attempts = 0
        
        while attempts < max_attempts:
            # Open an input dialog to get the current password from the user
            input_dialog = QInputDialog(self)
            input_dialog.setWindowTitle("Reset Password")
            input_dialog.setLabelText("Enter current password:")
            input_dialog.setModal(True)  # Make the dialog modal so it blocks interaction with other windows
            input_dialog.setFixedSize(500, 300)  # Set the size of the dialog
            
            if input_dialog.exec_() == QDialog.Accepted:
                text = input_dialog.textValue()
                # Check if the entered password is correct
                if text == correct_password:
                    #QMessageBox.information(self, "Success", "Password is correct.")
                    self.display_thanhcong("Mật khẩu chính xác")
                    # Enable the desired UI elements
                    self.textbox_confirm_pw.setEnabled(True)
                    self.textbox_password.setEnabled(True)
                    self.textbox_username.setEnabled(True)
                    self.textbox_username.setStyleSheet("background-color: rgb(255, 255, 255);")  # Màu xám
                    self.textbox_confirm_pw.setStyleSheet("background-color: rgb(255, 255, 255);")  # Màu xám
                    self.textbox_password.setStyleSheet("background-color: rgb(255, 255, 255);")  # Màu xám
                    return True
                else:
                    attempts += 1
                    remaining_attempts = max_attempts - attempts
                    if remaining_attempts > 0:
                        QMessageBox.warning(self, "Error", f"Mật khẩu không đúng. Bạn còn {remaining_attempts} lần")
                    else:
                        QMessageBox.warning(self, "Error", "Bạn đã vượt quá số lần thử")
                        return False
            else:
                # Handle the case where the user cancels or closes the dialog
                QMessageBox.warning(self, "Error", "Password reset was cancelled.")
                return False
        
        return False
    def fcn_save_password(self):
        try:
            username_temp = self.textbox_username.toPlainText().strip()
            
            # Kiểm tra xem username có chứa text hay không
            if not username_temp:
                raise ValueError("Username is empty")  # Tạo lỗi nếu username rỗng

        except ValueError as e:
            self.display_thatbai("Error!")
            return

        try:
            password_temp = self.textbox_password.toPlainText()
        # Kiểm tra xem username có chứa text hay không
            if not password_temp:
                raise ValueError("New password is empty.")  # Tạo lỗi nếu username rỗng

        except ValueError as e:
            self.display_thatbai(e)
            return
        
        try:
            confirm_pw = self.textbox_confirm_pw.toPlainText()
            if not confirm_pw:
                raise ValueError("Confirm password is empty.")  # Tạo lỗi nếu username rỗng

        except ValueError as e:
            self.display_thatbai(e)
            return

        if password_temp == confirm_pw:
            
            self.username_global = username_temp
            self.password_global = password_temp

            Uti.setupRobot_sql_save(
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

            # Đối với QPlainTextEdit
            self.textbox_username.setStyleSheet("background-color: rgb(235, 235, 235);")  # Màu xám
            self.textbox_confirm_pw.setStyleSheet("background-color: rgb(235, 235, 235);")  # Màu xám
            self.textbox_password.setStyleSheet("background-color: rgb(235, 235, 235);")  # Màu xám
            self.textbox_username.clear()
            self.textbox_password.clear()
            self.textbox_confirm_pw.clear()
            self.textbox_confirm_pw.setEnabled(False)
            self.textbox_password.setEnabled(False)
            self.textbox_username.setEnabled(False)
            self.display_thanhcong("Mật khẩu đã được cập nhật")

        else:
          
            self.display_thatbai("Error: mật khẩu xác nhận sai")

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
            print(f"{vxt_auto_max_text}")
            self.VX_AUTO_MAX = float(vxt_auto_max_text) if vxt_auto_max_text else None
            self.validate_input(self.VX_AUTO_MAX, RobConf.AUTO_VX_MIN, RobConf.AUTO_VX_MAX, "[Auto] vận tốc thẳng.")

            vw_auto_max_text = self.textbox_vtx.toPlainText()
            print(f"{vw_auto_max_text}")
            self.VW_AUTO_MAX = float(vw_auto_max_text) if vw_auto_max_text else None
            self.validate_input(self.VW_AUTO_MAX, RobConf.AUTO_VW_MIN, RobConf.AUTO_VW_MAX, "[Auto] vận tốc xoay.")

            vx_manual_max_text = self.textbox_vtt_2.toPlainText()
            print(f"{vx_manual_max_text}")
            self.VX_MANUAL_MAX = float(vx_manual_max_text) if vx_manual_max_text else None
            self.validate_input(self.VX_MANUAL_MAX, RobConf.MANUAL_VX_MIN, RobConf.MANUAL_VX_MAX, "[Manual] vận tốc thẳng.")

            vw_manual_max_text = self.textbox_vtx_2.toPlainText()
            print(f"{vw_manual_max_text}")
            self.VW_MANUAL_MAX = float(vw_manual_max_text) if vw_manual_max_text else None
            self.validate_input(self.VW_MANUAL_MAX, RobConf.MANUAL_VW_MIN, RobConf.MANUAL_VW_MAX, "[Manual] vận tốc xoay.")

            dung_sai_diem_den_vitri_text = self.textbox_sskc.toPlainText()
            print(f"{dung_sai_diem_den_vitri_text}")
            self.DUNG_SAI_DIEM_DEN_VITRI = float(dung_sai_diem_den_vitri_text) if dung_sai_diem_den_vitri_text else None
            self.validate_input(self.DUNG_SAI_DIEM_DEN_VITRI, 0, RobConf.DUNG_SAI_KC_MAX, "Sai số khoảng cách.")

            dung_sai_diem_den_goc_text = self.textbox_ssgx.toPlainText()
           # print(f"{dung_sai_diem_den_goc_text}")
            self.DUNG_SAI_DIEM_DEN_GOC = float(dung_sai_diem_den_goc_text) if dung_sai_diem_den_goc_text else None
            self.validate_input(self.DUNG_SAI_DIEM_DEN_GOC, 0, RobConf.DUNG_SAI_GOC_MAX, "Sai số góc xoay.")

            chieu_cao_led_text = self.textbox_ccdl.toPlainText()
            print(f"{chieu_cao_led_text}")
            self.CHIEU_CAO_LED = float(chieu_cao_led_text) if chieu_cao_led_text else None
            self.validate_input(self.CHIEU_CAO_LED, RobConf.CHIEU_CAO_LED_MIN, RobConf.CHIEU_CAO_LED_MAX, "Chiều cao đèn LED.")

            muc_dien_ap_sac_pin_text = self.textbox_pin_sac_auto.toPlainText()
            print(f"{muc_dien_ap_sac_pin_text}")
            self.MUC_DIEN_AP_SAC_PIN = float(muc_dien_ap_sac_pin_text) if muc_dien_ap_sac_pin_text else None
            self.validate_input(self.MUC_DIEN_AP_SAC_PIN, RobConf.MUC_DIEN_AP_SAC_MIN, RobConf.MUC_DIEN_AP_SAC_MAX, "Mức pin sạc tự động.")
          
            delay_time = self.textbox_delay_time.toPlainText()
            print(f"{delay_time}")

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
            self.username_global = RobConf.USERNAME_DF 
            self.password_global = RobConf.PASSWORD_DF 
            
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
        self.fcn_load_data()
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

        start_time = time.time()

        yawdeg  = round((self.ros2_handle.odom_listener.data_odom[3]*180.0)/math.pi,3)
        print(yawdeg)
        result  = self.setup_dv.test_image(200, yawdeg, self.capC,  self.cap_qr)
        result_text = ""
        for idx, (coords, similarity) in enumerate(result[:1]):
            result_text += f"({coords[0]}, {coords[1]}, {coords[2]}, {coords[3]}) "
            result_text += f"{similarity:.6f} \n"

        print(result_text)
           
        end_time = time.time() - start_time
        print("THOI GIAN THUC THI", end_time)

        self.check_and_opencamera()



        #---- giao diện thông báo ------
        if similarity > 0.5:
            self.finish_cam()
            Uti.RobotSpeakWithPath('voice_hmi_new/finish_check_cam.wav')
        else:
            print(f"Không xác định được vị trí")
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
################### MOBILE ##################
    def start_mobile(self):
        self.broker_url = "c69aeca2d48441618b65f77f38e2d8dc.s1.eu.hivemq.cloud"
        self.port = 8883
        self.topic_subscribe_command = "command/car"  # Topic nhận lệnh
        self.topic_publish_camera = "camera/stream"    # Topic gửi hình ảnh

        # Thông tin đăng nhập
        self.username = "hivemq.webclient.1746280264795"
        self.password = "ay;Z9W0$L1k7fbS!xH?C"

        # Tạo client MQTT
        self.client = mqtt.Client()
        self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe

        # Thiết lập kết nối TLS
        self.client.tls_set()

        # Thực hiện kết nối (non-blocking)
        self.client.connect_async(self.broker_url, self.port, 60)

        # Bắt đầu vòng lặp MQTT để xử lý kết nối và tin nhắn
        self.client.loop_start()

        # Lưu ý: Không gọi start_camera_stream trực tiếp ở đây nữa

        self.btn_toggle_camera_mobile.setEnabled(False)

        self.isPressCameraON = False

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Đã kết nối thành công đến MQTT broker!")
            if self.label_status_3:
                self.label_status_3.setText("Đã kết nối thành công đến MQTT broker!")
            client.subscribe(self.topic_subscribe_command)  # Subscribe topic điều khiển
            # Bắt đầu camera stream trong một thread riêng sau khi kết nối thành công
            camera_thread = threading.Thread(target=self.start_camera_stream)
            camera_thread.daemon = True  # Cho phép thread thoát khi chương trình chính thoát
            self.btn_toggle_camera_mobile.setEnabled(True)
         #   camera_thread.start()
        else:
            print(f"Kết nối thất bại với mã lỗi {rc}")
            self.btn_toggle_camera_mobile.setEnabled(False)
            if self.label_status_3:
                self.label_status_3.setText(f"Kết nối thất bại với mã lỗi {rc}")
    def toggle_camera_mobile(self):
        if(self.isPressCameraON==True):
            self.Stop_send_data_camera()
            self.isPressCameraON = False
            self.label_status_camera.setText(f"Sending data camera OFF")
        else:
            self.Send_data_camera()
            self.isPressCameraON = True
            self.label_status_camera.setText(f"Sending data camera ON")

    def Send_data_camera(self):
        camera_thread = threading.Thread(target=self.start_camera_stream)
        camera_thread.daemon = True 
        camera_thread.start()
    def Stop_send_data_camera(self):
        self.stop_camera()

    # Hàm callback khi nhận được tin nhắn
    def on_message(self, client, userdata, msg):
        if msg.topic == self.topic_subscribe_command:
            command = msg.payload.decode().upper()
            print(f"Nhận được lệnh: {command}")
            self.process_command(command)
            self.label_nhan_lenh.setText(f"Nhận được lệnh: {command}")

    # Hàm callback khi đăng ký topic thành công
    def on_subscribe(self, client, userdata, mid, granted_qos):
        print(f"Đã đăng ký topic: {self.topic_subscribe_command} với QoS: {granted_qos}")
      #  self.label_status_3.setText(f"Đã đăng ký topic: {self.topic_subscribe_command} với QoS: {granted_qos}")

    def process_command(self, command):
        """
        Xử lý các lệnh điều khiển xe.
        Hỗ trợ các lệnh có giá trị (ví dụ: FORWARD-50) và không có giá trị (ví dụ: STOP, HOME, DOCK).
        """
        command = command.upper().strip()
        print(f"Nhận được lệnh: {command}")

        if '-' in command:
            parts = command.split('-')
            if len(parts) == 2:
                action = parts[0].strip()
                value_str = parts[1].strip()
                try:
                    value = int(value_str)
                    tocdo = value / 100.0
                    print(f"Lệnh hành động: {action}, Giá trị: {value}, Tốc độ: {tocdo}")
                    self.ros2_handle.cmd_vel_publisher.vx = tocdo
                    self.ros2_handle.cmd_vel_publisher.vw = tocdo * 1.0
                    print(f"vx:{self.ros2_handle.cmd_vel_publisher.vx}")
                    print(f"vw:{self.ros2_handle.cmd_vel_publisher.vw}")

                    if action == "FORWARD":
                        print(f"Di chuyển về phía trước với tốc độ: {tocdo}")
                        self.ros2_handle.cmd_vel_publisher.forward()
                    elif action == "BACKWARD":
                        print(f"Di chuyển về phía sau với tốc độ: {tocdo}")
                        self.ros2_handle.cmd_vel_publisher.backward()
                    elif action == "LEFT":
                        print(f"Rẽ trái với tốc độ: {tocdo}")
                        self.ros2_handle.cmd_vel_publisher.left()
                    elif action == "RIGHT":
                        print(f"Rẽ phải với tốc độ: {tocdo}")
                        self.ros2_handle.cmd_vel_publisher.right()
                    else:
                        print(f"Lệnh hành động không hợp lệ (có giá trị): {action}")

                except ValueError:
                    print(f"Giá trị '{value_str}' trong lệnh không phải là số.")
            else:
                print(f"Định dạng lệnh có dấu '-' không hợp lệ: {command}. Mong đợi 'COMMAND-VALUE'.")
        else:
            # Xử lý các lệnh không có dấu '-'
            action = command
            if action == "STOP":
                print("Dừng lại")
                self.ros2_handle.cmd_vel_publisher.stop_movement()
            elif action == "HOME":
                print("Go to home")
                self.ve_home_trong_chu_trinh()
            elif action == "DOCK":
                print("Go to dock")
                self.ve_dock_sac()
            else:
                print(f"Lệnh hành động không hợp lệ (không có giá trị): {action}")
                
    def start_camera_stream(self):
        # Mở camera
        self.cap = cv2.VideoCapture(RobConf.faceCame1)  # Thay đổi index nếu cần

        try:
            while True:
                if self.cap is None or not self.cap.isOpened():
                    print("Không thể truy cập camera.")
                    break
                ret, frame = self.cap.read()
                if not ret:
                    print("Không thể đọc frame từ camera.")
                    break

                # Mã hóa frame thành JPEG
                _, img_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                data_to_send = img_encoded.tobytes()

                # Publish frame lên topic camera
                if self.client and self.client.is_connected():
                    self.client.publish(self.topic_publish_camera, payload=data_to_send, qos=0)
                    # print(f"Đã gửi frame ảnh ({len(data_to_send)} bytes) lên topic: {self.topic_publish_camera}")
                    pass # In log gửi ảnh có thể làm chậm quá trình
                else:
                    print("Không thể publish ảnh vì không có kết nối MQTT.")
                    break

                time.sleep(0.1)  # Gửi mỗi 100ms (điều chỉnh tùy ý)
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("Dừng stream camera.")
        finally:
            self.stop_camera()

    def stop_camera(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            print("Đã đóng camera.")
            self.cap = None

    def stop_mobile(self):
        print("Đang ngắt kết nối MQTT...")
        if self.client:
            self.label_status_3.setText("Vui lòng ấn nút kết nối mobile")
            self.label_nhan_lenh.setText("Đang chờ lệnh")
            self.client.loop_stop()  # Dừng vòng lặp MQTT thread
            self.client.disconnect()
            print("Đã ngắt kết nối MQTT.")
        self.btn_toggle_camera_mobile.setEnabled(False)
        self.stop_camera() # Đảm bảo camera cũng được dừng

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
        self.bt_STOP.pressed.connect(self.ros2_handle.cmd_vel_publisher.stop_movement)
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
                self.listWidget_dsban.addItem("Điểm {0}".format(text_nhap))
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


     
    def xu_ly_ID(self, event):
        print("ID")
        try:
            if not hasattr(keyyyyy.Registe, "virtual_keyboard") or not keyyyyy.Registe.virtual_keyboard.isVisible():
                keyyyyy.Registe.virtual_keyboard = keyyyyy.VKBD(self.uic50.lineEdit_nhapID)
                keyyyyy.Registe.virtual_keyboard.show()
            else:
                # keyyyyy.Registe.virtual_keyboard.activateWindow()
                keyyyyy.Registe.virtual_keyboard.hide()
        except Exception as e:
            print(f"failed to open virtual keyboard ,already exist or error: {e}")
    def xu_ly_name(self, event):
        print("Name")
        try:
            if not hasattr(keyyyyy.Registe, "virtual_keyboard") or not keyyyyy.Registe.virtual_keyboard.isVisible():
                keyyyyy.Registe.virtual_keyboard = keyyyyy.VKBD(self.uic50.lineEdit_nhapten)
                keyyyyy.Registe.virtual_keyboard.show()
            else:
                # keyyyyy.Registe.virtual_keyboard.activateWindow()
                keyyyyy.Registe.virtual_keyboard.hide()
        except Exception as e:
            print(f"failed to open virtual keyboard ,already exist or error: {e}")

    def manhinh_nhap_ID(self):  # dùng cho nút cập nhật số bàn
        self.from_nhap_ID = QMainWindow()
        self.uic50 = Ui_Form_nhaptenID()
        self.uic50.setupUi(self.from_nhap_ID)

        self.from_nhap_ID.show()
     
        self.uic50.Button_huy_capnhat_name.clicked.connect(self.tatmanhinh_capnhat_ID)

        self.uic50.lineEdit_nhapID.mousePressEvent = self.xu_ly_ID
        self.uic50.lineEdit_nhapten.mousePressEvent = self.xu_ly_name
      
        # sau khi nhấn enter hoặc nút xác nhận
        self.uic50.lineEdit_nhapID.returnPressed.connect(self.kiem_tra_nhap_name)
        self.uic50.lineEdit_nhapten.returnPressed.connect(self.kiem_tra_nhap_name)
        self.uic50.Button_capnhat_name.clicked.connect(self.kiem_tra_nhap_name)
   
        self.uic50.Button_xoa_capnhat_name.clicked.connect(self.xoa_dulieu_name)
        self.uic50.Button_xem_capnhat_name.clicked.connect(self.show_ID)
     # Hàm kiem_tra_nhap_name đã được chỉnh sửa
    def xoa_dulieu_name(self):
        """
        Hiển thị QMessageBox cảnh báo và xóa toàn bộ dữ liệu trong bảng 'nameLocation'
        nếu người dùng xác nhận.
        """
        reply = QMessageBox.warning(
            self,
            "Xác nhận xóa dữ liệu",
            "Bạn có chắc chắn muốn xóa TOÀN BỘ DỮ LIỆU trong bảng 'nameLocation' không? "
            "Thao tác này KHÔNG THỂ HOÀN TÁC!",
            QMessageBox.Yes | QMessageBox.No, # Các nút Yes và No
            QMessageBox.No # Nút mặc định được chọn là No
        )

        if reply == QMessageBox.Yes:
            print("Người dùng đã xác nhận xóa dữ liệu.")
            success = sql.clear_name_location_table() # Gọi hàm xóa dữ liệu

            if success:
                QMessageBox.information(self, "Xóa thành công", "Đã xóa toàn bộ dữ liệu trong bảng 'nameLocation'.")
                # Sau khi xóa DB, cần cập nhật lại giao diện (ví dụ: các nút, list widget)
                self.cap_nhat_ten_nut_theo_db()
                self.hien_thi_nut_theo_id()
                # Có thể xóa trắng các lineEdit_nhapID và lineEdit_nhapten nếu muốn
                self.uic50.lineEdit_nhapID.clear()
                self.uic50.lineEdit_nhapten.clear()
              
            
            else:
                # Thông báo lỗi đã được hiển thị trong clear_name_location_table
                pass
        else:
            print("Người dùng đã hủy thao tác xóa dữ liệu.")
            QMessageBox.information(self, "Hủy bỏ", "Thao tác xóa dữ liệu đã bị hủy bỏ.")
    def kiem_tra_nhap_name(self):
        xuly_cham_ngoai() # Gọi hàm xuly_cham_ngoai()

        is_id_valid = False
        is_name_not_empty = False
        error_message = ""
        input_id = None # Khởi tạo để tránh lỗi nếu int() thất bại

        # 1. Kiểm tra self.uic50.lineEdit_nhapID
        id_text = self.uic50.lineEdit_nhapID.text().strip()
        try:
            input_id = int(id_text)
            if 1 <= input_id <= 10:
                is_id_valid = True
            else:
                error_message = f"Lỗi ID: ID '{id_text}' không nằm trong khoảng 1-10."
        except ValueError:
            error_message = f"Lỗi ID: ID '{id_text}' không phải là số nguyên hợp lệ."
        except Exception as e:
            error_message = f"Lỗi không xác định khi kiểm tra ID: {e}"

        # 2. Kiểm tra self.uic50.lineEdit_nhapten
        name_text = self.uic50.lineEdit_nhapten.text().strip() # Sử dụng .lineEdit_nhapten
        if name_text:
            is_name_not_empty = True
        else:
            if not error_message: # Chỉ gán lỗi tên nếu chưa có lỗi ID
                error_message = "Lỗi Tên: Tên không được để trống."

        # 3. Xử lý dựa trên điều kiện hợp lệ
        if is_id_valid and is_name_not_empty:
            # Nếu tất cả hợp lệ, gọi hàm cập nhật/chèn database
            print("Các điều kiện nhập liệu hợp lệ. Đang tiến hành cập nhật/chèn vào DB...")
            # Gọi hàm update_or_insert_location_name
            db_success = sql.update_or_insert_location_name(input_id, name_text)

            if db_success:
                QMessageBox.information(self, "Thành công", f"Dữ liệu cho ID {input_id} ('{name_text}') đã được cập nhật/thêm thành công vào database.")
                # Cập nhật lại các nút nếu cần sau khi DB thay đổi
                self.cap_nhat_ten_nut_theo_db()
                self.hien_thi_nut_theo_id()
                self.from_nhap_ID.close() # Đóng form nếu thành công
            else:
                # Lỗi DB đã được xử lý bằng QMessageBox trong update_or_insert_location_name
                # Nên không cần show thêm MessageBox ở đây.
                print("Lỗi DB đã xảy ra, không thể hoàn thành thao tác.")
                # Tùy chọn: Không đóng form nếu thao tác DB thất bại để người dùng sửa.
                # self.from_nhap_ID.close() # Bạn có thể chọn đóng hoặc không tùy ý.

        else:
            # Nếu có lỗi, hiện MessageBox và đóng form_nhap_ID
            if error_message:
                QMessageBox.warning(self, "Lỗi Nhập Liệu", error_message)
            print("Điều kiện không đủ. Dữ liệu không hợp lệ. Đóng form.")
            self.from_nhap_ID.close() # Đóng form khi có lỗi nhập liệu

    
    def tatmanhinh_capnhat_ID(self):
        self.from_nhap_ID.close()
      

    def show_ID(self):
        self.UI_page_ID()
    def UI_page_ID(self):
        self.load_ID = QMainWindow()
        self.ui29 = Ui_Form_show_ID()
        self.ui29.setupUi(self.load_ID)
        self.ui29.btn_close_ID.clicked.connect(self.close_ui_ID)
        self.load_ID.show()
        self.load_ID_sql()

    def close_ui_ID(self):
        self.load_ID.close()

    def load_ID_sql(self):
        self.fcn_load_ID()

    # HAM DUNG DE SET CHIEU RONG CUA COT
   # SET CHIEU RONG CUA COT
    def set_column_widths_4(self):
        self.ui29.tableWidget_ID.setColumnWidth(0, 200)  # Cột toadoX
        self.ui29.tableWidget_ID.setColumnWidth(1, 200)  # Cột toadoY
    # SET CHIEU CAO CUA HANG
    def set_row_heights_4(self):
        for row in range(self.ui29.tableWidget_ID.rowCount()):
            self.ui29.tableWidget_ID.setRowHeight(row, 30)  # Chiều cao mỗi hàng là 30 pixel

    # HAM LOAD DATA LEN BANG
    def fcn_load_ID(self):
        data = sql.doc_du_lieu_ID('readall')

        if data is not None:
            num_rows = len(data)
            num_columns = 2  # 5 cột dữ liệu

            self.ui29.tableWidget_ID.clear()  # Xóa dữ liệu cũ trước khi tải lại
            self.ui29.tableWidget_ID.setRowCount(num_rows)
            self.ui29.tableWidget_ID.setColumnCount(num_columns)
            self.ui29.tableWidget_ID.setHorizontalHeaderLabels(['ID', 'NAME'])

            if num_rows > 0:
                for row_idx, row_data in enumerate(data):
                    # Thêm dữ liệu vào các cột và căn giữa
                    for col_idx, item in enumerate(row_data):
                        item_widget = QTableWidgetItem(str(item))
                        item_widget.setTextAlignment(Qt.AlignCenter)
                        self.ui29.tableWidget_ID.setItem(row_idx, col_idx, item_widget)

                self.set_column_widths_4()
                self.set_row_heights_4()

            self.ui29.terminal_ID.setPlainText("Data loaded successfully")
        else:
            self.ui29.tableWidget_ID.clear()
            self.ui29.tableWidget_ID.setRowCount(0)
            self.ui29.tableWidget_ID.setColumnCount(2)  # Cập nhật số cột header
            self.ui29.tableWidget_ID.setHorizontalHeaderLabels(['ID', 'NAME'])
            self.ui29.terminal_ID.setPlainText("Failed to load data.")
    # ============ sắp xếp thứ tự bàn hien thi tren list ==============#
    def sap_xep_stt_ban(self):
        items = [self.listWidget_dsban.item(index).text() for index in
                 range(self.listWidget_dsban.count())]

        def custom_sort_key(item):
            if item == "Trạm sạc" or item == "Khu vực chờ":
                return (1, item)
            elif item.startswith("Điểm "):
                return (2, int(item.split("Điểm ")[-1]))
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
     #   self.x, self.y, self.z, self.w = self.ros2_handle.pose_listener.data_odom
        self.x, self.y, self.z, self.w = self.ros2_handle.odom_listener.data_odom
  
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
            elif item.startswith("Điểm "):
                return (2, int(item.split("Điểm ")[-1]))
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
                self.listWidget_dsban.addItem(f"Điểm {ban}")
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
                    self.uic4.listWidget_themvao.addItem(f"Điểm {ban}")
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

    def show_toado(self):
        self.UI_page_toado()
    def UI_page_toado(self):
        self.load_toado = QMainWindow()
        self.ui22 = Ui_Form_show_toado()
        self.ui22.setupUi(self.load_toado)
        self.ui22.btn_close_toado.clicked.connect(self.close_ui_toado)
        self.load_toado.show()
        self.load_toado_sql()
    def close_ui_toado(self):
        self.load_toado.close()
    def load_toado_sql(self):
        self.fcn_load_toado()

    # HAM DUNG DE SET CHIEU RONG CUA COT
   # SET CHIEU RONG CUA COT
    def set_column_widths_3(self):
        self.ui22.tableWidget_toado.setColumnWidth(0, 200)  # Cột toadoX
        self.ui22.tableWidget_toado.setColumnWidth(1, 200)  # Cột toadoY
        self.ui22.tableWidget_toado.setColumnWidth(2, 200)  # Cột toadoZ
        self.ui22.tableWidget_toado.setColumnWidth(3, 200)  # Cột toadoW
        self.ui22.tableWidget_toado.setColumnWidth(4, 200)  # Cột IDban
    # SET CHIEU CAO CUA HANG
    def set_row_heights_3(self):
        for row in range(self.ui22.tableWidget_toado.rowCount()):
            self.ui22.tableWidget_toado.setRowHeight(row, 30)  # Chiều cao mỗi hàng là 30 pixel

    # HAM LOAD DATA LEN BANG
    def fcn_load_toado(self):
        data = sql.doc_du_lieu_toado('readall')

        if data is not None:
            num_rows = len(data)
            num_columns = 5  # 5 cột dữ liệu

            self.ui22.tableWidget_toado.clear()  # Xóa dữ liệu cũ trước khi tải lại
            self.ui22.tableWidget_toado.setRowCount(num_rows)
            self.ui22.tableWidget_toado.setColumnCount(num_columns)
            self.ui22.tableWidget_toado.setHorizontalHeaderLabels(['toadoX', 'toadoY', 'toadoZ', 'toadoW', 'IDban'])

            if num_rows > 0:
                for row_idx, row_data in enumerate(data):
                    # Thêm dữ liệu vào các cột và căn giữa
                    for col_idx, item in enumerate(row_data):
                        item_widget = QTableWidgetItem(str(item))
                        item_widget.setTextAlignment(Qt.AlignCenter)
                        self.ui22.tableWidget_toado.setItem(row_idx, col_idx, item_widget)

                self.set_column_widths_3()
                self.set_row_heights_3()

            self.ui22.terminal_toado.setPlainText("Data loaded successfully")
        else:
            self.ui22.tableWidget_toado.clear()
            self.ui22.tableWidget_toado.setRowCount(0)
            self.ui22.tableWidget_toado.setColumnCount(7)  # Cập nhật số cột header
            self.ui22.tableWidget_toado.setHorizontalHeaderLabels(['toadoX', 'toadoY', 'toadoZ', 'toadoW', 'IDban'])
            self.ui22.terminal_toado.setPlainText("Failed to load data.")
    # HAM CHON CHECKBOX
    
################## GIAO DIEN THEM PHONG DE CHAY ##################

    def cap_nhat_ten_nut_theo_db(self):
        """
        Cập nhật text của các nút button dựa trên dữ liệu 'Name' từ bảng 'nameLocation'.
        Nếu ID không có trong DB, sử dụng tên mặc định (ví dụ: 'Điểm X').
        """
        button_map = {
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
            100: self.btn_tram_sac, # Thêm trạm sạc nếu bạn muốn tên nó cũng từ DB
            200: self.btn_khu_vuc_cho # Thêm khu vực chờ nếu bạn muốn tên nó cũng từ DB
        }

        for btn_id, button_obj in button_map.items():
            db_name = sql.get_location_name_from_db(btn_id)
            if db_name:
                button_obj.setText(db_name)
            else:
                # Nếu không có trong DB, đặt tên mặc định
                if btn_id == 100:
                    button_obj.setText("Trạm sạc")
                elif btn_id == 200:
                    button_obj.setText("Khu vực chờ")
                else:
                    button_obj.setText(f"Điểm {btn_id}")

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

    def kiem_tra_trung_ten_trong_listWidget_ds(self, item_text):
        """
        Kiểm tra xem một tên đã tồn tại trong listWidget_ds hay chưa.
        Trả về True nếu trùng lặp, False nếu không.
        """
        for i in range(self.listWidget_ds.count()):
            if self.listWidget_ds.item(i).text() == item_text:
                return True
        return False

    def toggle_item(self, id):
        # Dùng hàm get_location_name_from_db để lấy tên
        item_text = sql.get_location_name_from_db(id)

        if item_text is None:
            # Nếu không tìm thấy trong DB, mặc định là "Phòng X"
            if id == 100:
                item_text = "Trạm sạc"
            elif id == 200:
                item_text = "Khu vực chờ"
            else:
                item_text = f"Điểm {id}"

        if id not in self.item_states:
            self.item_states[id] = False # Đảm bảo ID được khởi tạo trạng thái

        if not self.item_states[id]:
            # Chỉ thêm nếu không trùng tên
            if not self.kiem_tra_trung_ten_trong_listWidget_ds(item_text):
                self.listWidget_ds.addItem(item_text)
                self.item_states[id] = True
            else:
                # Xử lý trường hợp trùng tên (ví dụ: thông báo cho người dùng)
                print(f"Mục '{item_text}' đã tồn tại trong danh sách.")
        else:
            for i in range(self.listWidget_ds.count()):
                if self.listWidget_ds.item(i).text() == item_text:
                    self.listWidget_ds.takeItem(i)
                    self.item_states[id] = False
                    break
        self.kiem_tra_du_dieu_khien_chay() # Giả định hàm này tồn tại
        self.sort_items()

    def sort_items(self):
        items = []
        for i in range(self.listWidget_ds.count()):
            items.append(self.listWidget_ds.item(i).text())

        def sort_key(item_name):
            if item_name == "Trạm sạc":
                return 0
            elif item_name == "Khu vực chờ":
                return 1
            else:
                # Cố gắng phân tích ID từ tên để sắp xếp
                # Đầu tiên, thử tìm ID trong DB
                db_id = None
                # Đây là một cách không hiệu quả, tốt hơn nên lưu ID cùng với tên trong ListWidget
                # Để đơn giản hóa ví dụ, tôi sẽ cố gắng lấy ID từ tên phòng nếu không có trong DB
                for potential_id in range(1, 11): # Kiểm tra các ID phòng thông thường
                    db_name = sql.get_location_name_from_db(potential_id)
                    if db_name == item_name:
                        db_id = potential_id
                        break
                if db_id is not None:
                    return db_id
                else:
                    # Nếu không tìm thấy trong DB, cố gắng phân tích "Phòng X"
                    try:
                        return int(item_name.split()[1])
                    except (ValueError, IndexError):
                        return float('inf')

        items.sort(key=sort_key)

        self.listWidget_ds.clear()
        for item in items:
            self.listWidget_ds.addItem(item)
    def xoaDS(self):
        self.listWidget_ds.clear()
        for key in self.item_states:
            self.item_states[key] = False

################## GIAO DIEN  CHAY DANH SACH ##################
    # SET CHIEU CAO CUA HANG
    def set_column_widths_0(self):
        self.tableWidget_4.setColumnWidth(0, 100)    # Cột checkbox
        self.tableWidget_4.setColumnWidth(1, 220)   # Cột ID
        self.tableWidget_4.setColumnWidth(2, 320)   # Cột NAME
        self.tableWidget_4.setColumnWidth(3, 200)   # Cột ROOM
    def set_row_heights_0(self):
        for row in range(self.tableWidget_4.rowCount()):
            self.tableWidget_4.setRowHeight(row, 50)    # Chiều cao mỗi hàng là 50 pixel

 # HAM LOAD DATA LEN BANG
    def fcn_load_list_0(self):
        data = sql.doc_du_lieu_list('readall')

        if data is not None:
            num_rows = len(data)
            num_columns = 4    # Đúng số cột dữ liệu

            self.tableWidget_4.clear()     # Xóa dữ liệu cũ trước khi tải lại
            self.tableWidget_4.setRowCount(num_rows)
            self.tableWidget_4.setColumnCount(num_columns)
            self.tableWidget_4.setHorizontalHeaderLabels(['Select','ID', 'Name', 'Room'])

            if num_rows > 0:
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
                    # Truyền thêm row_idx vào hàm get_selected_rows_0
                    checkbox.stateChanged.connect(lambda state, row=row_idx: self.handle_checkbox_change_0(state, row))

                    # Thêm widget checkbox vào bảng
                    self.tableWidget_4.setCellWidget(row_idx, 0, checkbox_widget)

                    # Thêm dữ liệu vào các cột còn lại và căn giữa
                    for col_idx, item in enumerate(row_data):
                        item_widget = QTableWidgetItem(str(item))
                        item_widget.setTextAlignment(Qt.AlignCenter)
                        self.tableWidget_4.setItem(row_idx, col_idx + 1, item_widget)

                self.set_column_widths_0()
                self.set_row_heights_0()

            else:
                self.tableWidget_4.clear()
                self.tableWidget_4.setRowCount(0)
                self.tableWidget_4.setColumnCount(4)     # Đúng số cột dữ liệu
                self.tableWidget_4.setHorizontalHeaderLabels(['Select','ID', 'Name', 'Room'])

    def handle_checkbox_change_0(self, state, row):
        # Lấy dữ liệu của hàng được chọn
        room_number = self.tableWidget_4.item(row, 3).text() # Cột 'Room' có index là 3
        id = room_number
        ID_location = sql.doc_name_theo_id(id)
        if state == Qt.Checked:
            # Nếu checkbox được chọn, kiểm tra trùng lặp trước khi thêm
            found = False
            for i in range(self.listWidget_ds.count()):
                if self.listWidget_ds.item(i).text() == ID_location[0]:
                    found = True
                    break
            if not found:
                self.listWidget_ds.addItem(ID_location[0])
                print(f"Đã chọn hàng {row + 1}, thêm Điểm: {room_number}")
                print(f"ID name: {id}")
                print(f"ID name: {ID_location[0]}")
                self.kiem_tra_du_dieu_khien_chay()
                self.sort_items() # Add this line to sort items after each toggle
            else:
                print(f"Điểm {room_number} đã tồn tại trong danh sách.")
                # Nếu đã tồn tại, có thể bỏ chọn lại checkbox (tùy vào logic bạn muốn)
                checkbox_widget = self.tableWidget_4.cellWidget(row, 0)
                if checkbox_widget and isinstance(checkbox_widget, QWidget):
                    checkbox = checkbox_widget.findChild(QCheckBox)
                    if checkbox:
                        checkbox.setChecked(False)

        else:
            # Nếu checkbox bị bỏ chọn, tìm và xóa số phòng khỏi listWidget_ds

            for i in range(self.listWidget_ds.count()):
                if self.listWidget_ds.item(i).text() == ID_location[0]:
                    self.listWidget_ds.takeItem(i)
                    print(f"Đã bỏ chọn hàng {row + 1}, xóa Điểm: {room_number}")
                    break

    def get_selected_rows_0(self):
        # Hàm này bây giờ có thể không cần thiết nếu bạn xử lý việc thêm/xóa trực tiếp
        # trong handle_checkbox_change_0, nhưng vẫn giữ lại nếu bạn có logic khác
        self.selected_rows_0 = []
        num_rows = self.tableWidget_4.rowCount()
        for row in range(num_rows):
            checkbox_widget = self.tableWidget_4.cellWidget(row, 0)
            if checkbox_widget and isinstance(checkbox_widget, QWidget):
                checkbox = checkbox_widget.findChild(QCheckBox)
                if checkbox and checkbox.isChecked():
                    self.selected_rows_0.append(row + 1)

        print("Selected rows:", self.selected_rows_0)
        return self.selected_rows_0
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
                            self.value_pin  = float(parts[0])  # Lấy phần tử đầu tiên và chuyển đổi
                            if self.value_pin  < 10.6:
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
                            elif self.value_pin  > 10.6:
                                print("Giá trị điện áp lớn hơn 10.8 :", self.value_pin )
                                print("Đủ điện")
                                time.sleep(2)
                            #    Uti.RobotSpeakWithPath('voice_hmi_new/thongbao_pin_day.wav')
                                self.navigation_thread.du_pin = True
                                self.read_arduino.doc_pin.disconnect(self.trangthai_pin)
                                self.read_arduino.stop()
                             #   self.ve_home_trong_chu_trinh()
                            else:
                                print("Giá trị điện áp bình thường :", self.value_pin )
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
            self.ui21.btn_sac_auto.setStyleSheet("background-color: rgb(0,255,0);border-radius:20px;")
            Uti.RobotSpeakWithPath('voice_hmi_new/bat_chedosac.wav')
             # time.sleep(1)
           
        else:
    
            self.ui21.btn_sac_auto.setStyleSheet("background-color: rgb(255,255,255);border-radius:20px;")
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
          # self.toado_z.setStyleSheet("background-color: white;")
     
     
        else:
            # Optional: Change background color when disabled
            self.toado_x.setStyleSheet("background-color: lightgray;")
            self.toado_y.setStyleSheet("background-color: lightgray;")
          #  self.toado_z.setStyleSheet("background-color: lightgray;")
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
        #    self.toado_z.mousePressEvent = self.xu_ly_nhap_banphim
        else:  
            print("Khóa Nhập")
            self.btn_toado_td.setEnabled(False)
    def fcn_nhap_toado_td(self):
      # self.toado_x.setText() = self.ros2_handle.odom_listener.data_odom[0]
    #   self.toado_y = self.ros2_handle.odom_listener.data_odom[1]
      # self.toado_z = (self.ros2_handle.odom_listener.data_odom[3]*180.0)/math.pi
       self.toado_x.setText(str(round(self.ros2_handle.odom_listener.data_odom[0], 3)))
       self.toado_y.setText(str(round(self.ros2_handle.odom_listener.data_odom[1], 3)))
   #    self.toado_z.setText(str(round(((self.ros2_handle.odom_listener.data_odom[3]*180.0)/math.pi),3)))
       print("Cap nhat toa do dinh vi")
       print("X:"+str(self.ros2_handle.odom_listener.data_odom[0]))
       print("Y:"+str(self.ros2_handle.odom_listener.data_odom[1]))
       print("Z:"+str((self.ros2_handle.odom_listener.data_odom[3]*180.0)/math.pi))
       print("Z:"+self.toado_z.currentText())

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
           # toa_do_z = float(self.toado_z.text())
            toa_do_z = float(self.toado_z.currentText())
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


      # HAM XOA DU LIEU
    def fcn_delete_data_all(self):

        
        self.setup_dv.delete_data_all()

        self.fcn_load_data()
        self.terminal.clear()
   
    
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
               # self.from_setup_cam.close()
                Uti.RobotSpeakWithPath('voice_hmi_new/finish_setup_cam.wav')
              #  self.finish_setup_cam()
            elif flag == 2:
                print("=============DONE DINH VI ==============")
            #   self.from_dang_dinhvi.close()
                Uti.RobotSpeakWithPath('voice_hmi_new/finish_dinhvi.wav')
             #   self.finish_dinhvi()
            elif flag == 10:
              #  self.error_setup_cam()
                pass
            else:
                pass
    #======================== ham cua nut nhan DINH VI =======================
     #========================== cac ham GUI nut nhan DINH VI ========================================
    def show_ui_dv(self):
        self.UI_DV()
    def UI_DV(self):
        self.dv = QMainWindow()
        self.ui25 = Ui_Form_ui_dv()
        self.ui25.setupUi(self.dv)

        self.ui25.Btn_ui_xn_dv.clicked.connect(self.close_ui_dv)
        self.ui25.Btn_ui_xn_dv.setEnabled(False)
        self.dv.show()
    def close_ui_dv(self):
        self.dv.close()

    #========================== cac ham xu ly cua nut nhan DINH VI ========================================
    def dinh_vi_fcn_1(self): 
            self.UI_DV()
            self.check_and_opencamera()       
          #  Uti.RobotSpeakWithPath('voice_hmi_new/dinhvi.wav')
            
            print('[dinh_vi_fcn_1]Dang dinh vi ')
      
            if self.dinh_vi_fcn():
                self.dongThongbaoKTSetup(2)
                self.ve_home_khan_cap()            
                self.back_setup()
            else:
              print('[dinh_vi_fcn_1]Dinh vi that bai')
              Uti.RobotSpeakWithPath('voice_hmi_new/dinh_vi_that_bai.mp3')
    def quaternion_to_euler(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_deg = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_deg = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_deg = math.degrees(math.atan2(t3, t4))

        return (roll_deg, pitch_deg, yaw_deg)

    def euler_to_quaternion(self,yaw = 0, pitch = 0, roll = 0):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return (qx, qy, qz, qw)
        
    def dinh_vi_fcn(self):
        self.fcn_load_data()
    
        self.navigation_thread.dang_dinhvi_status = True
        self.dang_dinhvi_cnt = 0
        Uti.RobotSpeakWithPath('voice_hmi_new/dinhvi.wav')

        
     
        self.id_voice = 1000    
        #self.run_file_code_thread.duongdan = 2
        print('[dinh_vi_fcn]---------------------------BAT DAU QUA TRINH DINH VI------------------------')

        #B1: TIM HINH TRONG MAU VANG TREN CAM: NEU CO DUONG TRON THI TINH TAM VA CAP NHAT VI TRI
        counterMove = 0
        timeReadcame = 10
        cricle_yes = False
        radius_step = 0.5
        yaw_step = 45
        radius_current = 0.0
        radius_step = 0.5
        yaw_index = 0
        yaw_current = 0
       # yaw_list = [0, 45, 90, 135, 180, -135, -90, -45]
        yaw_list = [ 45,  225]
        result = False

        while (not result) and (counterMove < timeReadcame): # quay lai qua trinh doc came
            #if self.navigation_thread.quatrinh_move == False:
                            #time.sleep(2)
        
            yawdeg  =round((self.ros2_handle.odom_listener.data_odom[3]*180.0)/math.pi,3)
            print(f"[DEBUG] Góc yaw đầu vào: {yawdeg} độ")
        #self.sub_win1.uic.terminal_2.setPlainText(str(yawdeg))
       # print("GOC YAWdeg now: ", yawdeg)
            print(f"X:{self.ros2_handle.odom_listener.data_odom[0]}")
            print(f"Y:{self.ros2_handle.odom_listener.data_odom[1]}")
            print(f"Z:{self.ros2_handle.odom_listener.data_odom[2]}")
            print(f"W:{self.ros2_handle.odom_listener.data_odom[3]}")
            result = self.setup_dv.test_image(200, yawdeg, self.capC,  self.cap_qr)
            print(f"Result:{result}")
            result_simlar = 0
            x_best, y_best, yaw_best = 0,0,0
    
            result_text = ""
            for idx, (coords, similarity) in enumerate(result[:1]):
                    result_text += f"({coords[0]}, {coords[1]}, {coords[2]}, {coords[3]}) "
                    result_text += f"{similarity:.6f} \n"
                    x_best, y_best, yaw_best, result_simlar = coords[0], coords[1], coords[2],similarity
                    print(result_text)
                

            # rdeg, pdeg, yawdeg = Uti.quaternion_to_euler(self.ros2_handle.odom_listener.data_odom[0],self.ros2_handle.odom_listener.data_odom[1], self.ros2_handle.odom_listener.data_odom[2], self.ros2_handle.odom_listener.data_odom[3])
            # yess, cx, cy, radius = Uti.dieuchinhvaTimduongtron(camID=headCamera, yaw_ros= yawdeg)
            if result_simlar > RobConf.DO_TUONG_DONG_CAM_DINHVIN:
                yess = True

            else:
                yess = False
                result = False
            if yess:
                # Tinh duoc vi tri hinhtron                    
                cricle_yes = True
                # Chinh Pose he thong
                    # doc thong so ti le cam/m va vi tri home
                    # Đọc các giá trị từ tệp văn bản
                xx, yy = x_best,y_best
           
                print(f'[dinh_vi_fcn]Vi tri Diem hien tai cho x: {xx}, y = {yy}, yaw ={yaw_best}.')
                self.navigation_thread.user_set_PoseXY(xx, yy,yaw_best)
                result = True
                # print(f"x_dinhvi: {xx};  y_dinhvi: {yy}")
                # THONG BAO TAT CAC POPUP
                #self.run_file_code_thread.duongdan = 2
                #GHI VI TRI HIEN TAI VAO FILE
                
                print('[dinh_vi_fcn]------------------- ĐÃ LẤY XONG TỌA ĐỘ ---------------------')
                # time.sleep(3)
              #  Uti.writePose2File(filePath = file_path_pose_data,x=xx,y=yy,z=self.ros2_handle.odom_listener.data_odom[2],w=self.ros2_handle.odom_listener.data_odom[3])
                #with open('file_text_odom/odom_data.txt', 'w') as file:
                #    file.write(f'x: {xx}, y: {yy}, z: {self.ros2_handle.odom_listener.data_odom[2]}, w: {self.ros2_handle.odom_listener.data_odom[3]}\n')
                print(f'[dinh_vi_fcn] x: {xx}, y: {yy}, z: {self.ros2_handle.odom_listener.data_odom[2]}, w: {self.ros2_handle.odom_listener.data_odom[3]}','-pose cam')
                print("[dinh_vi_fcn]DINH VI XONG!!!!!")
                break
            else: 
                yaw_current = yaw_list[yaw_index]
                
                x_desired = radius_current*np.cos(math.radians(yaw_current))
                y_desired = radius_current*np.sin(math.radians(yaw_current))
                z_desired = 0
              #  yaw_desired = math.radians(0)
                yaw_desired = math.radians(yaw_current)
                qx, qy, qz, qw = self.euler_to_quaternion(yaw_desired,0,0)
                w_desired = qw 
                print(f'[dinh_vi_fcn]-------Tim duong tron, Di chuyen den vi tri nay: goc = {yaw_current}, x = {x_desired}, y = {y_desired}')
                #self.desired_move(x_desired=x_desired, y_desired=y_desired, z_desired=qz, w_desired=qw,id_desired=200)      
                self.ros2_handle.goal_publisher.send_goal( x = x_desired, y = y_desired, z = z_desired, w = w_desired)
                print('[dinh_vi_fcn]CHO ROBOT TOI DIEM MOI XONG')

            
                # Cập nhật vòng lặp
                yaw_index += 1
                if yaw_index >= len(yaw_list):
                    yaw_index = 0
                    radius_current += radius_step
        #NEU KHONG CO TREN N LAN THI THOAT RA VA THONG BAO BANG HINH ANH VA GIONG NOI
        self.navigation_thread.dang_dinhvi_status = False 

        if not cricle_yes:
            print("DINH VI THAT BAI")
          #  Uti.RobotSpeakWithPath('khongtimthayhinhtron.mp3')
            self.label_status.setText("Robot định vị thất bại")
            self.ui25.label_ui_dv.setText("Robot định vị thất bại")
            self.ui25.Btn_ui_xn_dv.setEnabled(True)
            return False
        else:
            print("DINH VI XONG HOAN TOAN")
            self.label_status.setText("Robot định vị thành công")
            self.ui25.label_ui_dv.setText("Robot định vị thành công")
            self.ui25.Btn_ui_xn_dv.setEnabled(True)
            # Đóng cửa sổ thông báo sau khi hoàn thành
            #msg_box.close()
            return result         

################## Navigate thread  ##################
    def kiem_tra_du_dieu_khien_chay(self):
    #    self.read_arduino.start()
    #    self.read_arduino.doc_pin.connect(self.trangthai_pin)
        ## Kiem tra dieu khien pin
    #    print("du Pin")
   
        ## Kiem tra da nhap ten phong
    #    if( self.navigation_thread.du_pin==True):
   #         self.btn_batdau_danduong.setEnabled(True)
    #        self.read_arduino.stop()
    #        print("du Pin")
     #   else:
     #       self.btn_batdau_danduong.setEnabled(False)
    #        Uti.RobotSpeakWithPath('voice_hmi_new/vuilongsacpin.wav')
      #      self.read_arduino.stop()
     #       print("Thieu Pin")
        self.btn_batdau_danduong.setEnabled(True)

    
    def button_xn_pressed(self):
           if(self.id_voice==200):
            self.stackedWidget.setCurrentWidget(self.page_main)
            self.dinh_vi_fcn_1()
           elif( self.id_voice==100):
            self.stackedWidget.setCurrentWidget(self.page_main)
           else:
            pass

    def reached_goal(self, flag):
    
            if flag:
                self.btn_xac_nhan.setEnabled(True)
                if self.id_voice == RobConf.HOME_ID:
                    Uti.RobotSpeakWithPath('new_voice/new_hoan_thanh.wav')
                elif self.id_voice == 1000:
                    pass
                else:
                    Uti.RobotSpeakWithPath('voice_hmi_new/new_xac_nhan.wav')
                    self.listWidget_ds.takeItem(0)
                self.di_dendiem_DV = True  
            else: 
            
                print("fail go")
                self.di_dendiem_DV = False
               # self.man_hinh_dan_duong()
  
    def ve_home_trong_chu_trinh(self):
        # Lấy tọa độ từ database (vẫn dùng hàm cũ nếu logic tọa độ nằm riêng)
        # Giả định sql.doc_du_lieu_toado_robot trả về x,y,z,w,idout
        # Nếu hàm này vẫn dùng toado table, thì giữ nguyên
        x_goal, y_goal, z_goal, w_goal, idout = sql.doc_du_lieu_toado_robot(200)

        time.sleep(1)
        print('Go HOME')
        Uti.RobotSpeakWithPath('voice_hmi_new/ve_home.wav')
        self.navigation_thread.clear_done_navigation_status()
        self.navigation_thread.id = idout
        self.navigation_thread.quatrinh_move = True
        self.ros2_handle.goal_publisher.send_goal(x_goal, y_goal, z_goal, w_goal)

        self.listWidget_ds.clear()
        for id_item in self.item_states:
            self.item_states[id_item] = False
        self.checkBox_kvc.setCheckState(Qt.Unchecked)

    def ve_home_khan_cap(self):
        x_goal, y_goal, z_goal, w_goal, idout = sql.doc_du_lieu_toado_robot(200)
        self.ros2_handle.goal_publisher.send_goal(x_goal, y_goal, z_goal, w_goal)

    def ve_dock_sac(self):
        x_goal, y_goal, z_goal, w_goal, idout = sql.doc_du_lieu_toado_robot(100)
        time.sleep(1)
        print('Go DOCK')
        Uti.RobotSpeakWithPath('voice_hmi_new/batdau_quatrinh_sac.wav')
        self.navigation_thread.clear_done_navigation_status()
        self.navigation_thread.id = idout
        self.navigation_thread.quatrinh_move = True
        self.ros2_handle.goal_publisher.send_goal(x_goal, y_goal, z_goal, w_goal)
        Uti.RobotSpeakWithPath('voice_hmi_new/toidangdisac.wav')
        self.listWidget_ds.clear()
        for id_item in self.item_states:
            self.item_states[id_item] = False

    def auto_clear(self,id):
        x_goal, y_goal, z_goal, w_goal, idout = sql.doc_du_lieu_toado_robot(id)
        time.sleep(1)
        self.navigation_thread.clear_done_navigation_status()
        self.navigation_thread.id = idout
        self.navigation_thread.quatrinh_move = True
        self.ros2_handle.goal_publisher.send_goal(x_goal, y_goal, z_goal, w_goal)

    def man_hinh_dan_duong(self):
        self.btn_xac_nhan.setEnabled(False)
        goals_text = [self.listWidget_ds.item(index).text() for index in range(self.listWidget_ds.count())]
        print('Goals (text): ', goals_text)

        self.goal_ids = []
        for goal_text in goals_text:
            # Lấy ID từ tên hiển thị bằng cách đảo ngược quá trình
            # Cách này hơi phức tạp vì bạn cần dò lại trong DB hoặc trong logic chuyển đổi
            # Một cách tốt hơn là lưu ID cùng với tên trong ListWidget (ví dụ, dùng QListWidgetItem.setData)
            # Tạm thời, chúng ta sẽ cố gắng suy luận ID từ tên
            
            # Ưu tiên các ID đặc biệt
            if goal_text == "Trạm sạc":
                self.goal_ids.append(100)
            elif goal_text == "Khu vực chờ":
                self.goal_ids.append(RobConf.HOME_ID) # 200
            else:
                # Cố gắng tìm ID trong DB dựa trên tên
                found_id = None
                for i in range(1, 11): # Giả định các ID phòng từ 1 đến 10
                    db_name = sql.get_location_name_from_db(i)
                    if db_name == goal_text:
                        found_id = i
                        break
                if found_id is not None:
                    self.goal_ids.append(found_id)
                else:
                    # Nếu không tìm thấy trong DB, giả định là "Phòng X"
                    try:
                        # Lấy số X từ "Phòng X"
                        phong_id = int(goal_text.split()[-1])
                        self.goal_ids.append(phong_id)
                    except (ValueError, IndexError):
                        print(f"Cảnh báo: Không thể phân tích ID từ '{goal_text}'. Bỏ qua.")
                        # Hoặc bạn có thể gán một ID mặc định/lỗi
                        pass # Bỏ qua mục này nếu không thể phân tích

        print('Goal ids: ', self.goal_ids)

        if goals_text:
            first_item_text = goals_text[0]
            self.label_tt.setText(first_item_text)

        if not self.goal_ids:
            if(self.sac_pin_tu_dong==True):
                self.label_tt.setText("Trạm sạc")
                self.id_voice = 100
                self.ve_dock_sac()
            else:
                self.label_tt.setText("Khu vực chờ")
                self.id_voice = 200
                self.ve_home_trong_chu_trinh()
        else:
            Uti.RobotSpeakWithPath('voice_hmi_new/new_nhuong_duong.wav')
            
            # Sử dụng self.goal_ids[0] để xác định đích đến
            current_goal_id = self.goal_ids[0]

            if (current_goal_id == RobConf.HOME_ID): # ID 200
                self.id_voice = 200
                self.ve_home_trong_chu_trinh()
            elif(current_goal_id == 100):
                self.id_voice = 100
                self.ve_dock_sac()
            else:
                self.id_voice = current_goal_id # Giữ nguyên ID phòng
                self.auto_clear(current_goal_id)
          
      #  else:
          #  Uti.RobotSpeakWithPath('voice_hmi_new/toidangdisac.wav')
          #  print('toi dang di sac')
                   
################## Checklist  ##################
 #Setup list
    def export_to_excel(self):
        export_path = "/home/robot/ROBOT_HD/Excel"

        try:
            workbook = Workbook()
            sheet = workbook.active

            # Thêm header (bỏ qua cột checkbox ở index 0)
            header_labels = [self.tableWidget_2.horizontalHeaderItem(i).text() for i in range(1, self.tableWidget_2.columnCount())]
            sheet.append(header_labels)

            # Thêm toàn bộ dữ liệu từ tableWidget_2 (bỏ qua cột checkbox)
            num_rows = self.tableWidget_2.rowCount()
            num_cols = self.tableWidget_2.columnCount()
            for row in range(num_rows):
                row_data = []
                for col in range(1, num_cols):  # Bắt đầu từ cột 1 để bỏ qua checkbox
                    item = self.tableWidget_2.item(row, col)
                    row_data.append(item.text() if item else "")
                sheet.append(row_data)

            # Tạo tên file dựa trên ngày hiện tại
            now = datetime.now()
            file_name = f"exported_all_data_{now.strftime('%Y%m%d_%H%M%S')}.xlsx"
            full_file_path = os.path.join(export_path, file_name)

            # Đảm bảo thư mục tồn tại
            os.makedirs(export_path, exist_ok=True)

            workbook.save(full_file_path)
            self.terminal_3.setPlainText(f"All data exported successfully to: {full_file_path}")

        except ImportError:
            QMessageBox.critical(self, "Error", "The 'openpyxl' library is not installed. Please install it using 'pip install openpyxl'.")
            self.terminal_3.setPlainText("Error: 'openpyxl' not installed.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred during export: {e}")
            self.terminal_3.setPlainText(f"Error during export: {e}")
   # HAM DUNG DE SET CHIEU RONG CUA COT
    def set_column_widths_1(self):
        self.tableWidget_2.setColumnWidth(0, 85)   # Cột checkbox
        self.tableWidget_2.setColumnWidth(1, 100)  # Cột ID
        self.tableWidget_2.setColumnWidth(2, 150)  # Cột NAME
        self.tableWidget_2.setColumnWidth(3, 100)  # Cột ROOM
        self.tableWidget_2.setColumnWidth(4, 150)  # Cột DateTime checkin
        self.tableWidget_2.setColumnWidth(5, 150)  # Cột DateTIme checkout
        self.tableWidget_2.setColumnWidth(6, 350)  # Cột image checkout
        self.tableWidget_2.setColumnWidth(7, 350)  # Cột image checkout
    # SET CHIEU CAO CUA HANG
    def set_row_heights_1(self):
        for row in range(self.tableWidget_2.rowCount()):
            self.tableWidget_2.setRowHeight(row, 30)  # Chiều cao mỗi hàng là 30 pixel

    # HAM LOAD DATA LEN BANG
    def fcn_load_list(self):
        data = sql.doc_du_lieu_list('readall')

        if data is not None:
            num_rows = len(data)
            num_columns = 8  # 7 dữ liệu và 1 checkbox

            self.tableWidget_2.clear()  # Xóa dữ liệu cũ trước khi tải lại
            self.tableWidget_2.setRowCount(num_rows)
            self.tableWidget_2.setColumnCount(num_columns)
            self.tableWidget_2.setHorizontalHeaderLabels(['Select', 'ID', 'Name', 'Room', 'DateTime_Checkin', 'DateTime_CheckOut', 'Image_Checkin', ' Image_Checkout'])

            if num_rows > 0:
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
                    checkbox.stateChanged.connect(self.get_selected_rows_2)

                    # Thêm widget checkbox vào bảng
                    self.tableWidget_2.setCellWidget(row_idx, 0, checkbox_widget)

                    # Thêm dữ liệu vào các cột còn lại và căn giữa
                    for col_idx, item in enumerate(row_data):
                        item_widget = QTableWidgetItem(str(item))
                        item_widget.setTextAlignment(Qt.AlignCenter)
                        self.tableWidget_2.setItem(row_idx, col_idx + 1, item_widget)

                self.set_column_widths_1()
                self.set_row_heights_1()

            self.terminal_3.setPlainText("Data loaded successfully")
        else:
            self.tableWidget_2.clear()
            self.tableWidget_2.setRowCount(0)
            self.tableWidget_2.setColumnCount(8)  # Vẫn giữ số cột để hiển thị header
            self.tableWidget_2.setHorizontalHeaderLabels(['Select', 'ID', 'Name', 'Room', 'DateTime_Checkin', 'DateTime_CheckOut', 'Image_Checkin', ' Image_Checkout'])
            self.terminal_3.setPlainText("Failed to load data.")

    # HAM CHON CHECKBOX

    def get_selected_rows_2(self):
        self.selected_rows = []
        num_rows = self.tableWidget_2.rowCount()
        for row in range(num_rows):
            # Lấy widget checkbox từ ô
            checkbox_widget = self.tableWidget_2.cellWidget(row, 0)
            if checkbox_widget and isinstance(checkbox_widget, QWidget):
                checkbox = checkbox_widget.findChild(QCheckBox)
                if checkbox and checkbox.isChecked():
                    # Thêm chỉ số hàng vào mảng, +1 để chỉ số bắt đầu từ 1
                    self.selected_rows.append(row + 1)
        
        print("Selected rows:", self.selected_rows)
        return self.selected_rows

    # HAM XOA DU LIEU
    def fcn_delete_list_all(self):
            """Hiển thị hộp thoại xác nhận xóa tất cả dữ liệu và thực hiện xóa nếu người dùng đồng ý."""
            reply = QMessageBox.question(
                None,  # Hoặc widget cha nếu có
                'Xác nhận xóa',
                'Bạn có chắc chắn muốn xóa *tất cả* dữ liệu trong bảng LIST không?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No  # Nút mặc định được chọn
            )

            if reply == QMessageBox.Yes:
                sql.delete_all_data_list()
                self.fcn_load_list()
                self.terminal_3.setPlainText("Delete all data successfully")
                
            else:
                self.terminal_3.setPlainText("Deletion cancelled by user.")
    
            self.fcn_load_list_0()
            self.fcn_load_list_1()     
    
    def UI_xn_chupanh(self):
        self.chup_hinh = QMainWindow()
        self.ui20 = Ui_Form_xn_chupanh()
        self.ui20.setupUi(self.chup_hinh)
        self.ui20.btn_finish_camera.clicked.connect(self.handle_chup_anh)
        self.chup_hinh.show()

    def save_data_chupanh(self):
        self.UI_xn_chupanh()

    def capture_image(self):
        """Chụp ảnh điểm danh và lưu đường dẫn."""
        user_id = getattr(self, "current_id", None)

        if user_id is None:
            QMessageBox.warning(self, "Lỗi", "Không có ID hiện tại để lưu ảnh.")
            return

        if not self.checkin_active and not self.checkout_active:
            QMessageBox.warning(self, "Cảnh báo", "Vui lòng chỉ định loại điểm danh (Check In hoặc Check Out).")
            return

        if self.checkin_active and self.checkout_active:
            QMessageBox.warning(self, "Cảnh báo", "Không thể đồng thời Check In và Check Out.")
            return

        filename = f"{user_id}_1.png" if self.checkin_active else f"{user_id}_0.png"

        if self.checkin_active:
            folder_path = "/home/robot/ROBOT_HD/imageList/checkin"
        elif self.checkout_active:
            folder_path = "/home/robot/ROBOT_HD/imageList/checkout"
        else:
            return

        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        save_path = os.path.join(folder_path, filename)
        

        ret, frame = self.cap.read()
        if ret:
            cv2.imwrite(save_path, frame)
            QMessageBox.information(self, "Thông báo", f"Đã lưu ảnh thành công")
            self.image_save_path = save_path # Lưu đường dẫn ảnh
            self.back_setup()
        else:
            QMessageBox.warning(self, "Lỗi", "Không thể chụp ảnh.")

    def check_checkbox_state(self, state):
        if state == Qt.Checked:
            self.btn_opencamera_2.setEnabled(True)
        else:
            self.btn_opencamera_2.setEnabled(False)
    def toggle_camera(self):
        if self.camera_running:
            self.close_camera()
        else:
            self.start_camera()
    def uncheck_checkout(self, checked):
        if checked:
            self.checkbox_checkout_2.setChecked(False)
    
    def uncheck_checkin(self, checked):
        if checked:
            self.checkbox_checkin_2.setChecked(False)
      
    def update_checkin_state(self, checked):
        self.checkin_active = checked

    def update_checkout_state(self, checked):
        self.checkout_active = checked

    def close_camera(self):
        self.camera_running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.cap = None
        self.chua_camera.clear()
        self.chua_camera_3.setText("Camera đã đóng")
        self.last_qr_text = "Không tìm thấy mã QR"

    def start_camera(self):
        self.camera_running = True
        self.cap = cv2.VideoCapture(6)
        if not self.cap or not self.cap.isOpened():
            QtWidgets.QMessageBox.critical(self, "Lỗi", "Không mở được camera.")
            return
        self.timer.start(30)
        self.chua_camera_3.setText(self.last_qr_text) # Hiển thị lại text QR cuối cùng

    def update_frame(self):
        if not self.cap or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        frame, qr_data = self.process_qr(frame)

        # Hiển thị ảnh camera
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qt_img = QtGui.QImage(rgb_frame.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.chua_camera.setPixmap(QtGui.QPixmap.fromImage(qt_img))

        # Nếu có dữ liệu QR và camera đang chạy, lưu vào buffer
        if qr_data and self.camera_running:
            self.qr_data_buffer = qr_data

    def process_qr(self, frame):
        decoded_objects = decode(frame)
        result_text = ""
        qr_data_dict = {}  # Dictionary để lưu trữ dữ liệu QR đã tách

        for obj in decoded_objects:
            # Giải mã dữ liệu
            try:
                qr_raw_data = obj.data.decode('utf-8')
            except UnicodeDecodeError:
                qr_raw_data = obj.data.hex()

            # Tách từng dòng và lưu vào dictionary
            lines = qr_raw_data.strip().split('\n')
            for line in lines:
                if ":" in line:
                    key, value = line.split(":", 1)
                    qr_data_dict[key.strip()] = value.strip()
                else:
                    # Xử lý các dòng không có dấu ":" nếu cần
                    pass
      
                
            # Tạo text hiển thị
            display_lines = []
            if "Tên" in qr_data_dict:
                display_lines.append(f"Họ và tên :{qr_data_dict['Tên']}")
            if "ID" in qr_data_dict:
                display_lines.append(f"ID :{qr_data_dict['ID']}")
                self.current_id = qr_data_dict["ID"]
            if "Điểm" in qr_data_dict:
                display_lines.append(f"Điểm :{qr_data_dict['Điểm']}")

            result_text = "\n".join(display_lines)

            # Vẽ khung quanh QR code
            points = obj.polygon
            if len(points) > 4:
                hull = cv2.convexHull(np.array([p for p in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = points

            n = len(hull)
            for j in range(n):
                cv2.line(frame, hull[j], hull[(j + 1) % n], (0, 255, 0), 2)

        # Cập nhật QLabel chứa kết quả
        if result_text:
            self.chua_camera_3.setText(result_text.strip())
            self.last_qr_text = result_text.strip() # Lưu lại text QR
            return frame, qr_data_dict  # Trả về dictionary chứa dữ liệu QR
        else:
            self.chua_camera_3.setText(self.last_qr_text) # Giữ nguyên text QR cuối cùng
            return frame, None
    def handle_chup_anh(self):
        self.capture_image()
        if self.image_save_path:
            # Cập nhật đường dẫn ảnh vào qr_data_buffer để lưu vào database
            if self.qr_data_buffer:
                if self.checkbox_checkin_2.isChecked() and self.is_checkin_processed:
                    self.qr_data_buffer["Image_Checkin"] = self.image_save_path
                    self.add_qr_data_to_db(self.qr_data_buffer, checkin=True) # Gọi lưu DB sau khi chụp
                    self.is_checkin_processed = False # Reset flag sau khi lưu
                elif self.checkbox_checkout_2.isChecked() and self.is_checkout_processed:
                    self.qr_data_buffer["Image_Checkout"] = self.image_save_path
                    self.add_qr_data_to_db(self.qr_data_buffer, checkout=True) # Gọi lưu DB sau khi chụp
                    self.is_checkout_processed = False # Reset flag sau khi lưu
            # Đóng cửa sổ sau khi chụp
            button = self.sender()
            window = button.window()
            window.close()
            self.close_camera()
            self.image_save_path = None # Reset đường dẫn ảnh
        else:
            QMessageBox.warning(self, "Lỗi", "Không thể lưu đường dẫn ảnh.")
    def check_existing_checkin_checkout(self, user_id):
        """Kiểm tra xem ID đã có dữ liệu check-in hoặc check-out hay chưa."""
        has_checkin = False
        has_checkout = False
        try:
            mydb = mysql.connector.connect(
                user="robot",
                password="12345678",
                host="127.0.0.1",
                database="sql_rsr"
            )
            mycursor = mydb.cursor()
            sql_check = "SELECT DateTime_Checkin, DateTime_Checkout FROM LIST WHERE ID = %s"
            mycursor.execute(sql_check, (user_id,))
            existing_record = mycursor.fetchone()
            if existing_record:
                checkin_time, checkout_time = existing_record
                if checkin_time is not None:
                    has_checkin = True
                if checkout_time is not None:
                    has_checkout = True
        except mysql.connector.Error as err:
            print(f"Lỗi kết nối MySQL: {err}")
        finally:
            if mydb.is_connected():
                mycursor.close()
                mydb.close()
        return has_checkin, has_checkout
    def process_and_add_qr_data(self):
        """Xử lý dữ liệu QR đã quét và thêm vào cơ sở dữ liệu dựa trên checkbox.
        Ngăn chặn mở UI chụp ảnh nếu đã có dữ liệu check-in hoặc check-out tương ứng.
        """
        if self.qr_data_buffer:
            id_to_check = self.qr_data_buffer.get("ID")
            if id_to_check:
                has_checkin, has_checkout = self.check_existing_checkin_checkout(id_to_check)

                if self.checkbox_checkin_2.isChecked():
                    if has_checkin:
                        QMessageBox.warning(self, "Cảnh báo", f"ID {id_to_check} đã có dữ liệu check-in trước đó.")
                        return
                  
                    self.UI_xn_chupanh() # Mở UI chụp ảnh cho check-in
                    self.is_checkin_processed = True
                    self.is_checkout_processed = False # Reset checkout flag
                elif self.checkbox_checkout_2.isChecked():
                    if has_checkout:
                        QMessageBox.warning(self, "Cảnh báo", f"ID {id_to_check} đã có dữ liệu check-out trước đó.")
                        
                        return
                
                    self.UI_xn_chupanh() # Mở UI chụp ảnh cho check-out (nếu cần)
                    self.is_checkout_processed = True
                    self.is_checkin_processed = False # Reset checkin flag
                else:
                    QMessageBox.warning(self, "Cảnh báo", "Vui lòng chọn Check In hoặc Check Out.")
                    return
            else:
                QMessageBox.warning(self, "Cảnh báo", "Không tìm thấy ID trong dữ liệu QR.")
                return
        else:
            QMessageBox.warning(self, "Cảnh báo", "Chưa có dữ liệu QR nào được quét.")
    def add_qr_data_to_db(self, qr_data, checkin=False, checkout=False):
        """Thêm hoặc cập nhật dữ liệu QR code vào cơ sở dữ liệu, bao gồm đường dẫn ảnh.
        Ngăn chặn lưu ảnh và cập nhật nếu đã có dữ liệu check-in hoặc check-out tương ứng.
        """
        try:
            mydb = mysql.connector.connect(
                user="robot",
                password="12345678",
                host="127.0.0.1",
                database="sql_rsr"
            )
            mycursor = mydb.cursor()
        except mysql.connector.Error as err:
            print(f"Lỗi kết nối MySQL: {err}")
            return

        try:
            if "ID" in qr_data and "Tên" in qr_data and "Điểm" in qr_data:
                id_to_check = qr_data["ID"]
                image_checkin_path = qr_data.get("Image_Checkin")
                image_checkout_path = qr_data.get("Image_Checkout")

                sql_check = "SELECT ID, DateTime_Checkin, DateTime_Checkout, Image_Checkin, Image_Checkout FROM LIST WHERE ID = %s"
                mycursor.execute(sql_check, (id_to_check,))
                existing_record = mycursor.fetchone()

                if existing_record:
                    record_id, checkin_time, checkout_time, existing_image_in, existing_image_out = existing_record
                    if checkin:
                        if checkin_time is not None or existing_image_in is not None:
                            print(f"Cảnh báo: ID {id_to_check} đã có dữ liệu check-in trước đó. Không lưu lại.")
                            QMessageBox.warning(self, "Cảnh báo", f"ID {id_to_check} đã được check-in trước đó.")
                            return
                        else:
                            update_fields = ["DateTime_Checkin = %s"]
                            update_values = [datetime.now()]
                            if image_checkin_path:
                                update_fields.append("Image_Checkin = %s")
                                update_values.append(image_checkin_path)

                            sql_update = f"UPDATE LIST SET {', '.join(update_fields)} WHERE ID = %s"
                            mycursor.execute(sql_update, (*update_values, id_to_check))
                            mydb.commit()
                            print(f"Đã cập nhật check-in cho ID: {id_to_check} với ảnh (nếu có).")
                    elif checkout:
                        if checkout_time is not None or existing_image_out is not None:
                            print(f"Cảnh báo: ID {id_to_check} đã có dữ liệu check-out trước đó. Không lưu lại.")
                            QMessageBox.warning(self, "Cảnh báo", f"ID {id_to_check} đã được check-out trước đó.")
                            return
                        else:
                            update_fields = ["DateTime_Checkout = %s"]
                            update_values = [datetime.now()]
                            if image_checkout_path:
                                update_fields.append("Image_Checkout = %s")
                                update_values.append(image_checkout_path)

                            sql_update = f"UPDATE LIST SET {', '.join(update_fields)} WHERE ID = %s"
                            mycursor.execute(sql_update, (*update_values, id_to_check))
                            mydb.commit()
                            print(f"Đã cập nhật check-out cho ID: {id_to_check} với ảnh (nếu có).")
                else:
                    insert_fields = ["ID", "Name", "Room"]
                    insert_values = [qr_data["ID"], qr_data["Tên"], qr_data["Điểm"]]
                    if checkin and image_checkin_path:
                        insert_fields.append("DateTime_Checkin")
                        insert_values.append(datetime.now())
                        insert_fields.append("Image_Checkin")
                        insert_values.append(image_checkin_path)
                    elif checkout and image_checkout_path:
                        insert_fields.append("DateTime_Checkout")
                        insert_values.append(datetime.now())
                        insert_fields.append("Image_Checkout")
                        insert_values.append(image_checkout_path)
                    elif checkin:
                        insert_fields.append("DateTime_Checkin")
                        insert_values.append(datetime.now())
                    elif checkout:
                        insert_fields.append("DateTime_Checkout")
                        insert_values.append(datetime.now())

                    sql_insert = f"INSERT INTO LIST ({', '.join(insert_fields)}) VALUES ({', '.join(['%s'] * len(insert_fields))})"
                    mycursor.execute(sql_insert, insert_values)
                    mydb.commit()
                    print(f"Đã thêm dữ liệu ID: {qr_data['ID']} với thông tin check-in/check-out và ảnh (nếu có).")

            else:
                print("Lỗi: Dữ liệu QR không đầy đủ các trường cần thiết (ID, Tên, Điểm).")
        except mysql.connector.Error as err:
            mydb.rollback()
            print(f"Lỗi thực thi SQL: {err}")
        finally:
            if mydb.is_connected():
                mycursor.close()
                mydb.close()

    def closeEvent(self, event):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.timer.stop()
        event.accept()
    # Load du lieu danh sach
    def set_column_widths_2(self):
       
        self.tableWidget_3.setColumnWidth(0, 100)  # Cột ID
        self.tableWidget_3.setColumnWidth(1, 170)  # Cột NAME
        self.tableWidget_3.setColumnWidth(2, 100)  # Cột ROOM
        self.tableWidget_3.setColumnWidth(3, 160)  # Cột DateTime checkin
        self.tableWidget_3.setColumnWidth(4, 160)  # Cột DateTIme checkout

    # SET CHIEU CAO CUA HANG
    def set_row_heights_2(self):
        for row in range(self.tableWidget_3.rowCount()):
            self.tableWidget_3.setRowHeight(row, 30)  # Chiều cao mỗi hàng là 30 pixel

    # HAM LOAD DATA LEN BANG
    def fcn_load_list_1(self):
        self.tableWidget_3.clear()  # Xóa dữ liệu cũ trước khi tải lại
        data = sql.doc_du_lieu_list('readall')

        if data is not None:
            num_rows = len(data)
            num_columns = 5  # Đúng số cột dữ liệu

            self.tableWidget_3.clear()  # Xóa dữ liệu cũ trước khi tải lại
            self.tableWidget_3.setRowCount(num_rows)
            self.tableWidget_3.setColumnCount(num_columns)
            self.tableWidget_3.setHorizontalHeaderLabels(['ID', 'Name', 'Room', 'DateTime_Checkin', 'DateTime_CheckOut'])

            if num_rows > 0:
                for row_idx, row_data in enumerate(data):

                    # Thêm dữ liệu vào các cột và căn giữa
                    for col_idx, item in enumerate(row_data):
                        item_widget = QTableWidgetItem(str(item))
                        item_widget.setTextAlignment(Qt.AlignCenter)
                        self.tableWidget_3.setItem(row_idx, col_idx, item_widget)  # Sử dụng col_idx trực tiếp

                self.set_column_widths_2()
                self.set_row_heights_2()

            else:
                self.tableWidget_3.clear()
                self.tableWidget_3.setRowCount(0)
                self.tableWidget_3.setColumnCount(5)  # Đúng số cột dữ liệu
                self.tableWidget_3.setHorizontalHeaderLabels(['ID', 'Name', 'Room', 'DateTime_Checkin', 'DateTime_CheckOut'])

################## Tra cuu  ##################
    def speak_vietnamese_gg(self,text): 
        
        try:
            tts = gTTS(text=text, lang='vi')
            filename = "temp_speech.mp3"
            tts.save(filename)
            playsound(filename)
            os.remove(filename)
        except Exception as e:
            print(e)
    def speak_vietnamese_gg_ai(self,text): 
        
        try:
            tts = gTTS(text=text, lang='vi')
            filename = "temp_speech_ai.mp3"
            tts.save(filename)
            playsound(filename)
            os.remove(filename)
        except Exception as e:
            print(e)

    def load_excel(self):
        
        try:
            df = pd.read_excel("data_v1.xlsx", header=None, engine='openpyxl')
            df = df.iloc[:, :2]
            df.columns = ['Câu hỏi', 'Câu trả lời']
            df.dropna(how="any", inplace=True)

            self.tc_nha_truong_2.clear()
            for _, row in df.iterrows():
                item = QListWidgetItem(row['Câu hỏi'])
                item.setData(Qt.UserRole, row['Câu trả lời'])
                item.setTextAlignment(Qt.AlignLeft)  # Canh giữa
                self.tc_nha_truong_2.addItem(item)
        except Exception as e:
            QMessageBox.critical(self, "Lỗi", f"Không đọc được file Excel: {str(e)}")

    def on_question_clicked(self):
        item = self.tc_nha_truong_2.currentItem()
        if item:
            answer = item.data(Qt.UserRole)
            test = answer
            print(test)
            self.tra_loi_tc_nha_truong_4.setText(test)

            if self.checkBox_giong_noi_5.isChecked():
                self.tra_loi_tc_nha_truong_4.setText(answer)
                self.speak_vietnamese_gg(answer)

    def home_nha_truong(self):

        self.tt_ai = False 
        self.cau_hoi_ai.clear()
        self.tra_loi_ai.clear()
        self.tra_loi_ai.setText(" Cảm ơn bạn đã sử dụng.")
     
        phat_tieng_viet("Cảm ơn bạn đã sử dụng.")        
                        
        self.bt_stop_ai.setEnabled(False)
        pygame.mixer.init()
        pygame.mixer.music.stop()
        print(" Đã dừng phát âm thanh!")
        self.stackedWidget.setCurrentWidget(self.page_tra_cuu)
  

    def reload_ai(self):
        # Dừng phát âm thanh nếu đang chạy
        pygame.mixer.init()
        pygame.mixer.music.stop()
        print(" Đã dừng phát âm thanh!")

        # Xóa file âm thanh tạm (nếu tồn tại)
        if hasattr(self, "out_file") and os.path.exists(self.out_file):
            os.remove(self.out_file)
            print(f" Đã xóa file tạm: {self.out_file}")

        # Reset nội dung câu hỏi trên giao diện
        self.cau_hoi_ai.clear()
        self.tra_loi_ai.clear()
        print("Đã reset, sẵn sàng nhập câu hỏi mới!")

        thread_stop = threading.Thread(target=self.lang_nghe_reload, daemon=True)
        thread_stop.start()
        
    def tra_cuu_ai (self):
        self.stackedWidget.setCurrentWidget(self.page_ai)
        self.tt_ai = True
        self.nghe_lenh_kich_hoat()
        self.cau_hoi_ai.clear()
        self.tra_loi_ai.clear()

    def lang_nghe_reload(self):
        r = sr.Recognizer()
        mic = sr.Microphone()

        with mic as source:
            r.adjust_for_ambient_noise(source)

        while self.tt_ai:  # Đang hoạt động
            with mic as source:
                try:
                    print("Đang chờ lệnh 'reload'...")
                    audio = r.listen(source, timeout=5)
                    text = r.recognize_google(audio, language="vi-VN").lower()
                    #print("Luồng kết thúc nhận :", text)

                    if "dừng lại" in text:
                        # Dừng phát âm thanh nếu đang chạy
                        pygame.mixer.init()
                        pygame.mixer.music.stop()
                        print(" Đã dừng phát âm thanh!")

                        # Xóa file âm thanh tạm (nếu tồn tại)
                        if hasattr(self, "out_file") and os.path.exists(self.out_file):
                            os.remove(self.out_file)
                            print(f" Đã xóa file tạm: {self.out_file}")

                        # Reset nội dung câu hỏi trên giao diện
                        self.cau_hoi_ai.clear()
                        self.tra_loi_ai.clear()
                        print("Đã reset, sẵn sàng nhập câu hỏi mới!")
                        
                        break
                except:
                    continue

    def nghe_lenh_kich_hoat(self):
        def run():
            r = sr.Recognizer()
            mic = sr.Microphone()

            with mic as source:
                r.adjust_for_ambient_noise(source)
                self.tra_loi_ai.setText(" Bạn hãy nói 'Chào Robot' để bắt đầu...")
                phat_tieng_viet("Bạn hãy nói 'Chào Robot' để bắt đầu..")
                
            
            while (self.tt_ai == True) :
                print("loop1")
                with mic as source:
                    try:
                        r.pause_threshold = 1.5
                        print("Đang đợi câu 'Chào Robot'...")
                        audio = r.listen(source)
                        text = r.recognize_google(audio, language="vi-VN").lower()
                        print("Bạn nói:", text)

                        if "chào robot" in text:
                            self.tra_loi_ai.setText("   Xin Chào ! Tôi là Robot lễ tân,bạn hãy đặt câu hỏi hoặc nói,bấm kết thúc để dừng ")
                            #speak_vietnamese_gg("  Xin Chào ! Tôi là Robot lễ tân,bạn hãy đặt câu hỏi hoặc nói , bấm kết thúc để dừng ")
                            phat_tieng_viet("  Xin Chào ! Tôi là Robot lễ tân,bạn hãy đặt câu hỏi hoặc nói, bấm kết thúc để dừng ")
                            self.bt_stop_ai.setEnabled(True)
                            QApplication.processEvents()
                            # Bắt đầu thread nghe lệnh "kết thúc"
                           # threading.Thread(target=self.nghe_nhieu_cau_hoi, daemon=True).start()
                            self.nghe_nhieu_cau_hoi()

                    except sr.WaitTimeoutError:
                        continue
                    except sr.UnknownValueError:
                        continue
                    except Exception as e:
                        print(f"Lỗi: {e}")

        threading.Thread(target=run, daemon=True).start()
    
    def nghe_nhieu_cau_hoi(self):
        r = sr.Recognizer()
        mic = sr.Microphone()

        thread_stop = threading.Thread(target=self.lang_nghe_reload, daemon=True)
        thread_stop.start()
        with mic as source:
            r.adjust_for_ambient_noise(source)

        while (self.tt_ai == True) :
            print("loop2")
            with mic as source:
                try:
                    self.tra_loi_ai.setText(" Tôi đang nghe câu hỏi...")
                    QApplication.processEvents()

                    audio = r.listen(source, timeout=7)
                    text = r.recognize_google(audio, language="vi-VN").lower()
                    print("Bạn nói :", text)
                
                    if ("kết thúc" in text):  
                        self.tra_loi_ai.setText(" Cảm ơn bạn đã sử dụng.")
                    
                        phat_tieng_viet("Cảm ơn bạn đã sử dụng.")
                        #speak_vietnamese_gg(" Cảm ơn bạn đã sử dụng.")
                        self.tt_ai = False 
                        self.cau_hoi_ai.clear()
                        self.tra_loi_ai.clear()
                        self.stackedWidget.setCurrentWidget(self.page_tra_cuu)
                        self.bt_stop_ai.setEnabled(False)
                        break

                    else:
                        self.cau_hoi_ai.setText(f" Bạn hỏi : {text}")
                        print("dang xu li AI")
                        self.xu_ly_cau_hoi(text) 
                        QApplication.processEvents()
                      
                     
                               
                except sr.WaitTimeoutError:
                    #speak_vietnamese_gg(" Tôi không nghe thấy gì, bạn vui lòng nói xong và đợi 1 giây để có câu trả lời nhé ")
                    phat_tieng_viet(" Tôi không nghe thấy gì, bạn vui lòng nói xong và đợi 1 giây để có câu trả lời nhé ")
                    print(" Không nghe thấy gì, tiếp tục...")
                    continue
                except sr.UnknownValueError:
                    print(" Không hiểu, xin nói lại...")
                    #speak_vietnamese_gg("Tôi nghe không rõ,xin nói lại ")
                    phat_tieng_viet("Tôi nghe không rõ,xin nói lại ")
                    continue
                except Exception as e:
                    print(f" Lỗi: {e}")

    def xu_ly_cau_hoi(self, cau_hoi_text):
        self.tra_loi_ai.setText(" Đang xử lý câu hỏi...")
      
        QApplication.processEvents()
        try:
            response = self.model.generate_content(cau_hoi_text)
            if hasattr(response, "text") and response.text:

                tra_loi = response.text.strip()

                # Loại bỏ dấu **
                tra_loi = tra_loi.replace("**", "")
                # Loại bỏ dấu * đơn lẻ
                tra_loi = tra_loi.replace("*", "")
                self.tra_loi_ai.setText(f" {tra_loi}")
                self.tra_loi_ai.setWordWrap(True)  # Tự động ngắt dòng
                self.tra_loi_ai.setAlignment(Qt.AlignCenter)
                QApplication.processEvents()
                phat_tieng_viet(tra_loi)
                #speak_vietnamese_gg(tra_loi)
            else:
                self.tra_loi_ai.setText(" Không nhận được phản hồi.")
                QApplication.processEvents()
             

        except Exception as e:
            self.tra_loi_ai.setText(f" Lỗi: {str(e)}")
            QApplication.processEvents()

if __name__ == "__main__":
    app = QApplication(sys.argv)
  # window = MainApp()

 #  window.show()
    #window.showFullScreen()

    main_app = MainApp()
    loading_screen = LoadingScreen(main_app)
    login_screen = LoginScreen(loading_screen)  # Truyền loading_screen vào LoginScreen

    login_screen.show_login()
    sys.exit(app.exec_())