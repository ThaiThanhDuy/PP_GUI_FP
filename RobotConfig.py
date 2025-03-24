import sys
import mysql.connector
import SQL as sql

file_path_thongso_headcam = 'txt/thongso_cam.txt'
file_path_pose_data = 'txt/odom_data.txt'
file_path_trangthaiPin = 'txt/data_ard.txt'

#Charging
chargeCamera = "/dev/video0"
full_vol = 10.2
path_run_vaodocsac = 'python3 charging.py'

# MEDIA PATH
IMAGE_FOLDER = "images"
VOICE_FOLDER = "voices"

#Arduino
Mega2560_PORT = 'gnome-terminal -- rosrun rosserial_python serial_node.py /dev/usb_mega' 
MegaNANO_PORT = '/dev/usb_nano'
#Port_eye = '/dev/ttyUSB2'
#ROBOT_NODE = 'robottieptan_node'
RESET_NANO_CODE = 'reset\n'

# LINK MAP
MAP_FOLDER = '/home/robot/ros2_ws/src/library_robot2/maps'
Map_Lidar = MAP_FOLDER + '/my_map.pgm'
Path_yaml = MAP_FOLDER + '/my_map.yaml'

# ======================= CAC BIEN THEM MOI 20240923 ================================
# THONG SO CHO DIEU KHIEN MANUAL
VX_MANUAL_MAX = sql.doc_du_lieu_param_robot('manual_vt_thang')
VW_MANUAL_MAX = sql.doc_du_lieu_param_robot('manual_vt_xoay')
VX_AUTO_MAX = sql.doc_du_lieu_param_robot('auto_vt_thang')
VW_AUTO_MAX = sql.doc_du_lieu_param_robot('auto_vt_xoay')

DUNG_SAI_DIEM_DEN_VITRI = sql.doc_du_lieu_param_robot('sai_so_kc')#0.5 #m
DUNG_SAI_DIEM_DEN_GOC = sql.doc_du_lieu_param_robot('sai_so_goc')#9 #do

chieu_cao_led = sql.doc_du_lieu_param_robot('chieu_cao_led') #caidatrobot
MUC_DIEN_AP_SAC = sql.doc_du_lieu_param_robot('volt_sac_auto')

faceCame1 = sql.doc_du_lieu_param_robot('front_R')
headCamera = sql.doc_du_lieu_param_robot('headC')
chargCame = sql.doc_du_lieu_param_robot('backC')
faceCam2 = sql.doc_du_lieu_param_robot('front_L')
DELAY_TIME = sql.doc_du_lieu_robot_ros('delay_time')
USERNAME = sql.doc_du_lieu_param_robot('username')
PASSWORD = sql.doc_du_lieu_param_robot('password') #caidatrobot

#THONG SO ROBOT DO KY SU TINH TOAN =================
AUTO_VX_MAX = 3
AUTO_VW_MAX = 3
AUTO_VX_MIN = 0
AUTO_VW_MIN = 0
MANUAL_VX_MAX = 3
MANUAL_VW_MAX = 3
MANUAL_VX_MIN = 0
MANUAL_VW_MIN = 0
DUNG_SAI_KC_MAX = 1
DUNG_SAI_GOC_MAX = 20 # 20deg
CHIEU_CAO_LED_MAX = 600
CHIEU_CAO_LED_MIN = 0
MUC_DIEN_AP_SAC_MIN = 10 # 0% tuong duong 10 Volt => 
MUC_DIEN_AP_SAC_MAX = 12 #100% tuong duong 12 volt

# THONG SO MAC DINH CUA ROBOT =====================

USERNAME_DF = ''
PASSWORD_DF = ''
AUTO_VX_DF = 0.1*AUTO_VX_MAX
AUTO_VW_DF = 0.1*AUTO_VW_MAX
MANUAL_VX_DF = 0.1*MANUAL_VX_MAX
MANUAL_VW_DF = 0.1*MANUAL_VW_MAX
DUNG_SAI_KC_DF = 1*DUNG_SAI_KC_MAX
DUNG_SAI_GOC_DF = 1*DUNG_SAI_GOC_MAX
CHIEU_CAO_LED_DF  = 0.5*CHIEU_CAO_LED_MAX
MUC_DIEN_AP_DF = 10.2


admin = 'admin'
pw = '1'

#========= 
#ID DAC BIET CUA HE THONG
HOME_ID = 200
TruocDockSacID = 123
DockSacID = 100
KHOANG_CACH_VE_TRUOC_DOCKSAC = 0.5


# CAC CHE DO VAN TOC
LockedVelMode = 2
UnLockedVelMode = 5
ManualVelMode = 1
AutoVelMode = 0