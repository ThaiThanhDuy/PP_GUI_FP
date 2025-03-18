import sys
import mysql.connector
import Utilities as Uti

file_path_thongso_headcam = 'cam_odom/thongso_cam.txt'
file_path_pose_data = 'file_text_odom/odom_data.txt'
file_path_trangthaiPin = 'data_ard.txt'

chargeCamera = "/dev/video0"
full_vol = 10.2



db = mysql.connector.connect(user="robot", password='12345678', host='127.0.0.1',
                             database='sql_rsr')  # Thiết lập kết nối đến cơ sở dữ liệu

# MEDIA PATH
IMAGE_FOLDER = "images"
VOICE_FOLDER = "voices"

Mega2560_PORT = 'gnome-terminal -- rosrun rosserial_python serial_node.py /dev/usb_mega' 
MegaNANO_PORT = '/dev/usb_nano'
#Port_eye = '/dev/ttyUSB2'
ROBOT_NODE = 'robottieptan_node'

# LINK MAP
MAP_FOLDER = '/home/robot/ros2_ws/src/library_robot2/maps'
SAVE_MAP_PATH = '/home/robot/catkin_ws/src/navigation/maps/my_map'
Map_Lidar = MAP_FOLDER + '/my_map.pgm'
Path_yaml = MAP_FOLDER + '/my_map.yaml'

RESET_NANO_CODE = 'reset\n'

path_run_vaodocsac = 'python3 pid_new4_19_06_2024.py'

# CAC CHE DO VAN TOC
LockedVelMode = 2
UnLockedVelMode = 5
ManualVelMode = 1
AutoVelMode = 0

# CHE DO SAC TU DONG
h_des = 130
vx_vaotram = -0.15
vx_ratram = 0.5
vw_sac = 0.3
GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC = 0.3
#0.25
SO_LAN_THU_VAO_SAC_MAX = 10

# SO LAN THU SAI MAX
SO_LAN_THU_DINH_VI_MAX = 6
SO_LAN_THU_VECACVITRIKHAC_MAX = 6
SO_LAN_THU_VEHOME_MAX = 6

# GIA TRI CHAY MU
VANTOC_XOAY = 0.5
VANTOC_TIEN = 0.15

#ID DAC BIET CUA HE THONG
HOME_ID = 200
TruocDockSacID = 123
DockSacID = 100
KHOANG_CACH_VE_TRUOC_DOCKSAC = 0.5

#NGUONG XAC DINH ROBOT DI CHUYEN HAY KHONG
NGUONG_XACDINH_DICHUYEN_VX = 1
#NGUONG_XACDINH_DICHUYEN_VX = 0.01
NGUONG_XACDINH_DICHUYEN_VW = 0.01

# THONG SO CHO DIEU KHIEN MANUAL
VX_MANUAL_MAX = 0.6 #0.5
VW_MANUAL_MAX = 1.0 #1.5

#THONG SO KY THUAT CHO GIOI HAN GOC QUET VAF BUOC QUET CUAR QT0 TRONG SAC PI
QT0_GIOI_HAN_QUET = 90
QT0_BUOC_QUET = 10
QT0_SOLANQUET_CHOPHEP = 10

# CAU HINH THONG SO DINH VI HE THONG
DUNG_SAI_XOAY_ROBOT_DINHVI = 4 #do


# CAU HINH THONG SO ROBOT KHI DEN CAC DIEM
#DUNG_SAI_DIEM_DEN_VITRI = 0.7 #m
#DUNG_SAI_DIEM_DEN_GOC = 9 #do


# CAU HINH MQTT TOPIC

def hello1():
    print('Hello')

data_excel = "document/data.xlsx"
excel_file = "List_Checkin.xlsx"
excel_file_checkout = "check_out.xlsx"
pdf_file = "check_in.pdf"
pdf_file_checkout = "check_out.pdf"
# CAU HINH MQTT TOPIC
data_excel = "document/data.xlsx"
excel_file = "List_Checkin.xlsx"
excel_file_checkout = "check_out.xlsx"
pdf_file = "check_in.pdf"
pdf_file_checkout = "check_out.pdf"
# CAU HINH MQTT TOPIC
ID_ROBOT = "ASR03"

topicControl  = "optimarobotics/robotltv3/mqtt/control"
topicVantocVx = "optimarobotics/robotltv3/mqtt/vantocvx"
topicVantocVw = "optimarobotics/robotltv3/mqtt/vantocvw"
topicIdRobot  = "optimarobotics/robotltv3/mqtt/idrobot"
topicStatus   = "optimarobotics/robotltv3/mqtt/status"

topicCame     = "optimarobotics/robotltv3/mqtt/camera"
topicVoice    = "optimarobotics/robotltv3/mqtt/voice"
topicRecord   = "optimarobotics/robotltv3/mqtt/record"
topicIdRobot  = "optimarobotics/robotltv3/mqtt/idrobot"
topicOpenCame = "optimarobotics/robotltv3/mqtt/opencamera"
topicDelegateInfo = "optimarobotics/robotltv2/mqtt/delegate_info"

topicResponseId = "optimarobotics/robotltv3/mqtt/rpid"
topicResponseVt = "optimarobotics/robotltv3/mqtt/rpvt"
topicResponseOpenC = "optimarobotics/robotltv3/mqtt/rpopenc"
topicResponseConnect = "optimarobotics/robotltv3/mqtt/rpconnected"

mail_user = "robotreception2024orb@gmail.com"
password_mail = "xqgh lahe mdrn bner"


recipientEmail = "lehuunhiemm0812@gmail.com"
# Define AI gemini chatbox =================================================================




#===================================== DIEM DANH ==========================
save_dir_test = "image_test"
path_file_mssv = 'mssv.txt'
path_in_data_training = 'data_server/'
path_out_data_training = 'data_training/data_faces/'
path_train_features = 'features_all.csv'
path_model = 'data_training/data_dlib/'
#===========================================================================================

MODE_CHECKIN_FACEID = True
MODE_CHECKIN_QR_CODE = True

######### KHU VUC BIEN DINH VI
DO_TUONG_DONG_CAM_DINHVIN = 0.5

# ======================= CAC BIEN THEM MOI 20240923 ================================
# THONG SO CHO DIEU KHIEN MANUAL
VX_MANUAL_MAX = Uti.doc_du_lieu_sql('manual_vt_thang')#0.5 #0.4
VW_MANUAL_MAX = Uti.doc_du_lieu_sql('manual_vt_xoay')#1.5 #1.5
# CAU HINH THONG SO ROBOT KHI DEN CAC DIEM
DUNG_SAI_DIEM_DEN_VITRI = Uti.doc_du_lieu_sql('sai_so_kc')#0.5 #m
DUNG_SAI_DIEM_DEN_GOC = Uti.doc_du_lieu_sql('sai_so_goc')#9 #do
USERNAME = Uti.doc_du_lieu_sql('username')
PASSWORD = Uti.doc_du_lieu_sql('password') #caidatrobot
chieu_cao_led = Uti.doc_du_lieu_sql('chieu_cao_led') #caidatrobot
VX_AUTO_MAX = Uti.doc_du_lieu_sql('auto_vt_thang')#0.5 #0.4
VW_AUTO_MAX = Uti.doc_du_lieu_sql('auto_vt_xoay')#0.5 #0.4
MUC_DIEN_AP_SAC = Uti.doc_du_lieu_sql('volt_sac_auto')

faceCame1 = Uti.doc_du_lieu_sql('front_R')
headCamera = Uti.doc_du_lieu_sql('headC')
chargCame = Uti.doc_du_lieu_sql('backC')
faceCam2 = Uti.doc_du_lieu_sql('front_L')

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
