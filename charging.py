print('=' * 50)
print('----QR CHECK---')
print('=' * 50)
import pygame
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion  # Để chuyển đổi từ quaternion sang góc Euler
import math
import time
import threading
import cv2
from pyzbar.pyzbar import decode
import threading
import time
from geometry_msgs.msg import Twist
import rospy
import serial
import subprocess
from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np
import cal_yaw_sql as cal_yaw_sql
from geometry_msgs.msg import Vector3Stamped


import RobotConfig as RobConf
import Utilities as Uti

with open(RobConf.file_path_trangthaiPin, 'w') as file:
    file.write('')
# yaw_des: 275, x_des: 60
yaw_dock_sac =cal_yaw_sql.a
print(yaw_dock_sac)
usb_arduino = RobConf.MegaNANO_PORT #'/dev/ttyUSB3'
usb_camera = RobConf.chargeCamera #"/dev/video2"
brightness_value = 70
done_quytrinh = False
yaw_des = 350 

h_des = RobConf.h_des #300 
x_des = 0
h_current = 0
yaw_current = 0

vantoc_w = 0.7
vantoc_x = 0.2
error_chophep = 30
data_arduino = ''
break_camera = False
flag_timmaqr = False
flag_timmaqr_cnt = 0
robot_dichuyen = False
value_ard = ""
# command = f"chmod a+rw {usb_arduino}"
# process = subprocess.run(command, shell=True, check=True)
# time.sleep(1)

class Kiemtra_Robot_Move(QThread):
    send_arduino_dichuyen = pyqtSignal(bool)
    def __init__(self):
        
        super().__init__()
        self.speed_subscriber = rospy.Subscriber('/speed', Vector3Stamped, self.speed_callback)
        self.speed_data = None
        #robot_dichuyen = False
        self.cout_dichuyen = 0 
        self.cout_dung = 0
        self.last_x = 0
        self.last_y = 0

    def speed_callback(self, msg):
        global robot_dichuyen
        # Lấy ra hai giá trị đầu tiên (x và y) từ thông điệp Vector3Stamped
        x = msg.vector.x
        y = msg.vector.y
        z = msg.vector.z
        self.speed_data = (x - self.last_x, y - self.last_y)
        print(f"--------------Speed Data - x: {x - self.last_x}, y: {y - self.last_y}, Vx: {z}, robot di chuyen: {robot_dichuyen}, cnt dung: {self.cout_dung},cnt di chuyen: {self.cout_dichuyen} -----------------")
        if (abs(x - self.last_x) >= RobConf.NGUONG_XACDINH_DICHUYEN_VX) or (abs(y - self.last_y) >= RobConf.NGUONG_XACDINH_DICHUYEN_VX):
            # print("robot di chuyen")
            self.cout_dichuyen += 1
            # self.cout_dung = 0
            if self.cout_dichuyen >= 20:
            #    print('[KIEMTRA_ROBOT_MOVE] PHAT HIEN ROBOT DI CHUYEN')
                robot_dichuyen = True
                #self.send_arduino_dichuyen.emit(True)
                self.cout_dichuyen = 0
                self.cout_dung = 0
        if (abs(x - self.last_x) == 0) or (abs(y - self.last_y) == 0):
            self.cout_dung += 1
            # self.cout_dichuyen = 0
            if self.cout_dung >= 80:
            # print("robot dung yen")
            #    print('[KIEMTRA_ROBOT_MOVE] PHAT HIEN ROBOT DUNG')
                robot_dichuyen = False
               # self.send_arduino_dichuyen.emit(False)
                self.cout_dichuyen = 0
                self.cout_dung = 0
        self.last_x = x
        self.last_y = y

    def run(self):
        # Chờ cho đến khi nhận được dữ liệu
        #rospy.spin()
        pass

class PIDController:
    def __init__(self, kp=0, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.dt = 1


    def compute_control(self, error):
        self.integral += error
        derivative = error - self.prev_error
        #max_khaui = self.ki * self.integral


        control = self.kp * error + self.integral*self.ki + self.kd * derivative
        self.prev_error = error

        return control



class RobotController:
    def __init__(self, target_yaw, target_x):
        rospy.init_node('robot_controller_1')
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pid_angular = PIDController(kp=0.005/2, ki=0, kd=0) #4
        self.pid_linear = PIDController(kp=0.04/3, ki=0, kd=0)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.Vw = 0.0
        self.Vx = 0.0
        # Target position
        self.target_yaw = float(target_yaw)
        self.target_x = float(target_x)
        self.yaw_ros = 0.0
        self.z_ros = 0.0
        self.w_ros = 0.0
        self.get_yaw_first = False
        self.status_Vw = False
        # Robot's current position
        self.robot_yaw = 0.0
        self.robot_x = 0.0
        # Control loop rate
        self.pid_rate = rospy.Rate(100)  # 100 Hz for PID calculations
        self.quytrinh = 0
        self.quytrinh_1 = 2
        self.count_time = 0
        self.count_time_2 = 0
        self.count_check_yaw = 0
        self.yawControlCnt = 0



        self.cmd_vel_rotate_msg = Twist()

        #----------------- trang thai robot -----------------
        # self.speed_subscriber = rospy.Subscriber('/speed', Vector3Stamped, self.kiemtra_trangthai_robot)
        # self.speed_data = None
        # self.robot_dichuyen = False
        # self.cout_dichuyen = 0 
        # self.cout_dung = 0
        # self.trangthai_robot = False
        # self.previous_flag = False

    #def kiemtra_trangthai_robot(self, msg):
        # # Lấy ra hai giá trị đầu tiên (x và y) từ thông điệp Vector3Stamped
        # x = msg.vector.x
        # y = msg.vector.y
        # self.speed_data = (x, y)
        # # print(f"Speed Data - x: {x}, y: {y}")
        # if x != 0 or y != 0:
        #     # print("robot di chuyen")
        #     self.cout_dichuyen += 1
        #     # self.cout_dung = 0
        #     if self.cout_dichuyen == 70:
        #         self.robot_dichuyen = True
        #         self.trangthai_robot = True
        #         self.cout_dichuyen = 0
        #         print("robot di chuyen: ",self.cout_dichuyen)
                
        # else:
        #     self.cout_dung += 1
        #     # self.cout_dichuyen = 0
        #     if self.cout_dung == 70:
        #     # print("robot dung yen")
        #         self.robot_dichuyen = False
        #         self.trangthai_robot = False
        #         self.cout_dung = 0
        #         print("robot dung: ",self.cout_dung)

        # if self.trangthai_robot != self.previous_flag:
        #     if self.trangthai_robot:
        #         print("robot di chuyennnnnnnnnnnnnn")
        #     else:
        #         print("robot dunggggggggggggggg")
                    

        #         # print(f'Gửi dữ liệu: {flag}')

        # self.previous_flag = self.trangthai_robot
            
    
    def odom_callback(self, data):
        self.x_ros = data.pose.pose.position.x
        self.y_ros = data.pose.pose.position.y
        self.z_ros = data.pose.pose.orientation.z
        self.w_ros = data.pose.pose.orientation.w
        self.yaw_ros = self.calcular_yaw(z=self.z_ros, w=self.w_ros)
        # (roll, pitch, yaw) = euler_from_quaternion([0, 0, self.z_ros, self.w_ros])
        # print(f"yawMinh: {self.yaw_ros} yawDong: {yaw*180/math.pi}")

    def calcular_yaw(self, z, w):
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
        yaw = yaw * 180 / math.pi
        
        #print("gia tri",yaw)
        # if(yaw > 90 or yaw < -90):
        #     yaw = - yaw
        
        return yaw
    @staticmethod
    def XacDinhYaw(yaw_robot, yaw_dock):
        delta_yaw = yaw_robot - yaw_dock
        value = 0
        if delta_yaw > 180:
            delta_yaw -= 360
        elif delta_yaw < -180:
            delta_yaw += 360

        '''
        value = 1 -> lech phai
        value = -1 -> lech trai
        value = 0 -> trong khoang cho phep
        '''
        
        if abs(delta_yaw) < 1:
            value = 0
        elif delta_yaw > 0:
        #if delta_yaw > 0:
            value = 1
        else:
            value = -1
        return value
        
    def xacdinh_Vw(self):
        status = False

        value = self.XacDinhYaw(self.yaw_ros,yaw_dock= yaw_dock_sac)
        print(value)

        if value == 1:
            status = True
        elif value == -1:
            status = False
        
        return status
        #return self.XacDinhYaw(self.yaw_ros,yaw_dock= yaw_dock_sac)
        
        
        # if yaw_xd > 0:
        #     status = True
        # else:
        #     status = False
        # return status

    @staticmethod
    # def bu_duong_tron(saiso_yaw):
    #     if saiso_yaw > 180:
    #         saiso = saiso_yaw - 360
    #     elif saiso_yaw <-180:
    #         saiso = saiso_yaw + 360
    #     else:
    #         saiso = saiso_yaw
    #     print(f"saiso dau ra: {saiso}, saiso dau vao: {saiso_yaw}")
    #     return saiso
    # ===== HAM TINH UDK XOAY ROBOT TOI VI TRI MONG MUON=========
    #@staticmethod
    def udk_rotateRobot2Angle(self, yaw_c = 0, yaw_d = 0):     
        #tinh goc yaw hien tai      
        
        yaw_err = Uti.bu_duong_tron(yaw_d - yaw_c)
        vw = 0   
        result = False  
        if (abs(yaw_err)>9):
            self.yawControlCnt = self.yawControlCnt + 1
            if(self.yawControlCnt >= 250): #vuot qua so lan cho phep
                self.yawControlCnt = 250
                vw =0
                result=True 
            else:
                vw = yaw_err*0.03#41#25%gghv   #BO DIEU KHIEN P
                if(vw > 1.5):
                    vw = 1.5
                if(vw < -1.5):
                    vw = -1.5
                result=False
        else:
            self.yawControlCnt = 0
            vw =0
            result=True            
        return (result, vw)
                #rospy.sleep(0.1)  # Chờ 0.1 giây
        #    return False #sai so chua cho phep
       # else:
        #    return True #sai so cho phep

    #============== TINH TOAN YAW_D CHO QT0
    def tinh_yawd_qt0(self, yaw_c=0, yawd_min = 0, yawd_max = 1, yaw_step = 1):
        yaw_next = yaw_c + yaw_step
        step_out = yaw_step
        lim = False
        if(yaw_next > yawd_max):  #gioi han va giam la goc yaw
            yaw_next = yawd_max
            step_out = -yaw_step
            lim = True
        elif (yaw_next < yawd_min):  #gioi han va tang lai goc yaw
            yaw_next = yawd_min
            step_out = -yaw_step
            lim = True
        return (yaw_next, step_out, lim)

    #============= GOI VAN TOC XUONG ARDUINO====================
    def goiVantocxuongArduino(self, vx = 0, vw = 0, mode = 0):
        self.cmd_vel_rotate_msg.angular.z = vw
        self.cmd_vel_rotate_msg.linear.x = vx
        self.cmd_vel_rotate_msg.linear.y = mode
        self.cmd_vel_pub.publish(self.cmd_vel_rotate_msg)
        time.sleep(1)

    def PID(self):
        global yaw_current, yaw_des, vantoc_w, vantoc_x, data_arduino, break_camera, flag_timmaqr, done_quytrinh, h_current, h_des
        global value_ard, robot_dichuyen
        previousTime = 0.0
        self.done_quytrinh_0 = False
        done_quytrinh_0_temp = False

        self.done_quytrinh_1 = False
        self.done_quytrinh_2 = False
        done_quytrinh_2_temp = False
        self.done_quytrinh_3 = False

        error_yaw_led = 0
        error_yaw = 0
        count_1 = 0
        count_2 = 0
        count_3 = 0

        count_break = 0
        cnt_robot_dung = 0        
        yaw_d_qt0 = yaw_dock_sac
        yaw_d_qt0_max = yaw_dock_sac + RobConf.QT0_GIOI_HAN_QUET
        yaw_d_qt0_min = yaw_dock_sac - RobConf.QT0_GIOI_HAN_QUET
        qt0_buoc_quet = RobConf.QT0_BUOC_QUET
        qt0_lim_cnt = 0
        pre_h_current=0
        h_current_cnt =0

        #self.quytrinh = 1
        while not rospy.is_shutdown():

            done_quytrinh_0_temp = self.done_quytrinh_0
            done_quytrinh_2_temp = self.done_quytrinh_2
            #print("gia tri yaw_ros curent: ",self.yaw_ros)
            if self.quytrinh == 0 and not self.done_quytrinh_0:
                print("00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000")
                cnt_robot_dung = 0
                
                self.done_quytrinh_2 = False
                self.done_quytrinh_1 = False
                self.done_quytrinh_0 = False
                # if not self.get_yaw_first:
                #     temp1 = self.xacdinh_Vw() 
                #     if(temp1 >= 0):
                #         self.status_Vw = True
                #     else: 
                #         self.status_Vw = False
                #     self.count_check_yaw += 1
                #     if self.count_check_yaw == 2:
                #         self.get_yaw_first = True
                #         self.count_check_yaw = 0

                if flag_timmaqr and (abs(yaw_current - 340) < 300) and (abs(Uti.bu_duong_tron(self.yaw_ros - yaw_dock_sac)) < 90):
                    
                   # if (abs(yaw_current - 340) < 300) and (abs(self.bu_duong_tron(self.yaw_ros - yaw_dock_sac)) < 90):
                   # self.get_yaw_first = False
                   # self.status_Vw = False
                    print('tim thay ma QR -- hoan thanh quy trinh 0')
                    self.Vx = 0.0
                    self.Vw = 0.0
                    self.done_quytrinh_0 = True
                    self.goiVantocxuongArduino(vx = self.Vx, vw = self.Vw, mode = 10)
                    # chuan bi di chuyen den qui trinh 1
                    self.quytrinh_1 = 0
                    self.quytrinh = 1
                    self.done_quytrinh_0 = True
                    self.done_quytrinh_1 = False
                    print('=' * 50)
                    # else:
                    #     if self.status_Vw:
                    #         self.Vw = RobConf.vw_sac
                    #     else:
                    #         self.Vw = -RobConf.vw_sac
                    #     self.Vx = 0.0
                else:
                    print('----------------dang tim ma QR----------------')
                    # if self.status_Vw:
                    #     self.Vw =RobConf.vw_sac
                    # else:
                    #     self.Vw = -RobConf.vw_sac
                    # Qyet cam bang cach xoay robot toi cac vi tri mong muon
                    self.Vx = 0.0
                    if(self.quytrinh_1 == 2):  # MOi chuyen tu qui trinh 2 qua
                       # yaw_d_qt0 = self.yaw_ros
                        yaw_d_qt0 = yaw_dock_sac
                        lim = False
                        qt0_lim_cnt = 0                        
                        
                        
                    if(qt0_lim_cnt > RobConf.QT0_SOLANQUET_CHOPHEP): # HET SO LAN QUET  -> DUNG HE THONG -> THONG BAO CHO DUNG BIET
                        self.Vw = 0
                        self.goiVantocxuongArduino(vx = self.Vx, vw = self.Vw, mode = 10)
                        done_quytrinh = True
                        break_camera = True
                        Uti.RobotSpeakWithPath(pathFile='voice_hmi_new/qui_trinh_0_that_bai.mp3')
                        time.sleep(5)
                    else:    # CHO ROBOT DEN CAC QUOC QUET MONG MUON
                        yaw_d = yaw_d_qt0
                        yaw_c = self.yaw_ros
                        yaw_err = Uti.bu_duong_tron(yaw_d - yaw_c)
                        vw = 0   
                        result = False  
                        if (abs(yaw_err)>9):
                            self.yawControlCnt = self.yawControlCnt + 1
                            if(self.yawControlCnt >= 1000): #vuot qua so lan cho phep
                                self.yawControlCnt = 1000
                                vw =0
                                result=True 
                            else:
                                vw = yaw_err*0.02#41#25%gghv   #BO DIEU KHIEN P 0.03
                                if(vw > 0.5):
                                    vw = 0.5
                                if(vw < -0.5):
                                    vw = -0.5
                                result=False
                        else:
                            self.yawControlCnt = 0
                            vw =0
                            result=True  
                        #print(F'yaw_d: {yaw_d_qt0}, yaw_c: {yaw_c}, udk: {vw}, result: {result}')

                        #result, vw = self.udk_rotateRobot2Angle(yaw_c =0, yaw_d = 0)
                        #result, vw = self.udk_rotateRobot2Angle(yaw_c = self.yaw_ros, yaw_d = yaw_d_qt0)
                        self.Vw = vw
                        self.goiVantocxuongArduino(vx = self.Vx, vw = self.Vw, mode = 10)
                        if(result): # di chuyen toi buoc tiep theo
                            self.yawControlCnt = 0
                            time.sleep(1)
                            #print(f'======>yaw_d_quet tr:  {yaw_d_qt0}, buoc quet: {qt0_buoc_quet}')
                            yaw_d_qt0, qt0_buoc_quet, lim = self.tinh_yawd_qt0(yaw_c=yaw_d_qt0, yawd_min =  yaw_d_qt0_min, yawd_max = yaw_d_qt0_max, yaw_step = qt0_buoc_quet)
                            #print(f'======>yaw_d_quet sa:  {yaw_d_qt0}, buoc quet: {qt0_buoc_quet}')
                            if(lim):
                                qt0_lim_cnt = qt0_lim_cnt + 1

                    # Cap nhat qui trinh
                    self.quytrinh_1 = 0
                 

            elif self.quytrinh == 1 and not self.done_quytrinh_1:
                print("1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111")
            #    self.done_quytrinh_0 = False
            #    self.done_quytrinh_1 = False
                self.done_quytrinh_2 = False
                if h_current==pre_h_current:
                    h_current_cnt +=1
                else:
                    h_current_cnt=0

                if h_current_cnt>=10:
                    h_current_cnt=10
                pre_h_current=h_current

                if flag_timmaqr and h_current_cnt<10:
                    self.count_time_2 = 0
                    print('----Quy trinh 1 & co QR-----')
                    
                    error_yaw = Uti.bu_duong_tron(self.yaw_ros - yaw_dock_sac)
                    self.Vw = -0.01*(error_yaw)
                  #  if self.Vw > 0.5*RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC:
                  #      self.Vw = 0.5*RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC
                  #  elif self.Vw < -0.5*RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC:
                   #     self.Vw = -0.5*RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC

                    error_yaw_led = yaw_current - yaw_des
                    self.Vw = -0.001*error_yaw_led - 0*self.Vw*0.1
                    
                    print('===========================: ',self.Vw)
                    #self.Vx = RobConf.vx_vaotram#-0.1
                    #giới hạn vận tốc Vx
                    if self.Vw > RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC:
                        self.Vw = RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC
                    elif self.Vw < -RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC:
                        self.Vw = -RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC

                    if value_ard == 'A': #A
                        self.Vw = 0.0
                        self.Vx = 0.0
                        break_camera = True
                        self.done_quytrinh_1 = True
                        print('------Hoan thanh quy trinh sac-----------')
                        print('=' * 50)
                    else:
                        if value_ard == "B":
                            print("55555555555555555555555555555555555555555555555555555555555555555555555555555555")
                            value_ard = ''
                            self.Vw = 0.0
                            self.Vx = 0.0
                            self.count_time = 0
                            self.quytrinh = 2
                            count_break += 1
                            self.done_quytrinh_1 = True
                            print('=' * 50)
                        count_2 = 0
                        # LOI VAO DOC SAC SAI HUONG
                        if (h_current < h_des): 
                            count_3 += 1
                            #cho xem giam toc
                            self.Vx = 0.99*self.Vx
                        else: 
                            #cho xe tang toc
                            count_3 = 0
                            self.Vx = self.Vx - 0.01
                            if(self.Vx <= RobConf.vx_vaotram):
                                self.Vx =RobConf.vx_vaotram
                        
                        if count_3 >= 50:
                            self.Vw = 0.0
                            self.Vx = 0.0
                            self.done_quytrinh_1 = True
                            self.quytrinh = 2
                            count_break += 1                        
                            count_3 = 100
                else:
                    # self.Vw = 0
                    # self.Vx = 0
                    count_3 += 1
                    print(f'----Quy trinh 1 & khong co QR. count2: {count_2}-----')
                    count_2 += 1
                    if count_2 >= 100:  #TAM CHAP NHAN DO NHIEU QUA MUC GIOI HAN
                        self.quytrinh = 2
                        count_break += 1
                        self.done_quytrinh_1 = True
                        count_2 = 200
                        self.Vw = 0.0
                        self.Vx = 0.0
                    else: #giam toc do
                        self.Vw = 0.8*self.Vw
                        self.Vx = 0.8*self.Vx
                # XU LY TINH HUONG ROBOT DUNG
                if(not robot_dichuyen):
                    cnt_robot_dung += 1                    
                    if (cnt_robot_dung > 50): # CHUYEN QUA QUI TRINH 2
                        self.quytrinh = 2
                        self.done_quytrinh_1 = True
                        count_break += 1
                        #self.Vw = 0.0
                        #self.Vx = 0.0
                        print('[QTRINH 1] PHAT HIEN ROBOT DUNG QUA THOI GIAN QUI DINH')
                    else:#giam toc do
                        self.Vw = 0.6*self.Vw
                        self.Vx = 0.6*self.Vx
                        print(f"[QTRINH 1] robot k di chuyen, dang giam toc: {cnt_robot_dung}")
                else:
                    cnt_robot_dung = 0
                    print("[QTRINH 1] robot di chuyen")
                # Xuat tinh hieu dk
                print(F'[QTRINH 1] vx: {self.Vx}, vw: {self.Vw}')
                # #  test QT0
                # self.Vw = 0.0
                # self.Vx = 0.0
                ########################
                self.goiVantocxuongArduino(vx = self.Vx, vw = self.Vw, mode = 10)
                self.quytrinh_1 = 1
            elif self.quytrinh == 2:
                print("222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222")
                # with open('/home/robottieptan/Desktop/letan_24052024/data_ard.txt', 'w') as file:
                #     file.write('')
                #value_ard = ""
                self.done_quytrinh_0 = False
                self.done_quytrinh_1 = False
                print("---Quy trinh 2 --lui ra dock sac------")
                # self.status_Vw = self.xacdinh_Vw()                

                self.Vw = - Uti.bu_duong_tron( self.yaw_ros - yaw_dock_sac) * 0.1

                if self.Vw > RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC:
                    self.Vw = RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC
                elif self.Vw < -RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC:
                    self.Vw = -RobConf.GIOI_HAN_VAN_TOC_XOAY_TRAM_SAC
                self.Vx = RobConf.vx_ratram

                # #  test QT0
                # self.Vw = 0.0
                # self.Vx = 0.0
                ###########################

                #for _ in range(10):
                self.goiVantocxuongArduino(vx=self.Vx, vw=self.Vw, mode=10)
                time.sleep(5.5)

                print(f"--------------------------[QT2 lui ra] Vx= {self.Vx}, Vw= {self.Vw}---------------------")
                #time.sleep(15)
                # self.Vw = 0
                # self.Vx = 0
                # self.goiVantocxuongArduino(vx = self.Vx, vw = self.Vw, mode = RobConf.ManualVelMode)
                # time.sleep(2)
                #xong qui trinh 2
                self.done_quytrinh_2 = True
                self.quytrinh = 0
                self.quytrinh_1 = 2

       

            # if not done_quytrinh_0_temp and  self.done_quytrinh_0:
            #     time.sleep(2)
            #     print("chuyen quy trinh tu 0 -> 1")
            #     self.quytrinh = 1

            # if self.done_quytrinh_2:
            #     time.sleep(2)
            # #     print("chuyen quy trinh tu 1 -> 2: ",done_quytrinh_2_temp)
            #     print("77777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777")
                
            #     if not done_quytrinh_2_temp:
            #         print("6666666666666666666666666666666666666666666666666666666666666666666666666666666666")
            #         self.cmd_vel_rotate_msg.angular.z = self.Vw
            #         self.cmd_vel_rotate_msg.linear.x = self.Vx
            #         self.cmd_vel_pub.publish(self.cmd_vel_rotate_msg)


            #         # with open('/home/robottieptan/Desktop/letan_24052024/data_ard.txt', 'w') as file:
            #         #     file.write('')
            #         #value_ard = ""
            #     else:
            #         time.sleep(2)
                

            # if  self.done_quytrinh_0 or self.done_quytrinh_1 :
            #     print('dung lai')
            #     time.sleep(2.5)
            #     self.done_quytrinh_0 = False
            #     self.done_quytrinh_1 = False
            #     self.count_time_2 = 0

            # elif self.done_quytrinh_2: #lui ra
            #     #done_quytrinh = False
                
            #     print("------- done")
            #     time.sleep(8)
            #     print("------- done delay")
            #     self.quytrinh = 4
            #     # self.cmd_vel_rotate_msg.angular.z = 0
            #     # self.cmd_vel_rotate_msg.linear.x = 0
            #     # self.cmd_vel_pub.publish(self.cmd_vel_rotate_msg)
            #     # self.count_time_2 = 0
            #     # self.done_quytrinh_3 = False
            #     # self.done_quytrinh_2 = False

            if value_ard == 'A': 
                self.Vw = 0.0
                self.Vx = 0.0
                break_camera = True
                self.done_quytrinh_1 = True
                print('------Hoan thanh quy trinh sac-----------')
                print('=' * 50)
                self.goiVantocxuongArduino(vx = self.Vx, vw = self.Vw, mode =0)
                time.sleep(2)
            if count_break >= RobConf.SO_LAN_THU_VAO_SAC_MAX:

                time.sleep(1)
                self.cmd_vel_rotate_msg.angular.z = 0
                self.cmd_vel_rotate_msg.linear.x = 0
                self.cmd_vel_pub.publish(self.cmd_vel_rotate_msg)
                time.sleep(1)
                
                pygame.mixer.init()
                pygame.mixer.music.load('voice_hmi_new/huynhiemvu.wav')
                pygame.mixer.music.play()
                time.sleep(7)

                pygame.mixer.init()
                pygame.mixer.music.load('voice_hmi_new/huynhiemvu.wav')
                pygame.mixer.music.play()
                time.sleep(5)

                done_quytrinh = True
                print("------> BREAK HUY NHIEM VU")
                break



            if break_camera:
                done_quytrinh = True
                print("------> BREAK TOAN BO QUY TRINH")

                
               # with open('file_text_odom/odom_data.txt', 'w') as file:
                with open(RobConf.file_path_pose_data, 'w') as file:
                    file.write(f'x: {self.x_ros}, y: {self.y_ros}, z: {self.z_ros}, w: {self.w_ros}\n')
                    #print("da ghi text")

                break
                #cv2.destroyAllWindows()

            # print(f"count2: {count_2} ---------- count3: {count_3}---------- count_break: {count_break}")
            # print("gia tri ard: ",value_ard)  
            #print("  h_current: ",h_current) 
            # print(f" e_error_yaw: { self.yaw_ros - yaw_dock_sac }")
            # print(f' yaw_current = {self.yaw_ros}', f'yaw_dock = {yaw_dock_sac}', ' ' * 5, ' ' * 5, f'Vx = {self.Vx}', f'Vw = {self.Vw}')
            print(f' Quy trinh hien tai: {self.quytrinh}')
            #print(f' Trang thai QR: {flag_timmaqr}')
            # print(' yaw', self.yaw_ros)
            # # self.status_Vw = self.xacdinh_Vw()
            # print(' status Vww', self.status_Vw)
            print("------------------------------------------------------")
            self.pid_rate.sleep()

            #rospy.spin()
class arduino(threading.Thread):
    def __init__(self):
        super().__init__()
        self.ser = None
        self.data = ''
        self.is_running = False
        self.connected_ard = False
        self.done_quytrinh = False

    def connect_arduino(self):
        try:
            self.ser = serial.Serial(usb_arduino, 38400)
            print('-----------> ket noi arduino thanh cong')
            time.sleep(0.5)
            self.connected_ard = True
        except Exception as ex:
            print(ex)

    def read_data_from_arduino(self):
        data = self.ser.readline().decode('utf-8').strip()
        # if len(data) > 1:
        #     data = 'not'
        if len(data) > 1 and not self.done_quytrinh:
            data = 'not'
        return data

    def run(self):
        global data_arduino,done_quytrinh
        self.data = ""
        self.is_running = True
        self.connect_arduino()
        if self.connected_ard:
            while self.is_running:
                data_arduino = self.read_data_from_arduino()
                #print(data_arduino)
                if data_arduino == 'A':
                    time.sleep(1.5)
                    if done_quytrinh:
                        break
                    else:
                        data_arduino =''
    
            #print('ngat ket noi arduino')
            self.ser.close()
            self.connected_ard = False

class LED_Code(threading.Thread):
    def __init__(self):
        super().__init__()
        self.camera = cv2.VideoCapture(usb_camera)
        self.default_brightness = self.camera.get(cv2.CAP_PROP_BRIGHTNESS)
        
        # Đặt độ sáng (giá trị từ 0 đến 255, tùy thuộc vào camera)
        brightness_value = 85  # Điều chỉnh giá trị này để giảm độ sáng
        self.camera.set(cv2.CAP_PROP_BRIGHTNESS, brightness_value)

        self.flag_barcode = False
        self.is_ret = False
        self.flag_coqr = False
    
    def get_led_color(self, frame, contour):
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        if radius > 0:
            led_color = frame[center[1], center[0]]
            return led_color, center, radius
        return None, None, None

    def run(self):
        global flag_timmaqr, flag_timmaqr_cnt
        global yaw_current, h_current

        print('-' * 20, 'START QR CODE', '-' * 20)
        global break_camera, brightness_value, done_quytrinh
        break_camera = False

        while not done_quytrinh:
                
            ret, frame = self.camera.read()
            if ret:
                self.is_ret = True
            # frame_height, frame_width = frame.shape[:2]
            # frame_height, frame_width = 480, 680
            frame = cv2.resize(frame, (680, 480))
            # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # lower_red = np.array([0, 100, 100])
            # upper_red = np.array([10, 255, 255])
            # mask1 = cv2.inRange(hsv, lower_red, upper_red)
            # lower_red = np.array([160, 100, 100])
            # upper_red = np.array([179, 255, 255])
            # mask2 = cv2.inRange(hsv, lower_red, upper_red)

            # mask = mask1 + mask2



            # rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # lower_red = np.array([100, 0, 0])
            # upper_red = np.array([255, 50, 50])
            # mask = cv2.inRange(rgb_frame, lower_red, upper_red)
            # mask = cv2.GaussianBlur(mask, (15, 15), 0)
            # mask = cv2.erode(mask, None, iterations=2)
            # mask = cv2.dilate(mask, None, iterations=2)

            # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # lower_red = np.array([160, 100, 100])
            # upper_red = np.array([179, 255, 255])
            # mask2 = cv2.inRange(hsv, lower_red, upper_red)
            # #mask =  mask1

            # mask3 = mask2

            

            # mask3 = cv2.GaussianBlur(mask3, (15, 15), 0)
            # mask3 = cv2.erode(mask3, None, iterations=2)
            # mask3 = cv2.dilate(mask3, None, iterations=2)


            # rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # lower_red = np.array([100, 0, 0])
            # upper_red = np.array([255, 50, 50])
            # mask = cv2.inRange(rgb_frame, lower_red, upper_red)
            # mask3 = mask + mask3

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            lower_red = np.array([160, 100, 100])
            upper_red = np.array([179, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)

            mask = mask1 + mask2

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
           # flag_timmaqr = False
            find_led_result = False
            for contour in contours:   
                if cv2.contourArea(contour) > 100:
                    led_color, center, radius = self.get_led_color(frame, contour)
                    #print(f" led {led_color[0]}")
                    if led_color is not None:
                        r, g, b = led_color
                        if 200 <= r <= 255:
                            find_led_result = True
                            flag_timmaqr = True
                            self.flag_coqr = True
                            cv2.circle(frame, center, radius, (0, 255, 0), 2)
                            #print("ban kinh",radius)
                            
                            cv2.putText(frame, f'RGB: {center}', (center[0] + 15, center[1] + 5), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5, (0, 255, 0), 2)
                            yaw_current = center[0]
                            h_current = center[1]
                            #print(F'[Led_code] h_current: {h_current}')
                        # else:
                        #     if r >= 0 and r < 230:
                        #         flag_timmaqr = False
            #print(f'--------------------------------->Distance to center: {yaw_current} , {flag_timmaqr} ')
                            
            if(not find_led_result):
                h_current =h_des
                flag_timmaqr_cnt += 1
                if(flag_timmaqr_cnt >= 10):
                    flag_timmaqr_cnt = 10
                    flag_timmaqr = False
            else:
                flag_timmaqr_cnt = 0
                flag_timmaqr = True
            cv2.imshow('LED Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.camera.release()
        cv2.destroyAllWindows()

        if break_camera or done_quytrinh:
            print("da break camera")
            #print(f"Giá trị độ sáng mặc định: {self.default_brightness}")


# def read_yaw():
#     while True:
#         pid_yaw = RobotController(target_yaw=yaw_des, target_x=x_des)


class read_txt_ard(threading.Thread):
    def __init__(self):
        super().__init__()
    def run_read(self):
        global value_ard, done_quytrinh
        value_ard = ""
        while not done_quytrinh:
            #with open("/home/robottieptan/Desktop/letan_24052024/data_ard.txt", 'r') as file:
            with open(RobConf.file_path_trangthaiPin, 'r') as file:
                value_ard = file.read()
           # print("[read_txt_ard]gia tri pin:", value_ard)
        if done_quytrinh:
            print("da break luong doc arduino")
            
def main():

    kiemtra_robot_dichuyen = Kiemtra_Robot_Move()
    kiemtra_robot_dichuyen.start()

    # #start arduino
    # ard = arduino()
    # ard.start()
    # # start camera

    pid_yaw = read_txt_ard()
    pid_thread_yaw = threading.Thread(target=pid_yaw.run_read)
    pid_thread_yaw.start()

    qr_reader = LED_Code()
    qr_reader.start()
    time.sleep(2)

    pid_yaw = RobotController(target_yaw=yaw_des, target_x=x_des)
    pid_thread_yaw = threading.Thread(target=pid_yaw.PID)
    pid_thread_yaw.start()
    pid_thread_yaw.join()
    print('=' * 20, 'done', '=' * 20)





if __name__ == '__main__':
 
    # start arduino
    # ard = arduino()
    # ard.start()
    # # start camera
    # qr_reader = QRCodeReader()
    # qr_reader.start()
    # time.sleep(2)
    # # start pid
    # pid_yaw = RobotController(target_yaw=yaw_des, target_x=x_des)
    # time.sleep(2)
    # while abs(int(pid_yaw.yaw_ros)) == 0:
    #     print('dang doi tinh goc yaw')
    #     print(pid_yaw.yaw_ros)
    #     if abs(pid_yaw.yaw_ros) > 0:
    #         break
    #     time.sleep(0.5)
    # pid_thread_yaw = threading.Thread(target=pid_yaw.PID)
    # pid_thread_yaw.start()
    # pid_thread_yaw.join()
    # print('=' * 20, 'done', '=' * 20)
    # read_yaw()

    main()

    #a = LED_Code()
    #a.start()
    









