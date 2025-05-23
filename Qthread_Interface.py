from PyQt5.QtCore import QThread, pyqtSignal
# import rospy
import math
# from tf.transformations import euler_from_quaternion

# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped
import pygame.mixer
import speech_recognition as sr

import os
import time
import signal
import subprocess
# from move_base_msgs.msg import MoveBaseActionGoal
import serial
# from geometry_msgs.msg import Twist, PoseStamped, Pose
# from actionlib_msgs.msg import GoalStatusArray
import mysql.connector
import sys

# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import actionlib
# from actionlib_msgs.msg import GoalStatus
from datetime import datetime
import time
import os
import wikipedia
# import playsound
from gtts import gTTS
import speech_recognition as sr
import pygame

import json
import difflib
import re
import requests

import pandas as pd
import requests
from bs4 import BeautifulSoup
import speech_recognition as sr
from gtts import gTTS
import os
from openpyxl import load_workbook
import pygame
from datetime import datetime
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
# Thêm thư viện keyboard để lắng nghe phím bấm
import pyaudio
import numpy as np
import RobotConfig as RobConf
import Utilities as Uti
import subprocess

db = mysql.connector.connect(
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
#db = mysql.connector.connect(user="admin", password='123456', host='127.0.0.1',
#                             database='sql_rsr')  # Thiết lập kết nối đến cơ sở dữ liệu
done_goal = False


#Mega2560_PORT = 'gnome-terminal -- rosrun rosserial_python serial_node.py /dev/ttyUSB0'
Mega2560_PORT = RobConf.Mega2560_PORT
#NAVIGATION_S2M1 = RobConf.NAVIGATION_S2M1
full_vol = RobConf.full_vol #10.7
hieu = 0


class run_roscore(QThread):
    
    def __init__(self) :
        super().__init__()
        self.is_running = False
    
    def run(self):
        self.run_login()

    def run_login(self):

        # rospy.Subscriber('/slamware_ros_sdk_server_node/odom', Odometry, send_odom)
        # rospy.Subscriber('/cmd_vel', Twist, send_cmd_vel)
        
        

        # time.sleep(1)
        # os.system(Mega2560_PORT)
        # time.sleep(5)
        pass

        # rplidar_ros_process = subprocess.Popen("gnome-terminal -- bash -c 'roslaunch rplidar_ros rplidar_s2.launch; exec bash'", shell=True)
        # rplidar_ros_process.wait()
        # subprocess.Popen("gnome-terminal -- bash -c 'roslaunch navigation navigation.launch; exec bash'", shell=True)
        # subprocess.Popen("gnome-terminal -- bash -c 'roslaunch navigation navigation.launch port:=/dev/usb_lidar; exec bash'", shell=True)
        # subprocess.Popen("gnome-terminal -- bash -c 'roslaunch rplidar_ros rplidar_s2.launch; exec bash'", shell=True)
        
        # os.system(NAVIGATION_S2M1)
        # # os.system('gnome-terminal -- rosrun map_server map_server /home/dong/catkin_ws/src/navigation_bot/map/{map_file}.yaml')
        # time.sleep(2)
        #os.system('gnome-terminal -- roslaunch slamware_ros_sdk slamware_ros_sdk_server_node.launch ip_address:=192.168.11.1')
        # time.sleep(2)
        
        #os.system('gnome-terminal -- rosrun rosserial_python serial_node.py /dev/ttyUSB2')

        #------------------- chay file final cam ------------------
        #process3 = subprocess.Popen("python3 /home/truong/Desktop/letan_24052024/reset_lidar.py", shell=True)
        # process3 = subprocess.Popen("python3 reset_lidar.py", shell=True)
        # process3.wait()

    #     self.spin_ros()
    #     print('done')
    # def spin_ros(self):
    #     rospy.spin()


class send_goal(QThread):
    def __init__(self) :
        super().__init__()
        self.is_running = False
        #self.file_path = '/home/truong/Desktop/letan_24052024/file_text_odom/odom_data.txt'
        self.file_path = RobConf.file_path_pose_data #'/home/truong/Desktop/letan_24052024/file_text_odom/odom_data.txt'
        
        self.x_pose = 0
        self.y_pose = 0
        self.z_pose = 0
        self.w_pose = 0
        self.no_set_pose = False
        
        
    def run(self):
        # self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        # os.system('rosservice call /move_base/clear_costmaps')
        # pub = rospy.Publisher('/slamware_ros_sdk_server_node/set_pose', Pose, queue_size=10)
        time.sleep(1)
        if not self.no_set_pose:
            with open(self.file_path, 'r') as file:
                lines = file.readlines()
                if lines:
                    last_line = lines[-1]  
                    data = last_line.strip().split(',')
                    for item in data:
                        key, value = item.split(':')
                        key = key.strip()
                        value = value.strip()
                        if key == 'x':
                            self.x_pose = float(value)
                        elif key == 'y':
                            self.y_pose = float(value)
                        elif key == 'z':
                            self.z_pose = float(value)
                        elif key == 'w':
                            self.w_pose = float(value)

        #     pose_msg = Pose()
        #     pose_msg.position.x = self.x_pose
        #     pose_msg.position.y = self.y_pose
        #     pose_msg.orientation.z = self.z_pose
        #     pose_msg.orientation.w = self.w_pose
        #     # print("zzzz: ",pose_msg.orientation.z)
        #     # print("wwww: ",pose_msg.orientation.w)
        #     pub.publish(pose_msg)
    
        #     time.sleep(1)

        # goal = PoseStamped()
        # self.no_set_pose = False

        # goal.header.frame_id = "map"  
        # goal.pose.position.x = self.x_goal   
        # goal.pose.position.y = self.y_goal   
        # goal.pose.orientation.z = self.z_goal  
        # goal.pose.orientation.w = self.w_goal  
        # self.goal_publisher.publish(goal) 
        # time.sleep(1)
        print('publish done')
        
        # quatrinh_movebase = 1   

    def rec_goal(self,x,y,z,w):
        self.x_goal = x 
        self.y_goal = y
        self.z_goal = z
        self.w_goal = w

class read_data_lidar(QThread):
    send_bool = pyqtSignal(bool)
    text_finish = pyqtSignal(bool)
    enable_btn = pyqtSignal(bool)
    clear_auto = pyqtSignal(bool)
    arduino_charing = pyqtSignal(bool)
    x_sac = 0.0
    y_sac = 0.0
    z_sac = 0.0
    w_sac = 0.0
    error_x = 0.0
    error_y = 0.0
    error_yaw = 0.0


    def __init__(self) :
        super().__init__()
        self.is_running = False
        self.data_odom = []
        self.auto_charging = False
        self.quatrinh_move = False
        self.no_voice = False
        self.giaotoi = 10
        self.dung = 0 
        self.dung1aidix3 = 0
        self.id = 0
        self.voice = 0
        self.done_1 = False

        self.dem = 0
        self.saiso = 0
        
        self.voice_huy = 0
        self.biethuy = False

        self.thoat = False
        
        
    def run(self):
        self.is_running = True
        # rospy.Subscriber('/slamware_ros_sdk_server_node/odom', Odometry, self.send_odom)
        # self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.client.wait_for_server()

        # # Khởi tạo subscriber để theo dõi trạng thái của goal
        # rospy.Subscriber('/move_base/status', GoalStatus, self.status_callback)

        # self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pass
    
    def move_robot_backwards(self):
        # Tạo lệnh điều khiển di chuyển lùi
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -0.085  # Tốc độ lùi
        cmd_vel_msg.angular.z = 0.0   # Không xoay
        # Gửi lệnh điều khiển cho một khoảng thời gian
        start_time = time.time()
        while time.time() - start_time < 5:  # Lùi trong 3 giây
            self.voice_huy += 1
            print("voice: ",self.voice_huy)
            if self.voice_huy == 15:
                # --------------------- phat am thanh -------------------------
                Uti.RobotSpeakWithPath('voice_hmi_new/huynhiemvu.wav')

                self.voice_huy = 0

            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(0.1)  # Chờ 0.1 giây
        
    def move_robot_forward(self, giay = 1.5):
        # Tạo lệnh điều khiển di chuyển lùi
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.06  # Tốc độ lùi
        cmd_vel_msg.angular.z = 0.0   # Không xoay
        # Gửi lệnh điều khiển cho một khoảng thời gian
        start_time = time.time()
        while time.time() - start_time < giay:  # Lùi trong 3 giây
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(0.1)  # Chờ 0.1 giây

    def set_vitri(self,x_goal,y_goal,z_goal,w_goal):
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.z_goal = z_goal
        self.w_goal = w_goal

    def yaw_from_quaternion(self,quaternion):
    # Chuyển đổi quaternion sang góc Euler (roll, pitch, yaw)
        (roll, pitch, yaw) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw
    
    def status_callback(self, msg):
        global hieu
        for goal_status in msg.status_list:
            # print("status: ", goal_status.status)
            hieu = goal_status.status
            # if goal_status.status == GoalStatus.ABORTED:
            #     return goal_status.status  
            # else:
            #     return False

    def kiemtra_hoanthanh_nv(self,x,x_goal,y,y_goal,z,z_goal,w,w_goal,id,status):

        
        yaw_des = self.tinhgoc_yaw(z=z_goal,w=w_goal)
        yaw = self.tinhgoc_yaw(z=z,w=w)
        # print("id_hamnhiemvu",id)
        if id == 200:
            if ((abs(x-x_goal) < 0.27) and (abs(y-y_goal)<0.27) and (abs(yaw - yaw_des)<20)) or status == 3:
                return True
            else:
                return False
        elif id == 123:
            if ((abs(x-x_goal) < 0.27) and (abs(y-y_goal)<0.27) and (abs(yaw - yaw_des)<20)) or status == 3:
                print(f"x ={x}, y = {y},yaw = {yaw} ,x_goal = {x_goal},y_goal = {y_goal}, yaw_des = {yaw_des}, ---- status = {status}")
                print("da den vi tri cho sac")
                time.sleep(2)
                return True
            else:
                print(f"x ={x}, y = {y},yaw = {yaw} ,x_goal = {x_goal},y_goal = {y_goal}, yaw_des = {yaw_des}, ---- status = {status}")
                return False
        else:
            if ((abs(x-x_goal) < 0.27) and (abs(y-y_goal)<0.27) and (abs(yaw - yaw_des)<20)) or status == 3:
                    return True
            else:
                return False
        
    
    def tinhgoc_yaw(self,z,w):
        esp1 = 0
        esp2 = 0
        esp3 = z
        esp4= w
        r11 = 1 - 2*esp2*esp2 - 2*esp3*esp3
        r21 = 2*(esp1*esp2 + esp3*esp4)
        yaw = math.atan2(r21,r11)
        yaw = yaw*180/math.pi
        
        return yaw


            
    def send_odom(self,data):
        global done_goal, hieu
        # print("chay")
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        read_data_lidar.x_sac,read_data_lidar.y_sac,read_data_lidar.z_sac,read_data_lidar.w_sac = x,y,z,w
        orientation = data.pose.pose.orientation
        yaw = math.degrees(self.yaw_from_quaternion(orientation))
        self.data_odom = [x,y,z,w]
        if self.auto_charging:
            if self.kiemtra_hoanthanh_nv(x=x,x_goal = self.x_goal,y=y,y_goal=self.y_goal,w=w,w_goal=self.w_goal,z=z,z_goal=self.z_goal,id=123,status = hieu):
                print("=================================OK===============================================")
                self.auto_charging = False
                time.sleep(5)
                self.text_finish.emit(True)
                print('-----------------done trung gian------------')

            # else:
            #     print(f"x: {x}, y: {y}, z:{z}, w: {w}")
            #     print(f"ex: {abs(x-self.x_goal)}, ey: {abs(y-self.y_goal)}, ez:{abs(z-self.z_goal)}, ew: {abs(w-self.w_goal)}")


        # print(self.quatrinh_move)
        # print(self.data_odom)
                
        if self.quatrinh_move:
            # print("choooooo .....")

            # print("dem: ",self.dem)
            read_data_lidar.error_x = x - self.x_goal
            read_data_lidar.error_y = y - self.y_goal
            # if (abs(x-self.x_goal) < 0.13) and (abs(y-self.y_goal)<0.13) and (abs(z-self.z_goal)<0.14) and (abs(w-self.w_goal)<0.14):
            if self.kiemtra_hoanthanh_nv(x=x,x_goal = self.x_goal,y=y,y_goal=self.y_goal,w=w,w_goal=self.w_goal,z=z,z_goal=self.z_goal,id=self.id,status = hieu):
                self.dung += 1
                # print(self.dung)
                print("dungggggggggggggg",self.dung)
                if self.dung > 150:
                    if self.giaotoi == 0 and self.id != 200:
                        # --------------------- phat am thanh -------------------------
                        Uti.RobotSpeakWithPath('voice_hmi_new/new_den_vi_tri.wav')
                    # if self.giaotoi == 1:
                    #     self.done_1 = True
                    #     # --------------------- phat am thanh -------------------------
                    #     pygame.mixer.init()
                    #     pygame.mixer.music.load('voice_hmi_new/xongnhiemvu.wav')
                    #     pygame.mixer.music.play()
                    time.sleep(2)
                    print('ghi data')
                    print(x,'  ', y, '  ', z, '   ', w)
                    #with open('/home/truong/Desktop/letan_24052024/file_text_odom/odom_data.txt', 'w') as file:
                    with open(RobConf.file_path_pose_data, 'w') as file:
                        file.write(f'x: {x}, y: {y}, z: {z}, w: {w}\n')
                    # print(x,'  ', y, '  ', z, '   ', w)
                    # print(self.id)
                    time.sleep(1)

                    if self.id == 200:
                        print('da time sleep')
                        print("biet huy: ",self.biethuy)
                        # if self.biethuy:
                        #     # --------------------- phat am thanh -------------------------
                        #     pygame.mixer.init()
                        #     pygame.mixer.music.load('voice_hmi_new/biethuynv.wav')
                        #     pygame.mixer.music.play()
                        #     self.biethuy = False
                        # else:
                        #     # --------------------- phat am thanh -------------------------
                        #     pygame.mixer.init()
                        #     pygame.mixer.music.load('voice_hmi_new/xongnhiemvu.wav')
                        #     pygame.mixer.music.play()

                        # if (abs(x-self.x_goal) > 0.12):
                        #     self.move_robot_forward(1.5)
                        # print('da code')

                        try:
                            python_path = sys.executable  # Lấy đường dẫn đầy đủ đến trình thông dịch Python
                            subprocess.run([python_path, "/home/truong/Desktop/letan_24052024/cam_odom/final_cam_set.py"], check=True)
                            #subprocess.run([python_path, RobConf.file_pa, check=True)
                            self.id = 0
                            if self.biethuy:
                                # --------------------- phat am thanh -------------------------
                                Uti.RobotSpeakWithPath('voice_hmi_new/biethuynv.wav')
                                self.biethuy = False
                            else:
                                # --------------------- phat am thanh -------------------------
                                Uti.RobotSpeakWithPath('voice_hmi_new/new_hoan_thanh.wav')
                            self.arduino_charing.emit(True)

                        except subprocess.CalledProcessError as e:
                            print(f"Quá trình chạy tập lệnh Python bị lỗi: {e}")


                    self.dung = 0
                    self.enable_btn.emit(True)
                    self.quatrinh_move = False
                
            else:
                self.dung = 0
                self.voice += 1
                if self.voice > 150:
                    # print("da phat")
                    self.voice = 0
                    # --------------------- phat am thanh -------------------------
                    Uti.RobotSpeakWithPath('new_voice/new_nhuong_duong_2.wav')

                # if hieu == 4:
                #     self.dung1aidix3 += 1
                #     self.biethuy = True
                    
                #     print("dong nn",self.dung1aidix3)
                #     if self.dung1aidix3 > 40:
                #         # --------------------- phat am thanh -------------------------
                #         pygame.mixer.init()
                #         pygame.mixer.music.load('voice_hmi_new/huynhiemvu.wav')
                #         pygame.mixer.music.play()

                #         if self.thoat == False:
                #             self.move_robot_backwards()
                #             self.thoat = True
                #         elif self.thoat:
                #             self.move_robot_forward(1.5)
                #             self.thoat = False

                #         self.clear_auto.emit(True)
                #         self.x_goal = 0
                #         self.y_goal = 0
                #         self.z_goal = 0
                #         self.w_goal = 1
                #         os.system('rosservice call /move_base/clear_costmaps')
                #         time.sleep(1)
                #         goal = PoseStamped()
                #         goal.header.frame_id = "map"  
                #         goal.pose.position.x = self.x_goal   
                #         goal.pose.position.y = self.y_goal   
                #         goal.pose.orientation.z = self.z_goal  
                #         goal.pose.orientation.w = self.w_goal  
                #         self.goal_publisher.publish(goal) 
                #         # time.sleep(1)
                #         print('publish done')
                #         self.id = 200
                #         self.dung1aidix3 = 0


                
                # self.status_callback()


        # if self.quatrinh_move:
        #     # self.dung = 0
        #     # print('qqq')
        #     print("choooooo .....")
        #     if(abs(x-self.x_goal)<0.12) and (abs(y-self.y_goal)<0.12):
        #         print(f"ex: {self.x_goal}, ey: {self.y_goal}, ez:{self.z_goal}, ew: {self.w_goal}")
        #     # if abs((x-self.x_goal) < 0.12) and (abs(y-self.y_goal)<0.12) and (abs(z-self.z_goal)<0.12) and (abs(w-self.w_goal)<0.1):
        #         if abs((x-self.x_goal) < 0.12) and (abs(y-self.y_goal)<0.12) and (abs(z-self.z_goal)<0.12) and (abs(w-self.w_goal)<0.1):
        #             print("dungggggggggggggg")
        #         self.dung += 1
        #         print("bien dung: ", self.dung)

             
        #         # print(self.dung)
        #         if self.dung == 100:  
        #             if self.giaotoi == 0:
        #                 # --------------------- phat am thanh -------------------------
        #                 pygame.mixer.init()
        #                 pygame.mixer.music.load('voice_hmi_new/toibanbatky.wav')
        #                 pygame.mixer.music.play()
        #             if self.giaotoi == 1:
        #                 self.done_1 = True
        #                 # --------------------- phat am thanh -------------------------
        #                 pygame.mixer.init()
        #                 pygame.mixer.music.load('voice_hmi_new/xongnhiemvu.wav')
        #                 pygame.mixer.music.play()


        #         if self.dung >= 250:
                   
        #             # print('ghi data')
        #             with open('/home/truong/Desktop/letan_24052024/file_text_odom/odom_data.txt', 'w') as file:
        #                 file.write(f'x: {x}, y: {y}, z: {z}, w: {w}\n')
        #             #enable button xac nhan
                    
        #             # print(self.id)

        #             if self.id == 200 and self.done_1:
        #                 print(self.x_goal, '     ', self.y_goal, '    ', self.z_goal, '     ', self.w_goal)
        #                 # print('da code')
        #                 try:
        #                     python_path = sys.executable  # Lấy đường dẫn đầy đủ đến trình thông dịch Python
        #                     subprocess.run([python_path, "/home/truong/Desktop/letan_24052024/cam_odom/final_cam_set.py"], check=True)
        #                     self.id = 0
        #                 except subprocess.CalledProcessError as e:
        #                     print(f"Quá trình chạy tập lệnh Python bị lỗi: {e}")

        #             self.dung = 0
        #             self.enable_btn.emit(True)
        #             self.quatrinh_move = False
        #             self.done_1 = False
        #             # self.no_voice = True

        #     else:
              
        #         # print(self.voice)
        #         self.voice += 1
        #         if self.voice > 250:
        #             # print("da phat")
        #             self.voice = 0
        #             # --------------------- phat am thanh -------------------------
        #             pygame.mixer.init()
        #             pygame.mixer.music.load('voice_hmi_new/dichuyen.wav')
        #             pygame.mixer.music.play()
                    

                # if self.dung == 50:    
                
                
class send_cmd_vel_arduino(QThread):
    def __init__(self) :
        super().__init__()
        self.is_running = False
        
        self.twist_msg = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def run(self):
        self.cmd_vel_pub.publish(self.twist_msg)
        print("da pub cmd")
        print(self.twist_msg.linear.x)

    def send_cmd_vel(self, x, z, mode = 0):
        # z_max = 0.08
        # z_max_goal = 0.04
        # z_out = z
        # if abs(read_data_lidar.error_x) < 0.2 and abs(read_data_lidar.error_y) < 0.2:
        #     if z > z_max_goal:
        #         z_out = z_max_goal
        #     elif z < -z_max:
        #         z_out = -z_max_goal
        # else:
        #     if z > z_max:
        #         z_out = z_max
        #     elif z < -z_max:
        #         z_out = -z_max

        # print("z_out: ", z_out)
        self.twist_msg.linear.x =  x
        self.twist_msg.linear.y =  mode
        self.twist_msg.angular.z =  z

class kill_terminal(QThread):
    
    def __init__(self) :
        super().__init__()
        self.is_running = False
    
    def run(self):
        self.kill()

    def kill(self):
        # os.system('killall -9 rosmaster')
        # subprocess.call(['pkill', '-f', 'ros2'])
        # subprocess.call(['pkill', '-TERM', 'gnome-terminal'])
        

        print('1111111111111111111111')

            
        print("Đã tắt tất cả các terminal.")

class speech_recognition(QThread):
    text_finish = pyqtSignal(str)
    def __init__(self) :
        super().__init__()
        self.is_running = False
        self.speech_enabled = False
        self.recognizer = sr.Recognizer()
        self.text = ''
        
    def run(self):
        if self.speech_enabled:
            self.is_running = True
        while self.is_running:
            data = self.reco_voice()
            if self.text_finish != '':
                self.text_finish.emit(data)
            if not self.is_running:
                break
        print("stop speech recognition")

    def reco_voice(self):
        self.text = ""
        with sr.Microphone() as source:
            while pygame.mixer.music.get_busy():
                 self.recognizer.adjust_for_ambient_noise(source)
            print("Đang nghe...")
            audio = self.recognizer.listen(source)
        try:
            if self.speech_enabled:
                self.text = self.recognizer.recognize_google(audio, language="vi-VN")
                print("Đã nhận diện giọng nói:", self.text)
                return self.text

        except sr.UnknownValueError:
            print("Không thể nhận diện giọng nói.")

class move_base(QThread):
    def __init__(self) :
        super().__init__()
        self.is_running = False
        self.giaotoiban = 10
        
    def run(self):
        # rospy.Subscriber("/move_base/status", GoalStatusArray, self.check_goal)
        # --------------------- phat am thanh -------------------------
        self.is_running = True

        while self.is_running :
            Uti.RobotSpeakWithPath('new_voice/new_nhuong_duong_2.wav')
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)
                print('dang phat')
        print('da dung luong voice')
    def stop(self):
        self.is_running = False

    def play_voice(self):
        Uti.RobotSpeakWithPath('new_voice/new_nhuong_duong_2.wav')
    # def check_goal(self,data):
    #     pass
        # for status in data.status_list:
        #     if status.status == 1:  # 1 means the goal is currently being processed
        #         print("Robot is moving towards the goal")
        #     elif status.status == 3:  # 3 means the goal has been achieved
        #         # if self.giaotoiban == 0:
        #         #     # --------------------- phat am thanh -------------------------
        #         #     pygame.mixer.init()
        #         #     pygame.mixer.music.load('voice_hmi_new/toibanbatky.wav')
        #         #     pygame.mixer.music.play()
        #         # if self.giaotoiban == 1:
        
        #         print("Robot has reached the goal")
        #     elif status.status == 4:  # 4 means the goal has been aborted
        #         print("Robot has aborted the goal")
        #     elif status.status == 5:  # 5 means the goal has been canceled
        #         print("Robot has canceled the goal")

class run_file(QThread):
    oke_da_sac = pyqtSignal(int)

    def __init__(self) :
        super().__init__()
        self.is_running = False
        #self.path_run_1 = 'python3 /home/truong/Desktop/letan_24052024/pid_new4_19_06_2024.py'
        self.path_run_1 = RobConf.path_run_vaodocsac
        # self.path_run_2 = 'python3 /home/truong/Desktop/letan_24052024/qr_code_check_1.py'
        self_done_quytrinh = False
        self.done_full = False
        self.data_arduino_dock = ""
        self.x_chamdock = 0
        self.y_chamdock = 0
        self.z_chamdock = 0
        self.w_chamdock = 0

        self.sac_thanh_cong = False

        #self.worker_thread  = LED_Code()
        #self.camera_instance = Camera()
        #self.pid_yaw = A()
    def run(self):

        
        
        #pid_thread_yaw = threading.Thread(target=pid_yaw.run_code)
 

        # camera_thread = LED_Code()
        # camera_thread.start()

        #self.worker_thread.start()

        # data_2class_docksac = self.data_arduino_dock

        # qr_reader = LED_Code()
        # a = threading.Thread(target=qr_reader.run)
        # a.start()
        # time.sleep(2)
        # pid_yaw = RobotController(target_yaw=yaw_des, target_x=x_des)
        # pid_thread_yaw = threading.Thread(target=pid_yaw.PID)
        # pid_thread_yaw.start()
        # pid_thread_yaw.join()

 
        # self.is_running = True
        # self.done_full = False
        # self.done_quytrinh = False
        # while True:
        #     # process1 = subprocess.Popen(self.path_run_1, shell=True)
        #     # process1.wait()
        #     # self.is_running = False
        #     # self.done_full= True
        #     # break
        #     if  not self.done_quytrinh :
        # pygame.mixer.init()
        # pygame.mixer.music.load('new_voice/dang_lui_vao_dock.wav')
        # pygame.mixer.music.play()
        #time.sleep(2)

        process1 = subprocess.Popen(self.path_run_1, shell=True)
        process1.wait()

        Uti.RobotSpeakWithPath('voice_hmi_new/done_sac_tu_dong.wav')
        time.sleep(2)
        
        self.sac_thanh_cong = True



        

        


            #     self.done_quytrinh = True
            #     self.done_full = False
            #     print('done quy trinh PID')
            # else:
            #     process1 = subprocess.Popen(self.path_run_2, shell=True)
            #     process1.wait()
            #     self.done_full = True
            #     print('done quy trinh qr check')
            #     break
        
        # if self.done_full:
        # time.sleep(2.5)
        # with open('/home/truong/Desktop/letan_24052024/file_text_odom/odom_data.txt', 'w') as file:
        #         file.write(f'x: {self.x_chamdock}, y: {self.y_chamdock}, z: {self.z_chamdock}, w: {self.w_chamdock}\n')
        # print(f'x: {self.x_chamdock}, y: {self.y_chamdock}, z: {self.z_chamdock}, w: {self.w_chamdock}\n')
        # print('Hoan thanh quy trinh sac')
        # print('-'*50)

class arduino(QThread):
    auto_charing = pyqtSignal(bool)
    def __init__(self):
        super().__init__()
        self.ser = None
        self.data = ''
        self.is_running = False
        self.connected_ard = False
        self.count = 0
        self.nhan_nutsac = False
        self.nhan_nutbatdau = False
        self.batdau_sac = False
        
    def connect_arduino(self):
        try :
            #self.ser = serial.Serial('/dev/ttyUSB1', 9600)
            self.ser = serial.Serial(RobConf.MegaNANO_PORT, 38400)
            print('ket noi arduino thanh cong')
            time.sleep(0.5)
            self.connected_ard = True
        except Exception as ex:
            print(ex) 
        
    def read_data_from_arduino(self):
        data = self.ser.readline().decode('utf-8').strip()
        return data
  
    def run(self):
        self.data = ""
        self.count = 0
        self.is_running = True
        # self.no_voice = False
        # self.batdau_sac = False
        self.nhan_nutsac = False
        self.nhan_nutbatdau = False
        self.batdau_sac = False
        self.connect_arduino()
        if self.connected_ard:
            print('bat dau doc dien ap')
            while self.is_running:
                self.data = self.read_data_from_arduino()
                print('---volage battery ---: ',self.data)
                if (float(self.data)) < full_vol:
                    if not self.nhan_nutsac and not self.nhan_nutbatdau:
                        Uti.RobotSpeakWithPath('voice_hmi_new/vuilongsacpin.wav')
                        self.batdau_sac = True

                    else:
                        pass
                else:
                    self.batdau_sac = False
                self.is_running = False
                break
            if self.batdau_sac :
                self.auto_charing.emit(True)
            else:
                print('du pin')
            print('ngat ket noi arduino')
            self.ser.close()
            self.connected_ard = False
class set_pose_goal(QThread):
    def __init__(self) :
        super().__init__()
        self.file_path = 'file_text_odom/odom_data.txt'
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        self.done = False
        # Khởi tạo Class1 và kết nối tín hiệu send_bool của Class1 với phương thức receive_bool của Class2
        class1_instance = read_data_lidar()
        class1_instance.send_bool.connect(self.receive_bool)

        # Bắt đầu chạy thread của Class1
        class1_instance.start()

    def receive_bool(self, data):
        print("Received boolean in Class2:", data)
        self.done = data

    def run(self):
        global done_goal
        while True:
            print("Received boolean in Class2:", done_goal)
            if self.done:
                print('1')
                time.sleep(5)
                print('2')
               # with open('/home/truong/Desktop/letan_24052024/file_text_odom/odom_data.txt', 'w') as file:
                with open(RobConf.file_path_pose_data, 'w') as file:
                    file.write(f'0')

                # with open(self.file_path, 'r') as file:
                #     lines = file.readlines()
                #     if lines:
                #         last_line = lines[-1]  
                #         data = last_line.strip().split(',')
                #         for item in data:
                #             key, value = item.split(':')
                #             key = key.strip()
                #             value = value.strip()
                #             if key == 'x':
                #                 self.x = float(value)
                #             elif key == 'y':
                #                 self.y = float(value)
                #             elif key == 'z':
                #                 self.z = float(value)
                #             elif key == 'w':
                #                 self.w = float(value)
                






