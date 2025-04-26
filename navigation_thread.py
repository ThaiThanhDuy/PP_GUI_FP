from __future__ import absolute_import, division
import time
import math
from PIL import Image
import numpy as np
import yaml
# import rospy
# from nav_msgs.msg import OccupancyGrid
import os
import subprocess
import pygame
from tf_transformations import euler_from_quaternion
#------------------------------------- IMPORT PYQT5 ---------------------------------
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from Qthread_Interface import *
from PyQt5.QtCore import QThread, pyqtSignal


#------------------------------------- IMPORT THƯ VIỆN ROS ---------------------------------
# from tf.transformations import euler_from_quaternion
# from nav_msgs.msg import Odometry
# from move_base_msgs.msg import MoveBaseAction
# from geometry_msgs.msg import Twist, PoseStamped, Pose
# import actionlib
# from actionlib_msgs.msg import GoalStatus
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from sensor_msgs.msg import Imu

#------------------------------------ IMPORT CLASS FROM UI ---------------------------------
import Utilities as Uti
import RobotConfig as RobConf

#------------------------------------- IMPORT THÓNG SỐ TOÀN CỤC ---------------------------------
file_path_pose_data = RobConf.file_path_pose_data


#----------------------------------------------------------------- CLASS NAVIGATION CHÍNH -----------------------------------------------------------------

class NavigationThread(QThread):                
    send_bool = pyqtSignal(bool)
    text_finish = pyqtSignal(bool)
    enable_btn = pyqtSignal(bool)
    clear_auto = pyqtSignal(bool)
    arduino_charing = pyqtSignal(bool)
    done_navigation = pyqtSignal(bool)
    mapping_status = pyqtSignal(bool)

    x_sac = 0.0
    y_sac = 0.0
    z_sac = 0.0
    w_sac = 0.0
    error_x = 0.0
    error_y = 0.0
    error_yaw = 0.0
    
    def __init__(self):
        super().__init__()
    
        self.du_pin = True
        self.is_running = True
        self.data_odom = []
        self.auto_charging = False
        self.quatrinh_move = False
        self.no_voice = False
        self.giaotoi = 10
        self.dung = 0
        self.dung1aidix3 = 0
        self.id = 0
        self.voice = 0
        self.dem = 0
        self.saiso = 0
        self.voice_huy = 0
        self.biethuy = False
        self.thoat = False
        self.x_data_odom = 0
        self.y_data_odom = 0
        self.z_data_odom = 0
        self.w_data_odom = 0

        self.is_running = True
        self.file_path = file_path_pose_data #'file_text_odom/odom_data.txt'

        self.x_pose = 0
        self.y_pose = 0
        self.z_pose = 0
        self.w_pose = 0
        self.no_set_pose = False


        self.ve_home = False
        self.solanthu_vehome = 0
        self.solanthu_vecacvitrikhac = 0
        self.robot_dang_di_chuyen = False
        self.trangthai_navi = 0
        self.emervalue = 1 # NOrmal mode
        self.dang_dinhvi_status = False # bien status dang dinh vị
        self.dang_dinhvi_cnt = 0 # bien status dang dinh vị

        # cac bien tien xu ly khi bi ket
        self.daxoaytrai = True
        self.datientoi = True
        self.publicTwist_msg = None
        self.yawControlCnt = 0
        self.dangxoay_devedich = False # bien trang thai robot dang xoay de ve dich

        self.yaw_imu = 0
        self.mapping_processes = []
        self.navigation_processes = []
        self.map_viewer_subcriber = None
        

    def run(self):
        if self.is_running:  # KHAI BAO CAC SUBSCRIBER VẦ  PUBLISHER CUA ROS
            # rospy.Subscriber('/odom', Odometry, self.send_odom)
            # self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            # self.client.wait_for_server()

            # rospy.Subscriber('/move_base/status', GoalStatus, self.status_callback)

            # self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
            # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            # self.publicTwist_msg = Twist()

            # rospy.Subscriber('/imu/data', Imu, self.imu_callback)
            pass

    def imu_callback(self, data): # Hàm lưu trữ dữ liệu từ IMU
        self.imu_data = data
        self.yaw_imu = math.degrees(self.yaw_from_quaternion(self.imu_data.orientation))
        #print("yaw_imu: ", self.yaw_imu)

    def publish_pose(self, x, y, z, w):

        # Khởi tạo publisher cho topic 'initialpose'
        # pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        # # Đợi một chút để đảm bảo publisher khởi tạo hoàn tất
        # rospy.sleep(1)

        # # Thiết lập một tin nhắn PoseWithCovarianceStamped trong ROS
        # fixed_frame = "map"
        # pose_msg = PoseWithCovarianceStamped()
        # pose_msg.header.frame_id = fixed_frame
        # pose_msg.header.stamp = rospy.Time.now()

        # # Gán giá trị tọa độ x và y
        # pose_msg.pose.pose.position.x = x
        # pose_msg.pose.pose.position.y = y

        # # Gán giá trị orientation z và w
        # pose_msg.pose.pose.orientation.z = z
        # pose_msg.pose.pose.orientation.w = w

        # # Thiết lập covariance (độ phủ). Giá trị này có thể được tùy chỉnh tùy thuộc vào độ tin cậy của cảm biến
        # pose_msg.pose.covariance = [0.0] * 36  # Ma trận 6x6 của covariance matrix

        # # Xuất bản thông điệp lên topic 'initialpose'
        # pub.publish(pose_msg)

        # rospy.loginfo(f"[NavigationThread] Published pose: x={x}, y={y}, z={z}, w={w}")
        pass

    # --------------------SET POSE FROM FILE DATA-----------------
    def setPoseFromFile(self, fileName = file_path_pose_data):
        read_result, self.x_pose, self.y_pose, self.z_pose, self.w_pose = Uti.readPosefromFile(filePath = fileName)
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
            #                 self.x_pose = float(value)
            #             elif key == 'y':
            #                 self.y_pose = float(value)
            #             elif key == 'z':
            #                 self.z_pose = float(value)
            #             elif key == 'w':
            #                 self.w_pose = float(value)
        if read_result:
            os.system('rosservice call /move_base/clear_costmaps')

            self.publish_pose(self.x_pose,self.y_pose,self.data_odom[2],self.data_odom[3])
            
            # pub = rospy.Publisher('/slamware_ros_sdk_server_node/set_pose', Pose, queue_size=10)
            # time.sleep(1)
            # pose_msg = Pose()
            # pose_msg.position.x = self.x_pose
            # pose_msg.position.y = self.y_pose
            # #pose_msg.orientation.z = self.z_pose
            # #pose_msg.orientation.w = self.w_pose
            # pose_msg.orientation.z = self.data_odom[2]
            # pose_msg.orientation.w = self.data_odom[3]        
            # pub.publish(pose_msg)
            # time.sleep(0.5)
            return True
        else:
            return False
        
    def setpose_abs(self, xd=0, yd=0,zd=0,wd=1):
        # os.system('rosservice call /move_base/clear_costmaps')

        # self.publish_pose(self.x_pose, self.y_pose, zd, wd)

        # pub = rospy.Publisher('/slamware_ros_sdk_server_node/set_pose', Pose, queue_size=10)
        # time.sleep(1) 
        # pose_msg = Pose()
        # pose_msg.position.x = self.x_pose
        # pose_msg.position.y = self.y_pose 
        # #pose_msg.orientation.z = self.z_pose
        # #pose_msg.orientation.w = self.w_pose
        # pose_msg.orientation.z = zd #self.data_odom[2]
        # pose_msg.orientation.w = wd #self.data_odom[3]        
        # pub.publish(pose_msg) 
        time.sleep(1)
      #========================== HAM SET POSE VI TRI TU ID SQL NHAN DUOC =======================================
    def setpose_tu_sql(self, id):
        data_vitri_tramsac = Uti.nhandulieu_mysql(idban=id)
        self.data_send_sql = data_vitri_tramsac
        x_sql, y_sql, z_sql, w_sql, idout = self.data_send_sql #map(lambda x: float(x.strip("'")), self.data_send_sql)
        x_sql = float(x_sql)
        y_sql = float(y_sql)

        
        print(f"x_sql: {x_sql}, y_sql: {y_sql}")

        z_pose_sac = self.data_odom[2]
        w_pose_sac = self.data_odom[3]
        self.publish_pose(x_sql,y_sql,z_pose_sac,w_pose_sac)

        # pub = rospy.Publisher('/slamware_ros_sdk_server_node/set_pose', Pose, queue_size=10)
        # pose_msg = Pose()
        # pose_msg.position.x = x_sql
        # pose_msg.position.y = y_sql
        # z_pose_sac = self.navigation_thread.data_odom[2]
        # w_pose_sac = self.navigation_thread.data_odom[3]
        # pose_msg.orientation.z = z_pose_sac
        # pose_msg.orientation.w = w_pose_sac
        # pub.publish(pose_msg)    
            
        print("da set pose tu sql")

    def rotateRobot2Angle(self,z = 0, zd = 0,w = 1, wd = 1 ):  
          
        #tinh goc yaw hien tai
        yaw = self.tinhgoc_yaw(z, w)
        yawd = self.tinhgoc_yaw(zd, wd)
        
        yaw_err = Uti.bu_duong_tron(yawd - yaw)
    
        if (abs(yaw_err)>RobConf.DUNG_SAI_DIEM_DEN_GOC):
        
            self.yawControlCnt = self.yawControlCnt + 1
            if(self.yawControlCnt >= 250): #vuot qua so lan cho phep
                self.yawControlCnt = 250
                vw =0
                result=True 
                
            else:
                vw = yaw_err*0.01#41#25%gghv   #BO DIEU KHIEN P
                if(vw > 0.45):
                    vw = 0.45
                if(vw < -0.45):
                    vw = -0.45
                result=False
                
        else:
            self.yawControlCnt = 0
            vw =0
            result=True    
            
        print("----------------------[rotateRobot2Angle] yaw_err = ",yaw_err, "yaw: ",yaw,"yawd: ",yawd)
        self.publicTwist_msg.linear.x = 0  # Tốc độ lùi
        self.publicTwist_msg.linear.y = 7 #7 neu khong = 7 thi robot se bi xoay dan
        self.publicTwist_msg.angular.z = vw   # Không xoay
        self.cmd_vel_pub.publish(self.publicTwist_msg)
    
        return result
                #rospy.sleep(0.1)  # Chờ 0.1 giây
        #    return False #sai so chua cho phep
        # else:
        #    return True #sai so cho phep
                


    #reset done_navigation status
    def clear_done_navigation_status(self):
        self.done_navigation.emit(False)
    #-------- --------- ---- ----- HAM PUBLISH KE HOACH DI CHUYEN NAVIGATION LEN ROS ---------------------------
    def send_goal(self):
        # pub = rospy.Publisher('/slamware_ros_sdk_server_node/set_pose', Pose, queue_size=10)
        # time.sleep(1)
        print("ve home set pose: ",self.ve_home)
        
        #----------------- SET POSE------------------
        if not self.no_set_pose:
            print("co set pose")
            self.no_set_pose = self.setPoseFromFile()
            # read_result, self.x_pose, self.y_pose, self.z_pose, self.w_pose = readPosefromFile(filePath = self.file_path)
            # # with open(self.file_path, 'r') as file:
            # #     lines = file.readlines()
            # #     if lines:
            # #         last_line = lines[-1]
            # #         data = last_line.strip().split(',')
            # #         for item in data:
            # #             key, value = item.split(':')
            # #             key = key.strip()
            # #             value = value.strip()
            # #             if key == 'x':
            # #                 self.x_pose = float(value)
            # #             elif key == 'y':
            # #                 self.y_pose = float(value)
            # #             elif key == 'z':
            # #                 self.z_pose = float(value)
            # #             elif key == 'w':
            # #                 self.w_pose = float(value)
            # if read_result:
            #     pose_msg = Pose()
            #     pose_msg.position.x = self.x_pose
            #     pose_msg.position.y = self.y_pose
            #     #pose_msg.orientation.z = self.z_pose
            #     #pose_msg.orientation.w = self.w_pose
            #     pose_msg.orientation.z = self.data_odom[2]
            #     pose_msg.orientation.w = self.data_odom[3]
            
            #     pub.publish(pose_msg)

            #     time.sleep(1)

        if(self.id == 200):
            self.ve_home = True
        else:
            self.ve_home = False

        goal = PoseStamped()
        self.no_set_pose = False

        os.system('rosservice call /move_base/clear_costmaps')

        #----------------- PUBLISH GOAL ------------------
        goal.header.frame_id = "map"
        goal.pose.position.x = self.x_goal
        goal.pose.position.y = self.y_goal
        goal.pose.orientation.z = self.z_goal
        goal.pose.orientation.w = self.w_goal
        self.goal_publisher.publish(goal)

        self.done_navigation.emit(False)
        print(f'publish goal, id: {self.id}')

    ##--------------------HAM CHO PHEP SET POSE CHO ROBOT--------------
    def user_set_PoseXY(self, xP = 0, yP = 0,yaw=0):
        os.system('rosservice call /move_base/clear_costmaps')# ROS1
        # setpose
        self.publish_pose(xP,yP,0.0,yaw)


        # pub = rospy.Publisher('/slamware_ros_sdk_server_node/set_pose', Pose, queue_size=10)
        # time.sleep(1)
        
        # #----------------- SET POSE------------------
        # # if self.quatrinh_move:   
        # #     print("He thong van chua dung!!!")
        # #     return
        # pose_msg = Pose()
        # pose_msg.position.x = xP
        # pose_msg.position.y = yP
        # #pose_msg.orientation.z = self.z_pose
        # #pose_msg.orientation.w = self.w_pose
        # pose_msg.orientation.z = self.data_odom[2]
        # pose_msg.orientation.w = self.data_odom[3]
    
        # pub.publish(pose_msg)

        print('publish done')
        time.sleep(1)
        return True

        
    def send_goal_without_Setpose(self, xd = 0, yd = 0, zd = 0, wd = 1):
        #os.system('rosservice call /move_base/clear_costmaps')
        #pub = rospy.Publisher('/slamware_ros_sdk_server_node/set_pose', Pose, queue_size=10)
        #time.sleep(1)
        #print("ve home set pose: ",self.ve_home)
        
        # #----------------- SET POSE------------------
        # if not self.no_set_pose:
        #     print("co set pose")
            
        #     with open(self.file_path, 'r') as file:
        #         lines = file.readlines()
        #         if lines:
        #             last_line = lines[-1]
        #             data = last_line.strip().split(',')
        #             for item in data:
        #                 key, value = item.split(':')
        #                 key = key.strip()
        #                 value = value.strip()
        #                 if key == 'x':
        #                     self.x_pose = float(value)
        #                 elif key == 'y':
        #                     self.y_pose = float(value)
        #                 elif key == 'z':
        #                     self.z_pose = float(value)
        #                 elif key == 'w':
        #                     self.w_pose = float(value)
            
        #     pose_msg = Pose()
        #     pose_msg.position.x = self.x_pose
        #     pose_msg.position.y = self.y_pose
        #     #pose_msg.orientation.z = self.z_pose
        #     #pose_msg.orientation.w = self.w_pose
        #     pose_msg.orientation.z = self.data_odom[2]
        #     pose_msg.orientation.w = self.data_odom[3]
        
        #     pub.publish(pose_msg)

        #     time.sleep(1)

        # if(self.id == 200):
        #     self.ve_home = True
        # else:
        #     self.ve_home = False

        goal = PoseStamped()
        #self.no_set_pose = False

        #----------------- PUBLISH GOAL ------------------
        goal.header.frame_id = "map"
        goal.pose.position.x = xd
        goal.pose.position.y = yd
        goal.pose.orientation.z = zd
        goal.pose.orientation.w = wd
        self.goal_publisher.publish(goal)
        print('publish done')

    def set_vitri(self, x_goal, y_goal, z_goal, w_goal):
        self.x_goal = float(x_goal)
        self.y_goal = float(y_goal)
        self.z_goal = float(z_goal)
        self.w_goal = float(w_goal)

    def yaw_from_quaternion(self, quaternion):
        # Chuyển đổi quaternion sang góc Euler (roll, pitch, yaw)
        (roll, pitch, yaw) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw

    def tinhgoc_yaw(self, z, w):
        # esp1 = 0
        # esp2 = 0
        # esp3 = z
        # esp4 = w
        # r11 = 1 - 2 * esp2 * esp2 - 2 * esp3 * esp3
        # r21 = 2 * (esp1 * esp2 + esp3 * esp4)
        # yaw = math.atan2(r21, r11)
        # yaw = yaw * 180 / math.pi

        (roll, pitch, yaw) = euler_from_quaternion([0, 0,z, w])
        yaw = yaw * 180 / math.pi
        

        return yaw

    def status_callback(self, msg): #trang thai navigation
        for goal_status in msg.status_list:
            self.trangthai_navi = goal_status.status

    def kiemtra_hoanthanh_nv_tructiep(self):
        x, y,z, w = self.data_odom
        return self.kiemtra_hoanthanh_nv(x=x, x_goal=self.x_goal, y=y, y_goal=self.y_goal, z=z,
                                         z_goal=self.z_goal,w=w, w_goal=self.w_goal, id=self.id, status=self.trangthai_navi) 
    def kiemtra_hoanthanh_nv(self, x, x_goal, y, y_goal, z, z_goal, w, w_goal, id, status=1): #kiem tra sai so hoan thanh nhiem vu

        yaw_des = self.tinhgoc_yaw(z=z_goal, w=w_goal)
        yaw = self.tinhgoc_yaw(z=z, w=w)
        #print(f"[kiemtra_hoanthanh_nv] x ={x}, y = {y},yaw = {yaw} ,x_goal = {x_goal},y_goal = {y_goal}, yaw_des = {yaw_des}, ---- status = {status}")
        # print("[kiemtra_hoanthanh_nv] id_hamnhiemvu",id)
        # print(f"[kiemtra_hoanthanh_nv] data odom: {x} {y}")  
        # print(f"[kiemtra_hoanthanh_nv] dung sai {RobConf.DUNG_SAI_DIEM_DEN_VITRI}")
        if id == 200:
            if (abs(x - x_goal) < RobConf.DUNG_SAI_DIEM_DEN_VITRI) and (abs(y - y_goal) < RobConf.DUNG_SAI_DIEM_DEN_VITRI) and status == 3:
                self.solanthu_vehome = 0        
                return True
            else:
                return False
        elif id == 123:
            if (abs(x - x_goal) < RobConf.DUNG_SAI_DIEM_DEN_VITRI) and (abs(y - y_goal) < RobConf.DUNG_SAI_DIEM_DEN_VITRI) and status==3:
                # print(
                #     f"x ={x}, y = {y},yaw = {yaw} ,x_goal = {x_goal},y_goal = {y_goal}, yaw_des = {yaw_des}, ---- status = {status}")
                print("da den vi tri cho sac")
                self.solanthu_vecacvitrikhac = 0
                #time.sleep(1)
                return True
            else:
                # print(
                #     f"x ={x}, y = {y},yaw = {yaw} ,x_goal = {x_goal},y_goal = {y_goal}, yaw_des = {yaw_des}, ---- status = {status}")
                return False
        else:
            if ((abs(x - x_goal) < RobConf.DUNG_SAI_DIEM_DEN_VITRI) and (abs(y - y_goal) < RobConf.DUNG_SAI_DIEM_DEN_VITRI) ) and status == 3:
                self.solanthu_vecacvitrikhac = 0
                
                return True
            else:
                # print("dang di chuyen")
                return False

    def send_odom(self, data):  #---------- doc gia tri odom hien tai va xu ly qua trinh di chuyen --------------
        global done_goal
        # print("chay")
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        read_data_lidar.x_sac, read_data_lidar.y_sac, read_data_lidar.z_sac, read_data_lidar.w_sac = x, y, z, w
        orientation = data.pose.pose.orientation
        yaw = math.degrees(self.yaw_from_quaternion(orientation))

        self.data_odom = [x, y, z, w]           

        # print("auto: ", self.auto_charging)
        if self.auto_charging:
            if self.dangxoay_devedich or self.kiemtra_hoanthanh_nv(x=x, x_goal=self.x_goal, y=y, y_goal=self.y_goal, z=z,
                                         z_goal=self.z_goal,w=w, w_goal=self.w_goal,  id=123, status=self.trangthai_navi):
                self.quatrinh_move = False
            
                if (not self.rotateRobot2Angle(z = z,  zd = self.z_goal, w=w, wd = self.w_goal)):
                    self.dangxoay_devedich = True

                else:  
                    self.dangxoay_devedich = False 
                    print("=================================OK===============================================")
                    self.auto_charging = False
                    self.quatrinh_move = False
                    time.sleep(5)
                    self.text_finish.emit(True)
                    print('-----------------done trung gian------------')

        if self.quatrinh_move or self.dangxoay_devedich:
            self.robot_dang_di_chuyen = True
            #print("---------- ve home ------------------: ",self.ve_home)
            
            # print("choooooo .....")

            # print("dem: ",self.dem)
            read_data_lidar.error_x = x - self.x_goal
            read_data_lidar.error_y = y - self.y_goal
            #print(f"[send_odom] data odom: {x} {y}") 
            # if (abs(x-self.x_goal) < 0.13) and (abs(y-self.y_goal)<0.13) and (abs(z-self.z_goal)<0.14) and (abs(w-self.w_goal)<0.14):
            if self.dangxoay_devedich or (self.kiemtra_hoanthanh_nv(x=x, x_goal=self.x_goal, y=y, y_goal=self.y_goal, z=z,
                                         z_goal=self.z_goal,w=w, w_goal=self.w_goal, id=self.id, status=self.trangthai_navi) and not self.ve_home):
                self.quatrinh_move=False
                print("[send_odon] bien dang ve dich:", self.dangxoay_devedich)
                if (not self.rotateRobot2Angle(z = z,  zd = self.z_goal, w=w, wd = self.w_goal)):
                    self.dangxoay_devedich = True

                else:  
                                
                    self.dung += 1
                    print(self.dung)
                    if(self.dung > 0):
                        print("dungggggggggggggg", self.dung)
                    if self.id != 123:
                        if self.dung > 0:
                            self.dangxoay_devedich = False  
                            

                            print(f'ghi data id: {self.id}')
                            print(x, '  ', y, '  ', z, '   ', w)
                            Uti.writePose2File(filePath = file_path_pose_data,x=x,y=y,z=z,w=w)
                            #with open('file_text_odom/odom_data.txt', 'w') as file:
                            #    file.write(f'x: {x}, y: {y}, z: {z}, w: {w}\n')
                            # print(x,'  ', y, '  ', z, '   ', w)
                            # print(self.id)
                            time.sleep(1)

                            self.dung = 0
                            self.robot_dang_di_chuyen = False
                            self.enable_btn.emit(True)
                            self.quatrinh_move = False
                            print('Done navigation TRUE')
                            self.done_navigation.emit(True)

                    
                    else:
                        if self.dung > 0: #250
                            self.dangxoay_devedich = False  
                            

                            print('ghi data 123:')
                            print(x, '  ', y, '  ', z, '   ', w)
                            Uti.writePose2File(filePath = file_path_pose_data,x=x,y=y,z=z,w=w)
                            #with open('file_text_odom/odom_data.txt', 'w') as file:
                            #    file.write(f'x: {x}, y: {y}, z: {z}, w: {w}\n')
                            # print(x,'  ', y, '  ', z, '   ', w)
                            # print(self.id)
                            time.sleep(1)

                            self.dung = 0
                            self.robot_dang_di_chuyen = False
                            self.enable_btn.emit(True)
                            self.quatrinh_move = False
                            print('Done navigation TRUE')
                            self.done_navigation.emit(True)
                  #  self.dangxoay_devedich = False  
            elif self.ve_home:
                
                # self.enable_btn.emit(True)
                # self.robot_dang_di_chuyen = False
                # # self.done_navigation.emit(True)
                # self.quatrinh_move = False    
                if  self.dangxoay_devedich or self.kiemtra_hoanthanh_nv(x=x, x_goal=self.x_goal, y=y, y_goal=self.y_goal, z=z,
                                         z_goal=self.z_goal,w=w, w_goal=self.w_goal,  id=self.id, status=self.trangthai_navi):
                    self.quatrinh_move=False
                    print("[send_odon] [vehome] bien dang ve dich:", self.dangxoay_devedich)
                    if (not self.rotateRobot2Angle(z = z,  zd = self.z_goal, w=w, wd = self.w_goal)):
                        self.dangxoay_devedich = True
                    else:
                   #     self.dangxoay_devedich = False
                        self.dung += 1
                        # print(self.dung)
                        print("dungggggggggggggg home", self.dung)
                        if self.dung > 0:
                            self.dangxoay_devedich = False  
                          #  self.done_navigation.emit(True)

                            print(f'ghi data id: {self.id}')
                            print(x, '  ', y, '  ', z, '   ', w)
                            Uti.writePose2File(filePath = file_path_pose_data,x=x,y=y,z=z,w=w)
                            #with open('file_text_odom/odom_data.txt', 'w') as file:
                            #    file.write(f'x: {x}, y: {y}, z: {z}, w: {w}\n')
                            # print(x,'  ', y, '  ', z, '   ', w)
                            # print(self.id)
                            time.sleep(1)

                            self.dung = 0
                            self.robot_dang_di_chuyen = False
                            self.enable_btn.emit(True)
                            self.quatrinh_move = False
                            print('Done navigation TRUE')
                            self.done_navigation.emit(True)
                            #self.dangxoay_devedich = False  
                else:                    
                    print("[SEND_ODOM][vehome]-----------------trangthai_navi: ----------------------: ",self.trangthai_navi)
                    if((self.trangthai_navi==3 or self.trangthai_navi==4) and self.id==200): #xu ly khi robot dax ve lai Home nhung chua dat dc do chinh xac
                        if(self.emervalue < RobConf.LockedVelMode): # ơ che do thuong
                            if self.dang_dinhvi_status: #dang o che do dinh vi
                                self.dang_dinhvi_cnt = self.dang_dinhvi_cnt + 1  #tang so lan thu 
                                if(self.dang_dinhvi_cnt >= RobConf.SO_LAN_THU_DINH_VI_MAX):
                                    self.dang_dinhvi_cnt = RobConf.SO_LAN_THU_DINH_VI_MAX
                                    self.quatrinh_move=False
                                    print('[SEND_ODOM][vehome]So lan thu dinh vi toi da')
                                else:
                                    self.ve_lai_vi_tri(x=self.x_goal,y=self.y_goal,z=self.z_goal,w=self.w_goal, id_velai=self.id) # ve lai home lan nua
                            else:
                                self.solanthu_vehome = self.solanthu_vehome + 1
                                if(self.solanthu_vehome >= RobConf.SO_LAN_THU_VEHOME_MAX):
                                    self.quatrinh_move=False
                                    self.solanthu_vehome = RobConf.SO_LAN_THU_VEHOME_MAX
                                    print('[SEND_ODOM][vehome]So lan thu ve hom toi da')
                                    Uti.RobotSpeakWithPath('voice_hmi_new/Toi_da_bi_ket.mp3')
                                else:
                                    print("[SEND_ODOM][vehome]-----------------quay ve lai diem home ----------------------")
                                    self.ve_lai_vi_tri(x=0,y=0,z=0,w=1, id_velai=200) # ve lai home lan nua
                        else:
                            print("[SEND_ODOM] Dong co dang bi khoa")


            else:
                self.done_navigation.emit(False)
                self.dung = 0
                self.voice += 1
                if self.voice > 150:
                    self.voice = 0
                    # --------------------- phat am thanh -------------------------
                    Uti.RobotSpeakWithPath('voice_hmi_new/new_nhuong_duong_2.wav')
                
                if self.trangthai_navi == 3 or self.trangthai_navi == 4: 
                    if(self.emervalue < RobConf.LockedVelMode): # ơ che do thuong
                        if self.dang_dinhvi_status: #dang o che do dinh vi
                            self.dang_dinhvi_cnt = self.dang_dinhvi_cnt + 1  #tang so lan thu 
                            if(self.dang_dinhvi_cnt >= RobConf.SO_LAN_THU_DINH_VI_MAX):
                                self.dang_dinhvi_cnt = RobConf.SO_LAN_THU_DINH_VI_MAX
                                self.quatrinh_move=False
                                print('[SEND_ODOM][ELSE][DV]]So lan thu dinh vi toi da')
                            else:
                                #print('[SEND_ODOM][ELSE][DV]] dang thu dinh vi lai')
                                
                                self.ve_lai_vi_tri(x=self.x_goal,y=self.y_goal,z=self.z_goal,w=self.w_goal, id_velai=self.id) # ve lai home lan nua

                        else: 
                            # TIEN XU LY KHI BI KET
                            self.biethuy = True
                            if self.trangthai_navi == 3:
                            # tiep tuc di nua
                                self.robotXoayTraiPhai()                                
                            
                            if self.trangthai_navi == 4:
                                self.robotTienLui()
                            
                            if self.id == RobConf.HOME_ID: # XU LY TAI DIEM HOME
                                self.solanthu_vecacvitrikhac = 0 
                                self.solanthu_vehome = self.solanthu_vehome + 1
                                if(self.solanthu_vehome >= RobConf.SO_LAN_THU_VEHOME_MAX):
                                    self.solanthu_vehome = RobConf.SO_LAN_THU_VEHOME_MAX
                                    self.quatrinh_move=False
                                    print('[SEND_ODOM][vehome]So lan thu ve hom toi da')
                                    Uti.RobotSpeakWithPath('voice_hmi_new/Toi_da_bi_ket.mp3')
                                else:
                                    print("[SEND_ODOM][ELSE]-----------------quay ve lai diem home ----------------------")
                                    self.ve_lai_vi_tri(x=0,y=0,z=0,w=1, id_velai=RobConf.HOME_ID) # ve lai home lan nua
                            else:     # VE LAI DIEM KHAC
                                self.solanthu_vehome = 0
                                self.solanthu_vecacvitrikhac = self.solanthu_vecacvitrikhac + 1
                                if(self.solanthu_vecacvitrikhac >= RobConf.SO_LAN_THU_VECACVITRIKHAC_MAX): #Huy nhiem vu, cho robot ve lai home
                                    self.solanthu_vecacvitrikhac = RobConf.SO_LAN_THU_VECACVITRIKHAC_MAX
                                    self.quatrinh_move=False
                                    Uti.RobotSpeakWithPath('voice_hmi_new/Huy_nhiem_vu_quay_ve.mp3')
                                    self.ve_lai_vi_tri(x=0,y=0,z=0,w=1, id_velai=RobConf.HOME_ID) # ve lai home
                                else:
                                    print(F'[SEND_ODOM][ELSE] thu di tiep den diem {self.id}, current:{x},{y},{z},{w}, Target: {self.x_goal}, {self.y_goal}, {self.z_goal}, {self.w_goal}')
                                    self.ve_lai_vi_tri(x=self.x_goal,y=self.y_goal,z=self.z_goal,w=self.w_goal, id_velai=self.id) # thu ve vi tri cu lan nua lan nua
                    else:
                        print("[SEND_ODOM][ELSE] Dong co dang bi khoa")
                else:
                    pass

    

    def ve_lai_vi_tri(self,x=0,y=0,z=0,w=1,id_velai = 200): 
        self.x_goal = x
        self.y_goal = y
        self.z_goal = z
        self.w_goal = w
        self.id = id_velai
        os.system('rosservice call /move_base/clear_costmaps')
        time.sleep(1)
        goal = PoseStamped()
        goal.header.frame_id = "map"  
        goal.pose.position.x = self.x_goal   
        goal.pose.position.y = self.y_goal   
        goal.pose.orientation.z = self.z_goal  
        goal.pose.orientation.w = self.w_goal  
        self.goal_publisher.publish(goal) 

    def goi_vantocCuongbuc(self, mode = RobConf.LockedVelMode):
        # Tạo lệnh điều khiển di chuyển lùi
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0  # Tốc độ lùi
        cmd_vel_msg.linear.y = mode 
        cmd_vel_msg.angular.z = 0.0   # Không xoay
        # Gửi lệnh điều khiển cho một khoảng thời gian
        start_time = time.time()
        while time.time() - start_time < 1:  # Lùi trong 3 giây
            # self.voice_huy += 1
            # print("voice: ",self.voice_huy)
            # if self.voice_huy == 15:
            #     # --------------------- phat am thanh -------------------------
            #     pygame.mixer.init()
            #     pygame.mixer.music.load('voice_hmi_new/huynhiemvu.wav')
            #     pygame.mixer.music.play()

            #     self.voice_huy = 0

            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(0.1)  # Chờ 0.1 giây

    def move_robot_backwards(self, giay):
            # Tạo lệnh điều khiển di chuyển lùi
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -RobConf.VANTOC_TIEN  # Tốc độ lùi
        cmd_vel_msg.linear.y = RobConf.ManualVelMode 
        cmd_vel_msg.angular.z = 0.0   # Không xoay
        # Gửi lệnh điều khiển cho một khoảng thời gian
        start_time = time.time()
        while time.time() - start_time < giay:  # Lùi trong 3 giây
            # self.voice_huy += 1
            # print("voice: ",self.voice_huy)
            # if self.voice_huy == 15:
            #     # --------------------- phat am thanh -------------------------
            #     pygame.mixer.init()
            #     pygame.mixer.music.load('voice_hmi_new/huynhiemvu.wav')
            #     pygame.mixer.music.play()

            #     self.voice_huy = 0

            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(0.1)  # Chờ 0.1 giây
        
    def move_robot_forward(self, giay = 1.5):
        # Tạo lệnh điều khiển di chuyển lùi
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = RobConf.VANTOC_TIEN  # Tốc độ lùi
        cmd_vel_msg.linear.y = RobConf.ManualVelMode 
        cmd_vel_msg.angular.z = 0.0   # Không xoay
        # Gửi lệnh điều khiển cho một khoảng thời gian
        start_time = time.time()
        while time.time() - start_time < giay:  # Lùi trong 3 giây
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(0.1)  # Chờ 0.1 giây

    def move_robot_phai(self, giay = 1.5):
        # Tạo lệnh điều khiển di chuyển lùi
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0  # Tốc độ lùi
        cmd_vel_msg.linear.y = RobConf.ManualVelMode 
        cmd_vel_msg.angular.z = RobConf.VANTOC_XOAY   # Không xoay
        # Gửi lệnh điều khiển cho một khoảng thời gian
        start_time = time.time()
        while time.time() - start_time < giay:  # Lùi trong 3 giây
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(0.1)  # Chờ 0.1 giây

    def move_robot_trai(self, giay = 1.5):
        # Tạo lệnh điều khiển di chuyển lùi
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0  # Tốc độ lùi
        cmd_vel_msg.linear.y = RobConf.ManualVelMode 
        cmd_vel_msg.angular.z = -RobConf.VANTOC_XOAY   # Không xoay
        # Gửi lệnh điều khiển cho một khoảng thời gian
        start_time = time.time()
        while time.time() - start_time < giay:  # Lùi trong 3 giây
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(0.1)  # Chờ 0.1 giây

    # HAM DIEU KHIEN ROBOT TIEN TOI - LUI BAN TU DONG
    def robotTienLui(self):
        if self.datientoi == False:
            self.move_robot_forward(3)
            self.datientoi = True
        else:
            self.move_robot_backwards(1.5)
            self.datientoi = False
    # HAM DIEU KHIEN ROBOT XOAY TRAI-PHAI BAN TU DONG
    def robotXoayTraiPhai(self):
        if self.daxoaytrai == False:
            self.move_robot_phai(3)
            self.daxoaytrai = True
        else:
            self.move_robot_trai(1.5)
            self.daxoaytrai = False
    
    # CAC HAM XU LY MAPPING TREN GIOA DIEN
    