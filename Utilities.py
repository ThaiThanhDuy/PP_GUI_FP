import cv2
import numpy as np
import math
import time
import threading
import time
from geometry_msgs.msg import Twist, PoseStamped, Pose
import subprocess
from nav_msgs.msg import Odometry
import subprocess
import sys
import mysql.connector
from mysql.connector import Error
import pygame
import pygame.mixer
import os
import shutil
import pickle

import RobotConfig as RobConf
##------------------------------------
# HAM TIM THONG TIN DUONG TRONG LON NHAT CO TRONG ANH: TRA VE: CO/KHONG, TAM_X, TAM_Y, BK
# QUY TRINH:
# NEU KO DOC DC CAM -> THOAT
# NEU DOC DUOC CAM: 1) DOC CAM -> 2) LOC NHIEU -> TIM DUONG TRON LON NHAT -> 
#                       + NEU CO --->THOAT
#                       + NEU KHONG -> TANG DO SANG THEM 5 : NEU DO SANG > 255 --> THOAT
#                                    NEU KHONG -> QUAY LAI BUOC 1

def image_path(relative_file_path):
  return f"{RobConf.IMAGE_FOLDER}/{relative_file_path}"

def voice_path(relative_file_path):
  return f"{RobConf.VOICE_FOLDER}/{relative_file_path}"

def findLargestCircleFromCam(Cam_ID = 0, radius_max = 500, goc_came_nghieng = 0, error_radius = 10):
  print('------------ bat dau tim tam hinh tron -----------')
  cx, cy = 0, 0
  main_radius = 0
  cap = cv2.VideoCapture(Cam_ID)  # Mở camera
  if not cap.isOpened():
    print("Không thể mở camera.")
    return False
  
  brightness_value = 150
  cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness_value)  # Thiết lập độ sáng ban đầu
  
  circle_found_count = 0
  largest_circle = None  # Biến lưu trữ thông tin hình tròn lớn nhất tìm được

  while True:
    ret, frame = cap.read()
    if not ret:
      print("Không thể đọc khung hình từ camera.")
      return (False,0, 0, 0)

    frame = cv2.resize(frame, (1000, 600))  # Thay đổi kích thước khung hình
    blurred_frame = cv2.GaussianBlur(frame, (25, 25), 0)  # Làm mờ khung hình để giảm nhiễu
    hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)  # Chuyển đổi sang không gian màu HSV

    color_ranges = {
        'yellow': ([20, 100, 100], [36, 255, 255])  # Định nghĩa khoảng màu vàng trong không gian HSV
          # 'green': ([36, 25, 25], [86, 255, 255])
      }

    combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)  # Tạo một mặt nạ kết hợp

    for color_name, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))  # Tạo mặt nạ cho màu sắc
        combined_mask = cv2.bitwise_or(combined_mask, mask)  # Kết hợp các mặt nạ lại với nhau
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Tìm các đường viền

        for contour in contours:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)  # Tìm hình tròn bao quanh đường viền
            center = (int(x), int(y))
            radius = int(radius)
            main_radius = radius
            if radius < radius_max:
                if largest_circle is None: 
                    largest_circle = (center[0], center[1], radius)  # Cập nhật thông tin hình tròn lớn nhất
                    main_radius = radius
                else:
                    if radius > largest_circle[2]:
                      largest_circle = (center[0], center[1], radius)  # Cập nhật thông tin hình tròn lớn nhất
                      main_radius = radius
                    else:
                       pass

    if not (largest_circle is None):
      origin_point_x, origin_point_y, radius = largest_circle
      cv2.circle(frame, (origin_point_x, origin_point_y), radius, (0, 255, 0), 2)  # Vẽ hình tròn lớn nhất
      cv2.circle(frame, (origin_point_x, origin_point_y), 5, (0, 0, 255), -1)  # Vẽ tâm của hình tròn
      print(f"Tâm hình tròn: {origin_point_x} | {origin_point_y}")
      home_xy_cam = pixel_to_xy_cam(origin_point_y, origin_point_x, 500, 300, goc_came_nghieng)  # Chuyển đổi tọa độ tâm
      cx = home_xy_cam[0]
      cy = home_xy_cam[1]
      #print(f"home: {home_xy_cam}")
      circle_found_count += 1
      break

    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2
    third_width = width // 2
    third_height = height // 2

    cv2.line(frame, (0, third_height), (width, third_height), (255, 0, 0), 2)  # Vẽ đường ngang giữa khung hình
    cv2.line(frame, (third_width, 0), (third_width, height), (255, 0, 0), 2)  # Vẽ đường dọc giữa khung hình

    the_d = goc_came_nghieng * math.pi / 180  # Góc goc_came_nghieng độ tính theo radian
    d_draw = 500
    cv2.line(frame, (center_x, center_y), 
              (int(center_x + d_draw * math.cos(the_d)), int(center_y + d_draw * math.sin(the_d))), 
              (0, 0, 255), 2)  # Vẽ đường chéo thứ nhất
    cv2.line(frame, (center_x, center_y), 
              (int(center_x - d_draw * math.cos(the_d)), int(center_y - d_draw * math.sin(the_d))), 
              (0, 0, 255), 2)  # Vẽ đường chéo thứ hai
    cv2.line(frame, (center_x, center_y), 
              (int(center_x - d_draw * math.sin(the_d)), int(center_y + d_draw * math.cos(the_d))), 
              (0, 0, 255), 2)  # Vẽ đường chéo thứ ba
    cv2.line(frame, (center_x, center_y), 
              (int(center_x + d_draw * math.sin(the_d)), int(center_y - d_draw * math.cos(the_d))), 
              (0, 0, 255), 2)  # Vẽ đường chéo thứ tư

    cv2.imshow('Webcam', frame)  # Hiển thị khung hình

    if not largest_circle:
        brightness_value += 20  # Tăng độ sáng nếu không tìm thấy hình tròn
        if brightness_value > 255:
            brightness_value = 255
            break
        cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness_value)
        print(f"Không tìm thấy hình tròn. Độ sáng được tăng lên: {brightness_value}")
        time.sleep(0.5)

    '''if (cv2.waitKey(1) & 0xFF == ord('q')) or circle_found_count == 25:'''

    if (cv2.waitKey(1) & 0xFF == ord('q')) or circle_found_count == 50:
        time.sleep(0.5)
        break

  cap.release()
  cv2.destroyAllWindows()

  if largest_circle:
      check_circle = True
      return (check_circle,cx, cy, main_radius)
  else:
      check_circle = False
      return (check_circle,0, 0, 0)
  
  
  
  
  
#   print("Hello, " + name)
def ma_tran_xoay(x, y, theta):
  theta_rad = math.radians(theta)  # Chuyển đổi góc từ độ sang radian
  cos_theta = math.cos(theta_rad)  # Tính cos(theta)
  sin_theta = math.sin(theta_rad)  # Tính sin(theta)
  x_new = x * cos_theta - y * sin_theta  # Công thức tính tọa độ x mới
  y_new = x * sin_theta + y * cos_theta  # Công thức tính tọa độ y mới
  return round(x_new), round(y_new)  # Trả về tọa độ mới sau khi xoay

# Hàm chuyển đổi tọa độ pixel thành tọa độ của camera
def pixel_to_xy_cam(pixelV, pixelH, centerX, centerY, theta_d):
  t1 = tinh(pixelV, pixelH, centerX, centerY)  # Tính tọa độ chênh lệch so với tâm
  t2 = ma_tran_xoay(t1[0], t1[1], theta_d)  # Xoay tọa độ đó theo góc theta_d
  return t2
# Hàm tính tọa độ chênh lệch giữa pixel hiện tại và tâm (centerX, centerY)
def tinh(pixelV, pixelH, centerX, centerY):
  pixelY = pixelH - centerX
  pixelX = pixelV - centerY
  print(f"pixelX: {pixelX}, pixelY: {pixelY}, pixelV: {pixelV}, pixelH: {pixelH}")
  return pixelX, pixelY
# Hàm để xoay điểm (x, y) một góc theta (độ)
def ma_tran_xoay(x, y, theta):
  theta_rad = math.radians(theta)  # Chuyển đổi góc từ độ sang radian
  cos_theta = math.cos(theta_rad)  # Tính cos(theta)
  sin_theta = math.sin(theta_rad)  # Tính sin(theta)
  x_new = x * cos_theta - y * sin_theta  # Công thức tính tọa độ x mới
  y_new = x * sin_theta + y * cos_theta  # Công thức tính tọa độ y mới
  return round(x_new), round(y_new)  # Trả về tọa độ mới sau khi xoay

##-----------------------------------------
# DOC VI TRI KHU VUC CHO: RETURN SUCCESS/NOT, X, Y, Z, W
def read_kvc():
  #global z_pose, w_pose
  x_pose, y_pose, z_pose, w_pose = 0,0,0,1

  result = False
  with open('file_text_odom/KHUVUCCHO_data.txt', 'r') as file:
    lines = file.readlines()
  if lines:
    last_line = lines[-1]  
    data = last_line.strip().split(',')
    for item in data:
      key, value = item.split(':')
      key = key.strip()
      value = value.strip()
      if key == 'x':
        x_pose = float(value)
        result = True
      elif key == 'y':
        y_pose = float(value)
        result = True
      elif key == 'z':
        z_pose = float(value)
        result = True
      elif key == 'w':
        w_pose = float(value)
        result = True
  print("----------------------------------------------------", z_pose,"          ",w_pose)
  return (result, x_pose, y_pose, z_pose, w_pose)
  #print("----------------------------------------------------", z_pose,"          ",w_pose)

def rotate_robot_free(quaternion_d, timeout_d):
    # global daquayxong
    # global z_pose, w_pose, robot_yaw_goal_degree, robot_yaw_degree, robot_x, robot_y, robot_z, robot_w
  data = Odometry  
  robot_x = data.pose.pose.position.x
  robot_y = data.pose.pose.position.y
  robot_z = data.pose.pose.orientation.z
  robot_w = data.pose.pose.orientation.w
    
    # Chuyển đổi từ quaternion sang góc Euler để lấy góc yaw
  quaternion = (
      data.pose.pose.orientation.x,
      data.pose.pose.orientation.y,
      data.pose.pose.orientation.z,
      data.pose.pose.orientation.w
  )
  _, _, robot_yaw = euler_from_quaternion(quaternion)
    # quaternion1 = (
    #     0,
    #     0,
    #     0,
    #     1
    #     # z_pose,
    #     # w_pose
    #     # 0,1

    # )
  _, _, robot_yaw_goal = euler_from_quaternion(quaternion_d) 

  robot_yaw_degree = np.degrees(robot_yaw)
  robot_yaw_goal_degree = np.degrees(robot_yaw_goal)    

  error = robot_yaw_goal_degree - robot_yaw_degree

    # print(round(robot_yaw_goal_degree,2), "   |   ", round(robot_yaw_degree,2), "    |    ", round(error,2) )
  daquayxong == False
  cnt = 0
  # pub_twist = rospy.Publisher('/cmd_vesl', Twist, queue_size=10)
  twist_msg = Twist()
  while not daquayxong and cnt < timeout_d:
    cnt +=1
    if error < 5 and error > -5:
        twist_msg.angular.z = 0
        twist_msg.linear.x=0
        twist_msg.linear.y = 1
        pub_twist.publish(twist_msg)
        print(round(robot_yaw_goal_degree,2), "   |   ", round(robot_yaw_degree,2), "    |    ", round(error,2) )
        # print("dung ")
        daquayxong = True
    else:  
        vw = error*0.1   #BO DIEU KHIEN P
        if(vw > 1.5):
            vw = 1.5
        if(vw < -1.5):
            vw = -0.5
        twist_msg.angular.z = vw
        twist_msg.linear.x=0
        twist_msg.linear.y = 1
        pub_twist.publish(twist_msg)
  return daquayxong

##--------------HAM SET UP CAM
def quytrinhSetupCam(camID = 4, yellowCirRadi = 0.03):
  hoan_thanh = False
  tinh_x, tinh_y = 0, 0
  bankinh_duongtrong_vang = yellowCirRadi
  #B1: DICH ROBOT VE GOC THANG
  print('------------------------- XOAY ROBOT -----------------------')
    
  # quaternion_d = (
  #       0,
  #       0,
  #       0,
  #       1
  #       # z_pose,
  #       # w_pose
  #       # 0,1

  #   )
  # rotate_robot_free( quaternion_d, 500)
  python_path = sys.executable  # Lấy đường dẫn đầy đủ đến trình thông dịch Python
  subprocess.run([python_path, "cam_odom/robot_rotate.py"],
                                   check=True)     # DIEU KHIEN ROBOT VE VI TTRI 0 DO
  time.sleep(0.5)
  print('-------------------- TÌM TÂM HÌNH TRÒN ---------------------')
  result, cx, cy, radius = findLargestCircleFromCam(Cam_ID = camID, radius_max = 70)   # TRA VE BIEN CO HOAC KHONG, VA THONG TIN TAM, BAN KINH DUONG TRON MAU VANG
  error_radius = 10
  if result: #TINH tinh_x, tuin, luu file, hien thong bao va giong noi
    #TINH TI LE GIUA PIXEL VS MET 
    tinh_x = float( bankinh_duongtrong_vang / radius )  
    tinh_y = -float( bankinh_duongtrong_vang / radius )
    #-------------------- Ghi vao file text -----------------
    with open('cam_odom/thongso_cam.txt', 'w') as file:
        file.write(f"{tinh_x}\n")
        file.write(f"{tinh_y}\n")
        file.write(f"{radius}\n")
        file.write(f"{error_radius}\n")
        file.write(f"{cx}\n")
        file.write(f"{cy}\n")
        #file.write(f"{home_xy_cam[0]}\n")
        #file.write(f"{home_xy_cam[1]}\n")
    print("Đã ghi hai biến float vào tệp thongso_cam.txt")
    hoan_thanh = True

  else: #HIEN THONG BAO "SETUP CAM THAT BAI"
    print("Khong tim thay hinh trong dinh vi")
  return (hoan_thanh, tinh_x, tinh_y)


##--------------HAM SET UP CAM
def dieuchinhvaTimduongtron(camID = 4, yaw_ros = 0): 
  #B1: DICH ROBOT VE GOC THANG
  print('------------------------- XOAY ROBOT -----------------------')
    
  # quaternion_d = (
  #       0,
  #       0,
  #       0,
  #       1
  #       # z_pose,
  #       # w_pose
  #       # 0,1

  #   )
  # rotate_robot_free( quaternion_d, 500)
  if(abs(yaw_ros) > RobConf.DUNG_SAI_XOAY_ROBOT_DINHVI): # tien hanh xoay robot khi goc lech lon
    time.sleep(1)
    python_path = sys.executable  # Lấy đường dẫn đầy đủ đến trình thông dịch Python
    subprocess.run([python_path, "cam_odom/robot_rotate.py"],
                                   check=True)     # DIEU KHIEN ROBOT VE VI TTRI 0 DO
    time.sleep(1)
  print('-------------------- TÌM TÂM HÌNH TRÒN ---------------------')
  result, cx, cy, radius = findLargestCircleFromCam(Cam_ID = camID, radius_max = 70)   # TRA VE BIEN CO HOAC KHONG, VA THONG TIN TAM, BAN KINH DUONG TRON MAU VANG
  print(f'thong tin duong trong: tam x = {cx}, y = {cy}, bk = {radius}')
  return (result, cx, cy, radius)


def quaternion_to_euler( x, y, z, w):  
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  X = math.degrees(math.atan2(t0, t1))

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  Y = math.degrees(math.asin(t2))

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  Z = math.degrees(math.atan2(t3, t4))

  return (X, Y, Z)

def euler_to_quaternion( yaw = 0, pitch = 0, roll = 0):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return (qx, qy, qz, qw)

##--------------------------------------------
# READ POSE DATA FROM FILE
#----------------------------------------------
def readPosefromFile(filePath = 'file_text_odom/odom_data.txt'):
  result = 0
  x,y,z,w = 0,0,0,1
  try:
    with open(filePath, 'r') as file:
      lines = file.readlines()
    if lines:
        last_line = lines[-1]
        data = last_line.strip().split(',')
        for item in data:
            key, value = item.split(':')
            key = key.strip()
            value = value.strip()
            if key == 'x':
              x = float(value)
              result += 1    
            elif key == 'y':
              y = float(value)
              result += 1 
            elif key == 'z':
              z = float(value)
              result += 1 
            elif key == 'w':
              w = float(value)
              result += 1 
            
  except:
    return (False, 0,0,0,1)

  return (result, x,y,z, w)

##--------------------------------------------
# WRITE POSE DATA TO FILE
#----------------------------------------------
def writePose2File(filePath = 'file_text_odom/odom_data.txt',x=0,y=0,z=0,w=1):
  with open(filePath, 'w') as file:
    file.write(f'x: {x}, y: {y}, z: {z}, w: {w}\n')
  time.sleep(0.1)
  return True

# ROBOT SPEAK
def RobotSpeakWithPath(pathFile):
  pygame.mixer.init()
  pygame.mixer.music.load(voice_path(pathFile))
  pygame.mixer.music.play()

# def xulyTocdo
# def xulyTocdoTinhtien(vx_in = 0):
#   temp = vx_in*RobConf.VX_MANUAL_MAX
#   if(temp > RobConf.VX_MANUAL_MAX):
#       temp = RobConf.VX_MANUAL_MAX
#   if(temp < -RobConf.VX_MANUAL_MAX):
#       temp = -RobConf.VX_MANUAL_MAX  
#   return (temp)
# def xulyTocdoXoay(vw_in = 0):
#   temp = vw_in*RobConf.VW_MANUAL_MAX
#   if(temp > RobConf.VW_MANUAL_MAX):
#       temp = RobConf.VW_MANUAL_MAX
#   if(temp < -RobConf.VW_MANUAL_MAX):
#       temp = -RobConf.VW_MANUAL_MAX  
#   return (temp)

# def xulyTocdo
def xulyTocdoTinhtien(vx_in = 0):
  vel_manual_vt_thang = doc_du_lieu_sql('manual_vt_thang')
  temp = vx_in*vel_manual_vt_thang #RobConf.VX_MANUAL_MAX
  if(temp > vel_manual_vt_thang):
      temp = vel_manual_vt_thang
  if(temp < - vel_manual_vt_thang):
      temp = - vel_manual_vt_thang  
  return (temp)

def xulyTocdoXoay(vw_in = 0):
  vel_manual_vt_xoay = doc_du_lieu_sql('manual_vt_xoay')
  temp = vw_in*vel_manual_vt_xoay
  if(temp > vel_manual_vt_xoay):
      temp = vel_manual_vt_xoay
  if(temp < - vel_manual_vt_xoay):
      temp = - vel_manual_vt_xoay
  return (temp)

#----- BU DUONG TRON---
def bu_duong_tron(saiso_yaw):
  temp1 = saiso_yaw
  while(temp1 > 180) or (temp1 < -180):
    if temp1 > 180:
        temp1 = temp1 - 360
    if temp1 <-180:
        temp1 = temp1 + 360    
  #print(f"saiso dau ra: {saiso}, saiso dau vao: {saiso_yaw}")
  return temp1

    #========================= ham nhan du lieu tu sql ======================
def nhandulieu_mysql(idban=1):
    # Load dữ liệu từ MySQL dựa trên IDban
    select_query = 'SELECT * FROM toado WHERE `IDban` = %s LIMIT 0, 1'  # Lấy chỉ một dòng dữ liệu
    mycursor = RobConf.db.cursor()
    mycursor.execute(select_query, (idban,))
    result = mycursor.fetchone()  # Sử dụng fetchone() để lấy chỉ một dòng dữ liệu
    # b = numpy.array(result)
    # print(b[0])
    # print(b[1])
    # print(b[2])
    # print(b[3])
    if result:
        x,y,z,w = result[0:4]
        idout = idban
        return  (x,y,z,w, idout)
    else:
        return None 
    

# =================== CODE THEM VO 20240923 ==========================================
def setupRobot_sql_delete():

    """Xóa tất cả dữ liệu trong bảng parameter_robot."""
    
    mycursor = RobConf.db.cursor()
    
    # Truy vấn để xóa tất cả dữ liệu
    delete_query = "DELETE FROM parameter_robot;"
    
    mycursor.execute(delete_query)
    RobConf.db.commit()
    
    print("Đã xóa tất cả dữ liệu trong bảng parameter_robot.")

def setupRobot_sql_save( aVx=0, aVw=0, mVx=0, mVw=0, ss_kc=0, ss_gx=0, led=0, user=0, pw=0, volt = 0, hC = 0, bC = 2, FL = 4, FR = 6):
    setupRobot_sql_delete()
    """Chèn tọa độ mới vào cơ sở dữ liệu mà không cần kiểm tra ID trùng lặp."""
    
    mycursor = RobConf.db.cursor()
    
    # Chuẩn bị dữ liệu để chèn
    data_to_insert = (aVx, aVw, mVx, mVw, ss_kc, ss_gx, led, user, pw, volt, hC, bC, FL, FR)  # Thay 'username' và 'password' theo yêu cầu
    insert_query = """
        INSERT INTO parameter_robot 
        (`auto_vt_thang`, `auto_vt_xoay`, `manual_vt_thang`, `manual_vt_xoay`, 
        `sai_so_kc`, `sai_so_goc`, `chieu_cao_led`, `username`, `password`, `volt_sac_auto`, `backC`, `headC`, `front_L`, `front_R`) 
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s);
    """
    
    mycursor.execute(insert_query, data_to_insert)
    RobConf.db.commit()
    
    print("Đã chèn tọa độ mới vào cơ sở dữ liệu.")

def load_ros_config_sql():
    """Đọc dữ liệu từ bảng ros_config"""
    mycursor = RobConf.db.cursor()
    
    # Truy vấn để lấy dữ liệu
    select_query = "SELECT * FROM ros_config;"
    mycursor.execute(select_query)
    
    # Lấy tất cả các bản ghi
    results = mycursor.fetchone()

    if results:
        output = {
            'default_map_name': results[0],
            'delay_time': results[1]
        }
        return output
    else:
        return None

def update_ros_config_sql(data):
    """Cập nhật dữ liệu vào bảng ros_config"""
    # import RobotConfig as RobConf
    mycursor = RobConf.db.cursor()
    
    update_query = f"update ros_config set "
    for key, value in data.items():
      update_query += f"{key}='{value}',"
    update_query = update_query[:-1] + ";"
    mycursor.execute(update_query)
    RobConf.db.commit()

def doc_du_lieu_sql(input):
    """Đọc dữ liệu từ bảng parameter_robot và trả về giá trị hoặc danh sách các bản ghi."""
    
    mycursor = RobConf.db.cursor()
    
    # Truy vấn để lấy dữ liệu
    select_query = "SELECT * FROM parameter_robot;"
    mycursor.execute(select_query)
    
    # Lấy tất cả các bản ghi
    results = mycursor.fetchall()

    # Mapping input keys to corresponding row indices and data types
    input_mapping = {
        'auto_vt_thang': (0, float),
        'auto_vt_xoay': (1, float),
        'manual_vt_thang': (2, float),
        'manual_vt_xoay': (3, float),
        'sai_so_kc': (4, float),
        'sai_so_goc': (5, float),
        'chieu_cao_led': (6, float),
        'username': (7, str),
        'password': (8, str),
        'volt_sac_auto': (9, str),
        'backC': (10, int),
        'headC': (11, int),
        'front_L': (12, int),
        'front_R': (13, int),
        'readall': (None, None)  # Special case for readall
    }

    for row in results:
        if input in input_mapping:
            index, data_type = input_mapping[input]
            if index is not None:
                return data_type(row[index])
            elif input == 'readall':
                return [
                    (float(row[0]), float(row[1]), float(row[2]),
                     float(row[3]), float(row[4]), float(row[5]),
                     float(row[6]), str(row[7]), str(row[8]))
                ]

    return None  # Trả về None nếu input không hợp lệ
  
def insert_navigation_goal(map_name, goal_name, x, y, theta):
    """
    Update a record in the table based on the ID.

    Parameters:
        connection (mysql.connector.connection_cext.CMySQLConnection): The database connection.
        id (int): The ID of the record to update.
        map_name (str): The new map name.
        goal_name (str): The new goal name.
        x (float): The new x coordinate.
        y (float): The new y coordinate.
        theta (float): The new theta value.
    """
    try:
        mycursor = RobConf.db.cursor()
        query = """
            INSERT INTO navigation_goals (map_name, goal_name, x, y, theta)
            VALUES (%s, %s, %s, %s, %s)
        """
        mycursor.execute(query, (map_name, goal_name, x, y, theta))
        RobConf.db.commit()
        print(f"Record with ID {id} updated successfully.")
    except Error as e:
        print(f"Error while updating record: {e}")
        
def get_all_by_map_name(map_name):
    """
    Retrieve all records from the table with the given map_name.

    Parameters:
        connection (mysql.connector.connection_cext.CMySQLConnection): The database connection.
        map_name (str): The map name to filter by.

    Returns:
        list: A list of tuples representing the records.
    """
    try:
        mycursor = RobConf.db.cursor()
        query = "SELECT * FROM navigation_goals WHERE map_name = %s"
        mycursor.execute(query, (map_name,))
        results = mycursor.fetchall()
        return results
    except Error as e:
        print(f"Error while retrieving records: {e}")
        return []
      
# insert_navigation_goal("map_name", "phòng khách", x=0.9, y=0.9, theta=0.9)
#print(get_all_by_map_name("map_name"))
# import os
# os.environ['SDL_AUDIODRIVER'] = 'pulseaudio'
# RobotSpeakWithPath('bandathoat.mp3')
# print(1)
# data_vitri = nhandulieu_mysql(RobConf.HOME_ID)
# x_goal, y_goal, z_goal, w_goal = map(lambda x: float(x.strip("'")), data_vitri)
# print(x_goal, y_goal, z_goal, w_goal)


## ================================ MY SQL CHECKIN =========================
def delete_rows_by_name1(name):
    """
    Delete all rows from the checkin_list table where the name matches the given name.
    """
    try:
        # Connect to the database
        cursor = RobConf.db.cursor()

        # SQL query to delete rows with the specified name
        query = "DELETE FROM checkin_list WHERE name = %s"
        cursor.execute(query, (name,))
        RobConf.db.commit()

        print(f"Deleted {cursor.rowcount} row(s) with name = '{name}'")

    except mysql.connector.Error as err:
        print(f"Error: {err}")

def load_full_table():
    """
    Load all records from the checkin_list table.
    """
    try:
        # Connect to the database
        cursor = RobConf.db.cursor(dictionary=True)  # Return rows as dictionaries

        # SQL query to fetch all records
        query = "SELECT * FROM checkin_list"
        cursor.execute(query)

        # Fetch all records
        results = cursor.fetchall()

        if results:
            print(f"Loaded {len(results)} records from the checkin_list table.")
        else:
            print("No records found in the checkin_list table.")

        return results

    except mysql.connector.Error as err:
        print(f"Error: {err}")
        return []

def insert_to_checkin_list(name, position=None, department=None, image=None):
    """
    Insert a new record into the checkin_list table. """

    try:
        # Connect to the database
        cursor = RobConf.db.cursor()

        # SQL query for inserting a new record
        query = """
        INSERT INTO checkin_list (name, position, department, image)
        VALUES (%s, %s, %s, %s)
        """
        # Execute the query with provided data
        cursor.execute(query, (name, position, department, image))
        RobConf.db.commit()

        print(f"Record inserted successfully with ID {cursor.lastrowid}")

    except mysql.connector.Error as err:
        print(f"Error: {err}")


def fetch_rows_by_name(name):
    """
    Lấy và in toàn bộ thông tin từ bảng `checkin_list` dựa trên giá trị `name`.
    """
    try:
        # Kết nối đến cơ sở dữ liệu
        cursor = RobConf.db.cursor(dictionary=True)  # Sử dụng dictionary=True để kết quả dạng dictionary

        # Truy vấn SQL để lấy thông tin theo name
        query = "SELECT * FROM checkin_list WHERE name = %s"
        cursor.execute(query, (name,))

        # Lấy tất cả các hàng khớp với `name`
        rows = cursor.fetchall()

        if rows:
            print(f"Found {len(rows)} record(s) with name = '{name}':")
            for row in rows:
                # Trích xuất từng giá trị
                extracted_name = row.get('name')
                position = row.get('position')
                department = row.get('department')
                soghe = row.get('image')
                print(f"Name: {extracted_name}, Position: {position}, Department: {department}")
            return extracted_name, position, department, soghe  # Trả về tất cả các bản ghi dạng danh sách
        else:
            print(f"No records found with name = '{name}'.")
            return None

    except mysql.connector.Error as err:
        print(f"Error: {err}")

def load_features_from_pkl(file_path):
    with open(file_path, 'rb') as file:
        data = pickle.load(file)
    return data

def remove_label_from_pkl(file_path, label_to_remove):
    # Tải dữ liệu
    all_features = load_features_from_pkl(file_path)
    
    # Kiểm tra nếu dữ liệu là dictionary
    if not isinstance(all_features, dict):
        print("Dữ liệu không phải dictionary, không thể xóa nhãn.")
        return
    
    # Kiểm tra nhãn có tồn tại trong dữ liệu không
    if label_to_remove in all_features:
        del all_features[label_to_remove]
        print(f"Đã xóa nhãn: {label_to_remove}")
    else:
        print(f"Nhãn '{label_to_remove}' không tồn tại trong dữ liệu.")
    
    # Lưu lại dữ liệu sau khi xóa
    save_features_to_pkl(file_path, all_features)


def delete_rows_by_name(file_path, name):
    delete_folder_in_multiple_paths(name)
    remove_label_from_pkl(file_path, name)
    """
    Delete all rows from the checkin_list table where the name matches the given name.
    """
    try:
        # Connect to the database
        cursor = RobConf.db.cursor()

        # SQL query to delete rows with the specified name
        query = "DELETE FROM checkin_list WHERE name = %s"
        cursor.execute(query, (name,))
        RobConf.db.commit()

        print(f"Deleted {cursor.rowcount} row(s) with name = '{name}'")

    except mysql.connector.Error as err:
        print(f"Error: {err}")


def fetch_rows_by_name(name):
    """
    Lấy và in toàn bộ thông tin từ bảng `checkin_list` dựa trên giá trị `name`.
    """
    try:
        # Kết nối đến cơ sở dữ liệu
        cursor = RobConf.db.cursor(dictionary=True)  # Sử dụng dictionary=True để kết quả dạng dictionary

        # Truy vấn SQL để lấy thông tin theo name
        query = "SELECT * FROM checkin_list WHERE name = %s"
        cursor.execute(query, (name,))

        # Lấy tất cả các hàng khớp với `name`
        rows = cursor.fetchall()

        if rows:
            print(f"Found {len(rows)} record(s) with name = '{name}':")
            for row in rows:
                # Trích xuất từng giá trị
                extracted_name = row.get('name')
                position = row.get('position')
                department = row.get('department')
                soghe = row.get('image')
                print(f"Name: {extracted_name}, Position: {position}, Department: {department}")
            return extracted_name, position, department, soghe  # Trả về tất cả các bản ghi dạng danh sách
        else:
            print(f"No records found with name = '{name}'.")
            return None

    except mysql.connector.Error as err:
        print(f"Error: {err}")

def delete_folder_in_multiple_paths(folder_name):
    """
    Xóa thư mục con trong cả 'data_server' và 'data_training/data_faces'.

    Args:
        folder_name (str): Tên thư mục con cần xóa.
    """
    # Danh sách các thư mục gốc cần kiểm tra
    base_dirs = ['data_server', 'data_training/data_faces']

    for base_dir in base_dirs:
        target_path = os.path.join(base_dir, folder_name)

        # Kiểm tra thư mục có tồn tại hay không
        if os.path.exists(target_path):
            try:
                # Xóa thư mục và toàn bộ nội dung
                shutil.rmtree(target_path)
                print(f"Đã xóa thành công thư mục: {target_path}")
            except Exception as e:
                print(f"Lỗi khi xóa thư mục {target_path}: {e}")
        else:
            print(f"Thư mục không tồn tại: {target_path}")


# Hàm lưu dữ liệu vào file pkl
def save_features_to_pkl(file_path, data):
    with open(file_path, 'wb') as file:
        pickle.dump(data, file)