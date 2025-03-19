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
  with open('txt/khuvuccho_data.txt', 'r') as file:
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
      