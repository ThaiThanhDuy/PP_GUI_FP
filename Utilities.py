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

def RobotSpeakWithPath(pathFile):
  pygame.mixer.init()
  pygame.mixer.music.load(voice_path(pathFile))
  pygame.mixer.music.play()
  
def writePose2File(filePath = 'txt/odom_data.txt',x=0,y=0,z=0,w=1):
  with open(filePath, 'w') as file:
    file.write(f'x: {x}, y: {y}, z: {z}, w: {w}\n')
  time.sleep(0.1)
  return True

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