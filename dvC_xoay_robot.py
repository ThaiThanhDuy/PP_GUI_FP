#-------------------------------------- QUÁ TRÌNH SETUP ROBOT ---------------------------------------------
# B1. ĐĂNG NHẬP CHẾ ĐỘ ADMIN MỚI CHO SETUP HOME, CHỈ KHI ĐĂNG NHẬP ADMIN THÌ MỚI HIỆN CÁC NÚT NHẤN Ở BƯỚC 2.
# B2: TRONG TRANG ĐIỀU KHIỂN MANUAL THIẾT KẾ CÁC YÊU CẦU SAU
#   1. 2 NÚT NHẤN CHO PHÉP MOVE ROBOT ĐI TỚI VÀ ĐI LUI
#   2. KHI NHẤN ĐI TỚI HOẶC ĐI LUI THÌ ROBOT CHỈ ĐI N/4 XUNG SAU ĐÓ DỪNG LẠI
#   3. Sau khi robot đi xong thì chương trình tự động chụp ảnh 
#   4. QPlainText cho phép cập nhật tọa độ (x,y,z) thủ công. 
#   5. 1 Nút nhấn cho phép save ảnh kèm với tọa độ đã có từ QPlainText
''' Robot cũng có thể tự tính và cập nhật tọa độ, khác cách code 1 xíu
Là thay vì chỗ QPlainText mình nhập thủ công tọa độ, thì nhập vào số 1 2 3 4, tương ứng với 4 khu vực (x+, y+), (x+, y-), (x-, y+), (x-, y-)
Thì robot sẽ hiểu đang ở chỗ nào và từ đó có thể tự tính được tọa độ x, y.
Còn khi setup robot di chuyển trên trục x hoặc y thì nhập 0 để robot tự +- x hoặc y '''
#-------------------------------------- QUA TRÌNH ROBOT THỰC THI ----------------------------------------------------------
# B1: CHO ROBOT DI CHUYEN VE TOI HOME NẰM TRONG VÙNG KHÔNG GIAN CÓ BÁN KÍNH 1 MÉT
# B2: CHO ROBOT DỪNG, BẬC CHẾ ĐỘ ĐỊNH VỊ CAMERA
# B3: ĐỊNH VỊ LẠI GÓC YAW CHO ROBOT
#   1. Ban đầu phải setup các góc yaw bằng camera trước. 
#   2. Khi robot về tới home thì chụp ảnh từ camera phía trước
#   3. Tính độ tương đồng giữa các ảnh đã save trước đó. Để xem robot đang có hướng như nào
#   4. Xoay robot với 1 góc tùy ý muốn với công thức xung encoder = (L*theta*N) / (4*pi*R) 
#       4.1 L kc 2 bánh xe, R là bán kính bánh xe, N xung trên vòng (đã nhân tỉ số truyền)
#   5. Sau khi xoay robot đúng hướng ban đầu sang bước B4
# B4: CHỤP ẢNH TỪ HEADCAME VÀ TÍNH ĐỘ TƯƠNG ĐỒNG XEM ROBOT ĐANG Ở TỌA ĐỘ (X,Y) NHƯ NÀO.
#   1. Robot sẽ rơi vào 2 trường  hợp đặc biệt và 4 trường hợp bình thường.
#       1.1 Hai trường hợp đặc biệt sẽ là robot nằm trên trục x và y
#       1.2 Bốn trường hợp  khác là tại (x+, y+), (x+, y-), (x-, y+), (x-, y-)
#   2. Sau khi xác định được khu vực của robot đang đứng sang bước 5
# B5: LÚC NÀY CHIA ROBOT THÀNH 2 TRƯỜNG HỢP (góc yaw đã đúng)
#   1. Robot đang phía dương trục x thì cho robot đi lui cứ đi được N/4 xung encoder thì dừng và chụp ảnh kiểm tra, đến khi đạt ngưỡng cho phép 
#   2. Sau khi thõa được trục x thì xét xem robot đang bên dương hay âm trục y
#   3. Nếu phái dương thì cho robot xoay 90 độ sang phải và di chuyển robot cứ N/4 xung thì kiểm tra 1 lần
#--------------------------------------


# ================================================ CAC CAU LENH GIUP ROBOT DI CHUYEN ====================================================

import rospy
from geometry_msgs.msg import Twist
import subprocess
import time
import math
import cv2
import os
import numpy as np
import sqlite3
from sklearn.metrics.pairwise import cosine_similarity
from PIL import Image
import os
import json
import torch
import torchvision
import torchvision.transforms as transforms
from torchvision import models
import dvC_xacdinh_XY as xd_xy

class ImageDatabase:
    def __init__(self, db_path):
        self.db_path = db_path
        self.conn = sqlite3.connect(self.db_path)

    def load_images(self):
        query = "SELECT file_name, x, y, z FROM images"
        cursor = self.conn.execute(query)
        data = cursor.fetchall()
        return data

    def close(self):
        self.conn.close()

class RobotController:
    def __init__(self, port='/dev/ttyUSB2'):
        self.port = port
        self.pub = None
        self.headCame = 2
        self.ChargCame = 0
        self.faceCameT = 4
        self.faceCameN = 6

        # Nguong cho phep
        self.threshold = 0.9
        self.check_threshold = False

        # Load the ResNet50 model pre-trained on ImageNet
        self.model = models.resnet50(weights=torchvision.models.resnet.ResNet50_Weights.DEFAULT)
        self.model.eval()

        # Modify the model to remove the classification layer
        self.model = torch.nn.Sequential(*list(self.model.children())[:-1])

        self.preprocess = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])  

        self.start_roscore()
        self.start_rosserial()
        self.init_node()
        

    def start_roscore(self):
        # Start roscore in the background
        subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(4)  # Wait for roscore to fully start

    def start_rosserial(self):
        # Start rosserial in the background
        subprocess.Popen(['rosrun', 'rosserial_python', 'serial_node.py', self.port], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(4)  # Wait for rosserial to fully start

    def init_node(self):
        rospy.init_node('cmd_vel_publisher', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # Tần số publish là 10 Hz

    def up(self, linear_speed):
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = 0
        self.pub.publish(twist_msg)

    def down(self, linear_speed):
        twist_msg = Twist()
        twist_msg.linear.x = -linear_speed
        twist_msg.angular.z = 0
        self.pub.publish(twist_msg)

    def left(self, angular_speed):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = angular_speed
        self.pub.publish(twist_msg)

    def right(self, angular_speed):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = -angular_speed
        self.pub.publish(twist_msg)

    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.pub.publish(twist_msg)
     #   self.check_threshold = False

    def rotate_robot(self, angular_speed=0.7):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = -angular_speed
        self.pub.publish(twist_msg)
        time_to_rotate = (math.pi / 2 / angular_speed)  + 7  # Time to rotate 90 degrees
        #print(time_to_rotate)
        time.sleep(4)
        self.stop()

    def rotate_robot_update(self, initial_angular_speed=0.65, min_angular_speed=0.2, rotation_time=0.1, reduction_rate=0.1):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        angular_speed = initial_angular_speed
        

        while not self.check_threshold:
            twist_msg.angular.z = -angular_speed
            self.pub.publish(twist_msg)
            self.process_image()
            time.sleep(rotation_time)

            # Reduce angular speed
            angular_speed = max(min_angular_speed, angular_speed - reduction_rate)
            if angular_speed <= min_angular_speed:
                angular_speed = min_angular_speed
            print(f"Current angular speed: {angular_speed:.2f}")

        self.stop()


    def extract_features(self, image, model):
        image = self.preprocess(image)
        image = image.unsqueeze(0)
        with torch.no_grad():
            features = model(image)
        return features.squeeze().numpy()

    def find_similar_images(self, target_image, image_features, model):
        target_features = self.extract_features(target_image, model)
        similarities = {}
        for filename, features in image_features.items():
            similarity = cosine_similarity([target_features], [features])[0][0]
            similarities[filename] = similarity
        sorted_similarities = sorted(similarities.items(), key=lambda item: item[1], reverse=True)
        return sorted_similarities

    def Feature_Store(self, folder_path):
        image_features = {}
        for filename in os.listdir(folder_path):
            if filename.endswith('.jpg') or filename.endswith('.png'):
                image_path = os.path.join(folder_path, filename)
                image = Image.open(image_path).convert('RGB')
                features = self.extract_features(image, self.model)
                image_features[filename] = features
        return image_features
    
    def capture_image(self, save_path, image_name):
        cap = cv2.VideoCapture(self.faceCameT)
        ret, frame = cap.read()
        if ret:
            image_path = os.path.join(save_path, image_name)
            cv2.imwrite(image_path, frame)
            print(f"Image saved at {image_path}")  # Debug print
        else:
            print("Failed to capture image.")  # Debug print
        cap.release()
        return ret


    def run(self):
  
        while not rospy.is_shutdown():
            print("OK CHAY")
            self.rate.sleep()
            self.rotate_robot_update()
            if self.check_threshold:
                print("========================== OK DA DUNG HUONG CUA ROBOT ROI NHA =========================")
                return

    def process_image(self):
        db_path = './dvC_image_database_Z.db'
        image_dir = './dvC_images_Z'
        
        # Create the ImageDatabase instance
        image_db = ImageDatabase(db_path)

        # Capture a new image
        new_image_name = 'dvC_new_image.jpg'
        if self.capture_image(image_dir, new_image_name):
            print(f"Captured and saved {new_image_name}")

            # Construct the full path for the new image
            new_image_path = os.path.join(image_dir, new_image_name)
            try:
                target_image = Image.open(new_image_path).convert('RGB')
            except FileNotFoundError:
                print(f"File not found: {new_image_path}")
                return

            # Find the most similar image
            best_match = None
            best_similarity = -1
            best_coords = None

            for entry in image_db.load_images():
                file_name, x, y, z = entry
                image_path = os.path.join(image_dir, file_name)
                stored_image_features = self.extract_features(Image.open(image_path).convert('RGB'), self.model)
                similarity = cosine_similarity([stored_image_features], [self.extract_features(target_image, self.model)])[0][0]

                if similarity > best_similarity:
                    best_similarity = similarity
                    best_match = file_name
                    best_coords = (x, y, z)

            if best_match:
                if best_similarity > self.threshold:
                    self.check_threshold = True
                print(f"Best match is {best_match} with similarity {best_similarity:.6f}")
                print(f"Coordinates: ({best_coords[0]}, {best_coords[1]}, {best_coords[2]})")
            else:
                print("No matching images found.")
        else:
            print("Failed to capture image.")

        # Close the database connection
        image_db.close()

    def move_xy(self):
        toa_do = xd_xy.main_run()
        if toa_do == (0,0):
            self.stop()
        else:
            if toa_do != (0,0):
                self.run()
                self.left(0.5)
                time.sleep(10)
                self.stop()
                self.up(0.5)
                time.sleep(6)
                self.stop()
                self.check_threshold = False
                self.run()



if __name__ == '__main__':
    try:
        robot_controller = RobotController()
        robot_controller.move_xy()
    except rospy.ROSInterruptException:
        pass
