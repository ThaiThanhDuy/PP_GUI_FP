import sqlite3
import cv2
import os
import time
import RobotConfig as robotconf
import Utilities as Uti
import cv2
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
from multiprocessing import Pool
import pickle
import random
from sklearn.neighbors import KDTree

class SetupDinhVi():
    def __init__(self):
        self.db_path = './dvC_image_database_XY.db'
        self.image_dir = './dvC_images_XY'
        self.image_dir_test = Uti.image_path('dvC_images_test')
        
       
        self.toado_x = None
        self.toado_y = None
        self.toado_z = None
        self.id_vitri = None
        self.id_delete = None
        self.conn = sqlite3.connect(self.db_path)
      
        #self.model = models.resnet50(weights=torchvision.models.resnet.ResNet50_Weights.DEFAULT)
        #self.model.eval()

        self.model = None

        
        #self.model = torch.nn.Sequential(*list(self.model.children())[:-1])


      #  self.preprocess = transforms.Compose([
    #         transforms.Resize(256),
     #        transforms.CenterCrop(224),
      #       transforms.ToTensor(),
        #     transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
       #  ])

        # Trích xuất đặc trưng cho tất cả các ảnh đã lưu
        #self.extract_features_for_all_images()
        
        # Load all image features and metadata during initialization
        self.stored_data = []
        # self.array_storted_data()
        # self._build_kd_tree()
        print("[array_storted_data] Da load toan bo du lieu len")
        #self.create_table()

    def _build_kd_tree(self):
        # Prepare feature data and IDs
        features_list = [features for features, x, y, z, id in self.stored_data]
        self.features_array = np.array(features_list)
        self.ids = np.array([id for features, x, y, z, id in self.stored_data])

        # Create and build KDTree
        self.tree = KDTree(self.features_array)

    def create_table(self):
        # Xóa bảng nếu đã tồn tại để đảm bảo cấu trúc đúng
        self.conn.execute("DROP TABLE IF EXISTS images")

        # Tạo bảng với cột x, y, z, id
        query = """
        CREATE TABLE IF NOT EXISTS images (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            file_name TEXT,
            x REAL,
            y REAL,
            z REAL,
            image_id INTEGER
        )
        """
        self.conn.execute(query)
        self.conn.commit()

    
    def extract_features(self, image, model):
        image = self.preprocess(image)
        image = image.unsqueeze(0)
        with torch.no_grad():
            features = model(image)
        #print("Dac trung anh moi vua chup:", features.squeeze().numpy())
    
        return features.squeeze().numpy()


 

    def load_images(self):
        query = "SELECT file_name, x, y, z, image_id FROM images"
        cursor = self.conn.execute(query)
        data = cursor.fetchall()
        return data
    

    def update_data(self, file_name, x, y, z, image_id):
        # Kiểm tra xem bản ghi đã tồn tại chưa
        query_check = "SELECT COUNT(*) FROM images WHERE file_name = ?"
        cursor = self.conn.execute(query_check, (file_name,))
        count = cursor.fetchone()[0]
        
        if count > 0:
            # Cập nhật dữ liệu nếu bản ghi đã tồn tại
            query_update = "UPDATE images SET x = ?, y = ?, z = ?, image_id = ? WHERE file_name = ?"
            self.conn.execute(query_update, (x, y, z, image_id, file_name))
        else:
            # Thêm dữ liệu nếu bản ghi chưa tồn tại
            query_insert = "INSERT INTO images (file_name, x, y, z, image_id) VALUES (?, ?, ?, ?, ?)"
            self.conn.execute(query_insert, (file_name, x, y, z, image_id))
        
        self.conn.commit()


    
    def capture_imag(self, save_path, image_name, id_camera, cap):
        #cap = cv2.VideoCapture(id_camera)
        print("[capture_image] DA VO DE CHUP ANH")
        #while True:

        # Đọc khung hình từ camera
        ret, frame = cap.read()
        
        frame = cv2.flip(frame, 0)

        # Thay đổi kích thước khung hình
        if ret:
            image_path = os.path.join(save_path, image_name)
            cv2.imwrite(image_path, frame)
           
            #count = count + 1
            # if count == 25:
            #     break

        cap.release()
        return ret
    


    def capture_image(self, save_path, image_name, id_camera, cap, threshold=10):
        print("[capture_image] Đã vào để chụp ảnh")
        
        while True:
            # Đọc khung hình từ camera
            ret, frame = cap.read()
            frame = cv2.flip(frame, 0)

            if not ret:
                print("[capture_image] Không thể đọc khung hình từ camera.")
                cap.release()
                return False

            # Tính độ sáng trung bình của khung hình (tính trên kênh độ xám)
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            brightness = np.mean(gray_frame)
            
            print(f"[capture_image] Độ sáng khung hình: {brightness}")

            # Kiểm tra nếu độ sáng nhỏ hơn ngưỡng (threshold), tiếp tục chụp lại
            if brightness > threshold:
                # Thay đổi kích thước khung hình nếu cần thiết
                image_path = os.path.join(save_path, image_name)
                cv2.imwrite(image_path, frame)
                print(f"[capture_image] Đã lưu ảnh tại {image_path}")
                break
            else:
                print("[capture_image] Khung hình quá tối, chụp lại...")

        cap.release()
        print("[capture_image] dang dong camera")
        return ret



    def capture_and_extract_features(self, capC, capF):
        self.timestamp = int(time.time())

        # Capture and extract features for the ceiling image
        image_name_ceiling = '{}_ceiling.jpg'.format(self.timestamp)
        self.capture_image(self.image_dir, image_name_ceiling, robotconf.headCamera, capC)
        image_ceiling = Image.open(os.path.join(self.image_dir, image_name_ceiling)).convert('RGB')
        features_ceiling = self.extract_features(image_ceiling, self.model)

        # Capture and extract features for the front image
        image_name_front = '{}_front.jpg'.format( self.timestamp)
        self.capture_image(self.image_dir, image_name_front, robotconf.faceCame1,capF)
        image_front = Image.open(os.path.join(self.image_dir, image_name_front)).convert('RGB')
        features_front = self.extract_features(image_front, self.model)

        # Combine the features
        combined_features = np.concatenate((features_ceiling, features_front))
        return combined_features
    
    def capture_and_extract_features_new1(self):
    
        # Capture and extract features for the ceiling image
        image_name_ceiling = 'ceiling_test.jpg'
        self.capture_image(self.image_dir_test, image_name_ceiling, robotconf.headCamera)
        image_ceiling = Image.open(os.path.join(self.image_dir_test, image_name_ceiling)).convert('RGB')
        
        # Capture and extract features for the front image
        image_name_front = 'front_test.jpg'
        self.capture_image(self.image_dir_test, image_name_front, robotconf.faceCame1)
        image_front = Image.open(os.path.join(self.image_dir_test, image_name_front)).convert('RGB')


        features_front = self.extract_features(image_front, self.model)
        features_ceiling = self.extract_features(image_ceiling, self.model)
        # Combine the features
        combined_features = np.concatenate((features_ceiling, features_front))
        return combined_features
    
    def capture_and_extract_features_new(self, capC, capF):
        # Define image paths
        image_paths = {
            'ceiling': 'ceiling_test.jpg',
            'front': 'front_test.jpg'
        }
    
        # Capture images (assuming this saves them to disk)
        self.capture_image(self.image_dir_test, image_paths['ceiling'], robotconf.headCamera, capC)
        self.capture_image(self.image_dir_test, image_paths['front'], robotconf.faceCame1, capF)
        
        # Extract features from captured images
        features = {}
        for key in image_paths:
            image_path = os.path.join(self.image_dir_test, image_paths[key])
            with Image.open(image_path).convert('RGB') as img:
                features[key] = self.extract_features(img, self.model)
        
        # Combine features
        combined_features = np.concatenate((features['ceiling'], features['front']))
        
        return combined_features
    

    def add_data(self, x, y, z, image_id, capC, capF):
        combined_features = self.capture_and_extract_features(capC, capF)
        # Save combined features to a file
        features_path = os.path.join(self.image_dir, f'{self.timestamp}.pkl')
        with open(features_path, 'wb') as f:
            pickle.dump(combined_features, f)

        self.update_data(features_path, x, y, z, image_id)

    
    def test_image1(self, target_id, yawdeg):
        # Extract features for the target image
        target_features = self.capture_and_extract_features_new()

        # Query the KD-Tree
        dist, ind = self.tree.query([target_features], k=1)  # k is the number of nearest neighbors to find

        # Extract results
        results = []
        for i, idx in enumerate(ind[0]):
            stored_features, x, y, z, id = self.stored_data[idx]
            similarity = 1 - dist[0][i]  # cosine similarity is related to distance
            if id == target_id:
                results.append(((x, y, z, id), similarity))

        # Sort and return the top result based on similarity
        results.sort(key=lambda item: item[1], reverse=True)
        top_results = results[:1]

        return top_results


    def test_image(self, target_id, yawdeg, capC, capF): #test_image
        # Extract features for the target image
        target_features = self.capture_and_extract_features_new(capC, capF)
        similarity_results = []
        
        # Iterate over the preloaded data and calculate similarity
        for stored_features, x, y, z, id in self.stored_data:
            if id == target_id:
                #if (z < yawdeg + 15) and (z > yawdeg - 15):
                similarity = cosine_similarity([stored_features], [target_features])[0][0]
                similarity_results.append(((x, y, z, id), similarity))
    

        # Sort and return the top 2 results based on similarity
        similarity_results.sort(key=lambda item: item[1], reverse=True)
        top_results = similarity_results[:1]
        print("topbbbbbbb",top_results)

        return top_results
    
    def test_image2(self, target_id, yawdeg):
        # Trích xuất các đặc trưng cho hình ảnh mục tiêu
        target_features = self.capture_and_extract_features_new()
        
        # Ngưỡng tương đồng
        similarity_threshold = 0.7
        
        # Tính độ tương đồng và kiểm tra ngưỡng
        for stored_features, x, y, z, id in self.stored_data:
            if id == target_id:
                similarity = cosine_similarity([stored_features], [target_features])[0][0]
                
                # Nếu độ tương đồng đạt trên ngưỡng, trả về kết quả ngay lập tức
                if similarity >= similarity_threshold:
                    return [((x, y, z, id), similarity)]
        
        # Nếu không có kết quả nào đạt ngưỡng, sắp xếp và trả về kết quả tốt nhất
        similarity_results = [
            ((x, y, z, id), cosine_similarity([stored_features], [target_features])[0][0])
            for stored_features, x, y, z, id in self.stored_data
            if id != target_id
        ]
        
        similarity_results.sort(key=lambda item: item[1], reverse=True)
        top_results = similarity_results[:1]

        return top_results

    def array_storted_data(self):
        
        for entry in self.load_images():
            file_name, x, y, z, id = entry
            
            # Kiểm tra xem file_name có chứa sẵn đường dẫn chưa
            if not file_name.startswith(self.image_dir):
                features_path = os.path.join(self.image_dir, file_name)
            else:
                features_path = file_name
            
            print("[array_storted_data] features_path trong test_image:", features_path)
            
            # Mở và load dữ liệu từ file
            with open(features_path, 'rb') as f:
                stored_features = pickle.load(f)
            
            # Thêm dữ liệu đã load vào danh sách
            self.stored_data.append((stored_features, x, y, z, id))
        
    
    def check_postion(self, target_id, yawdeg): # check_postion
        target_features = self.capture_and_extract_features_new()
        similarity_results = []

        for entry in self.load_images():
            file_name, x, y, z, id = entry
            print("features_path trong test_image",file_name)
            if id == target_id :
                #if z < (yawdeg + 15) and z > (yawdeg - 15):
                features_path = os.path.join(self.image_dir,file_name)
                print("features_path trong test_image",features_path)
                with open(file_name, 'rb') as f:
                    stored_features = pickle.load(f)

                similarity = cosine_similarity([stored_features], [target_features])[0][0]
                similarity_results.append(((x, y, z, id), similarity))
              

        similarity_results.sort(key=lambda item: item[1], reverse=True)
        top_results = similarity_results[:5]

        for idx, (coords, similarity) in enumerate(top_results):
            print(f"Rank {idx + 1}:")
            print(f"  Similarity: {similarity:.6f}")
            print(f"  Coordinates: ({coords[0]}, {coords[1]}, {coords[2]}, {coords[3]})")
            print()
        return top_results
    

    def delete_image(self, file_name):
        # Delete the image file from the directory
        self.stored_data = []
        image_path = os.path.join(self.image_dir, file_name)
        if os.path.exists(image_path):
            os.remove(image_path)
            print(f"Deleted file {image_path}")
        file_name_with_extension = os.path.basename(file_name)
        # Tách phần mở rộng khỏi tên file
        file_name_delete_folder, _ = os.path.splitext(file_name_with_extension)
        print("tu khoa de xoa file ", file_name_delete_folder)
        self.delete_specific_files(file_name_delete_folder)

        # Delete the record from the database
        query = "DELETE FROM images WHERE file_name = ?"
        self.conn.execute(query, (file_name,))
        self.conn.commit()

    def delete_image_by_index(self, index):
        images = self.load_images()
        if 0 <= index < len(images):
            file_name = images[index][0]
            self.delete_image(file_name)
          
        else:
            print("Invalid index.")

    def delete_data(self, id_delete):
        id_delete = id_delete - 1
        self.delete_image_by_index(id_delete)


    def close(self):
        self.conn.close()

    def delete_specific_files(self, keyword):
        # Duyệt qua tất cả các file trong thư mục
        for file_name in os.listdir(self.image_dir):
            # Nếu tên file chứa từ khóa cần xóa
            if keyword in file_name:
                file_path = os.path.join(self.image_dir, file_name)
                if os.path.exists(file_path):
                    os.remove(file_path)
                    print(f"Deleted file: {file_path}")
                else:
                    print(f"File not found: {file_path}")


