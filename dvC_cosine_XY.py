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

headCame = 2
ChargCame = 0
faceCameT = 4
faceCameN = 6

# Load the ResNet50 model pre-trained on ImageNet
model = models.resnet50(weights=torchvision.models.resnet.ResNet50_Weights.DEFAULT)
model.eval()

# Modify the model to remove the classification layer
model = torch.nn.Sequential(*list(model.children())[:-1])

preprocess = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

def extract_features(image, model):
    image = preprocess(image)
    image = image.unsqueeze(0)
    with torch.no_grad():
        features = model(image)
    return features.squeeze().numpy()

def find_similar_images(target_image, image_features, model):
    target_features = extract_features(target_image, model)
    similarities = {}
    for filename, features in image_features.items():
        similarity = cosine_similarity([target_features], [features])[0][0]
        similarities[filename] = similarity
    sorted_similarities = sorted(similarities.items(), key=lambda item: item[1], reverse=True)
    return sorted_similarities

def Feature_Store(folder_path):
    image_features = {}
    for filename in os.listdir(folder_path):
        if filename.endswith('.jpg') or filename.endswith('.png'):
            image_path = os.path.join(folder_path, filename)
            image = Image.open(image_path).convert('RGB')
            features = extract_features(image, model)
            image_features[filename] = features
    return image_features

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

def capture_image(ip_camera_url, save_path, image_name):
    cap = cv2.VideoCapture(ip_camera_url)
    ret, frame = cap.read()
    if ret:
        image_path = os.path.join(save_path, image_name)
        cv2.imwrite(image_path, frame)
        print(f"Image saved at {image_path}")  # Debug print
    else:
        print("Failed to capture image.")  # Debug print
    cap.release()
    return ret

def main():
    db_path = './dvC_image_database_XY.db'
    image_dir = './dvC_images_XY'
    ip_camera_url = headCame

    # Create the ImageDatabase instance
    image_db = ImageDatabase(db_path)

    # Capture a new image
    new_image_name = 'dvC_new_image.jpg'
    if capture_image(ip_camera_url, image_dir, new_image_name):
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
            stored_image_features = extract_features(Image.open(image_path).convert('RGB'), model)
            similarity = cosine_similarity([stored_image_features], [extract_features(target_image, model)])[0][0]

            if similarity > best_similarity:
                best_similarity = similarity
                best_match = file_name
                best_coords = (x, y, z)

        if best_match:
            print(f"Best match is {best_match} with similarity {best_similarity:.6f}")
            print(f"Coordinates: ({best_coords[0]}, {best_coords[1]}, {best_coords[2]})")
        else:
            print("No matching images found.")
    else:
        print("Failed to capture image.")

    # Close the database connection
    image_db.close()


