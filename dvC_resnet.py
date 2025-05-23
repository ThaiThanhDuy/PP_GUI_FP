import os
import json

import cv2
import torch
import torchvision
import torchvision.transforms as transforms
from torchvision import models
from PIL import Image
from sklearn.metrics.pairwise import cosine_similarity
import pyodbc

headCame = 2
ChargCame = 0
faceCameT = 4
faceCameN = 6
model = models.resnet50(weights=torchvision.models.resnet.ResNet50_Weights.DEFAULT)
model.eval()

# Bỏ lớp classification để chuyển model thành một mạng chỉ để trích xuất đặc trưng
model = torch.nn.Sequential(*list(model.children())[:-1])

preprocess = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

# Trích xuất đặc trưng của ảnh
def extract_features(image, model):
    image = preprocess(image)
    image = image.unsqueeze(0)
    with torch.no_grad():
        features = model(image)
    return features.squeeze().numpy()

def find_similar_images(target_image, image_features, model):
    """Find and compare the similarity of a target image with stored images."""
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

# Example of extracting features from images in a folder and saving to the database
folder_path = 'dvC_images'
image_features = Feature_Store(folder_path)

# Load a target image and find similar images
target_image_path = 'dvC_images/dvC_new_image.jpg'
target_image = Image.open(target_image_path).convert('RGB')
similar_images = find_similar_images(target_image, image_features, model)

print("Top similar images:")
for image, similarity in similar_images:
    print(f"Image: {image}, Similarity: {similarity}")
