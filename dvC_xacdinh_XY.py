import cv2
import numpy as np
import os
import json
import torch
import torchvision.models as models
import torchvision.transforms as transforms
from PIL import Image
from sklearn.metrics.pairwise import cosine_similarity

def split_image(image_path, output_dir, tile_size=320):
    # Đọc ảnh bản đồ
    image = cv2.imread(image_path)

    # Kiểm tra kích thước ảnh
    height, width, _ = image.shape
    print(f'Original image size: {width}x{height}')

    # Danh sách lưu tọa độ các ảnh
    coords_list = []

    # Chia ảnh thành các ô vuông và lưu ảnh
    image_count = 0
    for y in range(0, height, tile_size):
        for x in range(0, width, tile_size):
            # Trích xuất ảnh con
            sub_image = image[y:y + tile_size, x:x + tile_size]

            # Kiểm tra kích thước của sub_image
            actual_height, actual_width = sub_image.shape[:2]
            if actual_height != tile_size or actual_width != tile_size:
                continue  # Bỏ qua ảnh không đủ kích thước

            # Lưu ảnh con với tên file là tọa độ
            image_count += 1
            output_path = os.path.join(output_dir, f'{x}_{y}.jpg')
            cv2.imwrite(output_path, sub_image)

            # Lưu tọa độ của ảnh
            coords_list.append({
                'image_path': output_path,
                'coordinates': (x, y)
            })

    # Lưu tọa độ vào file JSON
    with open('image_coords.json', 'w') as f:
        json.dump(coords_list, f)

    print(f'Saved {image_count} images to {output_dir}')
    return coords_list

def extract_features(image, model):
    preprocess = transforms.Compose([
        transforms.Resize(256),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    image = preprocess(image)
    image = image.unsqueeze(0)
    with torch.no_grad():
        features = model(image)
    return features.squeeze().numpy()

def capture_image(save_path, image_name):
    cap = cv2.VideoCapture(2)
    ret, frame = cap.read()
    if ret:
        image_path = os.path.join(save_path, image_name)
        cv2.imwrite(image_path, frame)
        print(f"Image saved at {image_path}")  # Debug print
    else:
        print("Failed to capture image.")  # Debug print
    cap.release()
    return ret

def main_run():
    image_path = 'dvC_map.jpg'
    output_dir = 'dvC_image'
    image_dir = 'dvC_image_Z'
    tile_size = 320

    # Tạo thư mục lưu ảnh nếu chưa tồn tại
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Kiểm tra nếu thư mục đã có đủ 12 ảnh con
    if len(os.listdir(output_dir)) == 12:
        print("The directory already contains 12 images. Skipping the splitting process.")
        with open('image_coords.json', 'r') as f:
            coords_list = json.load(f)
    else:
        # Chia ảnh thành các ô vuông và lưu lại
        coords_list = split_image(image_path, output_dir, tile_size)

    # Khởi tạo mô hình ResNet-50
    model = models.resnet50(weights=models.ResNet50_Weights.DEFAULT)
    model.eval()

    # Bỏ lớp classification để chuyển model thành một mạng chỉ để trích xuất đặc trưng
    model = torch.nn.Sequential(*list(model.children())[:-1])

    # Trích xuất đặc trưng của từng ảnh đã lưu và lưu lại đặc trưng kèm với tọa độ
    feature_dict = {}
    for item in coords_list:
        image_path = item['image_path']
        coordinates = tuple(item['coordinates'])  # Convert to tuple
        image = Image.open(image_path)
        features = extract_features(image, model)
        feature_dict[coordinates] = features

    # Lưu các đặc trưng vào file
    np.save('feature_dict.npy', feature_dict)

     # Capture a new image
    new_image_name = 'dvC_new_image.jpg'
    if capture_image(image_dir, new_image_name):
        print(f"Captured and saved {new_image_name}")

        # Construct the full path for the new image
        new_image_path = os.path.join(image_dir, new_image_name)
        try:
            input_image = Image.open(new_image_path).convert('RGB')
        except FileNotFoundError:
            print(f"File not found: {new_image_path}")
            return

    # Đọc ảnh đầu vào
    # input_image_path = 'input.jpg'
    # input_image = Image.open(input_image_path)

    # Trích xuất đặc trưng của ảnh đầu vào
    input_features = extract_features(input_image, model)

    # Tính độ tương đồng cosine
    similarities = {}
    for coord, features in feature_dict.items():
        similarity = cosine_similarity([input_features], [features])[0][0]
        similarities[coord] = similarity

    # Sắp xếp độ tương đồng
    sorted_similarities = sorted(similarities.items(), key=lambda item: item[1], reverse=True)

    # Lấy tọa độ có độ tương đồng cao nhất
    best_coord = sorted_similarities[0][0]
    best_image_path = None

    # Tìm ảnh có tọa độ tương đồng cao nhất
    for item in coords_list:
        if tuple(item['coordinates']) == best_coord:
            best_image_path = item['image_path']
            break

    print(f'Tọa độ của ảnh đầu vào: {best_coord}')
    print(f'Tên ảnh có độ tương đồng cao nhất: {best_image_path}')

    # Đọc ảnh gốc và khoanh tròn khu vực tương đồng nhất
    image = cv2.imread(image_path)
    cv2.rectangle(image, (best_coord[0], best_coord[1]), (best_coord[0] + tile_size, best_coord[1] + tile_size), (0, 255, 0), 3)
    cv2.imwrite('annotated_map.jpg', image)

    print(f'Annotated map saved as annotated_map.jpg')
    return best_coord


