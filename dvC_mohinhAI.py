import os
import numpy as np
from PIL import Image
from sklearn.model_selection import train_test_split
import tensorflow as tf
from tensorflow.keras import layers, models

# Chuẩn bị dữ liệu
def load_data(data_dir):
    class_names = os.listdir(data_dir)
    images = []
    labels = []
    class_dict = {class_name: i for i, class_name in enumerate(class_names)}

    for class_name in class_names:
        class_dir = os.path.join(data_dir, class_name)
        for filename in os.listdir(class_dir):
            img_path = os.path.join(class_dir, filename)
            img = Image.open(img_path)
            img = img.resize((128, 128))
            img = np.array(img)
            if img.shape == (128, 128, 3):  # Đảm bảo ảnh có 3 kênh màu
                images.append(img)
                labels.append(class_dict[class_name])
    
    images = np.array(images)
    labels = np.array(labels)
    return images, labels, class_names

data_dir = './data'
images, labels, class_names = load_data(data_dir)

# Chia dữ liệu thành tập huấn luyện và tập kiểm tra
X_train, X_test, y_train, y_test = train_test_split(images, labels, test_size=0.2, random_state=42)

# Xây dựng mô hình CNN
def create_model(input_shape, num_classes):
    model = models.Sequential([
        layers.Conv2D(32, (3, 3), activation='relu', input_shape=input_shape),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.Flatten(),
        layers.Dense(64, activation='relu'),
        layers.Dense(num_classes, activation='softmax')
    ])
    model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])
    return model

input_shape = (128, 128, 3)
num_classes = len(class_names)
model = create_model(input_shape, num_classes)

# Huấn luyện mô hình
history = model.fit(X_train, y_train, epochs=10, validation_data=(X_test, y_test))

# Lưu mô hình
model.save('image_classification_model.h5')
np.save('class_names.npy', class_names)
