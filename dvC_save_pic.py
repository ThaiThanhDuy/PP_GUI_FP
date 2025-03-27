import cv2
import os

def capture_and_save_image(folder_path, filename):
    # Kiểm tra và tạo thư mục nếu không tồn tại
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    
    # Đường dẫn đầy đủ đến tệp ảnh
    filepath = os.path.join(folder_path, filename)
    
    # Mở kết nối với camera (0 là camera mặc định)
    cap = cv2.VideoCapture(2)
    
    if not cap.isOpened():
        print("Không thể mở camera.")
        return
    
    print("Camera đã sẵn sàng. Nhấn 's' để lưu ảnh và 'q' để thoát.")
    
    while True:
        # Đọc khung hình từ camera
        ret, frame = cap.read()
        
        if not ret:
            print("Không thể lấy khung hình.")
            break
        
        # Hiển thị khung hình
        cv2.imshow('Camera', frame)
        
        # Chờ phím nhấn
        key = cv2.waitKey(1) & 0xFF
        
        # Nếu nhấn 's', lưu ảnh
        if key == ord('s'):
            cv2.imwrite(filepath, frame)
            print(f"Ảnh đã được lưu vào {filepath}")
            break
        # Nếu nhấn 'q', thoát
        elif key == ord('q'):
            print("Thoát mà không lưu ảnh.")
            break
    
    # Giải phóng tài nguyên
    cap.release()
    cv2.destroyAllWindows()

# Đường dẫn tới thư mục images và tên file ảnh
folder_path = 'images'
filename = 'anh_test.jpg'

# Gọi hàm chụp và lưu ảnh
capture_and_save_image(folder_path, filename)
