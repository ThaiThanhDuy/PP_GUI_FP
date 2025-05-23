import sys
import cv2
import numpy as np 
from PyQt5 import QtWidgets, QtGui, QtCore
from pyzbar.pyzbar import decode
from QT_Duong import Ui_MainWindow

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.cap = None
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_frame)

        self.ui.btn_opencamera.clicked.connect(self.start_camera)

        self.ui.chua_camera.setScaledContents(True)
        self.ui.chua_camera_2.setText("📷 Chưa quét được mã QR")

         # Kết nối nút
        self.btn_diemdanh = self.findChild(QtWidgets.QPushButton, 'btn_diemdanh')
        self.diemdanh_state = False  # False = Điểm danh, True = Chụp ảnh

        self.btn_diemdanh.clicked.connect(self.handle_toggle)  #toggle btn
    

    def handle_toggle(self):
    
        self.diemdanh_state = not self.diemdanh_state
        if self.diemdanh_state:
            self.btn_diemdanh.setText("Chụp ảnh")
            self.btn_diemdanh.setStyleSheet("background-color: orange; color: white;")

        else:
            self.btn_diemdanh.setText("Điểm danh")
            self.btn_diemdanh.setStyleSheet("background-color: green; color: white;")  

    def start_camera(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap or not self.cap.isOpened():
            QtWidgets.QMessageBox.critical(self, "Lỗi", "Không mở được camera.")
            return
        self.timer.start(30)

    def update_frame(self):
        if not self.cap or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        frame = self.process_qr(frame)

        # Hiển thị ảnh camera
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qt_img = QtGui.QImage(rgb_frame.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.ui.chua_camera.setPixmap(QtGui.QPixmap.fromImage(qt_img))

    def process_qr(self, frame):
        decoded_objects = decode(frame)
        result_text = ""

        for obj in decoded_objects:
            # Giải mã dữ liệu
            try:
                qr_data = obj.data.decode('utf-8')
            except UnicodeDecodeError:
                qr_data = obj.data.hex()

            # Tách từng dòng và gán biểu tượng tương ứng
            lines = qr_data.strip().split('\n')
            for line in lines:
                if "Tên" in line:
                    result_text += f"👤 {line}\n"
                elif "ID" in line:
                    result_text += f"🆔 {line}\n"
                elif "Phòng" in line or "phòng" in line:
                    result_text += f"🏢 {line}\n"
                else:
                    result_text += f"{line}\n"

            # Vẽ khung quanh QR code
            points = obj.polygon
            if len(points) > 4:
                hull = cv2.convexHull(np.array([p for p in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = points

            n = len(hull)
            for j in range(n):
                cv2.line(frame, hull[j], hull[(j + 1) % n], (0, 255, 0), 2)

        # Cập nhật QLabel chứa kết quả
        if result_text:
            self.ui.chua_camera_2.setText(result_text.strip())
        else:
            self.ui.chua_camera_2.setText("🔍 Không tìm thấy mã QR")

        return frame




    def closeEvent(self, event):
        if self.cap:
            self.cap.release()
        self.timer.stop()
        event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())