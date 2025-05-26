import sys
import os
import cv2 
import threading
import time
import pandas as pd
import pyttsx3
import textwrap
import openpyxl
import google.generativeai as genai
from gtts import gTTS 
from playsound3 import playsound
from PyQt5.QtWidgets import QFileDialog
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import Qt,QThread
from PyQt5.QtWidgets import QStackedWidget,QListWidgetItem, QMessageBox
from QT_Duong import Ui_MainWindow
import speech_recognition as sr
from playsound3 import playsound  # Thay đổi import ở đây
import gemini
import pygame

# Cấu hình API key
GOOGLE_API_KEY = "AIzaSyCxp0wD3-6nZOKaRn_WUkvzwlHOKfw-hJw"
if not GOOGLE_API_KEY:
    print("Lỗi: Vui lòng thiết lập API Key.")
    exit()
genai.configure(api_key=GOOGLE_API_KEY)
model = genai.GenerativeModel('gemini-1.5-flash')

def speak_vietnamese_gg(text): 
    try:
        tts = gTTS(text=text, lang='vi')
        filename = "temp_speech.mp3"
        tts.save(filename)
        playsound(filename)
        os.remove(filename)
    except Exception as e:
        print(e)

def phat_tieng_viet(text, out_file="out_file.mp3"):
    try:
        # Xử lý text để không lỗi
        text = text.strip()
        if not text:
            print("Không có nội dung để phát.")
            return
        if pygame.mixer.get_init():
            pygame.mixer.music.stop()  # Dừng nếu đang phát
            pygame.mixer.quit()        # Thoát mixer nếu cần
        
        # Nếu file đang tồn tại, xóa trước khi ghi đè
        if os.path.exists(out_file):
            os.remove(out_file)

        # Ghi ra file
        tts = gTTS(text=text, lang='vi')
        tts.save(out_file)
        print(f" Đã lưu file mp3: {out_file}")

        # Phát bằng pygame
        pygame.mixer.init()
        pygame.mixer.music.load(out_file)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)

        pygame.mixer.quit()
        os.remove(out_file)

    except Exception as e:
        print(f" Lỗi phát âm: {e}")



#print(f"Số luồng đang chạy: {threading.active_count()}")
#print(f"Luồng hiện tại: {threading.current_thread().name}")

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.stackedWidget.setCurrentWidget(self.ui.pape_tra_cuu)
        # xoa trang khi moi khoi dong 
        self.ui.tc_nha_truong.clear()  # Reset danh sách câu hỏi
        self.ui.cau_hoi_ai.clear()
        self.ui.tra_loi_ai.clear()
        self.ui.tra_loi_tc_nha_truong.clear()  # Reset chỗ hiển thị câu trả lời
        self.ui.bt_tc_ai.clicked.connect(self.tra_cuu_ai)
        self.ui.bt_kt_ai.clicked.connect(self.reload_ai)
        self.ui.bt_kt_ai.setEnabled(False)       
        self.ui.tra_loi_ai.setWordWrap(True)  # Tự động ngắt dòng
        self.ui.tra_loi_ai.setAlignment(Qt.AlignCenter)
         #canh giua cho Qlabel
        self.ui.tra_loi_tc_nha_truong.setAlignment(Qt.AlignCenter)

    def home_nha_truong (self):
        self.ui.stackedWidget.setCurrentWidget(self.ui.pape_tra_cuu)
        self.tt_ai = False 
        self.ui.bt_kt_ai.setEnabled(False)

    def reload_ai(self):
        # Dừng phát âm thanh nếu đang chạy
        pygame.mixer.init()
        pygame.mixer.music.stop()
        print(" Đã dừng phát âm thanh!")

        # Xóa file âm thanh tạm (nếu tồn tại)
        if hasattr(self, "out_file") and os.path.exists(self.out_file):
            os.remove(self.out_file)
            print(f" Đã xóa file tạm: {self.out_file}")

        # Reset nội dung câu hỏi trên giao diện
        self.ui.cau_hoi_ai.clear()
        print("Đã reset, sẵn sàng nhập câu hỏi mới!")
        
    def tra_cuu_ai (self):
        self.ui.stackedWidget.setCurrentWidget(self.ui.page_ai)
        self.tt_ai = True
        self.nghe_lenh_kich_hoat()
        self.ui.cau_hoi_ai.clear()
        self.ui.tra_loi_ai.clear()

    def lang_nghe_reload(self):
        r = sr.Recognizer()
        mic = sr.Microphone()

        with mic as source:
            r.adjust_for_ambient_noise(source)

        while self.tt_ai:  # Đang hoạt động
            with mic as source:
                try:
                    print("Đang chờ lệnh 'reload'...")
                    audio = r.listen(source, timeout=5)
                    text = r.recognize_google(audio, language="vi-VN").lower()
                    #print("Luồng kết thúc nhận :", text)

                    if "dừng lại" in text:
                        # Dừng phát âm thanh nếu đang chạy
                        pygame.mixer.init()
                        pygame.mixer.music.stop()
                        print(" Đã dừng phát âm thanh!")

                        # Xóa file âm thanh tạm (nếu tồn tại)
                        if hasattr(self, "out_file") and os.path.exists(self.out_file):
                            os.remove(self.out_file)
                            print(f" Đã xóa file tạm: {self.out_file}")

                        # Reset nội dung câu hỏi trên giao diện
                        self.ui.cau_hoi_ai.clear()
                        print("Đã reset, sẵn sàng nhập câu hỏi mới!")
                        
                        break
                except:
                    continue

    def nghe_lenh_kich_hoat(self):
        def run():
            r = sr.Recognizer()
            mic = sr.Microphone()

            with mic as source:
                r.adjust_for_ambient_noise(source)
                self.ui.tra_loi_ai.setText(" Bạn hãy nói 'Chào Robot' để bắt đầu...")
            
            while (self.tt_ai == True) :
                print("loop1")
                with mic as source:
                    try:
                        r.pause_threshold = 1.5
                        print("Đang đợi câu 'Chào Robot'...")
                        audio = r.listen(source)
                        text = r.recognize_google(audio, language="vi-VN").lower()
                        print("Bạn nói:", text)

                        if "chào robot" in text:
                            self.ui.tra_loi_ai.setText("   Xin Chào ! Tôi là Robot lễ tân,bạn hãy đặt câu hỏi hoặc nói,bấm kết thúc để dừng ")
                            #speak_vietnamese_gg("  Xin Chào ! Tôi là Robot lễ tân,bạn hãy đặt câu hỏi hoặc nói , bấm kết thúc để dừng ")
                            phat_tieng_viet("  Xin Chào ! Tôi là Robot lễ tân,bạn hãy đặt câu hỏi hoặc nói , bấm kết thúc để dừng ")
                            self.ui.bt_kt_ai.setEnabled(True)
                            QtWidgets.QApplication.processEvents()
                            # Bắt đầu thread nghe lệnh "kết thúc"
                            self.nghe_nhieu_cau_hoi()

                    except sr.WaitTimeoutError:
                        continue
                    except sr.UnknownValueError:
                        continue
                    except Exception as e:
                        print(f"Lỗi: {e}")

        threading.Thread(target=run, daemon=True).start()
    
    def nghe_nhieu_cau_hoi(self):
        r = sr.Recognizer()
        mic = sr.Microphone()

        thread_stop = threading.Thread(target=self.lang_nghe_reload, daemon=True)
        thread_stop.start()
        with mic as source:
            r.adjust_for_ambient_noise(source)

        while (self.tt_ai == True) :
            print("loop2")
            with mic as source:
                try:
                    self.ui.tra_loi_ai.setText(" Tôi đang nghe câu hỏi...")
                    QtWidgets.QApplication.processEvents()

                    audio = r.listen(source, timeout=7)
                    text = r.recognize_google(audio, language="vi-VN").lower()
                    print("Bạn nói :", text)
                
                    if ("kết thúc" in text):  
                        self.ui.tra_loi_ai.setText(" Cảm ơn bạn đã sử dụng.")
                        QtWidgets.QApplication.processEvents()
                        phat_tieng_viet("Cảm ơn bạn đã sử dụng.")
                        #speak_vietnamese_gg(" Cảm ơn bạn đã sử dụng.")
                        self.tt_ai = False 
                        self.ui.cau_hoi_ai.clear()
                        self.ui.tra_loi_ai.clear()
                        self.ui.stackedWidget.setCurrentWidget(self.ui.pape_tra_cuu)
                        self.ui.bt_kt_ai.setEnabled(False)
                        break

                    else:
                        self.ui.cau_hoi_ai.setText(f" Bạn hỏi : {text}")
                        QtWidgets.QApplication.processEvents()
                        self.xu_ly_cau_hoi(text) 
                               
                except sr.WaitTimeoutError:
                    #speak_vietnamese_gg(" Tôi không nghe thấy gì, bạn vui lòng nói xong và đợi 1 giây để có câu trả lời nhé ")
                    phat_tieng_viet(" Tôi không nghe thấy gì, bạn vui lòng nói xong và đợi 1 giây để có câu trả lời nhé ")
                    print(" Không nghe thấy gì, tiếp tục...")
                    continue
                except sr.UnknownValueError:
                    print(" Không hiểu, xin nói lại...")
                    #speak_vietnamese_gg("Tôi nghe không rõ,xin nói lại ")
                    phat_tieng_viet("Tôi nghe không rõ,xin nói lại ")
                    continue
                except Exception as e:
                    print(f" Lỗi: {e}")

    def xu_ly_cau_hoi(self, cau_hoi_text):
        self.ui.tra_loi_ai.setText(" Đang xử lý câu hỏi...")
        QtWidgets.QApplication.processEvents()

        try:
            response = model.generate_content(cau_hoi_text)
            if hasattr(response, "text") and response.text:
                tra_loi = response.text.strip()
                self.ui.tra_loi_ai.setText(f" {tra_loi}")
                self.ui.tra_loi_ai.setWordWrap(True)  # Tự động ngắt dòng
                self.ui.tra_loi_ai.setAlignment(Qt.AlignCenter)
                QtWidgets.QApplication.processEvents()
                phat_tieng_viet(tra_loi)
                #speak_vietnamese_gg(tra_loi)
            else:
                self.ui.tra_loi_ai.setText(" Không nhận được phản hồi.")
                QtWidgets.QApplication.processEvents()

        except Exception as e:
            self.ui.tra_loi_ai.setText(f" Lỗi: {str(e)}")
            QtWidgets.QApplication.processEvents()
    

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())     
    