from PyQt5.QtCore import QThread, pyqtSignal
import serial
import RobotConfig as RobConf
import Utilities as Uti


MegaNANO_PORT = RobConf.MegaNANO_PORT
full_vol = RobConf.full_vol #10.7


#----------------------------------------------------------- CLASS DOC MUC PIN ARDUINO NANO ---------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------------------------------------------------------

class ReadArduinoPin(QThread):
    auto_charing_lientuc = pyqtSignal(bool)
    doc_pin = pyqtSignal(str)

    def __init__(self, port= MegaNANO_PORT, baud_rate=38400, full_vol=10.7):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.full_vol = full_vol
        self.ser = None
        self.is_running = False
        self.voltage = 0

    def connect_arduino(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate)
            print('Kết nối Arduino thành công')
            self.is_running = True
        except Exception as ex:
            self.is_running = False

    def read_data_from_arduino(self):
        try:
            return self.ser.readline().decode('utf-8').strip()
        
        except Exception as ex:
            return ""

    def send_reset_2_arduino(self):
        try:
     
            return self.ser.write(RobConf.RESET_NANO_CODE.encode())

        except Exception as ex:
        
            return ""

    def run(self):
        self.connect_arduino()
        if self.is_running:
            print('Bắt đầu đọc điện áp')
            while self.is_running:
                self.voltage = self.read_data_from_arduino()
                self.doc_pin.emit(self.voltage)
                if self.voltage:
                    try:
                        if float(self.voltage) < self.full_vol: 
                            self.auto_charing_lientuc.emit(True)    
                        else:
                     
                            self.auto_charing_lientuc.emit(False)
                    except ValueError:
                        pass
    def stop(self):
        self.is_running = False
        print("dung doc dien ap")
            
# CLASS CONTROLL CHARING
class arduino(QThread):
    auto_charing = pyqtSignal(bool)
    def __init__(self):
        super().__init__()
        self.ser = None
        self.data = ''
        self.is_running = False
        self.connected_ard = False
        self.count = 0
        self.nhan_nutsac = False
        self.nhan_nutbatdau = False
        self.batdau_sac = False
        
    def connect_arduino(self):
        try :
            #self.ser = serial.Serial('/dev/ttyUSB1', 9600)
            self.ser = serial.Serial(RobConf.MegaNANO_PORT, 38400)
            print('ket noi arduino thanh cong')
            time.sleep(0.5)
            self.connected_ard = True
        except Exception as ex:
            print(ex) 
        
    def read_data_from_arduino(self):
        data = self.ser.readline().decode('utf-8').strip()
        return data
  
    def run(self):
        self.data = ""
        self.count = 0
        self.is_running = True
        # self.no_voice = False
        # self.batdau_sac = False
        self.nhan_nutsac = False
        self.nhan_nutbatdau = False
        self.batdau_sac = False
        self.connect_arduino()
        if self.connected_ard:
            print('bat dau doc dien ap')
            while self.is_running:
                self.data = self.read_data_from_arduino()
                print('---volage battery ---: ',self.data)
                if (float(self.data)) < full_vol:
                    if not self.nhan_nutsac and not self.nhan_nutbatdau:
                        Uti.RobotSpeakWithPath('voice_hmi_new/vuilongsacpin.wav')
                        self.batdau_sac = True

                    else:
                        pass
                else:
                    self.batdau_sac = False
                self.is_running = False
                break
            if self.batdau_sac :
                self.auto_charing.emit(True)
            else:
                print('du pin')
            print('ngat ket noi arduino')
            self.ser.close()
            self.connected_ard = False