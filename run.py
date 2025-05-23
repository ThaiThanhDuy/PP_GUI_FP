import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import pyqtSlot, QProcess
from FILE_QT.login import Ui_MainWindow_login

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow_login()
        self.ui.setupUi(self)
        self.ui.btn_dn_login.clicked.connect(self.handle_login_button)
        self.process = None

    @pyqtSlot()
    def handle_login_button(self):
        print("Login button pressed!")
        self.close()
        self.run_external_command()

    def run_external_command(self):
        command = "/bin/bash"
        arguments = ["-c", "cd /home/robot/ROBOT_HD && /home/robot/robot_lib/bin/python3 main.py"]

        self.process = QProcess()
        self.process.finished.connect(self.process_finished)
        self.process.errorOccurred.connect(self.process_error)
        self.process.readyReadStandardOutput.connect(self.read_output)
        self.process.readyReadStandardError.connect(self.read_error)
        self.process.start(command, arguments)

        print(f"Attempting to run command: {command} {' '.join(arguments)}")

    @pyqtSlot(int, QProcess.ExitStatus)
    def process_finished(self, exitCode, exitStatus):
        print(f"External process finished with exit code: {exitCode}, status: {exitStatus}")
        self.process = None  # Clean up the QProcess object

    @pyqtSlot(QProcess.ProcessError)
    def process_error(self, error):
        print(f"Error occurred with external process: {error}")
        self.process = None

    @pyqtSlot()
    def read_output(self):
        output = self.process.readAllStandardOutput().data().decode()
        print(f"External process output:\n{output}")

    @pyqtSlot()
    def read_error(self):
        error = self.process.readAllStandardError().data().decode()
        print(f"External process error:\n{error}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())