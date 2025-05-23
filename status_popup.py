from PyQt5 import QtCore, QtWidgets
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QObject
import os
from PyQt5.QtWidgets import (
    QPushButton, QVBoxLayout, QDialog, QComboBox, QDialogButtonBox, 
    QCheckBox, QMessageBox, QLabel, QLineEdit, QTextEdit, QHBoxLayout
)
from PyQt5.QtGui import QFont, QIcon

class StatusPopup(QDialog):
    """Popup to display status messages with error handling options"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Process Status")
        self.setFixedSize(400, 200)
        # self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)
        
        self.icons = {"starting": "🚀", "loading": "⏳", "success": "✅", "error": "❌"}
        self.colors = {"starting": "#007bff", "loading": "#ffc107", "success": "#28a745", "error": "#dc3545"}

        # Main container styling
        self.setStyleSheet("""
            QDialog {
                background-color: #ffffff;
                border-radius: 10px;
                border: 2px solid #007bff;
                padding: 10px;
            }
            QLabel#header {
                background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1, stop:0 #007bff, stop:1 #00c6ff);
                color: white;
                padding: 10px;
                font-weight: bold;
                font-size: 16px;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                text-align: center;
                text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.2);
            }
            QTextEdit {
                background-color: #f9f9f9;
                border: none;
                font-size: 14px;
                color: #333333;
            }
        """)

        # Header label
        self.header = QLabel("Process Status")
        self.header.setObjectName("header")
        self.header.setAlignment(Qt.AlignCenter)

        # TextEdit to show cumulative status updates
        self.status_text = QTextEdit(self)
        self.status_text.setReadOnly(True)
        self.status_text.setAlignment(Qt.AlignLeft)
        
        # Label Create
        self.label_width, self.label_height = 100, 100
        self.animation_label = QtWidgets.QLabel(self) 
        self.animation_label.setGeometry(QtCore.QRect(25, 25, self.label_width, self.label_height))
        # self.animation_label.setMinimumSize(QtCore.QSize(50, 50))
        # self.animation_label.setMaximumSize(QtCore.QSize(300, 300))
        self.animation_label.setStyleSheet("background: transparent;")
        self.animation_label.setObjectName("lb1")
        # FrontWindow.setCentralWidget(self.centralwidget)

        # Loading the GIF 
        self.loading_gif = QMovie("videos/loading.gif")
        self.loading_gif.setScaledSize(QtCore.QSize(self.label_width+20, self.label_height+20))
        self.successfully_gif = QMovie("videos/successfully.gif")
        self.successfully_gif.setScaledSize(QtCore.QSize(self.label_width, self.label_height))
        self.error_gif = QMovie("videos/error.gif")
        self.error_gif.setScaledSize(QtCore.QSize(self.label_width, self.label_height))
        self.animation_label.setMovie(self.loading_gif)

        # Layout for loading label
        loading_layout = QHBoxLayout()
        loading_layout.addStretch()
        loading_layout.addWidget(self.animation_label)
        loading_layout.addStretch()
        
        # Layout for the popup window
        layout = QVBoxLayout()
        layout.addLayout(loading_layout)
        layout.addWidget(self.status_text)

        # Error handling buttons (hidden by default)
        self.details_button = QPushButton("Watch Details")
        self.details_button.clicked.connect(self.show_details)
        self.restart_button = QPushButton("Restart")
        self.restart_button.clicked.connect(self.restart_process)

        # Buttons layout
        self.button_layout = QHBoxLayout()
        self.button_layout.addWidget(self.details_button)
        self.button_layout.addWidget(self.restart_button)
        layout.addLayout(self.button_layout)

        self.setLayout(layout)
        self.hide_error_buttons()
        
        # Timer to close the dialog automatically on success
        self.auto_close_timer = QTimer(self)
        self.auto_close_timer.setSingleShot(True)
        self.auto_close_timer.timeout.connect(self.close)

    def update_status(self, status_type, message):
        """Update the popup window with new status messages and styling."""
        self.animation_label.setMovie(self.loading_gif)
        self.loading_gif.start()
        if status_type == "success":
            self.status_text.clear()
            self.loading_gif.stop()
            self.animation_label.setMovie(self.successfully_gif)
            self.successfully_gif.start()
            # Automatically close the dialog on success after 2 seconds
            self.auto_close_timer.start(2000)  # 2000 ms (2 seconds)
        elif status_type == "error":
            self.status_text.clear()
            self.loading_gif.stop()
            self.animation_label.setMovie(self.error_gif)
            self.error_gif.start()

        # Status emojis/icons
        icon = self.icons.get(status_type, "")

        # Append new message with icon and color coding
        color = self.colors.get(status_type, "#000000")

        # Format the message
        formatted_message = f'<span style="color:{color};">{icon} {message}</span>'
        self.status_text.append(formatted_message)
        self.show()  # Ensure the popup stays visible

    def closeEvent(self, event):
        """Clear the status text and stop the timer when the popup is closed."""
        self.status_text.clear()
        self.hide_error_buttons()
        self.auto_close_timer.stop()  # Stop the timer in case it was still running
        super().closeEvent(event)

    def show_details(self):
        """Expand the popup to show full details of the error log."""
        self.setFixedSize(400, 400)  # Expand to show more details

    def restart_process(self):
        """Restart the current process (simulated here)."""
        self.hide_error_buttons()
        self.status_text.clear()
        # Emit signal or call a method to restart the process (Placeholder here)
        self.status_text.append('<span style="color:blue;">Restarting process...</span>')

    def show_error_buttons(self):
        """Show the error handling buttons."""
        self.details_button.show()
        self.restart_button.show()

    def hide_error_buttons(self):
        """Hide the error handling buttons."""
        self.details_button.hide()
        self.restart_button.hide()


class NamedMap(QDialog):
    def __init__(self, maps_folder, default_map_name):
        super().__init__()
        self.maps_folder = maps_folder
        self.default_map_name = default_map_name

        # Dialog setup with a nice title and size
        self.setWindowTitle("Save Map")
        self.setFixedSize(450, 250)
        self.setStyleSheet("""
            QDialog {
                background-color: #f7f7f7;
                border-radius: 12px;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
        """)

        # Create layout
        layout = QVBoxLayout(self)

        # Instructions Label with Stylish Font
        instructions_label = QLabel("Select an existing name or enter a new one:")
        instructions_label.setFont(QFont("Segoe UI", 12, QFont.Bold))
        instructions_label.setStyleSheet("""
            color: #333;
            margin-bottom: 12px;
            padding-left: 20px;
        """)
        # layout.addWidget(instructions_label)

        # ComboBox for Map Names with Editable Option
        self.combo_box = QComboBox(self)
        self.combo_box.setEditable(True)
        self.combo_box.setPlaceholderText("Enter a new map name or select from the list...")
        self.combo_box.setFont(QFont("Segoe UI", 10))
        self.combo_box.setStyleSheet("""
            QComboBox {
                padding: 10px;
                border: 1px solid #888;
                border-radius: 8px;
                background-color: #ffffff;
                font-size: 14px;
            }
            QComboBox:editable {
                background-color: #f3f3f3;
            }
            QComboBox QAbstractItemView {
                background-color: #ffffff;
                border-radius: 8px;
                selection-background-color: #4CAF50;
                padding: 10px;
            }
        """)
        self.combo_box.setToolTip("Type a new map name or select an existing one to overwrite.")
        self.combo_box.setCurrentText(self.default_map_name)
        self.load_map_names()
        layout.addWidget(self.combo_box)

        self.set_default_checkbox = QCheckBox("Set as Default", self)
        self.set_default_checkbox.setChecked(True)  # Checked by default
        self.set_default_checkbox.setStyleSheet("""
            QCheckBox {
                font-size: 12px;
                color: #333;
            }
            QCheckBox:checked {
                color: #4CAF50;
            }
        """)
        layout.addWidget(self.set_default_checkbox)

        # Buttons with modern styling
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)
        buttons.setStyleSheet("""
            QDialogButtonBox QPushButton {
                background-color: #4CAF50;
                color: white;
                padding: 10px 20px;
                font-size: 14px;
                border-radius: 8px;
                margin-top: 20px;
                border: none;
            }
            QDialogButtonBox QPushButton:disabled {
                background-color: #b0b0b0;
                color: #d1d1d1;
            }
            QDialogButtonBox QPushButton:hover {
                background-color: #45A049;
            }
            QDialogButtonBox QPushButton:pressed {
                background-color: #388E3C;
            }
        """)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        # Adjust the dialog layout and set spacing
        layout.setSpacing(20)
        layout.setContentsMargins(20, 20, 20, 20)

        self.combo_box.setFocus()

    def load_map_names(self):
        if os.path.exists(self.maps_folder):
            map_files = [f[:-5] for f in os.listdir(self.maps_folder) if f.endswith('.yaml')]
            self.combo_box.addItems(map_files)

    def get_selected_map_name(self):
        return self.combo_box.currentText().strip()

    def is_set_as_default(self):
        print('////////////////////////////', self.set_default_checkbox.isChecked())
        return self.set_default_checkbox.isChecked()

    def kill_instance(self ):
        self.deleteLater()