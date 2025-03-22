from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QHBoxLayout, QPushButton
from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QThread, pyqtSignal

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Import canvas từ module
from layout.sub_widgets import canvas, motor_slider
from Mission_planner.status import pc_status as status
# import sys
# import os

# print("Current sys.path:", sys.path)

infoStyle="""
            QWidget {
                border: 2px solid black;  /* Viền màu đen */
                border-radius: 10px;      /* Bo góc */
                background-color: lightgray; /* Màu nền nhẹ */
            }
        """

normal="""
        QLabel {
            background-color: transparent; 
            border: none;  
            font-size: 20px;     
            font-weight: bold;                                               
        }
    """

highlightValue="""
        QLabel {
                    background-color: transparent; 
                    border: none;  
                    font-size: 40px;     
                    font-weight: bold;                                               
                }
    """

buttonStyle = """
    QPushButton {
        font-size: 16px;
        font-weight: bold;
        padding: 5px; /* Giúp không làm ảnh hưởng đến hover */
        border: 1px solid black; /* Viền ngoài */
        border-radius: 8px; /* Bo góc */
        background-color: white;
    }

    QPushButton:hover {
        background-color: lightgray; 
    }

    QPushButton:pressed {
        background-color: #a8d5e2; 
        border-color: #2980b9;
    }
"""

class settingLayout(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(917,1029)
        #khoi tao view widget
        self.view_init()
        #khoi tao canvas
        self.canvas_init()
    
        #khoi tao bar thong so
        self.info_bar_init()

        #khoi tao bar setting
        self.setting_bar_init()

        #Khoi tao cac phim chuc nang
        self.setting_function_button_init()

    def view_init(self):
        self.viewWidget = QWidget(self)
        self.viewWidget.setFixedSize(850, 70)
        
        # View label
        self.viewLabel = QLabel("View", self.viewWidget)
        self.viewLabel.setStyleSheet("font-size: 48px; font-weight: bold;")
        self.viewLabel.move(20,10)  # Di chuyển label "View" tới vị trí (0, 10)
        
        # status text
        self.statusLabel = QLabel("Vehicle unconnected", self.viewWidget)
        self.statusLabel.setStyleSheet("font-size: 20px; ")
        self.statusLabel.move(605,36)  # Di chuyển label trạng thái gần vị trí mong muốn
        
        # status dot
        self.statusDot = QLabel(self.viewWidget)
        self.statusDot.setFixedSize(20, 20)
        self.statusDot.setStyleSheet("border-radius: 10px; background-color: gray; border: 1px solid black;")
        self.statusDot.move(568,39)  

        self.viewWidget.setGeometry(0, 0, 850, 70)

    def canvas_init(self):
        self.mainCanvas=canvas.CanvasWidget()
        self.mainCanvas.move(20,100)      
        self.mainCanvas.setParent(self) 

    def info_bar_init(self):

        #tao box hien thi nhiet do
        self.tempInfoWidget = QWidget(self)
        self.tempInfoWidget.setFixedSize(178,95)

        self.tempInfoWidget.setStyleSheet(infoStyle)
        tempLable=QLabel("Temp",self.tempInfoWidget)
        tempLable.setStyleSheet(normal)
        tempLable.move(60,10)

        tempValue=QLabel("40o",self.tempInfoWidget)
        tempValue.setStyleSheet(highlightValue)
        tempValue.move(50,36)
        
        self.tempInfoWidget.setGeometry(20,630,178,95)

        #tao box hien thi do sau
        self.depthInfoWidget = QWidget(self)
        self.depthInfoWidget.setFixedSize(178,95)

        self.depthInfoWidget.setStyleSheet(infoStyle)

        depthLable=QLabel("Depth",self.depthInfoWidget)
        depthLable.setStyleSheet(normal)
        depthLable.move(60,10)

        depthValue=QLabel("5m",self.depthInfoWidget)
        depthValue.setStyleSheet(highlightValue)
        depthValue.move(58,36)

        self.depthInfoWidget.setGeometry(210,630,178,95)

    def setting_bar_init(self):
        self.viewLabel = QLabel("Setting", self)
        self.viewLabel.setStyleSheet("font-size: 36px; font-weight: bold;")
        self.viewLabel.move(20,750)  # Di chuyển label "View" tới vị trí (0, 10)
        
        #add speed slider
        self.speedSlider=motor_slider.MotorSlider()
        self.speedSlider.move(20,780)      
        self.speedSlider.setParent(self) 

    def setting_function_button_init(self):
        #khoi tao vung chua button
        button_fnc_widget=QWidget(self)
        button_fnc_widget.setFixedSize(340,340)
        button_fnc_widget.move(480,630)
        button_fnc_widget.setStyleSheet(infoStyle)

        #button 1
        self.calib_button=QPushButton("Calib",self)
        self.calib_button.setFixedSize(130,70)
        self.calib_button.setStyleSheet(buttonStyle)
        self.calib_button.move(505,650)
        
        #button 2
        self.calib_button=QPushButton("Param set",self)
        self.calib_button.setFixedSize(130,70)
        self.calib_button.setStyleSheet(buttonStyle)
        self.calib_button.move(665,650)

        #button 3
        self.button3=QPushButton("Button3",self)
        self.button3.setFixedSize(130,70)
        self.button3.setStyleSheet(buttonStyle)
        self.button3.move(505,760)
        
        #button 4
        self.button4=QPushButton("Button4",self)
        self.button4.setFixedSize(130,70)
        self.button4.setStyleSheet(buttonStyle)
        self.button4.move(665,760)

        # #button 5
        self.button5=QPushButton("Button5",self)
        self.button5.setFixedSize(130,70)
        self.button5.setStyleSheet(buttonStyle)
        self.button5.move(505,870)
        
        #button 6
        self.button6=QPushButton("Button6",self)
        self.button6.setFixedSize(130,70)
        self.button6.setStyleSheet(buttonStyle)
        self.button6.move(665,870)

    
class StatusReaderThread(QThread):
    status_signal = pyqtSignal(str, object)

    def __init__(self, key, parent=None):
        super().__init__(parent)
        self.key = key
        self.running = True

    def run(self):
        while self.running:
            try:
                value = status.read_status(self.key)
                self.status_signal.emit(self.key, value)
            except Exception as e:
                print(f"Error reading status: {e}")

    def stop(self):
        self.running = False
        self.wait()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Status Reader")
        self.resize(400, 200)

        self.label = QLabel("Waiting for status...", self)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        self.status_thread = StatusReaderThread(key="depth")
        self.status_thread.status_signal.connect(self.update_label)
        self.status_thread.start()

    def update_label(self, key, value):
        self.label.setText(f"{key}: {value}")

    def closeEvent(self, event):
        self.status_thread.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication([])
    setter = settingLayout()
    setter.show()
    app.exec_()
