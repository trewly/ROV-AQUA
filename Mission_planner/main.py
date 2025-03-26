import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QSplashScreen, QDialog
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import QFont, QPalette, QColor
import time
from PyQt5.QtCore import QTimer

from layout import screen3d
from layout import control
from layout import new_setting
from layout.test_widget.just_widget import testLayout
from communication.system_update_timer import SystemStatusManager

# Định nghĩa đường dẫn tới STL
STL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                       "layout", "resources", "shell_assem.STL")

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AQUA MISSION PLANNER")
        self.setGeometry(0,0, 1890, 990)  

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#2C3333")) 
        self.setPalette(palette)
        self.setAutoFillBackground(True)

        #khoi tao timer cap nhat du lieu
        self.status_manager=SystemStatusManager()

        self.upscreen_init()
        self.down_screen_init()
        self.rightscreen_init()

    def upscreen_init(self):
        self.up_screen = screen3d.STLViewerWidget(STL_PATH)
        self.up_screen.setParent(self)
        self.up_screen.setGeometry(15, 20,950, 395)  

    def down_screen_init(self):
        self.down_screen = control.ControlPanel()
        #self.down_screen = testLayout()
        self.down_screen.setParent(self)
        self.down_screen.setGeometry(10, 425,960, 540)  

    def rightscreen_init(self):
        self.right_screen = new_setting.settingLayout(self.status_manager)
        self.right_screen.setParent(self)
        self.right_screen.setGeometry(975, 20, 910, 944)

class SplashScreen(QSplashScreen):
    def __init__(self):
        super().__init__(QPixmap("./layout/resources/splash_screen_v1.png"))  # Thay "logo.png" bằng đường dẫn ảnh của bạn
    
if __name__ == "__main__":
    app = QApplication(sys.argv)

    splash = SplashScreen()
    splash.show()

    window = MainWindow()

    QTimer.singleShot(1200, lambda: (window.show(), splash.finish(window)))
    sys.exit(app.exec_())