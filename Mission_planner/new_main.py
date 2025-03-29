import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QSplashScreen, QDialog
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import QFont, QPalette, QColor
import time
from PyQt5.QtCore import QTimer

from layout import screen3d_layout
from layout import canvas_layout
from layout import setting_layout
from layout.control_layout import ControlWidget

from communication.system_update_timer import SystemStatusManager

STL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                       "layout", "resources", "shell_assem.STL")
LOGO_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "layout", "resources", "splash_screen_v1.png")

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AQUA MISSION PLANNER")
        self.setGeometry(0,0, 1890, 990)  

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#2C3333")) 
        self.setPalette(palette)
        self.setAutoFillBackground(True)

        self.status_manager=SystemStatusManager()

        self.left_up_screen_init()
        self.left_down_screen_init()
        self.right_down_screen_init()
        self.right_up_screen_init()

    def left_up_screen_init(self):
        self.lup_screen = screen3d_layout.STLViewerWidget(STL_PATH,self.status_manager)
        self.lup_screen.setParent(self)
        self.lup_screen.setGeometry(12, 15,860,398)  

    def left_down_screen_init(self):
        self.ldown_screen = canvas_layout.CanvasWidget(self.status_manager)
        self.ldown_screen.setParent(self)
        self.ldown_screen.setGeometry(12, 425,860,545)  

    def right_down_screen_init(self):
        self.rdown_screen= setting_layout.SettingWidget()
        self.rdown_screen.setParent(self)
        self.rdown_screen.setGeometry(889,809,985, 162)

    def right_up_screen_init(self):
        self.rup_screen= ControlWidget()
        self.rup_screen.setParent(self)
        self.rup_screen.setGeometry(889,14,985, 785)

class SplashScreen(QSplashScreen):
    def __init__(self):
        super().__init__(QPixmap(LOGO_PATH))
    
if __name__ == "__main__":
    app = QApplication(sys.argv)

    splash = SplashScreen()
    splash.show()

    window = MainWindow()

    QTimer.singleShot(1200, lambda: (window.show(), splash.finish(window)))
    sys.exit(app.exec_())