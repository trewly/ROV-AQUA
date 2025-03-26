from PyQt5.QtCore import QObject, QTimer, pyqtSignal

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

#print(os.path.dirname(os.path.abspath(__file__)))

from status import pc_status as status

class SystemStatusManager(QObject):
    got_disconnected_info = pyqtSignal(bool)  
    got_temp_depth_info = pyqtSignal(float,float)

    def __init__(self):
        super().__init__()
        
        #in ra dam bao khoi tao
        print("Khoi tao thanh cong status manager")

        #timer theo giay
        self.timer_1 = QTimer()
        self.timer_1.timeout.connect(self.get_disconnected_info)
        self.timer_1.timeout.connect(self.get_temp_depth_info)
        self.timer_1.start(1000)  # Cập nhật mỗi 1 giây

        #timer theo ms

    def get_disconnected_info(self):
        try:
            self.disconnected=bool(status.read_status("disconnect"))
            self.got_disconnected_info.emit(self.disconnected)
        except:
            print("Error read connect status")

    def get_temp_depth_info(self):
        if self.disconnected:
            self.got_temp_depth_info.emit(0,1)
            #return
        else:
            try:
                self.temp=status.read_status("temp")
                self.depth=status.read_status("depth")
                self.got_temp_depth_info.emit(self.temp,self.depth)
            except:
                print("Error read temp,depth")