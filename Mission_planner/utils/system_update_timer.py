from PyQt5.QtCore import QObject, QTimer, pyqtSignal

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

#print(os.path.dirname(os.path.abspath(__file__)))

from status import pc_status as status
from config import raspi_config as config

class SystemStatusManager(QObject):
    got_disconnected_info = pyqtSignal(bool)  
    got_temp_depth_info = pyqtSignal(float,float)
    got_roll_pitch_yaw_info= pyqtSignal(float,float,float)
    got_x_velo_info = pyqtSignal(bool,float,bool)

    def __init__(self):
        super().__init__()

        #doc lan dau tien
        self.initial_read()
        
        #timer theo giay
        self.timer_1 = QTimer()
        self.timer_1.timeout.connect(self.get_disconnected_info)
        self.timer_1.timeout.connect(self.get_temp_depth_info)
        self.timer_1.start(1000)  # Cập nhật mỗi 1 giây

        #timer theo ms
        self.timer_2= QTimer()
        self.timer_2.timeout.connect(self.get_roll_pitch_yaw_info)
        self.timer_2.timeout.connect(self.get_x_velo_info)
        self.timer_2.start(50)

    def initial_read(self):
        try:
            self.disconnected=bool(status.read_status("disconnected"))
            self.temp=status.read_status("internal_temp")
            self.depth=status.read_status("depth")
            self.pitch=status.read_status("pitch")
            self.roll=status.read_status("roll")
            self.yaw=status.read_status("heading")
        except:
               print("Error initial read")

    def get_disconnected_info(self):
        try:
            self.disconnected=bool(status.read_status("disconnected"))
            self.got_disconnected_info.emit(self.disconnected)
        except:
            print("Error read connect status")

    def get_temp_depth_info(self):
        if self.disconnected:
            self.got_temp_depth_info.emit(6,6)
            #return
        else:
            try:
                self.temp=status.read_status("internal_temp")
                self.depth=status.read_status("depth")
                self.got_temp_depth_info.emit(self.temp,self.depth)
            except:
                print("Error read temp,depth")

    def get_roll_pitch_yaw_info(self):
        if False:
        #if self.disconnected:
            self.got_roll_pitch_yaw_info.emit(6,6,90)
            return
        else:
            try:
                self.pitch=status.read_status("pitch")
                self.roll=status.read_status("roll")
                self.yaw=status.read_status("heading")
               # print(status.read_status("heading"))
                self.got_roll_pitch_yaw_info.emit(self.roll,self.pitch,self.yaw)
            except:
                print("Error read roll,pitch,yaw")

    def get_x_velo_info(self):
        if False:
        #if self.disconnected:
            self.got_roll_pitch_yaw_info.emit(0,5,0)
            return
        else:
            try:
                self.x_velo=status.read_status("vertical_velocity")
                self.got_x_velo_info.emit(0,self.x_velo,0)
            except:
                print("Error read x velo")
