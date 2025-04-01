from PyQt5.QtCore import Qt, QTimer

from .mode_controller import ModeChangeDialog
from Mission_planner.connection.pc_mavlink import MAV

class ButtonController:
    KEY_MAPPINGS = {
        Qt.Key_W: 'forward',
        Qt.Key_S: 'backward',
        Qt.Key_A: 'left',
        Qt.Key_D: 'right',
        Qt.Key_Shift: 'surface',
        Qt.Key_Control: 'dive',
        Qt.Key_R: 'roll_right',
        Qt.Key_F: 'roll_left',
    }
    
    MOTION_BUTTONS = {'forward', 'backward', 'left', 'right'}
    VERTICAL_BUTTONS = {'surface', 'dive'}
    ROLL_BUTTONS = {'roll_right', 'roll_left'}
    
    def __init__(self):
        self.active_buttons = set()
        self.last_state = {}
        self.last_active_buttons = set()
        self.should_send_stop = False
        self.stop_sent = False
        
        self.check_timer = QTimer()
        self.check_timer.timeout.connect(self.check_control_state)
        self.check_timer.start(50)  # Giảm tần suất kiểm tra xuống, 50ms là đủ
    
    def check_control_state(self):
        active_motion = any(btn in self.MOTION_BUTTONS for btn in self.active_buttons)
        active_vertical = any(btn in self.VERTICAL_BUTTONS for btn in self.active_buttons)
        active_roll = any(btn in self.ROLL_BUTTONS for btn in self.active_buttons)
        
        current_state = {
            'motion': active_motion,
            'vertical': active_vertical,
            'roll': active_roll
        }
        
        # Phát hiện các sự thay đổi trong các nút đang hoạt động
        buttons_changed = self.last_active_buttons != self.active_buttons
        
        # Kiểm tra xem có nút nào đang hoạt động không
        active_buttons = len(self.active_buttons) > 0
        
        # Nếu trạng thái thay đổi, cập nhật và gửi lệnh phù hợp
        if self.last_state != current_state or buttons_changed:
            print(f"Control state changed: {self.active_buttons}")
            
            # Khi không có nút nào được bấm và trạng thái STOP chưa được gửi
            if not active_buttons and not self.stop_sent:
                print("No buttons active - sending STOP command")
                MAV.send_control_cmd(MAV.STOP)
                self.stop_sent = True
            # Nếu có nút đang được bấm, đánh dấu rằng cần gửi STOP khi thả
            elif active_buttons:
                self.stop_sent = False
            
            self.last_state = current_state.copy()
            self.last_active_buttons = self.active_buttons.copy()
    
    def on_button_clicked(self, button_type):
        # Đánh dấu rằng cần gửi STOP sau khi thả tất cả các nút
        self.stop_sent = False
        
        # Thêm nút vào danh sách hoạt động
        self.active_buttons.add(button_type)
        
        # Gửi lệnh tương ứng
        if button_type == 'forward':
            MAV.send_control_cmd(MAV.FORWARD)
        elif button_type == 'backward':
            MAV.send_control_cmd(MAV.BACKWARD)
        elif button_type == 'left':
            MAV.send_control_cmd(MAV.LEFT)
        elif button_type == 'right':
            MAV.send_control_cmd(MAV.RIGHT)
        elif button_type == 'surface':
            MAV.send_control_cmd(MAV.SURFACE)
        elif button_type == 'dive':
            MAV.send_control_cmd(MAV.DIVE)
        elif button_type == 'roll_right':
            MAV.send_control_cmd(MAV.ROLL_RIGHT)
        elif button_type == 'roll_left':
            MAV.send_control_cmd(MAV.ROLL_LEFT)
        else:
            MAV.send_control_cmd(MAV.STOP)
    
    def on_button_released(self, button_type):
        # Xóa nút khỏi danh sách hoạt động
        if button_type in self.active_buttons:
            self.active_buttons.remove(button_type)
        
        # Với các nút này, gửi STOP ngay khi thả
        if button_type in self.VERTICAL_BUTTONS or button_type in self.ROLL_BUTTONS:
            MAV.send_control_cmd(MAV.STOP)
            self.stop_sent = True
        
        # Nếu không còn nút nào được bấm, gửi lệnh STOP
        if not self.active_buttons and not self.stop_sent:
            print("All buttons released - sending STOP command")
            MAV.send_control_cmd(MAV.STOP)
            self.stop_sent = True
        
        # Kiểm tra trạng thái để cập nhật last_state
        self.check_control_state()
    
    def on_forward_button_clicked(self):
        self.on_button_clicked('forward')
        print("Forward button clicked")
    
    def on_forward_button_released(self):
        self.on_button_released('forward')
        print("Forward button released")
    
    def on_backward_button_clicked(self):
        self.on_button_clicked('backward')
        print("Backward button clicked")
    
    def on_backward_button_released(self):
        self.on_button_released('backward')
        print("Backward button released")

    def on_left_button_clicked(self):
        self.on_button_clicked('left')
        print("Left button clicked")

    def on_left_button_released(self):
        self.on_button_released('left')
        print("Left button released")
    
    def on_right_button_clicked(self):
        self.on_button_clicked('right')
        print("Right button clicked")

    def on_right_button_released(self):
        self.on_button_released('right')
        print("Right button released")
        
    def on_surface_button_clicked(self):
        self.on_button_clicked('surface')
        print("Surface button clicked")

    def on_surface_button_released(self):
        self.on_button_released('surface')
        print("Surface button released")

    def on_dive_button_clicked(self):
        self.on_button_clicked('dive')
        print("Dive button clicked")

    def on_dive_button_released(self):
        self.on_button_released('dive')
        print("Dive button released")

    def on_roll_right_button_clicked(self):
        self.on_button_clicked('roll_right')
        print("Roll right button clicked")

    def on_roll_right_button_released(self):
        self.on_button_released('roll_right')
        print("Roll right button released")
    
    def on_roll_left_button_clicked(self):
        self.on_button_clicked('roll_left')
        print("Roll left button clicked")
    
    def on_roll_left_button_released(self):
        self.on_button_released('roll_left')
        print("Roll left button released")

    def on_mode_change_button_clicked(self):
        dialog = ModeChangeDialog()
        dialog.exec_()

    def handle_key_press(self, event):
        if event.isAutoRepeat():
            return False

        key = event.key()
        if key == Qt.Key_Shift:
            self.on_surface_button_clicked()
            return True
        elif key == Qt.Key_Control:
            self.on_dive_button_clicked()
            return True
        elif key == Qt.Key_R:
            self.on_roll_right_button_clicked()
            return True
        elif key == Qt.Key_F:
            self.on_roll_left_button_clicked()
            return True
        elif key == Qt.Key_W:
            self.on_forward_button_clicked()
            return True
        elif key == Qt.Key_S:
            self.on_backward_button_clicked()
            return True
        elif key == Qt.Key_A:
            self.on_left_button_clicked()
            return True
        elif key == Qt.Key_D:
            self.on_right_button_clicked()
            return True
        return False

    def handle_key_release(self, event):
        if event.isAutoRepeat():
            return False

        key = event.key()
        if key == Qt.Key_Shift:
            self.on_surface_button_released()
            return True
        elif key == Qt.Key_Control:
            self.on_dive_button_released()
            return True
        elif key == Qt.Key_R:
            self.on_roll_right_button_released()
            return True
        elif key == Qt.Key_F:
            self.on_roll_left_button_released()
            return True
        elif key == Qt.Key_W:
            self.on_forward_button_released()
            return True
        elif key == Qt.Key_S:
            self.on_backward_button_released()
            return True
        elif key == Qt.Key_A:
            self.on_left_button_released()
            return True
        elif key == Qt.Key_D:
            self.on_right_button_released()
            return True
        return False

    def cleanup(self):
        if self.check_timer.isActive():
            self.check_timer.stop()
        # Đảm bảo luôn gửi STOP khi dọn dẹp
        MAV.send_control_cmd(MAV.STOP)

controller = ButtonController()