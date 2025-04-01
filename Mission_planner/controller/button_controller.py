from PyQt5.QtCore import Qt, QObject, pyqtSignal
import threading
import time

from .mode_controller import ModeChangeDialog
from Mission_planner.connection.pc_mavlink import MAV
from Mission_planner.utils.pc_logger import LOG

class ControlStateChecker(threading.Thread):    
    def __init__(self, controller, interval=0.05):
        super().__init__(daemon=True)
        self.controller = controller
        self.interval = interval
        self.running = True
        
    def run(self):
        while self.running:
            time.sleep(self.interval)
            
            if not self.running:
                break
            self.controller.check_state_signal.emit()
    
    def stop(self):
        self.running = False
        if self.is_alive():
            self.join(timeout=1.0)

class Controller(QObject):
    check_state_signal = pyqtSignal()
    
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
        super().__init__()
        self.active_buttons = set()
        self.last_state = {}
        self.last_active_buttons = set()
        self.should_send_stop = False
        self.stop_sent = False
        
        self.check_state_signal.connect(self.check_control_state)
        
        self.state_checker = ControlStateChecker(self)
        self.state_checker.start()
    
    def check_control_state(self):
        try:
            active_motion = any(btn in self.MOTION_BUTTONS for btn in self.active_buttons)
            active_vertical = any(btn in self.VERTICAL_BUTTONS for btn in self.active_buttons)
            active_roll = any(btn in self.ROLL_BUTTONS for btn in self.active_buttons)
            
            current_state = {
                'motion': active_motion,
                'vertical': active_vertical,
                'roll': active_roll
            }
            
            buttons_changed = self.last_active_buttons != self.active_buttons
            
            active_buttons = len(self.active_buttons) > 0
            
            if self.last_state != current_state or buttons_changed:
                LOG.debug(f"Control state changed: {self.active_buttons}")
                
                if not active_buttons and not self.stop_sent:
                    LOG.info("No buttons active - sending STOP command")
                    MAV.send_control_cmd(MAV.STOP)
                    self.stop_sent = True
                elif active_buttons:
                    self.stop_sent = False
                
                self.last_state = current_state.copy()
                self.last_active_buttons = self.active_buttons.copy()
        except Exception as e:
            LOG.error(f"Error in check_control_state: {e}")
    
    def on_button_clicked(self, button_type):
        self.stop_sent = False
        
        self.active_buttons.add(button_type)
        
        try:
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
        except Exception as e:
            LOG.error(f"Error sending control command for {button_type}: {e}")
    
    def on_button_released(self, button_type):
        if button_type in self.active_buttons:
            self.active_buttons.remove(button_type)
        
        try:
            if button_type in self.VERTICAL_BUTTONS or button_type in self.ROLL_BUTTONS:
                MAV.send_control_cmd(MAV.STOP)
                self.stop_sent = True
            
            if not self.active_buttons and not self.stop_sent:
                LOG.info("All buttons released - sending STOP command")
                MAV.send_control_cmd(MAV.STOP)
                self.stop_sent = True
            
            self.check_control_state()
        except Exception as e:
            LOG.error(f"Error in on_button_released for {button_type}: {e}")
    
    def on_forward_button_clicked(self):
        self.on_button_clicked('forward')
        LOG.debug("Forward button clicked")
    
    def on_forward_button_released(self):
        self.on_button_released('forward')
        LOG.debug("Forward button released")
    
    def on_backward_button_clicked(self):
        self.on_button_clicked('backward')
        LOG.debug("Backward button clicked")
    
    def on_backward_button_released(self):
        self.on_button_released('backward')
        LOG.debug("Backward button released")

    def on_left_button_clicked(self):
        self.on_button_clicked('left')
        LOG.debug("Left button clicked")

    def on_left_button_released(self):
        self.on_button_released('left')
        LOG.debug("Left button released")
    
    def on_right_button_clicked(self):
        self.on_button_clicked('right')
        LOG.debug("Right button clicked")

    def on_right_button_released(self):
        self.on_button_released('right')
        LOG.debug("Right button released")
        
    def on_surface_button_clicked(self):
        self.on_button_clicked('surface')
        LOG.debug("Surface button clicked")

    def on_surface_button_released(self):
        self.on_button_released('surface')
        LOG.debug("Surface button released")

    def on_dive_button_clicked(self):
        self.on_button_clicked('dive')
        LOG.debug("Dive button clicked")

    def on_dive_button_released(self):
        self.on_button_released('dive')
        LOG.debug("Dive button released")

    def on_roll_right_button_clicked(self):
        self.on_button_clicked('roll_right')
        LOG.debug("Roll right button clicked")

    def on_roll_right_button_released(self):
        self.on_button_released('roll_right')
        LOG.debug("Roll right button released")
    
    def on_roll_left_button_clicked(self):
        self.on_button_clicked('roll_left')
        LOG.debug("Roll left button clicked")
    
    def on_roll_left_button_released(self):
        self.on_button_released('roll_left')
        LOG.debug("Roll left button released")

    def on_mode_change_button_clicked(self):
        dialog = ModeChangeDialog()
        dialog.exec_()

    def handle_key_press(self, event):
        if event.isAutoRepeat():
            return False

        key = event.key()
        if key in self.KEY_MAPPINGS:
            button_type = self.KEY_MAPPINGS[key]
            method_name = f"on_{button_type}_button_clicked"
            if hasattr(self, method_name):
                getattr(self, method_name)()
                return True
        return False

    def handle_key_release(self, event):
        if event.isAutoRepeat():
            return False

        key = event.key()
        if key in self.KEY_MAPPINGS:
            button_type = self.KEY_MAPPINGS[key]
            method_name = f"on_{button_type}_button_released"
            if hasattr(self, method_name):
                getattr(self, method_name)()
                return True
        return False

    def cleanup(self):
        if hasattr(self, 'state_checker'):
            self.state_checker.stop()
        
        try:
            MAV.send_control_cmd(MAV.STOP)
        except Exception as e:
            LOG.error(f"Error sending final STOP command: {e}")

ButtonController = Controller()