from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt5.QtCore import Qt, QPointF, pyqtSignal
from PyQt5.QtGui import QBrush, QColor
from communication.pc_mavlink import MAV

class VirtualJoystick(QGraphicsView):
    joystickMoved = pyqtSignal(float, float)
    
    STOP = (0, 0)
    RIGHT = (1, 0)
    LEFT = (-1, 0)
    FORWARD = (0, 1)
    BACKWARD = (0, -1)
    
    KEY_MAP = {
        Qt.Key_A: LEFT,
        Qt.Key_D: RIGHT,
        Qt.Key_W: FORWARD,
        Qt.Key_S: BACKWARD
    }
    
    DIRECTION_TO_COMMAND = {
        STOP: (MAV.STOP, "STOP"),
        RIGHT: (MAV.RIGHT, "RIGHT"),
        LEFT: (MAV.LEFT, "LEFT"),
        FORWARD: (MAV.FORWARD, "FORWARD"),
        BACKWARD: (MAV.BACKWARD, "BACKWARD")
    }

    def __init__(self, parent=None, size=180, knob_size=50, max_distance=60):
        super().__init__(parent)
        self.size = size
        self.knob_size = knob_size
        self.max_distance = max_distance
        self.pressed_keys_order = []
        
        self._setup_ui()
        self.joystickMoved.connect(self.on_joystick_moved)

    def _setup_ui(self):
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setFixedSize(self.size, self.size)
        self.setStyleSheet("background: transparent;")
        self.setFrameShape(QGraphicsView.NoFrame)

        base_size = self.size - 10
        base_position = 5
        
        self.base = QGraphicsEllipseItem(0, 0, base_size, base_size)
        self.base.setBrush(QBrush(QColor(0, 0, 0, 0)))
        self.base.setPen(QColor("#F3F3E0"))
        self.base.setPos(base_position, base_position)
        self.scene.addItem(self.base)

        self.knob = QGraphicsEllipseItem(0, 0, self.knob_size, self.knob_size)
        self.knob.setBrush(QBrush(QColor("#F3F3E0")))
        self.center_position = (self.size - self.knob_size) / 2
        self.knob.setPos(self.center_position, self.center_position)
        self.scene.addItem(self.knob)

        self.center = QPointF(self.center_position, self.center_position)

    def move_knob(self, x, y):
        dx = x * self.max_distance
        dy = -y * self.max_distance
        self.knob.setPos(self.center.x() + dx, self.center.y() + dy)
        self.joystickMoved.emit(x, y)

    def keyPressEvent(self, event):
        if not self.hasFocus():
            return super().keyPressEvent(event)

        if event.isAutoRepeat():
            return

        key = event.key()
        if key in self.KEY_MAP:
            if key not in self.pressed_keys_order:
                self.pressed_keys_order.append(key)
                self.update_knob_position()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        key = event.key()
        if key in self.KEY_MAP:
            if key in self.pressed_keys_order:
                self.pressed_keys_order.remove(key)

            if not self.pressed_keys_order:
                self.move_knob(0, 0)
            else:
                self.update_knob_position()
        else:
            super().keyReleaseEvent(event)

    def update_knob_position(self):
        if not self.pressed_keys_order:
            self.move_knob(0, 0)
            return

        key = self.pressed_keys_order[-1]
        x, y = self.KEY_MAP.get(key, (0, 0))
        self.move_knob(x, y)

    def on_joystick_moved(self, x, y):
        direction = (x, y)
        if direction in self.DIRECTION_TO_COMMAND:
            command, label = self.DIRECTION_TO_COMMAND[direction]
            print(label)
            MAV.send_control_cmd(command)