import sys
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt5.QtCore import Qt, QPointF, pyqtSignal
from PyQt5.QtGui import QBrush, QColor
from Mission_planner.communication.pc_mavlink import MAV

class VirtualJoystick(QGraphicsView):
    joystickMoved = pyqtSignal(float, float)

    def __init__(self, parent=None, size=150, knob_size=40, max_distance=50):
        super().__init__(parent)
        self.size = size
        self.knob_size = knob_size
        self.max_distance = max_distance

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setFixedSize(size, size)
        self.setStyleSheet("background: transparent;")
        self.setFrameShape(QGraphicsView.NoFrame)

        self.base = QGraphicsEllipseItem(0, 0, size - 25, size - 25)
        self.base.setBrush(QBrush(QColor(0, 0, 0, 0)))
        self.base.setPen(QColor(100, 100, 100, 150))
        self.base.setPos(12.5, 12.5)
        self.scene.addItem(self.base)

        self.knob = QGraphicsEllipseItem(0, 0, knob_size, knob_size)
        self.knob.setBrush(QBrush(QColor(0, 0, 255, 150)))
        self.knob.setPos((size - knob_size) / 2, (size - knob_size) / 2)
        self.scene.addItem(self.knob)

        self.center = QPointF((size - knob_size) / 2, (size - knob_size) / 2)

        self.pressed_keys_order = []
        
        self.joystickMoved.connect(self.on_joystick_moved)

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
        if key in {Qt.Key_Left, Qt.Key_Right, Qt.Key_Up, Qt.Key_Down}:
            if key not in self.pressed_keys_order:
                self.pressed_keys_order.append(key)
                self.update_knob_position()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        key = event.key()

        if key in {Qt.Key_Left, Qt.Key_Right, Qt.Key_Up, Qt.Key_Down}:
            if key in self.pressed_keys_order:
                self.pressed_keys_order.remove(key)

            if not any(k in {Qt.Key_Left, Qt.Key_Right, Qt.Key_Up, Qt.Key_Down} for k in self.pressed_keys_order):
                self.move_knob(0, 0)
        else:
            super().keyReleaseEvent(event)

    def update_knob_position(self):
        if len(self.pressed_keys_order) == 0:
            self.move_knob(0, 0)
            return

        key = self.pressed_keys_order[0]
        x, y = 0, 0

        if key == Qt.Key_Left:
            x = -1
        elif key == Qt.Key_Right:
            x = 1
        elif key == Qt.Key_Up:
            y = 1
        elif key == Qt.Key_Down:
            y = -1

        self.move_knob(x, y)

    def on_joystick_moved(self, x, y):
        if x == 0 and y == 0:
            print("STOP")
            MAV.send_control_cmd(MAV.STOP)
        elif x == 1 and y == 0:
            print("RIGHT")
            MAV.send_control_cmd(MAV.RIGHT)
        elif x == -1 and y == 0:
            print("LEFT")
            MAV.send_control_cmd(MAV.LEFT)
        elif x == 0 and y == 1:
            print("FORWARD")
            MAV.send_control_cmd(MAV.FORWARD)
        elif x == 0 and y == -1:
            print("BACKWARD")
            MAV.send_control_cmd(MAV.BACKWARD)

