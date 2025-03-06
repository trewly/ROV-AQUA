import sys
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt5.QtCore import Qt, QPointF, pyqtSignal
from PyQt5.QtGui import QBrush, QColor

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

        self.current_x = 0
        self.current_y = 0

        self.joystickMoved.connect(self.on_joystick_moved)

    def move_knob(self, x, y):
        dx = x * self.max_distance
        dy = -y * self.max_distance
        self.knob.setPos(self.center.x() + dx, self.center.y() + dy)
        self.joystickMoved.emit(x, y)

    def reset_knob(self):
        self.knob.setPos(self.center)
        self.joystickMoved.emit(0, 0)

    def keyPressEvent(self, event):
        if not self.hasFocus():
            return super().keyPressEvent(event)

        if event.key() == Qt.Key_Left:
            self.current_x = -1
        elif event.key() == Qt.Key_Right:
            self.current_x = 1
        elif event.key() == Qt.Key_Up:
            self.current_y = 1
        elif event.key() == Qt.Key_Down:
            self.current_y = -1
        else:
            return super().keyPressEvent(event)

        self.move_knob(self.current_x, self.current_y)
    

    def keyReleaseEvent(self, event):
        if not self.hasFocus():
            return super().keyReleaseEvent(event)

        if event.key() in (Qt.Key_Left, Qt.Key_Right):
            self.current_x = 0
        if event.key() in (Qt.Key_Up, Qt.Key_Down):
            self.current_y = 0
        else:
            return super().keyReleaseEvent(event)
        self.move_knob(self.current_x, self.current_y)

    def on_joystick_moved(self, x, y):
        print(f"Joystick X: {x:.2f}, Y: {y:.2f}")