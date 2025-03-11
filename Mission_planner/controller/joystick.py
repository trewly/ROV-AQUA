import sys
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt5.QtCore import Qt, QPointF, QTimer, pyqtSignal
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

        self.keys_pressed = set()
        self.keys_pressed = set()
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_knob_position)
        self.timer.start()

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

        if event.key() in {Qt.Key_Left, Qt.Key_Right, Qt.Key_Up, Qt.Key_Down}:
            self.keys_pressed.add(event.key())
            self.update_knob_position()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not self.hasFocus():
            return super().keyReleaseEvent(event)

        if event.key() in {Qt.Key_Left, Qt.Key_Right, Qt.Key_Up, Qt.Key_Down}:
            self.keys_pressed.discard(event.key())
            self.update_knob_position()
        else:
            super().keyReleaseEvent(event)

    def update_knob_position(self):
        x = 0
        y = 0

        if Qt.Key_Left in self.keys_pressed:
            x -= 1
        if Qt.Key_Right in self.keys_pressed:
            x += 1
        if Qt.Key_Up in self.keys_pressed:
            y += 1
        if Qt.Key_Down in self.keys_pressed:
            y -= 1

        self.move_knob(x, y)

    def on_joystick_moved(self, x, y):
        print(f"Joystick X: {x:.2f}, Y: {y:.2f}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    joystick = VirtualJoystick()
    joystick.show()
    sys.exit(app.exec_())