import sys
# import os

# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtCore import Qt, QTimer

from Mission_planner.controller.joystick import VirtualJoystick
from Mission_planner.controller.video import VideoThread
from Mission_planner.controller import control_buttons as buttons_controller

class Ui_Form(QWidget):
    def __init__(self):
        super().__init__()
        self.shift_timer = QTimer(self)
        self.ctrl_timer = QTimer(self)
        self.light_on = False

        self.video_label = QLabel(self)
        self.video_label.setGeometry(0, 0, 1500, 500)

        self.overlay_widget = QWidget(self)
        self.overlay_widget.setGeometry(0, 0, 1500, 500)
        self.overlay_widget.setAttribute(Qt.WA_TranslucentBackground)
        self.overlay_widget.setWindowFlags(Qt.FramelessWindowHint)

        self.setupUi(self.overlay_widget)

        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.video_thread.start()

    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1500, 500)

        self.surface_button = QtWidgets.QPushButton(Form)
        self.surface_button.setGeometry(QtCore.QRect(1350, 0, 121, 51))
        self.surface_button.setObjectName("surface_button")
        self.surface_button.clicked.connect(buttons_controller.on_surface_button_clicked)

        self.dive_button = QtWidgets.QPushButton(Form)
        self.dive_button.setGeometry(QtCore.QRect(1350, 60, 121, 51))
        self.dive_button.setObjectName("dive_button")
        self.dive_button.clicked.connect(buttons_controller.on_dive_button_clicked)

        self.mode_chang_button = QtWidgets.QPushButton(Form)
        self.mode_chang_button.setGeometry(QtCore.QRect(1350, 120, 121, 51))
        self.mode_chang_button.setObjectName("mode_chang_button")
        self.mode_chang_button.clicked.connect(buttons_controller.on_mode_change_button_clicked)

        self.light_button = QtWidgets.QPushButton(Form)
        self.light_button.setGeometry(QtCore.QRect(1350, 180, 121, 51))
        self.light_button.setObjectName("light_button")
        self.light_button.clicked.connect(buttons_controller.on_light_button_clicked)

        self.joystick = VirtualJoystick(Form)
        self.joystick.setGeometry(QtCore.QRect(10, 350, 150, 150))

        self.shift_timer.timeout.connect(self.surface_button.click)
        self.ctrl_timer.timeout.connect(self.dive_button.click)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)


    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.surface_button.setText(_translate("Form", "Surface"))
        self.dive_button.setText(_translate("Form", "Dive"))
        self.mode_chang_button.setText(_translate("Form", "MODE CHANGE"))
        self.light_button.setText(_translate("Form", "LIGHT"))

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Shift and not self.shift_timer.isActive():
            self.shift_timer.start(100)
        elif event.key() == Qt.Key_Control and not self.ctrl_timer.isActive():
            self.ctrl_timer.start(100)
        elif event.key() == Qt.Key_L:
            self.light_button.click()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Shift:
            self.shift_timer.stop()
        elif event.key() == Qt.Key_Control:
            self.ctrl_timer.stop()
        else:
            super().keyReleaseEvent(event)

    def update_image(self, qt_image):
        scaled_image = qt_image.scaled(self.video_label.size(), Qt.KeepAspectRatio)
        self.video_label.setPixmap(QtGui.QPixmap.fromImage(scaled_image))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    MainWindow = Ui_Form()
    MainWindow.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())