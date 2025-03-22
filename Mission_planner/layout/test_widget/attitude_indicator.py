from PyQt5.QtWidgets import QWidget, QGraphicsView, QGraphicsScene, QVBoxLayout, QSlider, QApplication, QGraphicsItem
from PyQt5.QtGui import QTransform
from PyQt5.QtSvg import QGraphicsSvgItem
from PyQt5.QtCore import Qt
import sys

class QFI_AI(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._originalWidth = 400   
        self._originalHeight = 400
        self._faceZ = 1             
        self._caseZ = 2             
        self._backZ = 0             
        self._ringZ = 3             
        
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        self._view = QGraphicsView(self)
        self._scene = QGraphicsScene(self)
        self._view.setScene(self._scene)
        layout.addWidget(self._view)
        
        # Slider Roll
        self._sliderRoll = QSlider(Qt.Horizontal)
        self._sliderRoll.setRange(0, 45)
        self._sliderRoll.valueChanged.connect(self.updateRoll)
        layout.addWidget(self._sliderRoll)

        # Slider Pitch
        self._sliderPitch = QSlider(Qt.Horizontal)
        self._sliderPitch.setRange(0, 20)
        self._sliderPitch.valueChanged.connect(self.updatePitch)
        layout.addWidget(self._sliderPitch)

        self.init()
    
    def init(self):
        self.reset()

        # Background
        self._itemBack = QGraphicsSvgItem("./layout/resources/attitude_indicator/ai_back.svg")
        self._itemBack.setZValue(self._backZ)
        self._scene.addItem(self._itemBack)
        
        bbox = self._itemBack.boundingRect()
        self._originalAdiCtr = bbox.center()
        self._itemBack.setTransformOriginPoint(self._originalAdiCtr)

        # Face (mặt attitude)
        self._itemFace = QGraphicsSvgItem("./layout/resources/attitude_indicator/ai_face.svg")
        self._itemFace.setZValue(self._faceZ)
        self._scene.addItem(self._itemFace)
        self._itemFace.setTransformOriginPoint(self._originalAdiCtr)
        
        # Ring
        self._itemRing = QGraphicsSvgItem("./layout/resources/attitude_indicator/ai_ring.svg")
        self._itemRing.setZValue(self._ringZ)
        self._scene.addItem(self._itemRing)
        self._itemRing.setTransformOriginPoint(self._originalAdiCtr)

        # Case
        self._itemCase = QGraphicsSvgItem("./layout/resources/attitude_indicator/ai_case.svg")
        self._itemCase.setZValue(self._caseZ)
        self._scene.addItem(self._itemCase)

        self._view.centerOn(self.width() / 2.0, self.height() / 2.0)
        self.updateView()
    
    def reset(self):
        self._scene.clear()
    
    def updateView(self):
        self._view.viewport().update()

    def updateRoll(self, value):
        self.rotateFace(value, self._sliderPitch.value())

    def updatePitch(self, value):
        self.rotateFace(self._sliderRoll.value(), value)

    def rotateFace(self, roll_imu, pitch_imu):
       if 0 <= roll_imu <= 45 or 315 <= roll_imu <= 360:
        roll = roll_imu  # Giữ nguyên roll nếu hợp lệ
        self._itemFace.setRotation(roll)
        pitch = (pitch_imu + 180) % 360 - 180  # Chuyển đổi về -90 đến 90
        if -20 <= pitch <= 20:
            pitch_offset = -pitch * (self._originalHeight / 180)
            self._itemFace.setY(pitch_offset)
        # Cập nhật giao diện
        self.updateView()



