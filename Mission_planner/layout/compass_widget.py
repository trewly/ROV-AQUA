from PyQt5.QtWidgets import QWidget, QGraphicsView, QGraphicsScene, QVBoxLayout, QGraphicsItem, QApplication, QSlider
from PyQt5.QtSvg import QGraphicsSvgItem
from PyQt5.QtGui import QTransform
from PyQt5.QtCore import Qt
import sys

class QFI_HI(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self._faceZ = 1             # Z-value của face
        self._caseZ = 2             # Z-value của case

        layout = QVBoxLayout()
        self.setLayout(layout)
        
        self._view = QGraphicsView(self)
        self._scene = QGraphicsScene(self)
        self._view.setScene(self._scene)
        layout.addWidget(self._view)
        
        #them slider 
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, 360)
        self._slider.valueChanged.connect(self.rotateFace)
        layout.addWidget(self._slider)

        self.init()
    
    def init(self):
        self._scaleX = 1
        self._scaleY = 1


        self.reset()

        self._itemFace = QGraphicsSvgItem("./layout/resources/compass/hi_face.svg")
        self._itemFace.setCacheMode(QGraphicsItem.NoCache)
        self._itemFace.setZValue(self._faceZ)
        self._itemFace.setTransform(QTransform().scale(self._scaleX, self._scaleY), True)
        self._scene.addItem(self._itemFace)

        bbox = self._itemFace.boundingRect()
        self._originalHsiCtr = bbox.center()

        # Đặt điểm xoay đúng vị trí
        self._itemFace.setTransformOriginPoint(self._originalHsiCtr)

        self._itemCase = QGraphicsSvgItem("./layout/resources/compass/hi_case.svg")
        self._itemCase.setCacheMode(QGraphicsItem.NoCache)
        self._itemCase.setZValue(self._caseZ)
        self._itemCase.setTransform(QTransform().scale(self._scaleX, self._scaleY), True)
        self._itemCase.setTransformOriginPoint(self._originalHsiCtr)
        self._scene.addItem(self._itemCase)

        self._view.centerOn(self.width() / 2.0, self.height() / 2.0)
        self.updateView()
    
    def reset(self):
        self._scene.clear()
    
    def updateView(self):
        self._view.viewport().update()
    
    def rotateFace(self, angle):
        self._itemCase.setRotation(angle)
        print(angle)
        
    
app = QApplication(sys.argv)
hsi = QFI_HI()
hsi.show()
sys.exit(app.exec_())