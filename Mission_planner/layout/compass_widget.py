import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtGui import QIcon, QPixmap

class imageViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
    
    def initUI(self):
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('Image Viewer')

        self.label = QLabel(self)
        pixmap = QPixmap('./resources/compass.png')
        self.label.setPixmap(pixmap)
        self.resize(pixmap.width(), pixmap.height())

        self.show()

    


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = imageViewer()
    sys.exit(app.exec_())