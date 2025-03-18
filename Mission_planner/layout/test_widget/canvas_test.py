from PyQt5.QtWidgets import QApplication, QWidget, QGraphicsView, QGraphicsScene, QPushButton, QGraphicsProxyWidget, QVBoxLayout
from PyQt5.QtGui import QPainter, QPen
from PyQt5.QtCore import Qt

class GridCanvasWidget(QWidget):
    def __init__(self, grid_size=5, parent=None):
        super().__init__(parent)

        # Tạo layout chứa QGraphicsView
        layout = QVBoxLayout(self)
        self.setLayout(layout)

        # Tạo QGraphicsView
        self.view = QGraphicsView(self)
        layout.addWidget(self.view)

        # Tạo QGraphicsScene
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        self.view.setSceneRect(0, 0, 900, 500)

        # Vẽ lưới
        self.draw_grid(grid_size)

        # Thêm button vào scene
        self.add_button(400, 250, "Click Me")

    def draw_grid(self, grid_size_mm):
        grid_size_px = grid_size_mm * 5  # Quy đổi mm -> pixel
        pen = QPen(Qt.gray, 0.5, Qt.DashLine)

        for x in range(0, 901, grid_size_px):
            self.scene.addLine(x, 0, x, 500, pen)
        for y in range(0, 501, grid_size_px):
            self.scene.addLine(0, y, 900, y, pen)

    def add_button(self, x, y, text):
        button = QPushButton(text)
        proxy = QGraphicsProxyWidget()
        proxy.setWidget(button)
        proxy.setPos(x, y)
        self.scene.addItem(proxy)

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = GridCanvasWidget()
    window.resize(200, 200)  # Đặt kích thước của widget
    window.show()
    sys.exit(app.exec_())
