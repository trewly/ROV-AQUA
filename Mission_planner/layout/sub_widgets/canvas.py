from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGraphicsView, QGraphicsScene, QPushButton, QMenu, QGraphicsEllipseItem, QGraphicsLineItem
from PyQt5.QtGui import QPen, QColor
from PyQt5.QtCore import Qt

button_style="""
            QPushButton {
                background-color: #DDDDDD;  /* Màu nền trắng */
                color: black;             /* Chữ màu đen */
                font-size: 14px; 
                font-family: 'Roboto', sans-serif;                   
                border: 1px solid black;  /* Viền xám nhạt */
                padding: 8px;               /* Khoảng cách chữ */
            }
            QPushButton:hover {
                background-color: #E3F2FD;  /* Màu xanh nhẹ khi hover */
                border: 2px solid #90CAF9;  /* Viền xanh nhạt hơn */
            }
            QPushButton:pressed {
                background-color: #BBDEFB;  /* Màu xanh đậm khi nhấn */
                border: 2px solid #42A5F5;
            }
        """

class MyWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(800, 500)

        #canvas init
        self.canvas_init()

        #vehicle location track
        

        #canvas button
        self.button_init()

        #bien 
        self.selected_points = []
        self.go_to_point_mode = False  # Chế độ chọn waypoint

    def draw_grid(self, grid_size):
        pen = QPen(QColor(200, 200, 200))  
        pen.setWidth(1)
        for y in range(0, 5000, grid_size):
            self.scene.addLine(0, y,5000, y, pen)
        for x in range(0, 5000, grid_size):
            self.scene.addLine(x, 0, x,5000, pen)

    def canvas_init(self):
        self.canvas = QGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.canvas.setScene(self.scene)
        self.canvas.setDragMode(QGraphicsView.ScrollHandDrag)
        self.canvas.centerOn(2500, 2500)
        self.draw_grid(50)  

    def button_init(self):
        self.wbutton = QPushButton("Waypoint set", self)
        self.wbutton.setStyleSheet(button_style)
        self.wbutton.setFixedSize(120, 50)
        self.wbutton.move(15, 5) 
        self.wbutton.clicked.connect(self.show_waypoint_menu)

        self.waypoint_menu = QMenu(self)
        self.waypoint_menu.addAction("Go to point", lambda: self.select_waypoint_mode("Go to point"))
        self.waypoint_menu.addAction("Pattern", lambda: self.select_waypoint_mode("pattern"))
        #self.waypoint_menu.setStyleSheet(button_style)

        # self.button2d = QPushButton("2D",self)
        # self.button2d.setStyleSheet(button_style)
        # self.button2d.setFixedSize(90, 50)
        # self.button2d.move(600, 5) 

        # self.button3d = QPushButton("3D",self)
        # self.button3d.setStyleSheet(button_style)
        # self.button3d.setFixedSize(90, 50)
        # self.button3d.move(695, 5) 

        #clear button
        self.button_clear = QPushButton("Clear", self)
        self.button_clear.setStyleSheet(button_style)
        self.button_clear.setFixedSize(90, 50)
        self.button_clear.move(150, 5)
        self.button_clear.clicked.connect(self.clear_waypoints)
        self.button_clear.setVisible(False)  

        #upload button
        self.button_uploadmission = QPushButton("upload", self)
        self.button_uploadmission.setStyleSheet(button_style)
        self.button_uploadmission.setFixedSize(90, 50)
        self.button_uploadmission.move(240, 5)
        self.button_uploadmission.clicked.connect(self.upload_waypoints)
        self.button_uploadmission.setVisible(False)  

    def show_waypoint_menu(self):
        button_pos = self.wbutton.mapToGlobal(self.wbutton.rect().bottomLeft())
        self.waypoint_menu.exec_(button_pos)

    def select_waypoint_mode(self, mode):
        if mode == "Go to point":
            self.go_to_point_mode = True
            self.selected_points = []  # Reset danh sách điểm
            self.button_clear.setVisible(True)
            self.button_uploadmission.setVisible(True)  # Hiển thị nút "Clear"
        elif mode == "Pattern":
            print("Pattern mode selected (Chưa triển khai)")

    def clear_waypoints(self):
        """Xóa toàn bộ các điểm đã chọn trên canvas"""
        self.selected_points = []
        self.scene.clear()
        self.draw_grid(50)  # Vẽ lại lưới
        self.go_to_point_mode = False
        self.button_clear.setVisible(False)  # Ẩn nút "Clear"
        self.button_uploadmission.setVisible(False)

    def upload_waypoints(self):
        print(self.selected_points)
        pass

    def mousePressEvent(self, event):
        if self.go_to_point_mode and self.canvas.geometry().contains(event.pos()):
            scene_pos = self.canvas.mapToScene(event.pos() - self.canvas.pos())  # Chuyển đổi tọa độ

            # Vẽ điểm waypoint
            radius = 5
            point = QGraphicsEllipseItem(scene_pos.x() - radius, scene_pos.y() - radius, radius * 2, radius * 2)
            point.setBrush(QColor(255, 0, 0))  # Màu đỏ
            self.scene.addItem(point)

            # Nối đường nếu có ít nhất 2 điểm
            if self.selected_points:
                last_point = self.selected_points[-1]
                line = QGraphicsLineItem(last_point.x(), last_point.y(), scene_pos.x(), scene_pos.y())
                line.setPen(QPen(Qt.blue, 2))  # Vẽ đường màu xanh
                self.scene.addItem(line)

            self.selected_points.append(scene_pos)  # Lưu điểm đã chọn

if __name__ == "__main__":
    app = QApplication([])
    window = MyWidget()
    window.show()
    app.exec_()
