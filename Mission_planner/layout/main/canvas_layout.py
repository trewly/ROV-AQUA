import os
import sys
import math
import time

from PyQt5.QtWidgets import QLabel,QApplication, QWidget, QVBoxLayout, QGraphicsView, QGraphicsScene, QPushButton, QMenu, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsPixmapItem
from PyQt5.QtGui import QPen, QColor, QFont, QFontDatabase, QPixmap, QTransform
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))) 

from Mission_planner.layout.resources.style import canvas_button_style
from Mission_planner.status import pc_status as status
from Mission_planner.utils.system_update_timer import SystemStatusManager
from Mission_planner.status.log_viewer import LogViewer
from Mission_planner.status.state_graph import AttitudePlotter

FONT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "resources", "Orbitron", "static", "Orbitron-Regular.ttf")

class ROVSimulationThread(QThread):
    update_position = pyqtSignal(float, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True
        self.dt = 0.1  # Time step in seconds
        self.velocity = 0  # Forward velocity (m/s)
        self.position_x = 2200  # Initial X position
        self.position_y = 2400  # Initial Y position
        self.angle = 0  # Current heading angle (degrees)
        self.previous_angle = 0  # Previous heading angle

    def run(self):
        # Create timer INSIDE the thread where it will run
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.timer.setInterval(int(self.dt * 1000))  # Convert to milliseconds
        self.timer.start()
        
        # Start event loop
        self.exec_()

    def update_simulation(self):
        if not self.running:
            return
            
        # Read current status
        self.angle = status.read_status("heading")
        self.velocity = status.read_status("vertical_velocity")  # Get velocity directly
        
        # Calculate velocity components based on current heading
        velocity_x = self.velocity * math.cos(math.radians(self.angle))
        velocity_y = self.velocity * math.sin(math.radians(self.angle))
        
        # Update position
        self.position_x += velocity_x * self.dt
        self.position_y += velocity_y * self.dt
        
        # Emit updated position
        self.update_position.emit(self.position_x, self.position_y, self.angle)

    def stop(self):
        self.running = False
        if hasattr(self, 'timer'):
            self.timer.stop()
        self.quit()
        self.wait()

class CanvasWidget(QWidget):
    def __init__(self,status_manager: SystemStatusManager):
        super().__init__()
        self.setFixedSize(860, 545)

        #some info
        self.depth_info = 0
        self.temp_info = 0
        self.myfont = None
        #font setup
        font_id = QFontDatabase.addApplicationFont(FONT_PATH)
        if font_id != -1:
            font_family = QFontDatabase.applicationFontFamilies(font_id)[0]
            self.myfont = QFont(font_family, 10, QFont.Bold)
        else:
            print("Không thể tải font!")

        #canvas init
        self.canvas_init()

        #vehicle location track
        self.vehicle_locate_init()

        self.simulation_thread = ROVSimulationThread()
        self.simulation_thread.update_position.connect(self.update_motion_canvas)
        self.simulation_thread.start()

        #canvas button
        self.button_init()

        #state button 
        self.state_button_init()

        #vehicle status
        self.vehicle_status_init()

        #system info
        self.system_info_init()

        #bien 
        self.selected_points = []
        self.go_to_point_mode = False  # Chế độ chọn waypoint

        #Cap nhat su kien
        self.status_manager = status_manager
        self.status_manager.got_disconnected_info.connect(self.update_vehicle_status)
        self.status_manager.got_temp_depth_info.connect(self.update_temp_depth_info)
        # Đọc trạng thái ban đầu
        self.update_vehicle_status(0)

    #khoi tao canvas voi grid
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
        self.canvas.setStyleSheet("background-color: #F3F3E0;") 
        self.draw_grid(50)  

    #cac nut theo doi trang thai he thong - log viewer, pitch roll yaw
    def state_button_init(self):
        #khoi tao mode button
        self.mode_button = QPushButton("LOG", self)
        self.mode_button.setStyleSheet(canvas_button_style)
        self.mode_button.setFixedSize(110, 40)
        self.mode_button.move(630, 500) 
        self.mode_button.clicked.connect(self.show_log_viewer)
        #khoi tao state button -> bieu do he thogn cac gov raw, pitch, roll
        self.state_button = QPushButton("STATE", self)
        self.state_button.setStyleSheet(canvas_button_style)
        self.state_button.setFixedSize(110, 40)
        self.state_button.move(745, 500) 
        self.state_button.clicked.connect(self.show_state)

    def show_state(self):
        self.state_viewer = AttitudePlotter()  # Gán vào self để không bị thu hồi bộ nhớ
        self.state_viewer.show()

    def show_log_viewer(self):
        self.log_viewer = LogViewer()  # Gán vào self để không bị thu hồi bộ nhớ
        self.log_viewer.show()

    def vehicle_status_init(self):
        # status text
        self.statusLabel = QLabel("Vehicle unconnected", self)
        self.statusLabel.setFont(self.myfont)
        self.statusLabel.setStyleSheet("color: #395B64;")
        self.statusLabel.move(625,18)  
        
        # status dot
        self.statusDot = QLabel(self)
        self.statusDot.setFixedSize(20, 20)
        self.statusDot.setStyleSheet("border-radius: 10px; background-color: gray; border: 1px solid black;")
        self.statusDot.move(600,15)  

    def update_vehicle_status(self,disconnected: bool):
        if disconnected:
            self.statusLabel.setText("Vehicle unconnected")
        else:
            self.statusLabel.setText("Vehicle connected")
        #print("update connected info")

    def system_info_init(self):
        self.infoLabel = QLabel(f"Depth: {self.depth_info}   Temp: {self.temp_info}" , self)
        self.infoLabel.setFont(self.myfont)
        self.infoLabel.setStyleSheet("color: #395B64; font-size: 22px;")
        self.infoLabel.move(15, 510)

    def update_temp_depth_info(self,temp,depth):
        self.temp_info=temp
        self.depth_info=depth
        self.infoLabel.setText(f"Depth: {self.depth_info}   Temp: {self.temp_info}")
        #print("update tempdepth info")

    #phim chuc nang tren canvas
    def button_init(self):
        self.wbutton = QPushButton("WAYPOINT", self)
        self.wbutton.setStyleSheet(canvas_button_style)
        self.wbutton.setFixedSize(160, 50)
        self.wbutton.move(15, 5) 
        self.wbutton.clicked.connect(self.show_waypoint_menu)

        self.waypoint_menu = QMenu(self)
        self.waypoint_menu.addAction("Go to point", lambda: self.select_waypoint_mode("Go to point"))
        self.waypoint_menu.addAction("Pattern", lambda: self.select_waypoint_mode("pattern"))
        self.waypoint_menu.setStyleSheet("""
            QMenu {
                background-color: #2C3333;  /* Màu nền đỏ */
                border: 1px solid #2C3333;  /* Viền đỏ đậm */
                color: white;  /* Màu chữ trắng */
                font-size: 14px;  /* Cỡ chữ */
            }
            
            QMenu::item {
                padding: 8px 20px;
                background-color: transparent;
            }
            
            QMenu::item:selected {
                background-color: #395B64;  /* Màu đỏ đậm khi hover */
            }
        """)

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
        self.button_clear.setStyleSheet(canvas_button_style)
        self.button_clear.setFixedSize(90, 50)
        self.button_clear.move(180, 5)
        self.button_clear.clicked.connect(self.clear_waypoints)
        self.button_clear.setVisible(False)  

        #upload button
        self.button_uploadmission = QPushButton("Upload", self)
        self.button_uploadmission.setStyleSheet(canvas_button_style)
        self.button_uploadmission.setFixedSize(110, 50)
        self.button_uploadmission.move(275, 5)
        self.button_uploadmission.clicked.connect(self.upload_waypoints)
        self.button_uploadmission.setVisible(False)  

    #waypoint
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
        """Xóa toàn bộ các điểm đã chọn trên canvas mà không xóa vehicle_dot"""
        self.selected_points = []

        # Lưu lại vị trí của vehicle_dot trước khi xóa
        vehicle_pos = self.vehicle_dot.pos()

        # Xóa tất cả item trừ vehicle_dot
        items_to_remove = [item for item in self.scene.items() if item is not self.vehicle_dot]
        for item in items_to_remove:
            self.scene.removeItem(item)

        # Vẽ lại lưới trước
        self.draw_grid(50)

        # Đặt lại vehicle_dot lên trên cùng
        self.scene.addItem(self.vehicle_dot)  
        self.vehicle_dot.setPos(vehicle_pos)  # Đặt lại vị trí cũ

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

    def vehicle_locate_init(self):
        self.vehicle_dot = QGraphicsPixmapItem(QPixmap("./layout/resources/vehicle_sym.png"))
        self.scene.addItem(self.vehicle_dot)
        
        # Đặt tâm xoay về trung tâm hình ảnh
        self.vehicle_dot.setTransformOriginPoint(self.vehicle_dot.boundingRect().width() / 2, 
                                                self.vehicle_dot.boundingRect().height() / 2)

        pixmap = self.vehicle_dot.pixmap()
        transform = QTransform().rotate(90)
        rotated_pixmap = pixmap.transformed(transform, Qt.SmoothTransformation)
        
        self.vehicle_dot.setPixmap(rotated_pixmap)  # Cập nhật ảnh đã xoay

        # Đặt vị trí ban đầu
        self.vehicle_dot.setPos(2200, 2400)

    def update_motion_canvas(self,x,y,yaw):
        self.vehicle_dot.setPos(x, y)
        self.vehicle_dot.setTransform(QTransform().rotate(yaw))

        #add tracking
        track_path = QGraphicsEllipseItem(x, y, 3, 3)  # Vẽ hình tròn nhỏ (6x6 pixel)
        track_path.setBrush(Qt.green)  # Đặt màu đỏ cho waypoint
        track_path.setPen(QPen(Qt.NoPen))
        self.scene.addItem(track_path)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     status_manager = SystemStatusManager()  # Tạo đối tượng quản lý trạng thái hệ thống
#     main_window = CanvasWidget(status_manager)
#     main_window.show()
#     sys.exit(app.exec_())