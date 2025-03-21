import sys 
import numpy as np
import pyqtgraph.opengl as gl
from stl import mesh
from PyQt5.QtWidgets import QLabel, QApplication, QWidget,QFrame, QGraphicsScene,QGraphicsItem,QGraphicsView,QVBoxLayout, QHBoxLayout,QGraphicsItem
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QTransform
from PyQt5.QtCore import Qt
from PyQt5.QtSvg import QGraphicsSvgItem
import os
from scipy.spatial.transform import Rotation as R
#phan gia lap xoay
from PyQt5.QtGui import QMatrix4x4
import time
import random

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))) 

#from Mission_planner.controller import state_status #phan dung de doc du lieu

#phan gia lap xoay
class RotationThread(QThread):
    new_rotation = pyqtSignal(float, float, float)  # Signal gửi 3 giá trị (pitch, roll, yaw)

    def run(self):
        while True:
            #test ngau nhien
            # # ngau nhien roll,pitch,yaw
            # pitch = random.uniform(-20,20) 
            # roll = random.uniform(-90,90)   
            # yaw = random.uniform(0,360)    
            
            # self.new_rotation.emit(roll,pitch,yaw)
            # print(roll,pitch,yaw)
            # time.sleep(2)  
            
            #test goc pitch
            for pitch in range(-90, 90, 5):  
                self.new_rotation.emit(0, pitch, 0)  
                #print(f"Pitch: {pitch}")  
                time.sleep(0.05)  
            
            #test goc roll
            # for roll in range(-90, 90, 5):  
            #     self.new_rotation.emit(roll, 0, 0)  
            #     print(f"Roll: {roll}")  
            #     time.sleep(0.05) 

            #test goc yaw
            # for yaw in range(0,360, 5):  
            #     self.new_rotation.emit(0,0,yaw)  
            #     print(f"Yaw: {yaw}")  
            #     time.sleep(0.05) 

            # Reset về 0 và lặp lại
            self.new_rotation.emit(0, 0, 0)
            time.sleep(0.5)  

class STLLoaderThread(QThread):
    finished = pyqtSignal(np.ndarray, np.ndarray)

    def __init__(self, stl_file):
        super().__init__()
        self.stl_file = stl_file

    def run(self):
        your_mesh = mesh.Mesh.from_file(self.stl_file)
        vertices = np.zeros((len(your_mesh.vectors) * 3, 3), dtype=np.float32)
        faces = np.zeros((len(your_mesh.vectors), 3), dtype=np.int32)

        for i, triangle in enumerate(your_mesh.vectors):
            vertices[i * 3:i * 3 + 3] = triangle
            faces[i] = [i * 3, i * 3 + 1, i * 3 + 2]

        self.finished.emit(vertices, faces)

class STLViewerWidget(QWidget):
    def __init__(self, stl_file):
        super().__init__()
        self.setFixedSize(960,450)
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.setLayout(layout)

        #them widget opengl chua mohinh3d
        self.view = gl.GLViewWidget()
        layout.addWidget(self.view)
        self.view.setCameraPosition(distance=80)
        self.view.setBackgroundColor((255, 255, 255))
        self.view.setFixedSize(850, 450)
        self.mesh_item = None
        self.stl_loader = STLLoaderThread(stl_file)
        self.stl_loader.finished.connect(self.on_stl_loaded)
        self.stl_loader.start()

        #them truc the gioi
        self.add_axes()
        self.add_arrows()

        #layout chua instrument
        instrument_widget=QWidget()
        instrument_layout= QVBoxLayout(instrument_widget)
        #them instrument vao layout chinh
        layout.addWidget(instrument_widget)
        

        self._scaleX = 0.8
        self._scaleY = 0.8
        #them widget compass    

        self.compass_faceZ = 1
        self.compass_caseZ = 2  
        self.compass_view = QGraphicsView(self)
        self.compass_view.setFixedSize(250, 250) 
        self.compass_screen = QGraphicsScene(self)
        self.compass_view.setScene(self.compass_screen)
        instrument_layout.addWidget(self.compass_view)
        self.compass_init()

        self.compass_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.compass_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.compass_view.setFrameShape(QFrame.NoFrame)
        
        #them widget attitude indicator

        self.ai_faceZ = 1             
        self.ai_caseZ = 2             
        self.ai_backZ = 0             
        self.ai_ringZ = 3      
        self.ai_view = QGraphicsView(self)
        self.ai_view.setFixedSize(250,250)
        self.ai_screen=QGraphicsScene(self)
        self.ai_view.setScene(self.ai_screen)
        instrument_layout.addWidget(self.ai_view)
        self.attitude_indicator_init()

        self.ai_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.ai_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.ai_view.setFrameShape(QFrame.NoFrame)

        # Khởi động luồng đọc dữ liệu xoay giả lập
        self.rotation_thread = RotationThread()
        self.rotation_thread.new_rotation.connect(self.rotate_model)  # xoay theo truc the gioi
        self.rotation_thread.new_rotation.connect(self.attitude_indicator_position_change)  # xoay pitch roll
        self.rotation_thread.new_rotation.connect(self.compass_position_change)  # xoay compass
        self.rotation_thread.start()

    #ham khoi tao, chuc nang mo hinh
    def on_stl_loaded(self, vertices, faces):
        vertices -= np.mean(vertices, axis=0)  # Căn giữa
        scale_factor = 0.1
        vertices *= scale_factor

        # Xoay dữ liệu gốc
        rot_x = R.from_euler('x', 90, degrees=True).as_matrix()
        rot_z = R.from_euler('z', 180, degrees=True).as_matrix()

        vertices = vertices @ rot_x.T
        vertices = vertices @ rot_z.T

        # Tạo MeshItem với dữ liệu đã xoay
        mesh_color = (1, 1, 1, 0.1)
        self.mesh_item = gl.GLMeshItem(vertexes=vertices, faces=faces, shader="normalColor", color=mesh_color)

        self.view.addItem(self.mesh_item)
        
    def add_axes(self):
        axis_length = 40
        axis_width = 5

        x_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [axis_length, 0, 0]]),
            color=(1, 0, 0, 1), width=axis_width, antialias=True
        )
        self.view.addItem(x_axis)

        y_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, axis_length, 0]]),
            color=(0, 1, 0, 1), width=axis_width, antialias=True
        )
        self.view.addItem(y_axis)

        z_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 0, axis_length]]),
            color=(0, 0, 1, 1), width=axis_width, antialias=True
        )
        self.view.addItem(z_axis)

    def add_arrows(self):
        axis_length = 40

        arrow_X = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=20, radius=[0, 2.5], length=5),
            smooth=True,
            color=(1, 0, 0, 1),
            shader="shaded"
        )
        arrow_X.rotate(-90, 0, 1, 0)
        arrow_X.translate(axis_length + 2, 0, 0)
        self.view.addItem(arrow_X)

        arrow_Y = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=20, radius=[0, 2.5], length=5),
            smooth=True,
            color=(0, 1, 0, 1),
            shader="shaded"
        )
        arrow_Y.rotate(90, 1, 0, 0)
        arrow_Y.translate(0, axis_length + 2, 0)
        self.view.addItem(arrow_Y)

        arrow_Z = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=20, radius=[0, 2.5], length=5),
            smooth=True,
            color=(0, 0, 1, 1),
            shader="shaded"
        )
        arrow_Z.translate(0, 0, -axis_length - 2)
        arrow_Z.rotate(180, 1, 0, 0)
        self.view.addItem(arrow_Z)
    
    def rotate_model(self, x_angle, y_angle, z_angle):
        if not self.mesh_item:
            return

        transform = QMatrix4x4()
        transform.rotate(x_angle, 1, 0, 0)  # Xoay quanh trục X
        transform.rotate(y_angle, 0, 1, 0)  # Xoay quanh trục Y
        transform.rotate(z_angle, 0, 0, 1)  # Xoay quanh trục Z

        self.mesh_item.setTransform(transform)  # Cập nhật mô hình

    #ham khoi tao, chuc nang compass
    def compass_init(self):

        self.compass_reset()

        self.cp_itemFace = QGraphicsSvgItem("./layout/resources/compass/hi_face.svg")
        self.cp_itemFace.setCacheMode(QGraphicsItem.NoCache)
        self.cp_itemFace.setZValue(self.compass_faceZ)
        self.cp_itemFace.setTransform(QTransform().scale(self._scaleX, self._scaleY), True)
        self.compass_screen.addItem(self.cp_itemFace)

        bbox = self.cp_itemFace.boundingRect()
        self.cp_originalHsiCtr = bbox.center()

        # Đặt điểm xoay đúng vị trí
        self.cp_itemFace.setTransformOriginPoint(self.cp_originalHsiCtr)

        self.cp_itemCase = QGraphicsSvgItem("./layout/resources/compass/hi_case.svg")
        self.cp_itemCase.setCacheMode(QGraphicsItem.NoCache)
        self.cp_itemCase.setZValue(self.compass_caseZ)
        self.cp_itemCase.setTransform(QTransform().scale(self._scaleX, self._scaleY), True)
        self.cp_itemCase.setTransformOriginPoint(self.cp_originalHsiCtr)
        self.compass_screen.addItem(self.cp_itemCase)

        self.compass_view.centerOn(self.width() / 2.0, self.height() / 2.0)
        self.update_compass_view()
    
    def update_compass_view(self):
        self.compass_view.viewport().update()

    def compass_reset(self):
        self.compass_screen.clear()

    def compass_position_change(self, roll, pitch, yaw):
        self.cp_itemCase.setRotation(yaw)

    #ham khoi tao, chuc nang attitude indicator
    def attitude_indicator_init(self):
        self.attitude_reset()

        # **Background**
        self.ai_itemBack = QGraphicsSvgItem("./layout/resources/attitude_indicator/ai_back.svg")
        self.ai_itemBack.setZValue(self.ai_backZ)
        self.ai_itemBack.setTransform(QTransform().scale(self._scaleX, self._scaleY), True)
        self.ai_screen.addItem(self.ai_itemBack)

        bbox = self.ai_itemBack.boundingRect()
        self.ai_originalAdiCtr = bbox.center()
        self.ai_itemBack.setTransformOriginPoint(self.ai_originalAdiCtr)

        # **Face (mặt attitude)**
        self.ai_itemFace = QGraphicsSvgItem("./layout/resources/attitude_indicator/ai_face.svg")
        self.ai_itemFace.setZValue(self.ai_faceZ)
        self.ai_itemFace.setTransform(QTransform().scale(self._scaleX, self._scaleY), True)
        self.ai_screen.addItem(self.ai_itemFace)
        self.ai_itemFace.setTransformOriginPoint(self.ai_originalAdiCtr)

        # **Ring**
        self.ai_itemRing = QGraphicsSvgItem("./layout/resources/attitude_indicator/ai_ring.svg")
        self.ai_itemRing.setZValue(self.ai_ringZ)
        self.ai_itemRing.setTransform(QTransform().scale(self._scaleX, self._scaleY), True)
        self.ai_screen.addItem(self.ai_itemRing)
        self.ai_itemRing.setTransformOriginPoint(self.ai_originalAdiCtr)

        # **Case**
        self.ai_itemCase = QGraphicsSvgItem("./layout/resources/attitude_indicator/ai_case.svg")
        self.ai_itemCase.setZValue(self.ai_caseZ)
        self.ai_itemCase.setTransform(QTransform().scale(self._scaleX, self._scaleY), True)
        self.ai_screen.addItem(self.ai_itemCase)

        # **Căn giữa lại**
        self.ai_view.centerOn(self.width() / 2.0, self.height() / 2.0)

        self.update_attitude_view()

    def update_attitude_view(self):
        self.ai_view.viewport().update()
    
    def attitude_reset(self):
        self.ai_screen.clear()

    def attitude_indicator_position_change(self, roll, pitch, yaw):
        #dieu khien roll
        if roll < -90:
            roll=-90
        if roll > 90:
            roll=90
        self.ai_itemFace.setRotation(roll)
        #dieu khien pitch
        if pitch > 20:
            pitch = 20
        elif pitch < -20:  
            pitch = -20
        self.ai_itemFace.setY(pitch * 2 * self._scaleY)
        # Cập nhật giao diện
        self.update_attitude_view()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    stl_path = os.path.join(script_dir, "resources\\shell_assem.STL")

    viewer = STLViewerWidget(stl_path)
    viewer.show()

    sys.exit(app.exec_())