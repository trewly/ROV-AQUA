import sys 
import numpy as np
import pyqtgraph.opengl as gl
from stl import mesh
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import QThread, pyqtSignal
import os
#phan gia lap xoay
from PyQt5.QtGui import QMatrix4x4
import time
import random

#phan gia lap xoay
class RotationThread(QThread):
    new_rotation = pyqtSignal(float, float, float)

    def run(self):
        while True:
            # Xoay X từ 0 -> 360
            for angle in range(0, 361, 5):
                self.new_rotation.emit(angle, 0, 0)
                time.sleep(0.05)

            # Xoay Y từ 0 -> 360
            for angle in range(0, 361, 5):
                self.new_rotation.emit(360, angle, 0)
                time.sleep(0.05)

            # Xoay Z từ 0 -> 360
            for angle in range(0, 361, 5):
                self.new_rotation.emit(360, 360, angle)
                time.sleep(0.05)

            # Reset về 0 và lặp lại
            self.new_rotation.emit(0, 0, 0)
            time.sleep(0.5)  # Nghỉ 0.5s trước khi lặp lại

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
        layout = QVBoxLayout()
        self.setLayout(layout)

        self.view = gl.GLViewWidget()
        layout.addWidget(self.view)
        self.view.setCameraPosition(distance=80)
        self.view.setBackgroundColor((240, 240, 255))

        self.mesh_item = None
        self.stl_loader = STLLoaderThread(stl_file)
        self.stl_loader.finished.connect(self.on_stl_loaded)
        self.stl_loader.start()

        self.add_axes()
        self.add_arrows()

        # Khởi động luồng đọc dữ liệu xoay giả lập
        self.rotation_thread = RotationThread()
        self.rotation_thread.new_rotation.connect(self.rotate_model)  # Kết nối tín hiệu
        self.rotation_thread.start()

    def on_stl_loaded(self, vertices, faces):
        vertices -= np.mean(vertices, axis=0)
        scale_factor = 0.1
        vertices *= scale_factor
        vertices -= np.mean(vertices, axis=0)

        mesh_color = (1, 1, 1, 0.1)
        self.mesh_item = gl.GLMeshItem(vertexes=vertices, faces=faces, shader="normalColor", color=mesh_color)
        self.mesh_item.rotate(90, 1, 0, 0)
        self.mesh_item.rotate(180, 0, 0, 1)
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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    stl_path = os.path.join(script_dir, "shell_assem.STL")

    viewer = STLViewerWidget(stl_path)
    viewer.show()

    sys.exit(app.exec_())