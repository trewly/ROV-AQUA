import sys
import numpy as np
import pyqtgraph.opengl as gl
from stl import mesh
from PyQt5.QtWidgets import QApplication

def load_stl(filename):
    your_mesh = mesh.Mesh.from_file(filename)
    vertices = np.zeros((len(your_mesh.vectors) * 3, 3), dtype=np.float32)
    faces = np.zeros((len(your_mesh.vectors), 3), dtype=np.int32)

    for i, triangle in enumerate(your_mesh.vectors):
        vertices[i * 3:i * 3 + 3] = triangle
        faces[i] = [i * 3, i * 3 + 1, i * 3 + 2]

    return vertices, faces

app = QApplication(sys.argv)
view = gl.GLViewWidget()
view.setWindowTitle("3D Model Viewer")
view.setCameraPosition(distance=10)
view.show()

# Load mô hình STL
vertices, faces = load_stl("Mission_planner\layout\shell_assem.STL")
vertices -= np.mean(vertices, axis=0)  # Dịch toàn bộ mô hình về gốc tọa độ
scale_factor = 0.1  # Thu nhỏ mô hình 50%
vertices *= scale_factor


# Vẽ mô hình 3D
mesh_item = gl.GLMeshItem(vertexes=vertices, faces=faces, shader="shaded", drawEdges=True)
view.addItem(mesh_item)
mesh_item.rotate(90, 1, 0, 0)  # Xoay 90 độ quanh trục X
view.setCameraPosition(distance=80)  # Tăng khoảng cách camera để mô hình trông nhỏ hơn

sys.exit(app.exec_())