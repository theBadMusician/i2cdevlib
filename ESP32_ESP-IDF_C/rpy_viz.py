import sys
import serial
import numpy as np
from PyQt6 import QtWidgets, QtCore
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui

from pyqtgraph.opengl import MeshData


def create_cube_mesh():
    verts = np.array([
        [1, 1, 1],
        [-1, 1, 1],
        [-1, -1, 1],
        [1, -1, 1],
        [1, 1, -1],
        [-1, 1, -1],
        [-1, -1, -1],
        [1, -1, -1]
    ])

    faces = np.array([
        [0, 1, 2], [0, 2, 3],  # Front
        [4, 7, 6], [4, 6, 5],  # Back
        [0, 4, 5], [0, 5, 1],  # Top
        [2, 6, 7], [2, 7, 3],  # Bottom
        [0, 3, 7], [0, 7, 4],  # Right
        [1, 5, 6], [1, 6, 2]   # Left
    ])

    return MeshData(vertexes=verts, faces=faces)


class IMUVisualizer(QtWidgets.QWidget):
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        super().__init__()
        self.setWindowTitle("IMU Orientation Viewer")
        self.resize(800, 600)

        # 3D view
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=10)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.view)
        self.setLayout(layout)

        # Create axis grid
        grid = gl.GLGridItem()
        grid.setSize(10, 10)
        grid.setSpacing(1, 1)
        self.view.addItem(grid)

        # Create a 3D box to represent orientation
        cube_mesh = create_cube_mesh()
        self.cube = gl.GLMeshItem(meshdata=cube_mesh, smooth=False, color=(
            0.2, 0.5, 1, 0.9), shader="shaded", drawEdges=True)
        self.cube.scale(1, 2, 0.2)
        self.view.addItem(self.cube)

        # Setup serial connection
        self.serial = serial.Serial(port, baudrate, timeout=1)

        # Timer to update the cube based on serial data
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_orientation)
        self.timer.start(50)

    def update_orientation(self):
        try:
            line = self.serial.readline().decode().strip()
            if line.startswith("YAW"):
                # Check if the line contains the expected format
                if "PITCH:" not in line or "ROLL:" not in line:
                    print(line)
                    return
                parts = line.replace("YAW:", "").replace(
                    "PITCH:", "").replace("ROLL:", "").split(",")
                yaw, pitch, roll = [float(p.strip()) for p in parts]

                # Convert degrees to radians
                yaw_rad = np.radians(yaw)
                pitch_rad = np.radians(pitch)
                roll_rad = np.radians(roll)

                # Rotation matrices
                Rx = np.array([
                    [1, 0, 0],
                    [0, np.cos(roll_rad), -np.sin(roll_rad)],
                    [0, np.sin(roll_rad), np.cos(roll_rad)]
                ])
                Ry = np.array([
                    [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                    [0, 1, 0],
                    [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
                ])
                Rz = np.array([
                    [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                    [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                    [0, 0, 1]
                ])

                # Combined rotation
                R = Rz @ Ry @ Rx

                # Apply rotation to the cube
                self.cube.resetTransform()
                m = QtGui.QMatrix4x4()
                for i in range(3):
                    col = QtGui.QVector4D(
                        float(R[0, i]),
                        float(R[1, i]),
                        float(R[2, i]),
                        0.0  # fourth component for homogeneous coordinates
                    )
                    m.setColumn(i, col)
                self.cube.setTransform(m)

        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = None
    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])
        port = sys.argv[1]
        win = IMUVisualizer(port=port, baudrate=baudrate)
    elif len(sys.argv) > 1:
        port = sys.argv[1]
        win = IMUVisualizer(port=port)
    else:
        win = IMUVisualizer()
    win.show()
    sys.exit(app.exec())
