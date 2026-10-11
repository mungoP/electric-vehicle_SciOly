import serial
import serial.tools.list_ports
import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

BAUD = 115200

PORT = "/dev/cu.usbmodem1201"
ser = serial.Serial(PORT, BAUD)

# Enable interactive mode
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d') # Matplotlib axis object (safe from overwriting)

points = np.array([
    [-1, -0.5, -0.2], [1, -0.5, -0.2], [1, 0.5, -0.2], [-1, 0.5, -0.2],
    [-1, -0.5,  0.2], [1, -0.5,  0.2], [1, 0.5,  0.2], [-1, 0.5,  0.2]
])

def rotate_x(pts, angle):
    rad = np.radians(angle)
    c, s = np.cos(rad), np.sin(rad)
    return pts.dot(np.array([[1, 0, 0], [0, c, -s], [0, s, c]]).T)

def rotate_y(pts, angle):
    rad = np.radians(angle)
    c, s = np.cos(rad), np.sin(rad)
    return pts.dot(np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]).T)

def rotate_z(pts, angle):
    rad = np.radians(angle)
    c, s = np.cos(rad), np.sin(rad)
    return pts.dot(np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T)

roll, pitch, yaw = 0.0, 0.0, 0.0

print("Visualizer running and parsing IMU data. Press Ctrl+C to exit.")

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        # Use accel_x instead of ax to keep the Matplotlib axis safe
        match = re.search(r'Ax:\s*(-?\d+).*Ay:\s*(-?\d+).*Az:\s*(-?\d+).*Gx:\s*(-?\d+).*Gy:\s*(-?\d+).*Gz:\s*(-?\d+)', line)
        if match:
            accel_x, accel_y, accel_z, gx, gy, gz = map(int, match.groups())
            
            # Scale down raw gyro values to drive rotation smoothly
            roll += -1 * (gy / 131.0) * 0.05
            pitch += (gx / 131.0) * 0.05
            yaw += (gz / 131.0) * 0.05

        ax.cla()
        ax.set_xlim([-3, 3])
        ax.set_ylim([-3, 3])
        ax.set_zlim([-3, 3])
        
        rotated = rotate_x(points, roll)
        rotated = rotate_y(rotated, pitch)
        rotated = rotate_z(rotated, yaw)

        faces = [
            [rotated[0], rotated[1], rotated[5], rotated[4]],
            [rotated[7], rotated[6], rotated[2], rotated[3]],
            [rotated[0], rotated[3], rotated[7], rotated[4]],
            [rotated[1], rotated[2], rotated[6], rotated[5]],
            [rotated[0], rotated[1], rotated[2], rotated[3]],
            [rotated[4], rotated[5], rotated[6], rotated[7]]
        ]
        
        poly3d = Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='black', alpha=0.6)
        ax.add_collection3d(poly3d)
        
        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    ser.close()
    print("Visualizer closed.")
