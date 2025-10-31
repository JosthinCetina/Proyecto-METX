# Código en Python para la lectura y gráficación de datos recibidos.

import serial, struct, time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

PORT = "COM7"   # Cambia según tu puerto
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Conectado a {PORT}")
dt = 1/3000.0     # 3 kHz
g = 9.81          # gravedad
alpha = 0.9       # factor de filtro pasa-bajo para ruido

vel = np.zeros(3)
pos = np.zeros(3)
acc_prev = np.zeros(3)
pos_hist = np.zeros((3000, 3))  # 1 segundo de historia

plt.style.use('seaborn-v0_8-darkgrid')
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_title("Posición estimada (m)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")

line, = ax.plot([], [], [], 'r-', lw=2)

def leer_muestra():
    data = ser.read(12)
    if len(data) == 12:
        ax_, ay_, az_, gx_, gy_, gz_ = struct.unpack('<hhhhhh', data)
        a = np.array([ax_, ay_, az_]) / 100.0  # m/s²
        a[2] -= g
        return a
    return None

def update(frame):
    global acc_prev, vel, pos, pos_hist
    a = leer_muestra()
    if a is not None:
        a_filt = alpha * acc_prev + (1 - alpha) * a
        acc_prev = a_filt
        vel += 0.5 * (a_filt + acc_prev) * dt
        pos += vel * dt
        pos_hist = np.roll(pos_hist, -1, axis=0)
        pos_hist[-1, :] = pos
        line.set_data(pos_hist[:,0], pos_hist[:,1])
        line.set_3d_properties(pos_hist[:,2])
    return line,

ani = animation.FuncAnimation(fig, update, interval=1, blit=True)
plt.show()
ser.close()
