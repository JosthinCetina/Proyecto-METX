


import serial, struct, time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

PORT = "COM7"     # Cambia según tu sistema
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Conectado a {PORT}")

dt_est = 1 / 200.0   # ~200 Hz aprox
g = 9.81              # gravedad
alpha = 0.9           # filtro pasa-bajo para suavizar aceleraciones
acc_prev = np.zeros(3)
vel = np.zeros(3)
pos = np.zeros(3)
hist_len = 2000
pos_hist = np.zeros((hist_len, 3))
plt.style.use('seaborn-v0_8-darkgrid')
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('Posición estimada (BNO055)')

line, = ax.plot([], [], [], 'r-', lw=2)
point, = ax.plot([], [], [], 'bo')

def leer_paquete():
    data = ser.read(12)
    if len(data) == 12:
        ax_, ay_, az_, gx_, gy_, gz_ = struct.unpack('<hhhhhh', data)
        a = np.array([ax_, ay_, az_]) / 100.0  # m/s²
        a[2] -= g 
        return a
    return None

last_time = time.time()

def update(frame):
    global acc_prev, vel, pos, pos_hist, last_time

    a = leer_paquete()
    if a is not None:
        now = time.time()
        dt = now - last_time
        last_time = now
        if dt <= 0 or dt > 0.05:  # limitar dt a valores razonables
            dt = dt_est

        a_filt = alpha * acc_prev + (1 - alpha) * a
        acc_prev = a_filt
        vel += a_filt * dt
        pos += vel * dt
        pos_hist = np.roll(pos_hist, -1, axis=0)
        pos_hist[-1, :] = pos

        line.set_data(pos_hist[:, 0], pos_hist[:, 1])
        line.set_3d_properties(pos_hist[:, 2])
        point.set_data([pos[0]], [pos[1]])
        point.set_3d_properties([pos[2]])

    return line, point

ani = animation.FuncAnimation(fig, update, interval=5, blit=True)
plt.show()
ser.close()
