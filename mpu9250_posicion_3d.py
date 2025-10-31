# Código en Python para la lectura y gráficación de datos recibidos.

import serial, struct, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT = "COM7"   # Cambia según tu sistema
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Conectado a {PORT}")

dt_est = 1 / 3000.0   # 3 kHz teórico
g = 9.81
alpha = 0.9            # Filtro pasa-bajo
acc_prev = np.zeros(3)
vel = np.zeros(3)
pos = np.zeros(3)
hist_len = 5000
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
ax.set_title('Posición estimada (MPU9250)')

trayectoria, = ax.plot([], [], [], 'r-', lw=2)
punto, = ax.plot([], [], [], 'bo')

def leer_paquete():
    data = ser.read(12)
    if len(data) == 12:
        ax_, ay_, az_, gx_, gy_, gz_ = struct.unpack('<hhhhhh', data)
        a = np.array([ax_, ay_, az_]) / 100.0  # convertir a m/s²
        a[2] -= g 
        return a
    return None

# === Bucle de animación ===
last_time = time.time()

def update(frame):
    global acc_prev, vel, pos, pos_hist, last_time

    a = leer_paquete()
    if a is not None:
        # Medir tiempo entre muestras
        now = time.time()
        dt = now - last_time
        last_time = now
        if dt <= 0 or dt > 0.05:  # proteger contra valores erróneos
            dt = dt_est
        a_filt = alpha * acc_prev + (1 - alpha) * a
        acc_prev = a_filt
        vel += a_filt * dt
        pos += vel * dt
        pos_hist = np.roll(pos_hist, -1, axis=0)
        pos_hist[-1, :] = pos
        trayectoria.set_data(pos_hist[:, 0], pos_hist[:, 1])
        trayectoria.set_3d_properties(pos_hist[:, 2])
        punto.set_data([pos[0]], [pos[1]])
        punto.set_3d_properties([pos[2]])

    return trayectoria, punto

ani = animation.FuncAnimation(fig, update, interval=5, blit=True)
plt.show()
ser.close()
