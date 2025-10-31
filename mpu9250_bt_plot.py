# Código en Python para la lectura y gráficación de datos recibidos.

import serial
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Cambia "COM7" por el puerto asignado al ESP32 en tu PC
# En Linux/Mac podría ser "/dev/rfcomm0" o "/dev/ttyESP32"
PORT = "COM7"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Conectado a {PORT} a {BAUD} bps")

MAX_POINTS = 300  # cantidad de puntos visibles (≈ 0.1 s a 3kHz)
acc_x, acc_y, acc_z = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
gyro_x, gyro_y, gyro_z = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

plt.style.use('seaborn-v0_8-darkgrid')
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

# Subplot 1: acelerómetros
line_acc_x, = ax1.plot([], [], label='Ax')
line_acc_y, = ax1.plot([], [], label='Ay')
line_acc_z, = ax1.plot([], [], label='Az')
ax1.set_ylim(-32768, 32767)
ax1.set_title("Acelerómetro (Raw)")
ax1.legend(loc="upper right")

# Subplot 2: giroscopios
line_gyro_x, = ax2.plot([], [], label='Gx')
line_gyro_y, = ax2.plot([], [], label='Gy')
line_gyro_z, = ax2.plot([], [], label='Gz')
ax2.set_ylim(-32768, 32767)
ax2.set_title("Giroscopio (Raw)")
ax2.legend(loc="upper right")

for a in [ax1, ax2]:
    a.set_xlim(0, MAX_POINTS)
    a.set_xlabel("Muestras")
    a.set_ylabel("Valor crudo")

# ======================
# FUNCIÓN DE LECTURA
# ======================
def read_mpu_frame():
    """Lee 12 bytes desde el puerto serial y los decodifica"""
    data = ser.read(12)
    if len(data) == 12:
        # '<hhhhhh' = 6 enteros de 16 bits (little endian)
        return struct.unpack('<hhhhhh', data)
    else:
        return None

def update(frame):
    reading = read_mpu_frame()
    if reading:
        ax, ay, az, gx, gy, gz = reading

        # Agregar al buffer
        acc_x.append(ax)
        acc_y.append(ay)
        acc_z.append(az)
        gyro_x.append(gx)
        gyro_y.append(gy)
        gyro_z.append(gz)

        line_acc_x.set_data(range(len(acc_x)), list(acc_x))
        line_acc_y.set_data(range(len(acc_y)), list(acc_y))
        line_acc_z.set_data(range(len(acc_z)), list(acc_z))
        line_gyro_x.set_data(range(len(gyro_x)), list(gyro_x))
        line_gyro_y.set_data(range(len(gyro_y)), list(gyro_y))
        line_gyro_z.set_data(range(len(gyro_z)), list(gyro_z))

    return (line_acc_x, line_acc_y, line_acc_z,
            line_gyro_x, line_gyro_y, line_gyro_z)

# ANIMACIÓN EN TIEMPO REAL
ani = animation.FuncAnimation(fig, update, interval=1, blit=True)
plt.tight_layout()
plt.show()

ser.close()



