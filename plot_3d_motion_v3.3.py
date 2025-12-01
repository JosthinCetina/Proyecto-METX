import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2R
from matplotlib.animation import FuncAnimation
from collections import deque
# Se elimina importación de matplotlib.widgets.Button y datetime

# ============================================================
# Detectar puerto automáticamente
# ============================================================
def detectar_puerto():
    for p in serial.tools.list_ports.comports():
        if ("USB" in p.description or "UART" in p.description 
            or "Silicon Labs" in p.description or "CP210" in p.description):
            return p.device
    return None

PORT = detectar_puerto()
ser = None
SERIAL_CONNECTED = False

if PORT:
    try:
        # ATENCIÓN: Baudrate a 9600
        ser = serial.Serial(PORT, 9600, timeout=1)
        SERIAL_CONNECTED = True
        print(f"Conexión serial establecida en: {PORT} a 9600 baudios.")
    except serial.SerialException as e:
        print(f"⚠️ Error al abrir el puerto {PORT}: {e}")
        print("Continuando sin conexión serial. No se recibirán datos en tiempo real.")
else:
    print("⚠️ No se detectó un puerto serial compatible.")
    print("Continuando sin conexión serial. No se recibirán datos en tiempo real.")

# ============================================================
# Filtro Madgwick
# ============================================================
madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])

# ============================================================
# Estado del sistema
# ============================================================
dt = 0.11 # Intervalo de tiempo (ajustar si la tasa de muestreo cambia)
v = np.zeros(3) # Vector de velocidad (mantenido para la lógica del ZUPT)
pos = np.zeros(3) # Vector de posición

tray_x, tray_y, tray_z = [], [], []

# ============================================================
# ZUPT SIMPLE (SOLO POR ROTACIÓN)
# ============================================================
WINDOW = 10
acc_magnitudes = deque(maxlen=WINDOW)
gyro_magnitudes = deque(maxlen=WINDOW)

TH_GYRO = 0.3 

def esta_quieto():
    if len(gyro_magnitudes) < WINDOW:
        return False
        
    avg_gyro = np.mean(gyro_magnitudes)
    return (avg_gyro < TH_GYRO)

# ============================================================
# SISTEMA DE RESET
# ============================================================
contador_quieto = 0
RESET_AFTER_QUIET = 1

def manejar_reset(quieto_actual):
    global contador_quieto, pos, v, tray_x, tray_y, tray_z
    
    if quieto_actual:
        contador_quieto += 1
        if contador_quieto >= RESET_AFTER_QUIET:
            tray_x.clear()
            tray_y.clear()
            tray_z.clear()
            pos[:] = 0.0
            v[:] = 0.0
            contador_quieto = 0
    else:
        contador_quieto = 0

# ============================================================
# CÁLCULO DE DESPLAZAMIENTO PROPORCIONAL (Lógica Revertida)
# ============================================================
ESCALA_ACELERACION = 0.01

def calcular_desplazamiento_proporcional(accel_global, dt):
    magnitud_acel = np.linalg.norm(accel_global)
    
    if magnitud_acel > 0.01:
        direccion = accel_global / magnitud_acel
    else:
        direccion = np.zeros(3)
    
    # Lógica de desplazamiento proporcional: Desplazamiento ~ (aceleración^2) * ESCALA
    desplazamiento = direccion * (magnitud_acel ** 2) * ESCALA_ACELERACION * dt
    return desplazamiento

# ============================================================
# Gráfica
# ============================================================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
linea, = ax.plot([], [], [], 'b-', lw=2)
punto_actual, = ax.plot([], [], [], 'ro', markersize=6)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.grid(True, alpha=0.3)
ax.set_title("Trayectoria 3D (IMU Fusión Madgwick + ZUPT de Giroscopio)")

# Se ajusta el espacio al pie, ya que el botón fue eliminado
plt.subplots_adjust(bottom=0.05) 

# ============================================================
# Loop principal
# ============================================================
def actualizar(frame):
    global q, v, pos

    if SERIAL_CONNECTED and ser.in_waiting:
        try:
            linea_serial = ser.readline().decode().strip()
            if linea_serial:
                datos = linea_serial.split(";")
                if len(datos) == 6:
                    axr, ayr, azr, gxr, gyr, gzr = map(float, datos)
                    
                    accel = np.array([axr, ayr, azr])
                    gyro = np.array([gxr, gyr, gzr])
                    
                    acc_mag = np.linalg.norm(accel)
                    gyro_mag = np.linalg.norm(gyro)
                    
                    acc_magnitudes.append(acc_mag)
                    gyro_magnitudes.append(gyro_mag)
                    
                    # Fusión y Orientación
                    q = madgwick.updateIMU(q, gyro, accel)
                    R = q2R(q)
                    
                    # Transformación y Compensación de Gravedad
                    accel_global = R @ accel
                    accel_global[2] -= 9.80665
                    
                    # ZUPT
                    quieto = esta_quieto()
                    manejar_reset(quieto)
                    
                    # Integración (Lógica Proporcional Revertida)
                    if quieto:
                        v[:] = 0.0
                        desplazamiento = np.zeros(3)
                    else:
                        desplazamiento = calcular_desplazamiento_proporcional(accel_global, dt)
                    
                    pos += desplazamiento
                    
                    # Actualizar Gráfica
                    tray_x.append(pos[0])
                    tray_y.append(pos[1])
                    tray_z.append(pos[2])
                    
                    if len(tray_x) > 0:
                        linea.set_data(tray_x, tray_y)
                        linea.set_3d_properties(tray_z)
                        
                        punto_actual.set_data([pos[0]], [pos[1]])
                        punto_actual.set_3d_properties([pos[2]])
                        
                        if len(tray_x) > 5:
                            margin = 0.5
                            ax.set_xlim(min(tray_x) - margin, max(tray_x) + margin)
                            ax.set_ylim(min(tray_y) - margin, max(tray_y) + margin)
                            ax.set_zlim(min(tray_z) - margin, max(tray_z) + margin)
            
        except Exception:
            # Esto captura errores de lectura o formato de datos
            pass 
    
    return linea, punto_actual

# Configurar animación
ani = FuncAnimation(fig, actualizar, interval=50, blit=True, cache_frame_data=False)

# Función para cerrar la conexión serial al cerrar la ventana
def on_close(event):
    if ser and ser.is_open:
        ser.close()

fig.canvas.mpl_connect('close_event', on_close)

plt.tight_layout()
plt.show()
