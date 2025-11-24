import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2R
from matplotlib.animation import FuncAnimation
from collections import deque
from matplotlib.widgets import Button
import datetime

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
if PORT is None:
    exit()

ser = serial.Serial(PORT, 115200, timeout=1)

# ============================================================
# Filtro Madgwick
# ============================================================
madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])

# ============================================================
# Estado del sistema
# ============================================================
dt = 0.11
v = np.zeros(3)
pos = np.zeros(3)

tray_x, tray_y, tray_z = [], [], []

# ============================================================
# Variables para captura de datos
# ============================================================
capturando_datos = False
datos_capturados = []
archivo_datos = None

# ============================================================
# ZUPT SIMPLE
# ============================================================
WINDOW = 10
acc_magnitudes = deque(maxlen=WINDOW)
gyro_magnitudes = deque(maxlen=WINDOW)

TH_ACC = 0.3
TH_GYRO = 0.3

def esta_quieto():
    if len(acc_magnitudes) < WINDOW:
        return False
    avg_acc = np.mean(acc_magnitudes)
    avg_gyro = np.mean(gyro_magnitudes)
    return (avg_acc < TH_ACC) and (avg_gyro < TH_GYRO)

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
# FUNCIONES PARA CAPTURA DE DATOS
# ============================================================
def iniciar_captura(event):
    global capturando_datos, datos_capturados, archivo_datos
    
    if not capturando_datos:
        capturando_datos = True
        datos_capturados = []
        
        # Crear archivo con timestamp: 
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        nombre_archivo = f"C:/Users/jocet/OneDrive/Documentos/Escuelaing/7 SEMESTRE/MTXE/Proyecto/Graficador_{timestamp}.txt"
        archivo_datos = open(nombre_archivo, 'w')
        archivo_datos.write("ax;ay;az;gx;gy;gz\n")  # Encabezado
        
        boton_captura.label.set_text("🛑 Detener Muestra")
        print(f"🎥 Captura iniciada: {nombre_archivo}")

def detener_captura():
    global capturando_datos, archivo_datos
    
    if capturando_datos:
        capturando_datos = False
        if archivo_datos:
            archivo_datos.close()
            archivo_datos = None
        
        boton_captura.label.set_text("📥 Capturar Muestra")
        print(f"💾 Captura finalizada. {len(datos_capturados)} muestras guardadas")

def toggle_captura(event):
    if capturando_datos:
        detener_captura()
    else:
        iniciar_captura(event)

def guardar_dato(dato):
    global datos_capturados, archivo_datos
    
    if capturando_datos and archivo_datos:
        # Guardar en memoria
        datos_capturados.append(dato)
        
        # Guardar en archivo inmediatamente
        archivo_datos.write(dato + '\n')
        archivo_datos.flush()  # Forzar escritura

# ============================================================
# FACTOR DE ESCALA PARA PROPORCIONALIDAD
# ============================================================
ESCALA_ACELERACION = 0.01

def calcular_desplazamiento_proporcional(accel_global, dt):
    magnitud_acel = np.linalg.norm(accel_global)
    
    if magnitud_acel > 0.01:
        direccion = accel_global / magnitud_acel
    else:
        direccion = np.zeros(3)
    
    desplazamiento = direccion * (magnitud_acel ** 2) * ESCALA_ACELERACION * dt
    return desplazamiento

# ============================================================
# Gráfica con botones
# ============================================================
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection="3d")
linea, = ax.plot([], [], [], 'b-', lw=2)
punto_actual, = ax.plot([], [], [], 'ro', markersize=6)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.grid(True, alpha=0.3)

# ============================================================
# Crear botones
# ============================================================
# Ajustar posición del gráfico para hacer espacio para botones
plt.subplots_adjust(bottom=0.15)

# Botón de captura
ax_boton_captura = plt.axes([0.7, 0.05, 0.25, 0.06])
boton_captura = Button(ax_boton_captura, '📥 Capturar Muestra', color='lightblue', hovercolor='cyan')

# Conectar el botón a la función
boton_captura.on_clicked(toggle_captura)

# ============================================================
# Loop principal
# ============================================================
def actualizar(frame):
    global q, v, pos

    try:
        if ser.in_waiting:
            linea_serial = ser.readline().decode().strip()
            if linea_serial:
                datos = linea_serial.split(";")
                if len(datos) == 6:
                    axr, ayr, azr, gxr, gyr, gzr = map(float, datos)
                    
                    # Guardar dato si estamos capturando
                    if capturando_datos:
                        guardar_dato(linea_serial)
                    
                    accel = np.array([axr, ayr, azr])
                    gyro = np.array([gxr, gyr, gzr])
                    
                    acc_mag = np.linalg.norm(accel)
                    gyro_mag = np.linalg.norm(gyro)
                    
                    acc_magnitudes.append(acc_mag)
                    gyro_magnitudes.append(gyro_mag)
                    
                    q = madgwick.updateIMU(q, gyro, accel)
                    R = q2R(q)
                    
                    accel_global = R @ accel
                    accel_global[2] -= 9.80665
                    
                    quieto = esta_quieto()
                    manejar_reset(quieto)
                    
                    if quieto:
                        v[:] = 0.0
                        desplazamiento = np.zeros(3)
                    else:
                        desplazamiento = calcular_desplazamiento_proporcional(accel_global, dt)
                    
                    pos += desplazamiento
                    
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
        pass

    return linea, punto_actual

# Configurar animación
ani = FuncAnimation(fig, actualizar, interval=50, blit=True, cache_frame_data=False)

# Función para cerrar el archivo si está abierto al cerrar la ventana
def on_close(event):
    if capturando_datos:
        detener_captura()
    if ser and ser.is_open:
        ser.close()

fig.canvas.mpl_connect('close_event', on_close)

plt.tight_layout()
plt.show()   
