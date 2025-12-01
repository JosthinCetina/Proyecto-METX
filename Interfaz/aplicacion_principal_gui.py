import serial
import serial.tools.list_ports
import numpy as np
import pandas as pd
import time
import tkinter as tk
from tkinter import ttk, messagebox
from collections import deque
import joblib
import warnings
import os
import threading
from queue import Queue
from sklearn.ensemble import RandomForestClassifier
from sklearn.preprocessing import LabelEncoder

# Deshabilitar advertencias de sklearn y numpy para una consola limpia
warnings.filterwarnings("ignore")

# === CONFIGURACIÓN GLOBAL (DEBE SER CONSISTENTE CON EL SENSOR) ===
AJUSTE_PASSWORD = "PINGPONG_ADMIN" 
BAUD_RATE = 9600
SAMPLE_RATE_REF = 50           
WINDOW_SIZE_COLLECTION = 100   
COLLECTION_TIME_S = WINDOW_SIZE_COLLECTION / SAMPLE_RATE_REF 
COLLECTION_TIMEOUT_S = 8.0 
SLIDE_STEP_CLASSIFICATION = 5 

# Configuración del Modelo de Clasificación
N_ESTIMATORS = 400        
COLUMNS = ['accX', 'accY', 'accZ', 'gyrX', 'gyrY', 'gyrZ']
EXPECTED_DATA_COUNT = len(COLUMNS) # DEBE SER 6
MODEL_FILENAME = 'modelo_segmentado_rf.pkl'
ENCODER_FILENAME = 'label_encoder.pkl'
MASTER_DATA_FILENAME = 'dataset_entrenamiento_MAESTRO.xlsx' 
SHEET_NAME_MASTER = 'Datos Combinados' 

MOVEMENTS = [
    'DRIVER O GOLPE DERECHO', 
    'REVES', 
    'SERVICIO O SAQUE', 
    'VOLEA DE DERECHA', 
    'VOLEA DE REVES'
]
NUM_REPETITIONS = 3 
# =================================================================

def detectar_puerto():
    """Detecta automáticamente el primer puerto serial disponible."""
    for p in serial.tools.list_ports.comports():
        if ("USB" in p.description or "UART" in p.description 
            or "Silicon Labs" in p.description or "CP210" in p.description):
            return p.device
    return None

def extract_features(window_df):
    """Calcula características estadísticas de la ventana de datos."""
    features = {}
    for col in COLUMNS:
        data = window_df[col]
        features[f'{col}_mean'] = data.mean()
        features[f'{col}_std'] = data.std()
        features[f'{col}_min'] = data.min()
        features[f'{col}_max'] = data.max()
        features[f'{col}_median'] = data.median()
        features[f'{col}_iqr'] = data.quantile(0.75) - data.quantile(0.25)
        features[f'{col}_rms'] = np.sqrt(np.mean(data**2))
        
    return pd.Series(features)

def parse_line(line):
    """
    Función de validación y limpieza estricta para una línea de datos.
    MODIFICADO: Usa ';' como separador.
    Retorna la lista de 6 flotantes si es válido, o None si falla.
    """
    line = line.strip()
    if not line:
        return None # Línea vacía

    try:
        # 1. Separar por punto y coma (;)
        parts = line.split(';')
        
        # 2. Verificar que tengamos el número exacto de columnas
        if len(parts) != EXPECTED_DATA_COUNT:
            # print(f"DEBUG REJECTED: Count mismatch ({len(parts)} != {EXPECTED_DATA_COUNT}) for: {line}")
            return None 
            
        # 3. Convertir todas las partes a float
        values = [float(p.strip()) for p in parts]
        
        return values
        
    except ValueError:
        # print(f"DEBUG REJECTED: Value error (non-numeric data) for: {line}")
        return None # Contiene texto u otros caracteres no numéricos
    except Exception as e:
        # print(f"DEBUG REJECTED: Unexpected error {e} for: {line}")
        return None # Otro error de formato

class SerialReader(threading.Thread):
    """
    Hilo dedicado para la lectura del puerto serial (Lectura no bloqueante). 
    """
    def __init__(self, ser_instance, data_queue):
        super().__init__()
        self.ser = ser_instance
        self.data_queue = data_queue # Cola para enviar datos a Tkinter
        self._stop_event = threading.Event()
        self.name = 'SerialReaderThread'

    def run(self):
        print(f"Hilo de lectura serial iniciado en {self.ser.port}.")
        while not self._stop_event.is_set():
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    # Usamos readline() con timeout corto para no bloquear indefinidamente
                    # ¡IMPORTANTE! El strip() se realiza en la función parse_line para una limpieza más controlada.
                    line = self.ser.readline().decode('utf-8', errors='ignore') 
                    if line:
                        self.data_queue.put(line)
                else:
                    time.sleep(0.005) 
            except serial.SerialException as e:
                self.data_queue.put(f"[SERIAL_ERROR] {e}")
                self.stop()
            except Exception as e:
                self.data_queue.put(f"[GENERIC_ERROR] {e}")

    def stop(self):
        """Detiene la ejecución del hilo."""
        self._stop_event.set()
        
class PasswordDialog(tk.Toplevel):
    # Clase PasswordDialog (se mantiene igual)
    def __init__(self, parent, title="Acceso Requerido"):
        super().__init__(parent)
        self.transient(parent) 
        self.grab_set()        
        self.title(title)
        self.result = None
        
        self.body()
        self.buttonbox()
        self.resizable(False, False)
        
        self.update_idletasks()
        width = self.winfo_width()
        height = self.winfo_height()
        x = parent.winfo_x() + (parent.winfo_width() // 2) - (width // 2)
        y = parent.winfo_y() + (parent.winfo_height() // 2) - (height // 2)
        self.geometry(f'+{x}+{y}')

        self.wait_window(self) 

    def body(self):
        frame = ttk.Frame(self, padding="10")
        frame.pack(padx=5, pady=5)
        
        self.pw_var = tk.StringVar()
        
        ttk.Label(frame, text="Contraseña de Ajuste:").grid(row=0, column=0, padx=10, pady=10, sticky="w")
        self.pw_entry = ttk.Entry(frame, textvariable=self.pw_var, show="*", width=30)
        self.pw_entry.grid(row=0, column=1, padx=10, pady=10, sticky="ew")
        self.pw_entry.focus_set()

        self.bind("<Return>", lambda event: self.ok())
        self.bind("<Escape>", lambda event: self.cancel())

    def buttonbox(self):
        box = ttk.Frame(self)
        
        w = ttk.Button(box, text="Aceptar", width=10, command=self.ok, default="active")
        w.pack(side="left", padx=5, pady=5)
        w = ttk.Button(box, text="Cancelar", width=10, command=self.cancel)
        w.pack(side="left", padx=5, pady=5)
        
        box.pack(pady=5)
        
    def ok(self):
        if self.pw_var.get() == AJUSTE_PASSWORD:
            self.result = True
            self.destroy()
        else:
            messagebox.showerror("Error", "Contraseña incorrecta.")
            self.pw_var.set("")
            self.pw_entry.focus_set()

    def cancel(self):
        self.result = False
        self.destroy()

class DataCollectorApp:
    def __init__(self, master):
        self.master = master
        master.title("Clasificador de Golpes de Ping Pong (v3: Lectura Reforzada)")
        
        # --- Variables de Estado de Conexión y Hilos ---
        self.ser = None
        self.reader_thread = None       
        self.data_queue = Queue()       
        self.is_connected = False
        self.after_id = None 
        
        # --- Variables de Estado de la Aplicación ---
        self.is_task_active = False        
        self.is_classifying = False        
        self.is_processing_result = False # NUEVO: Flag para evitar múltiples diálogos de confirmación

        # Variables de control del GUI
        self.status_var = tk.StringVar(value="Desconectado. Seleccione el puerto.")
        self.current_task_var = tk.StringVar(value="Modo: Inactivo")
        self.port_var = tk.StringVar() 
        self.level_var = tk.StringVar(value="Amateur") 
        self.classification_result_var = tk.StringVar(value="Esperando golpe...")
        
        # Almacenamiento de modelo y codificador
        self.model = None
        self.encoder = None
        
        # Variables de recolección
        self.collected_data_frames = []
        self.current_movement_index = 0
        self.current_repetition = 1 
        
        # Buffer de la ventana deslizante para clasificación
        self.current_buffer = deque(maxlen=WINDOW_SIZE_COLLECTION) 
        
        # Variables para la Recolección Secuencial (Máquina de Estados)
        self.recolection_start_time = 0 
        self.recolection_buffer = deque(maxlen=WINDOW_SIZE_COLLECTION)
        
        # Widgets para el historial de clasificación (inicialización temprana)
        self.classification_history_text = None

        self.setup_ui()
        self.update_port_list()
        self.load_model_and_encoder() 

        # Configurar el loop de procesamiento de la cola y el cierre
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.master.after(50, self.process_queue_loop) 

    # --- LECTURA SERIAL SEGURA (Multihilo + Cola) ---
    
    def process_queue_loop(self):
        """Lee periódicamente la cola de datos desde el hilo principal."""
        
        while not self.data_queue.empty():
            line_raw = self.data_queue.get_nowait()
            
            # Manejo de errores reportados desde el hilo de lectura
            if line_raw.startswith("[SERIAL_ERROR]") or line_raw.startswith("[GENERIC_ERROR]"):
                self._update_serial_monitor_gui(line_raw)
                messagebox.showerror("Error de Hilo Serial", "El hilo de lectura falló o la conexión se perdió.")
                self.toggle_connection() 
                break 

            # Mostrar el dato crudo en el monitor (con su limpieza básica)
            self._update_serial_monitor_gui(line_raw.strip())
            
            # --- Distribución de Datos ---
            if self.is_task_active:
                self._process_recolection_data(line_raw)
            elif self.is_classifying:
                self._process_live_data(line_raw)
                
        # Reprogramar la función para leer la cola de nuevo
        self.after_id = self.master.after(50, self.process_queue_loop) 

    def _update_serial_monitor_gui(self, line):
        """Lógica de actualización del widget Text."""
        self.serial_monitor_text.config(state='normal')
        current_time = time.strftime("[%H:%M:%S]")
        self.serial_monitor_text.insert(tk.END, f"{current_time} {line}\n")
        
        # Limitar el número de líneas para evitar saturación de memoria
        max_lines = 100 
        num_lines = int(self.serial_monitor_text.index('end-1c').split('.')[0])
        if num_lines > max_lines:
            self.serial_monitor_text.delete('1.0', f'{num_lines - max_lines + 1}.0')
        
        self.serial_monitor_text.see(tk.END)
        self.serial_monitor_text.config(state='disabled')

    # --- CONTROL DE CONEXIÓN Y PUERTOS (Se mantiene igual) ---

    def update_port_list(self):
        """Detecta los puertos seriales disponibles y actualiza el ComboBox."""
        try:
            ports = serial.tools.list_ports.comports()
            port_devices = [p.device for p in ports]
            
            self.port_combo['values'] = port_devices

            if port_devices:
                self.port_var.set(port_devices[0])
            else:
                self.port_var.set("No hay puertos")
        except Exception as e:
            print(f"Error al actualizar la lista de puertos: {e}")
            self.port_combo['values'] = ["Error al listar"]
            self.port_var.set("Error al listar")
            
    def toggle_connection(self):
        """Intenta conectar/desconectar el puerto serial y el hilo de lectura."""
        if self.is_connected:
            self.stop_classification()
            
            if self.reader_thread:
                self.reader_thread.stop()
                self.reader_thread.join(timeout=1) 
                self.reader_thread = None
                
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                
                self.is_connected = False
                self.ser = None
                self.status_var.set("Desconectado. [Modelo cargado: " + ("Sí" if self.model else "No") + "]")
                self.connect_btn.config(text="Conectar", style='TButton', state='normal')
                self.adjust_btn.config(state='disabled')
                self.classify_btn.config(state='disabled')
                self._update_serial_monitor_gui("--- CONEXIÓN CERRADA Y HILO DETENIDO ---")
            except Exception as e:
                messagebox.showerror("Error", f"Fallo al desconectar: {e}")
        else:
            port = self.port_var.get()
            if not port or port == "No hay puertos" or port == "Error al listar":
                messagebox.showerror("Error", "Seleccione un puerto COM válido.")
                return
            try:
                port_to_connect = port.split(' ')[0]
                
                self.ser = serial.Serial(port_to_connect, BAUD_RATE, timeout=0.01)
                time.sleep(2) 
                
                self.reader_thread = SerialReader(self.ser, self.data_queue)
                self.reader_thread.daemon = True 
                self.reader_thread.start()
                
                self.is_connected = True
                self.status_var.set(f"Conectado a {port_to_connect} @{BAUD_RATE}. Hilo de lectura activo.")
                self.connect_btn.config(text="Desconectar", style='Accent.TButton', state='normal')
                if self.model:
                    self.classify_btn.config(state='normal')
                self.adjust_btn.config(state='normal')
                    
                self._update_serial_monitor_gui(f"--- CONECTADO A {port_to_connect} ---")
                
            except serial.SerialException as e:
                messagebox.showerror("Error de Conexión", 
                    f"El puerto {port_to_connect} está ocupado o no existe.\n\nError: {e}"
                )
            except Exception as e:
                messagebox.showerror("Error", f"Error inesperado al conectar: {e}")
    
    # --- UI Setup (MODIFICADO) ---

    def setup_ui(self):
        # Configuración de estilos 
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#f0f0f0')
        style.configure('TLabel', background='#f0f0f0', font=('Arial', 10))
        style.configure('TButton', font=('Arial', 10, 'bold'))
        style.configure('Accent.TButton', foreground='white', background='#28a745', font=('Arial', 10, 'bold'))
        style.map('Accent.TButton', background=[('active', '#218838')])
        
        main_frame = ttk.Frame(self.master, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # --- Conexión ---
        connect_frame = ttk.LabelFrame(main_frame, text="1. Conexión del Sensor", padding="10")
        connect_frame.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        ttk.Label(connect_frame, text="Puerto COM:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.port_combo = ttk.Combobox(connect_frame, textvariable=self.port_var, state="readonly") 
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.connect_btn = ttk.Button(connect_frame, text="Conectar", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5, pady=5)
        connect_frame.columnconfigure(1, weight=1)

        # --- Estado y Progreso ---
        status_frame = ttk.LabelFrame(main_frame, text="Estado del Sistema", padding="10")
        status_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        ttk.Label(status_frame, text="ESTADO:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(status_frame, textvariable=self.status_var, wraplength=400).grid(row=0, column=1, padx=5, pady=2, sticky="w")
        ttk.Label(status_frame, text="TAREA:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(status_frame, textvariable=self.current_task_var, wraplength=400).grid(row=1, column=1, padx=5, pady=2, sticky="w")
        status_frame.columnconfigure(1, weight=1)

        # --- Control y Modelo (Recolección y Entrenamiento) ---
        control_frame = ttk.LabelFrame(main_frame, text="2. Ajuste (Recolección y Reentrenamiento)", padding="10")
        control_frame.grid(row=2, column=0, padx=5, pady=5, sticky="new") # sticky new para no expandir abajo
        
        ttk.Label(control_frame, text="Nivel Sujeto:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        level_options = ['Amateur', 'Profesional']
        self.level_combo = ttk.Combobox(control_frame, textvariable=self.level_var, values=level_options, state="readonly")
        self.level_combo.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        self.adjust_btn = ttk.Button(control_frame, text="AJUSTAR (Recolección + Reentrenamiento)", command=self.check_password_and_start_ajuste, state='disabled')
        self.adjust_btn.grid(row=1, column=0, columnspan=2, padx=5, pady=10, sticky="ew")
        control_frame.columnconfigure(1, weight=1)
        
        # --- Clasificación (MODIFICADO para incluir Historial) ---
        classify_frame = ttk.LabelFrame(main_frame, text="3. Clasificación en Vivo", padding="10")
        # Ocupa dos filas para darle espacio al monitor de historial.
        classify_frame.grid(row=2, column=1, rowspan=2, padx=5, pady=5, sticky="nsew") 
        
        self.classify_btn = ttk.Button(classify_frame, text="INICIAR CLASIFICACIÓN", command=self.toggle_classification, state='disabled')
        self.classify_btn.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        # Resultado de Clasificación (Row 1)
        result_label = ttk.Label(classify_frame, textvariable=self.classification_result_var, font=('Arial', 14, 'bold'), anchor='center')
        result_label.grid(row=1, column=0, padx=5, pady=5, sticky="ew")

        # NUEVO: Historial de Clasificación (Row 2, ocupa el espacio restante)
        monitor_clasif_frame = ttk.LabelFrame(classify_frame, text="Historial de Golpes Detectados", padding="5")
        monitor_clasif_frame.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")

        scrollbar_clasif = ttk.Scrollbar(monitor_clasif_frame)
        scrollbar_clasif.pack(side="right", fill="y")
        
        self.classification_history_text = tk.Text(monitor_clasif_frame, height=8, wrap='word', 
                                                   yscrollcommand=scrollbar_clasif.set, 
                                                   state='disabled', 
                                                   font=('Consolas', 9),
                                                   background='#f0fff0', # Color suave
                                                   foreground='#006400') # Verde oscuro
        self.classification_history_text.pack(side="left", fill="both", expand=True)
        scrollbar_clasif.config(command=self.classification_history_text.yview)

        # Configuración de expansión para el frame de Clasificación
        classify_frame.columnconfigure(0, weight=1)
        classify_frame.grid_rowconfigure(2, weight=1) # El historial es el que debe expandirse

        # --- Monitor Serial (Ahora en la fila 4) ---
        monitor_frame = ttk.LabelFrame(main_frame, text="4. Monitor Serial (Datos Crudos)", padding="10")
        monitor_frame.grid(row=4, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        scrollbar = ttk.Scrollbar(monitor_frame)
        scrollbar.grid(row=0, column=1, sticky='ns')
        
        self.serial_monitor_text = tk.Text(monitor_frame, height=10, wrap='none', 
                                           yscrollcommand=scrollbar.set, 
                                           state='disabled', 
                                           font=('Consolas', 9),
                                           background='#f8f8f8',
                                           foreground='#00008b') 
        self.serial_monitor_text.grid(row=0, column=0, sticky="nsew")
        scrollbar.config(command=self.serial_monitor_text.yview)
        
        monitor_frame.grid_rowconfigure(0, weight=1)
        monitor_frame.grid_columnconfigure(0, weight=1)

        # Configuración de expansión general
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=1)
        main_frame.grid_rowconfigure(4, weight=1) # Permitir que el monitor serial inferior se expanda verticalmente

    # --- GESTIÓN DE CONTROL Y ESTADOS ---
    
    def on_closing(self):
        """Se llama al cerrar la ventana para asegurar la desconexión serial."""
        self.stop_classification()
        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread.join(timeout=1)
        if self.after_id:
            self.master.after_cancel(self.after_id)
        if self.ser and self.ser.is_open:
            self.ser.close()
            
        self.master.destroy()
    
    def disable_control_buttons(self):
        """Deshabilita los botones de control durante una tarea activa."""
        self.adjust_btn.config(state='disabled')
        self.classify_btn.config(state='disabled')
        self.connect_btn.config(state='disabled')
        self.port_combo.config(state='disabled')
        self.level_combo.config(state='disabled')

    def enable_control_buttons(self):
        """Habilita los botones al finalizar el proceso."""
        self.adjust_btn.config(state='normal')
        self.connect_btn.config(state='normal')
        if self.model is not None and self.is_connected:
             self.classify_btn.config(state='normal')
        self.current_task_var.set("Modo: Inactivo")
        self.port_combo.config(state='readonly')
        self.level_combo.config(state='readonly')
        
    # --- PROCESO DE AJUSTE (Recolección y Entrenamiento) ---
    
    def check_password_and_start_ajuste(self):
        """Muestra el diálogo de contraseña antes de iniciar el Ajuste."""
        if not self.is_connected:
            messagebox.showerror("Error", "Debe estar conectado al sensor para Ajustar.")
            return

        dialog = PasswordDialog(self.master)
        
        if dialog.result:
            self.start_ajuste_process()
        elif dialog.result is False: 
            messagebox.showinfo("Ajuste Cancelado", "El proceso de Ajuste fue cancelado.")

    def start_ajuste_process(self):
        """Inicia la máquina de estados de Ajuste (Recolección + Entrenamiento)."""
        
        self.is_task_active = True 
        self.disable_control_buttons()
        
        self.collected_data_frames = []
        self.current_movement_index = 0
        self.current_repetition = 1 
        
        self.status_var.set("Iniciando recolección de datos...")
        self._update_serial_monitor_gui("--- INICIANDO RECOLECCIÓN SECUENCIAL ---")
        
        if self.ser and self.ser.is_open:
            self.ser.read_all()
            
        self.master.after(100, self.ajuste_stage_show_instruction) 
        
    def _process_recolection_data(self, line):
        """
        [MODIFICADO] Procesa la línea de datos y la agrega al buffer solo si es válida.
        """
        # Debe estar activo Y el temporizador corriendo (no 0)
        if not self.is_task_active or self.recolection_start_time == 0:
            return 
        
        # Si ya estamos procesando el resultado (mostrando el diálogo de confirmación), ignorar nuevos datos
        if self.is_processing_result:
            return

        # Intentar parsear y validar la línea
        values = parse_line(line)
        
        if values is not None:
            # Si el buffer aún no está lleno, agregar el dato válido
            if len(self.recolection_buffer) < WINDOW_SIZE_COLLECTION:
                self.recolection_buffer.append(values)
                self.current_task_var.set(
                    f"RECOLECCIÓN: {MOVEMENTS[self.current_movement_index]} - Rep {self.current_repetition}/{NUM_REPETITIONS} ({len(self.recolection_buffer)}/{WINDOW_SIZE_COLLECTION})"
                )
            # Si el buffer ya se llenó, forzar la detención y pasar a la siguiente etapa.
            if len(self.recolection_buffer) >= WINDOW_SIZE_COLLECTION and self.recolection_start_time > 0:
                self.recolection_start_time = 0 
                self.master.after(0, self.ajuste_stage_handle_result) 
        
    def ajuste_stage_show_instruction(self):
        """Etapa 1: Muestra la instrucción, limpia el buffer e inicia el temporizador de control."""
        
        # Verificar si hemos terminado todos los movimientos
        if self.current_movement_index >= len(MOVEMENTS):
            self.master.after(0, self.ajuste_stage_start_training) 
            return

        mov_index = self.current_movement_index
        rep = self.current_repetition
        movement_name = MOVEMENTS[mov_index]
        
        self.current_task_var.set(f"PREPARACIÓN: {movement_name} - Rep {rep}/{NUM_REPETITIONS}")
        
        messagebox.showinfo(
            "Instrucción de Recolección", 
            f"Movimiento: '{movement_name}'\nRepetición {rep}/{NUM_REPETITIONS}.\n"
            f"Ventana de grabación: {COLLECTION_TIME_S:.2f} segundos ({WINDOW_SIZE_COLLECTION} muestras)."
            f"\n\nPulse Aceptar para INICIAR la grabación. Realice el golpe de inmediato."
        )
        
        self.recolection_buffer.clear()
        self.recolection_start_time = time.time()
        self._update_serial_monitor_gui(f"--- GRABANDO {movement_name} - Rep {rep} ---")
        
        self.master.after_id_task = self.master.after(100, self.ajuste_stage_check_progress)

    def ajuste_stage_check_progress(self):
        """Etapa 2: Verifica el progreso de la recolección y el tiempo límite."""
        
        # Si la recolección ya fue forzada a parar (recolection_start_time = 0), no seguir checando.
        if self.recolection_start_time == 0:
            return

        collected_count = len(self.recolection_buffer)
        
        # 1. Comprobar tiempo límite
        if time.time() - self.recolection_start_time > COLLECTION_TIMEOUT_S:
            self._update_serial_monitor_gui(f"--- GRABACIÓN TERMINADA POR TIEMPO LÍMITE ({collected_count}/{WINDOW_SIZE_COLLECTION}) ---")
            self.recolection_start_time = 0 
            self.master.after(0, self.ajuste_stage_handle_result)
            return
            
        # 2. Comprobar finalización por muestras
        if collected_count >= WINDOW_SIZE_COLLECTION:
            self._update_serial_monitor_gui(f"--- GRABACIÓN FINALIZADA POR MUESTRAS ({collected_count}/{WINDOW_SIZE_COLLECTION}) ---")
            self.recolection_start_time = 0 
            self.master.after(0, self.ajuste_stage_handle_result)
            return
            
        # 3. Reprogramar la función de chequeo
        self.master.after_id_task = self.master.after(100, self.ajuste_stage_check_progress)

    def ajuste_stage_handle_result(self):
        """
        Etapa 3: Evalúa el resultado de la recolección y pide confirmación (Guardar o Repetir).
        MODIFICADO: Implementa el flag 'is_processing_result' para evitar ventanas duplicadas.
        """
        # NUEVA GUARDIA: Si ya estamos en el proceso de mostrar el diálogo/manejar el resultado, salir.
        if self.is_processing_result: 
            return
            
        self.is_processing_result = True # Bloquear nuevas entradas

        collected_count = len(self.recolection_buffer)
        mov_index = self.current_movement_index
        rep = self.current_repetition
        movement_name = MOVEMENTS[mov_index]

        # 1. Definir el mensaje de confirmación
        base_message = f"Movimiento: '{movement_name}' - Repetición {rep}/{NUM_REPETITIONS}.\n"
        
        if collected_count < WINDOW_SIZE_COLLECTION:
             base_message += f"Muestras capturadas: {collected_count} (Objetivo: {WINDOW_SIZE_COLLECTION}).\n"
             base_message += "La toma fue INCOMPLETA. Se recomienda repetir."
        else:
            base_message += f"Muestras capturadas: {collected_count} (¡Toma completa!).\n"

        prompt = base_message + ("\n\n¿Desea **GUARDAR** esta muestra y **CONTINUAR** al siguiente paso? "
                                 "(Seleccione 'No' para **REPETIR** la toma sin guardar.)")

        # 2. Presentar el diálogo de confirmación (True = Sí/Guardar/Continuar, False = No/Repetir)
        user_choice = messagebox.askyesno("Confirmación de Muestra Recolectada", prompt)

        if user_choice: # User chose 'Yes' (Guardar y Continuar)
            
            # --- GUARDAR DATOS ---
            if collected_count > 0:
                data_df = pd.DataFrame(list(self.recolection_buffer), columns=COLUMNS)
                data_df['label'] = movement_name
                data_df['subject_level'] = self.level_var.get()
                self.collected_data_frames.append(data_df)
                self._update_serial_monitor_gui(f"Muestra guardada: {movement_name} - Rep {rep} ({collected_count} puntos).")
            else:
                self._update_serial_monitor_gui("Muestra NO guardada (0 puntos válidos).")

            # --- AVANZAR ESTADO ---
            self.current_repetition += 1
            if self.current_repetition > NUM_REPETITIONS:
                self.current_repetition = 1
                self.current_movement_index += 1
                
        else: # User chose 'No' (Repetir)
            self._update_serial_monitor_gui(f"Toma de {movement_name} - Rep {rep} IGNORADA. Reiniciando la toma.")
            
        # 3. Pasar a la siguiente instrucción (ya sea la misma o la siguiente)
        self.is_processing_result = False # Liberar bloqueo
        self.master.after(100, self.ajuste_stage_show_instruction)

    # --- ENTRENAMIENTO Y ALMACENAMIENTO DE DATOS (Se mantiene igual) ---

    def ajuste_stage_start_training(self):
        """Etapa 4: Combinación de datos y Entrenamiento del Modelo."""
        self.current_task_var.set("ENTRENAMIENTO: Preparando datos...")
        self.status_var.set("Recolección terminada. Iniciando entrenamiento del modelo...")
        self._update_serial_monitor_gui("--- INICIANDO ENTRENAMIENTO DE CLASIFICADOR ---")
        
        if not self.collected_data_frames:
            messagebox.showerror("Error", "No se recolectaron datos para entrenar.")
            self.ajuste_stage_finalize()
            return
            
        combined_df = pd.concat(self.collected_data_frames, ignore_index=True)
            
        # Es necesario limpiar el buffer serial ANTES de empezar la recolección
        if self.ser and self.ser.is_open:
            self.ser.read_all()
        
        self.master.after(100, lambda: self.ajuste_stage_save_master_data(combined_df))

    def ajuste_stage_save_master_data(self, new_data_df):
        """Guarda los nuevos datos en el archivo maestro de Excel."""
        self.current_task_var.set("ENTRENAMIENTO: Guardando datos en maestro...")

        try:
            if os.path.exists(MASTER_DATA_FILENAME):
                with warnings.catch_warnings(record=True): 
                    master_df = pd.read_excel(MASTER_DATA_FILENAME, sheet_name=SHEET_NAME_MASTER, header=0)
                
                final_df = pd.concat([master_df, new_data_df], ignore_index=True)
            else:
                final_df = new_data_df

            with pd.ExcelWriter(MASTER_DATA_FILENAME, engine='openpyxl') as writer:
                final_df.to_excel(writer, sheet_name=SHEET_NAME_MASTER, index=False)

            self._update_serial_monitor_gui(f"Datos guardados en {MASTER_DATA_FILENAME}. Total de filas: {len(final_df)}.")
            print(f"Datos guardados. Total de filas: {len(final_df)}.")
            
            self.master.after(100, lambda: self.ajuste_stage_train_model(final_df))

        except Exception as e:
            messagebox.showerror("Error de Guardado", f"Fallo al guardar o leer el archivo maestro:\n{e}")
            self.ajuste_stage_finalize()

    def ajuste_stage_train_model(self, full_df):
        """Realiza la segmentación y entrena el modelo de Random Forest."""
        self.current_task_var.set("ENTRENAMIENTO: Segmentando y entrenando...")
        
        try:
            # 1. Segmentación de datos (Ventana deslizante)
            segmented_features = []
            labels = []
            
            for label in full_df['label'].unique():
                subset = full_df[full_df['label'] == label].reset_index(drop=True)
                
                for i in range(0, len(subset) - WINDOW_SIZE_COLLECTION, SLIDE_STEP_CLASSIFICATION):
                    window = subset.iloc[i:i + WINDOW_SIZE_COLLECTION]
                    
                    if len(window) == WINDOW_SIZE_COLLECTION:
                        features = extract_features(window[COLUMNS])
                        segmented_features.append(features)
                        labels.append(label)

            if not segmented_features:
                raise ValueError("No se pudieron generar características segmentadas. Revise el tamaño de la ventana.")
                
            X = pd.DataFrame(segmented_features)
            y = np.array(labels)

            # 2. Codificación de etiquetas (Encoder)
            self.encoder = LabelEncoder()
            y_encoded = self.encoder.fit_transform(y)
            
            # 3. Entrenamiento
            self.model = RandomForestClassifier(n_estimators=N_ESTIMATORS, random_state=42, n_jobs=-1)
            self.model.fit(X, y_encoded)
            
            # 4. Guardar Modelo y Encoder
            joblib.dump(self.model, MODEL_FILENAME)
            joblib.dump(self.encoder, ENCODER_FILENAME)

            self._update_serial_monitor_gui(f"Modelo entrenado y guardado como {MODEL_FILENAME}.")
            self.status_var.set("Modelo entrenado con éxito. Listo para Clasificar.")
            
        except Exception as e:
            messagebox.showerror("Error de Entrenamiento", f"Fallo en el entrenamiento o segmentación:\n{e}")
            print(f"[ERROR ENTRENAMIENTO] {e}")

        self.ajuste_stage_finalize()
        
    def ajuste_stage_finalize(self):
        """Finaliza el proceso de ajuste y limpia el estado."""
        self.is_task_active = False
        self.collected_data_frames = []
        self.recolection_buffer.clear()
        self.enable_control_buttons()

    # --- CLASIFICACIÓN EN VIVO (MODIFICADO) ---
    
    def load_model_and_encoder(self):
        """Carga el modelo y el encoder al iniciar la aplicación."""
        try:
            self.model = joblib.load(MODEL_FILENAME)
            self.encoder = joblib.load(ENCODER_FILENAME)
            self.status_var.set(f"Modelo '{MODEL_FILENAME}' cargado exitosamente.")
        except FileNotFoundError:
            self.model = None
            self.encoder = None
            self.status_var.set("Modelo NO encontrado. Realice el Ajuste primero.")
        except Exception as e:
            messagebox.showerror("Error de Carga", f"Error al cargar el modelo o encoder: {e}")
            self.model = None
            self.encoder = None

    def toggle_classification(self):
        """Alterna el estado de Clasificación en Vivo."""
        if not self.model or not self.encoder:
            messagebox.showerror("Error", "Debe cargar o entrenar el modelo antes de clasificar.")
            return
            
        if not self.is_connected:
            messagebox.showerror("Error", "Debe estar conectado al sensor.")
            return

        if self.is_classifying:
            self.stop_classification()
        else:
            self.start_classification_live()

    def start_classification_live(self):
        """Inicia el modo de clasificación en vivo."""
        self.is_classifying = True
        self.current_buffer.clear()
        self.classification_history_text.config(state='normal')
        self.classification_history_text.delete('1.0', tk.END)
        self.classification_history_text.config(state='disabled')
        self.classify_btn.config(text="DETENER CLASIFICACIÓN", style='TButton')
        self.current_task_var.set("Modo: CLASIFICACIÓN EN VIVO")
        self.classification_result_var.set("Esperando golpe...")
        
        self.status_var.set("CLASIFICANDO: Recibiendo datos en buffer deslizante...")

    def stop_classification(self):
        """Detiene el modo de clasificación en vivo."""
        self.is_classifying = False
        self.classify_btn.config(text="INICIAR CLASIFICACIÓN", style='Accent.TButton')
        self.current_task_var.set("Modo: Inactivo")
        self.classification_result_var.set("Clasificación Detenida.")
        
    def _process_live_data(self, line):
        """Procesa una línea de datos y evalúa el buffer para clasificación."""
        if not self.is_classifying or self.is_task_active:
            return 
            
        values = parse_line(line)
            
        if values is not None:
            # 1. Añadir al buffer (si el formato es correcto)
            self.current_buffer.append(values)
            
            # 2. Evaluar el buffer (solo cuando está lleno)
            if len(self.current_buffer) == WINDOW_SIZE_COLLECTION:
                self._classify_current_window()
                
                # 3. Deslizar la ventana manualmente al quitar elementos
                for _ in range(SLIDE_STEP_CLASSIFICATION):
                    if self.current_buffer:
                        self.current_buffer.popleft() 
        
    def _classify_current_window(self):
        """Extrae características y clasifica la ventana actual."""
        try:
            window_df = pd.DataFrame(list(self.current_buffer), columns=COLUMNS)
            
            features = extract_features(window_df)
            X_live = features.to_frame().T 

            prediction_encoded = self.model.predict(X_live)
            predicted_label = self.encoder.inverse_transform(prediction_encoded)[0]
            
            # 1. Actualizar resultado principal
            self.classification_result_var.set(f"DETECTADO: {predicted_label}")
            self._update_serial_monitor_gui(f"*** CLASIFICACIÓN: {predicted_label} ***")
            
            # 2. NUEVO: Actualizar el monitor de historial
            current_time = time.strftime("[%H:%M:%S]")
            history_line = f"{current_time} -> {predicted_label}\n"
            self.classification_history_text.config(state='normal')
            self.classification_history_text.insert(tk.END, history_line)
            
            # Limitar el historial (ej. a 50 líneas)
            max_lines = 50 
            num_lines = int(self.classification_history_text.index('end-1c').split('.')[0])
            if num_lines > max_lines:
                self.classification_history_text.delete('1.0', f'{num_lines - max_lines + 1}.0')
                
            self.classification_history_text.see(tk.END)
            self.classification_history_text.config(state='disabled')
            
        except Exception as e:
            self.classification_result_var.set("Error de Clasificación")
            print(f"[ERROR CLASIFICACIÓN DE VENTANA] {e}")


# --------------------------------------------------------------------
# MAIN
# --------------------------------------------------------------------
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    root = tk.Tk()
    app = DataCollectorApp(root)
    root.mainloop()