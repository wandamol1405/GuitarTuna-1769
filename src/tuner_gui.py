import serial
import threading
import tkinter as tk

# ---------------------------------------------------
# CONFIGURACIÓN PUERTO SERIAL
# ---------------------------------------------------
SERIAL_PORT = "COM12"  
BAUD_RATE = 9600

# ---------------------------------------------------
# MAPA DE CUERDAS
# ---------------------------------------------------
STRING_NAMES = ["E4", "B3", "G3", "D3", "A2", "E2"]

# ---------------------------------------------------
# INTERFAZ GRÁFICA
# ---------------------------------------------------
class TunerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GuitarTuna1769")
        self.root.geometry("400x500")
        self.root.configure(bg="#222222")

        title = tk.Label(root, text="GuitarTuna1769", font=("Arial", 22, "bold"), fg="white", bg="#222222")
        title.pack(pady=10)

        # Nueva: Indicador de Calibración
        self.calib_label = tk.Label(root, text="Calibración: Pendiente", font=("Arial", 14), fg="gray", bg="#222222")
        self.calib_label.pack(pady=5)
        
        self.string_label = tk.Label(root, text="Cuerda: --", font=("Arial", 20), fg="white", bg="#222222")
        self.string_label.pack(pady=10)

        self.freq_label = tk.Label(root, text="Frecuencia: -- Hz", font=("Arial", 18), fg="#cccccc", bg="#222222")
        self.freq_label.pack(pady=10)

        # Círculo indicador de afinación
        self.canvas = tk.Canvas(root, width=200, height=200, bg="#222222", highlightthickness=0)
        self.canvas.pack(pady=20)
        self.indicator = self.canvas.create_oval(20, 20, 180, 180, fill="gray")

        # Nueva: Estado General del Sistema (Detenido/Activo/Calibrando)
        self.system_state_label = tk.Label(root, text="Sistema: DESCONECTADO", font=("Arial", 18, "bold"), fg="#ff4444", bg="#222222")
        self.system_state_label.pack(pady=20)
        
        # Etiqueta de afinación
        self.state_label = tk.Label(root, text="Afinación: --", font=("Arial", 22, "bold"), fg="white", bg="#222222")
        self.state_label.pack(pady=10)

    def update_calibration_ui(self, status):
        """Actualiza el estado de la calibración y el estado general del sistema."""
        if status == "INIT":
            self.calib_label.config(text="Calibración: INICIANDO...", fg="#ffaa00")
            self.system_state_label.config(text="Sistema: CALIBRANDO...", fg="#ff8800")
        elif status == "OK":
            self.calib_label.config(text="Calibración: EXITOSA ✔", fg="#00ff00")
            self.system_state_label.config(text="Sistema: ACTIVO", fg="#00ff00")
        elif status == "FAIL":
            self.calib_label.config(text="Calibración: FALLIDA ❌", fg="red")
            self.system_state_label.config(text="Sistema: DETENIDO", fg="red")
        elif status == "STOP":
            self.calib_label.config(text="Calibración: Pendiente", fg="gray")
            self.system_state_label.config(text="Sistema: DETENIDO 🛑", fg="#ff4444")

    def update_tuner_ui(self, freq, state, string_id):
        """Actualiza la interfaz de afinación (frecuencia, estado, color)."""
        # Solo actualiza la afinación si el sistema está activo (calibración OK)
        if self.system_state_label.cget("text") != "Sistema: ACTIVO":
             # Esto evita que se muestren datos de afinación cuando no deberia
             return

        # Cuerda
        if 0 <= string_id < len(STRING_NAMES):
            self.string_label.config(text=f"Cuerda: {STRING_NAMES[string_id]}")
        else:
            self.string_label.config(text="Cuerda: --")

        # Frecuencia
        self.freq_label.config(text=f"Frecuencia: {freq} Hz")

        # Estado + color
        if state == "OK":
            color = "green"
            txt = "Afinada ✔"
        elif state == "TENSAR":
            color = "yellow"
            txt = "Tensar ↑"
        elif state == "DESTENSAR":
            color = "red"
            txt = "Destensar ↓"
        else:
            color = "gray"
            txt = "--"

        self.canvas.itemconfig(self.indicator, fill=color)
        self.state_label.config(text=f"Afinación: {txt}")

    # Necesitamos esta función para manejar los datos cuando el sistema está detenido o calibrando
    def update_ui(self, freq, state, string_id):
        # Esta función ahora solo es un wrapper por compatibilidad.
        # En el caso de mensajes de afinación, siempre llamaremos a update_tuner_ui
        self.update_tuner_ui(freq, state, string_id)


# ---------------------------------------------------
# LECTURA UART EN HILO SEPARADO
# ---------------------------------------------------
def serial_reader(gui):
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Puerto serial {SERIAL_PORT} abierto.")
        # Si se conecta, forzamos el estado a "STOP" o "Pendiente" si no hay mensajes
        gui.root.after(0, lambda: gui.update_calibration_ui('STOP'))
    except serial.SerialException:
        print(f"❌ No se pudo abrir el puerto serial {SERIAL_PORT}. Asegúrate de que el puerto sea correcto.")
        # Si no hay conexión, mostramos el error
        gui.root.after(0, lambda: gui.system_state_label.config(text="Sistema: ERROR SERIAL", fg="red"))
        return

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        
        # 1. Manejo de mensajes de Control/Calibración (ej. CONTROL:INIT)
        if line.startswith("CONTROL:"):
            control_state = line.split(":")[1]
            # Usar gui.root.after para actualizar la GUI desde el hilo
            gui.root.after(0, lambda: gui.update_calibration_ui(control_state))
            continue

        # 2. Manejo de mensajes de Afinación (ej. freq=110;state=TENSAR;string=3)
        try:
            parts = line.split(";")
            
            # Solo procesa si tiene las tres partes del protocolo de afinación
            if len(parts) != 3:
                continue

            freq = int(parts[0].split("=")[1])
            state = parts[1].split("=")[1]
            string_id = int(parts[2].split("=")[1])

            # Usar gui.root.after para actualizar la GUI desde el hilo
            gui.root.after(0, lambda: gui.update_tuner_ui(freq, state, string_id))

        except Exception as e:
            print(f"Error parseando mensaje de afinación: '{line}' | Error: {e}")


# ---------------------------------------------------
# MAIN
# ---------------------------------------------------
root = tk.Tk()
gui = TunerGUI(root)

# Inicializar el estado de la GUI
gui.system_state_label.config(text="Sistema: Conectando...", fg="#ffaa00")

thread = threading.Thread(target=serial_reader, args=(gui,), daemon=True)
thread.start()

root.mainloop()