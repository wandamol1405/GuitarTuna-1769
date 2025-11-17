import serial
import threading
import tkinter as tk
from tkinter import ttk

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
        self.root.title("AutoTune1769")
        self.root.geometry("400x500")
        self.root.configure(bg="#222222")

        title = tk.Label(root, text="AutoTune1769", font=("Arial", 22, "bold"), fg="white", bg="#222222")
        title.pack(pady=10)

        self.string_label = tk.Label(root, text="Cuerda: --", font=("Arial", 20), fg="white", bg="#222222")
        self.string_label.pack(pady=10)

        self.freq_label = tk.Label(root, text="Frecuencia: -- Hz", font=("Arial", 18), fg="#cccccc", bg="#222222")
        self.freq_label.pack(pady=10)

        # Círculo indicador
        self.canvas = tk.Canvas(root, width=200, height=200, bg="#222222", highlightthickness=0)
        self.canvas.pack(pady=20)
        self.indicator = self.canvas.create_oval(20, 20, 180, 180, fill="gray")

        self.state_label = tk.Label(root, text="Estado: --", font=("Arial", 22, "bold"), fg="white", bg="#222222")
        self.state_label.pack(pady=20)

    def update_ui(self, freq, state, string_id):
        """Actualiza todos los elementos visuales"""

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
        self.state_label.config(text=f"Estado: {txt}")


# ---------------------------------------------------
# LECTURA UART EN HILO SEPARADO
# ---------------------------------------------------
def serial_reader(gui):
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Puerto serial abierto.")
    except:
        print("❌ No se pudo abrir el puerto serial.")
        return

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        # Ejemplo esperado:
        # freq=110;state=TENSAR;string=3
        try:
            parts = line.split(";")
            freq = int(parts[0].split("=")[1])
            state = parts[1].split("=")[1]
            string_id = int(parts[2].split("=")[1])

            gui.update_ui(freq, state, string_id)

        except Exception as e:
            print("Error parseando:", line, e)


# ---------------------------------------------------
# MAIN
# ---------------------------------------------------
root = tk.Tk()
gui = TunerGUI(root)

thread = threading.Thread(target=serial_reader, args=(gui,), daemon=True)
thread.start()

root.mainloop()
