import serial
import matplotlib.pyplot as plt
from collections import deque

# Ajustá el puerto y velocidad según tu Arduino
PORT = 'COM9'
BAUD = 9600

ser = serial.Serial(PORT, BAUD, timeout=1)

# Guardamos los últimos 200 valores
data = deque([0]*200, maxlen=200)

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(data)
ax.set_ylim(0, 4095)  # Rango del ADC (12 bits)
ax.set_xlabel('Muestras')
ax.set_ylabel('Valor ADC')

while True:
    try:
        line_data = ser.readline().decode().strip()
        if line_data.isdigit():
            value = int(line_data)
            data.append(value)
            line.set_ydata(data)
            line.set_xdata(range(len(data)))
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.001)
    except KeyboardInterrupt:
        print("Cerrando...")
        break
    except Exception as e:
        print("Error:", e)
        continue

ser.close()
