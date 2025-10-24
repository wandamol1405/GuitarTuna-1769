# AutoTune-1769
## Afinador Digital de Guitarra Criolla (LPC1769)

### 🎼 Resumen del Proyecto

Este proyecto consiste en el desarrollo de un **afinador digital de guitarra criolla** implementado como un sistema embebido. Utiliza el microcontrolador **LPC1769** para capturar, procesar y analizar la señal acústica de las cuerdas, indicando al usuario si la afinación es correcta o si se requiere ajustar la tensión.

---

### 🎯 Objetivo

Desarrollar un sistema embebido capaz de analizar una señal acústica proveniente de una guitarra criolla y determinar si la cuerda seleccionada se encuentra correctamente afinada, utilizando el método de cruce por cero por su simplicidad y eficiencia computacional.

---

### ⚙️ Componentes Principales

| Componente | Función |
| :--- | :--- |
| **Microcontrolador (MCU)** | LPC1769 |
| **Sensor de Audio** | Módulo KY-037 (micrófono electret + preamplificador) |
| **Conversión A/D** | ADC interno del LPC1769 |
| **Comunicación** | UART (para visualización del estado de afinación) |
| **Interfaz de Usuario**| Botón para la selección secuencial de cuerdas |

---

### 💻 Funcionamiento del Sistema

El afinador opera en las siguientes etapas:

1.  **Captura de Señal:** El módulo KY-037 capta la vibración de la cuerda.
2.  **Muestreo:** El ADC del LPC1769 muestrea la señal analógica, con la frecuencia controlada por un *TIMER* para garantizar estabilidad. Las muestras se almacenan temporalmente en la SRAM.
3.  **Procesamiento de Frecuencia:** Se aplica el **Método de Cruce por Cero** para calcular la frecuencia fundamental ($f$):
    $$f = \frac{N_{cruces}}{2 \cdot T}$$
    Donde $N_{cruces}$ es el número de veces que la señal cruza el valor medio (cero lógico) y $T$ es el tiempo total de muestreo.
4.  **Comparación y Estado:** La frecuencia medida se compara con las frecuencias nominales de la guitarra (almacenadas en una tabla interna).
    * `f_medida < f_nominal` → **Tensar**
    * `f_medida > f_nominal` → **Destensar**
    * `f_medida ≈ f_nominal` → **Afinada**
5.  **Comunicación:** El estado de afinación se transmite al usuario mediante la interfaz **UART** (visualizado en una consola de PC).

---

### 🛠️ Diagrama de Bloques (Conceptual)

<img width="790" height="586" alt="Blank diagram" src="https://github.com/user-attachments/assets/10708772-a425-4dfa-bf95-a0b78df683b7" />

---

### 📡 Instrucciones de Uso (Interfaz UART)

1.  Conectar el LPC1769 a la PC vía UART.
2.  Utilizar el botón de interrupción para seleccionar la cuerda a afinar.
3.  Tocar la cuerda.
4.  La consola mostrará el estado actual: "Tensar", "Destensar" o "Afinada".

---

### 🚀 Tecnología Utilizada

* **Microcontrolador:** NXP LPC1769
* **Lenguaje:** C/C++ para programación embebida.
* **Algoritmo:** Detección de Cruce por Cero.
