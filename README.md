# 🎸 Afinador de Guitarra Digital (LPC1769)

## 📝 Descripción del Proyecto

Este proyecto implementa un **afinador de guitarra digital** utilizando el microcontrolador **NXP LPC1769**. El sistema captura la señal de audio de un micrófono mediante el conversor Analógico-Digital (**ADC**), utiliza el módulo de Acceso Directo a Memoria (**GPDMA**) para muestreo continuo y en tiempo real, y aplica algoritmos de procesamiento de señal para estimar la frecuencia de la cuerda que está vibrando.

El sistema proporciona retroalimentación al usuario a través de **LEDs** (Rojo, Amarillo, Verde) e información detallada vía **UART** para ser visualizada en una interfaz gráfica de usuario (GUI) externa.

---

## ✨ Características Clave

* **Muestreo en Tiempo Real:** Configuración de ADC y Timer para una tasa de muestreo precisa de **20 kHz**.
* **Transferencia Eficiente:** Uso de **DMA con Lista Enlazada (LLI)** y doble buffer (ping-pong) para la adquisición continua de muestras sin intervención de la CPU.
* **Calibración Automática:** Rutina de calibración al inicio para determinar el **offset DC** del micrófono y el **umbral de ruido** de fondo ($\sigma$).
* **Estimación de Frecuencia:** Implementación de **filtros digitales (HPF y LPF)** y detección de **cruces por cero** para calcular el periodo fundamental de la señal.
* **Interfaz Visual y Serial:**
    * **LEDs:** Indica el estado de afinación (TENSAR, OK, DESTENSAR).
    * **UART:** Envía información estructurada (`freq=XXX;state=YYY;string=Z\r\n`) para la comunicación con una GUI.

---

## 💻 Implementación Técnica (Hardware)

| Periférico | Función | Pin | Configuración |
| :--- | :--- | :--- | :--- |
| **ADC** | Entrada de Audio/Micrófono | P0.23 (AD0.0) | Disparado por Timer0 Match 1 (20 kHz) |
| **Timer0** | Generación de Frecuencia | MAT0.1 | Período de 50 us |
| **GPDMA** | Transferencia de Datos | Canal 0 | Periférico a Memoria (ADC $\to$ SRAM), LLI Dual Buffer |
| **UART0** | Comunicación Serial | P0.2 (TXD0), P0.3 (RXD0) | Envío de estado de afinación |
| **EINT0** | Control de Inicio | P2.10 | Activa/Desactiva el sistema y modo Calibración |
| **EINT1** | Selección de Cuerda | P2.11 | Cicla a través de las 6 cuerdas |
| **GPIO (LED)** | Indicador de Estado | P0.27 (Rojo), P0.28 (Verde), P2.13 (Amarillo) | Retroalimentación de afinación |

### Frecuencias Objetivo (Cuerdas Estándar)

El sistema soporta la afinación estándar de una guitarra de 6 cuerdas:

| Índice (curr\_string) | Cuerda | Frecuencia Objetivo (Hz) |
| :--- | :--- | :--- |
| 0 | Mi (E) - Aguda | 340 Hz |
| 1 | Si (B) | 266 Hz |
| 2 | Sol (G) | 222 Hz |
| 3 | Re (D) | 171 Hz |
| 4 | La (A) | 178 Hz |
| 5 | Mi (E) - Grave | 205 Hz |

---

## 🚀 Instrucciones de Uso

Para compilar y ejecutar este proyecto en un entorno de desarrollo integrado (IDE) como MCUXpresso o similar, sigue estos pasos:

### 1. Requisitos de Compilación

* Entorno de desarrollo **MCUXpresso IDE**.
* Herramientas de compilación **`arm-none-eabi-gcc`** (Toolchain).
* Librerías **CMSISv2p00\_LPC17xx** (Ya incluidas en la configuración del proyecto).

### 2. Ejecución

1.  **Cargar el Código:** Compila y carga el archivo `main.axf` en la placa LPC1769.
2.  **Iniciar Calibración:** Presiona el botón conectado a **EINT0 (P2.10)**. Esto inicia el sistema y entra en el modo de calibración.
3.  **Afinación:**
    * **Seleccionar Cuerda:** Pulsa el botón conectado a **EINT1 (P2.11)** para ciclar a través de las 6 cuerdas.
    * **Afinar:** Toca la cuerda que desees afinar. Observa los LEDs:
        * **🟢 Verde:** Afinado (Dentro de $ \pm 15$ Hz del objetivo).
        * **🟡 Amarillo:** Demasiado bajo (TENSAR).
        * **🔴 Rojo:** Demasiado alto (DESTENSAR).
4.  **Monitoreo:** Conecta un terminal serial (9600 baudios, 8N1) a **UART0 (P0.2/P0.3)** para visualizar los mensajes de estado:
    ```
    freq=215;state=TENSAR;string=2
    freq=222;state=OK;string=2
    ```

### 3. Vizualizacion en la GUI
La GUI requiere el módulo pyserial para comunicarse con la placa. Abre tu terminal o símbolo del sistema e instala pyserial:

    pip install pyserial

Ejecutas la GUI desde la carpeta actual donde se encuentra tuner_gui.py de la siguiente forma:

    python tuner_gui.py
