# AutoTune-1769

## Afinador Digital de Guitarra Criolla

### Descripción del Proyecto

Este proyecto desarrolla un **afinador digital de guitarra criolla** basado en el microcontrolador **LPC1769** y un módulo micrófono **KY-037**. El sistema permite afinar las cuerdas de una guitarra criolla de manera precisa mediante el análisis de frecuencias en tiempo real.

### Componentes Principales

- **Microcontrolador**: LPC1769 (ARM Cortex-M3)
- **Sensor de Audio**: Módulo micrófono KY-037
- **Interfaz de Comunicación**: UART
- **Interfaz de Usuario**: Botón de selección de cuerda

### Funcionamiento del Sistema

El afinador digital funciona mediante el siguiente proceso:

1. **Captura de Señal Acústica**: El módulo micrófono KY-037 capta la vibración sonora de la cuerda de la guitarra.

2. **Digitalización**: La señal analógica es convertida a formato digital mediante el conversor ADC (Analog-to-Digital Converter) integrado en el LPC1769.

3. **Cálculo de Frecuencia**: Se utiliza el **método de cruce por cero** para determinar la frecuencia fundamental de la señal captada. Este método cuenta el número de veces que la señal cruza el valor cero en un intervalo de tiempo determinado.

4. **Comparación con Frecuencia Nominal**: El sistema compara la frecuencia medida con la frecuencia nominal correspondiente a la cuerda seleccionada.

5. **Indicación de Afinación**: A través de la comunicación **UART**, el sistema proporciona retroalimentación al usuario indicando:
   - **Tensar**: Si la frecuencia es menor a la nominal (nota más grave)
   - **Destensar**: Si la frecuencia es mayor a la nominal (nota más aguda)
   - **Afinada**: Si la frecuencia está dentro del rango aceptable

6. **Selección de Cuerda**: Un botón permite al usuario seleccionar qué cuerda desea afinar (de la 1ª a la 6ª cuerda).

### Frecuencias Nominales de Guitarra Criolla

Las frecuencias estándar de afinación para una guitarra criolla son:

| Cuerda | Nota | Frecuencia (Hz) |
|--------|------|-----------------|
| 1ª     | E4   | 329.63          |
| 2ª     | B3   | 246.94          |
| 3ª     | G3   | 196.00          |
| 4ª     | D3   | 146.83          |
| 5ª     | A2   | 110.00          |
| 6ª     | E2   | 82.41           |

### Método de Cruce por Cero

El **método de cruce por cero** (Zero-Crossing Method) es una técnica eficiente para determinar la frecuencia de señales periódicas:

- Detecta cada vez que la señal cambia de signo (cruza el eje horizontal)
- Cuenta el número de cruces en un intervalo de tiempo conocido
- Calcula la frecuencia usando la fórmula: `f = N / (2 × T)`, donde:
  - `N` = número de cruces por cero
  - `T` = tiempo de medición
  - `f` = frecuencia resultante

### Características Técnicas

- **Procesamiento en tiempo real**: Análisis continuo de la señal de audio
- **Alta precisión**: Detección precisa de frecuencias mediante ADC de alta resolución
- **Interfaz simple**: Comunicación UART para fácil visualización de resultados
- **Portabilidad**: Sistema compacto basado en microcontrolador

### Aplicaciones

- Afinación de guitarras criollas (acústicas)
- Herramienta educativa para aprender sobre procesamiento de señales
- Base para desarrollos de afinadores más complejos con múltiples instrumentos

### Tecnologías Utilizadas

- **Lenguaje**: C/C++ (firmware embebido)
- **Hardware**: LPC1769, KY-037
- **Periféricos**: ADC, UART, GPIO
- **Algoritmos**: Procesamiento de señales digitales, detección de cruce por cero

---

**Desarrollado para aplicaciones de procesamiento de señales y sistemas embebidos**