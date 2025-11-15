/* afinador_systick_crono.c
 * LPC1769 — SysTick cronómetro (100 MHz) — ADC — MAX9814
 * 
 * Implementación con SysTick como cronómetro de alta precisión
 * para medir directamente el período entre cruces por cero
 */

#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ============================
   CONFIGURACIÓN GENERAL
   ============================ */

#define CPU_CLOCK      100000000UL   // Reloj del CPU: 100 MHz = 100,000,000 Hz
#define OFFSET_DEFAULT 1800          // Valor medio del ADC cuando no hay sonido (1.65V ≈ 1800)
#define SYSTICK_LOAD   0x00FFFFFFUL  // Valor máximo que puede contar SysTick (24 bits)
#define NOISE_TH       8             // Umbral de ruido: ignora variaciones menores a 8 unidades ADC
#define TARGET_FREQ    82.41f        // Frecuencia objetivo: Mi grave de guitarra (E2)
#define TOLERANCE      1.0f          // Tolerancia de afinación: ±1 Hz

// Buffer circular de módulo 2: guarda solo las 2 últimas muestras
typedef struct {
    uint16_t samples[2];  // Array de 2 muestras: [muestra_anterior, muestra_actual]
    uint8_t idx;          // Índice actual (0 o 1)
} circular_buffer_t;

// Variables globales volátiles (se modifican en interrupciones)
volatile circular_buffer_t buffer = {{0, 0}, 0};  // Buffer circular inicializado en ceros
volatile uint16_t offset = OFFSET_DEFAULT;        // Valor de referencia para señal centrada
volatile uint16_t noise_th = NOISE_TH;            // Umbral de ruido actual

volatile int cruce_count = 0;      // Contador de cruces por cero detectados
volatile uint32_t t_ini = 0;       // Tiempo inicial (primer cruce)
volatile uint32_t t_fin = 0;       // Tiempo final (segundo cruce)  
volatile float frecuencia = 0.0f;  // Frecuencia calculada
volatile int medicion_completa = 0; // Bandera: 1 cuando hay nueva medición lista

/* ============================
   FUNCIONES UART (COMUNICACIÓN SERIAL)
   ============================ */

// Envia un string por el puerto serial
void send_string(const char *s){
    UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t*)s, strlen(s), BLOCKING);
}

// Convierte número entero a string (ej: 123 -> "123")
void itoa_simple(int n, char s[]){
    int i = 0, sign = n;
    if (n < 0) n = -n;  // Si es negativo, lo hace positivo temporalmente
    do { 
        s[i++] = n % 10 + '0';  // Extrae último dígito y convierte a carácter
    } while((n /= 10) > 0);     // Divide por 10 hasta que sea 0
    if (sign < 0) s[i++] = '-'; // Agrega signo negativo si era negativo
    s[i] = '\0';  // Fin del string

    // Invierte el string porque los dígitos se extrajeron al revés
    int j = 0; 
    i--;
    while (j < i){
        char t = s[j];
        s[j] = s[i];
        s[i] = t;
        j++; i--;
    }
}

// Convierte float a string (ej: 82.41 -> "82.41")
void ftoa_simple(float f, char *str) {
    int integer_part = (int)f;  // Parte entera: 82
    int decimal_part = (int)((f - integer_part) * 100);  // Parte decimal: 41
    
    itoa_simple(integer_part, str);  // Convierte parte entera
    int len = strlen(str);
    str[len] = '.';  // Agrega punto decimal
    itoa_simple(decimal_part, &str[len + 1]);  // Convierte parte decimal
}

// Analiza la frecuencia y dice si hay que tensar o destensar
void reportarAfinacion(float f){
    char out[32];
    ftoa_simple(f, out);  // Convierte frecuencia a string

    send_string("Frecuencia: ");
    send_string(out);
    send_string(" Hz => ");

    // Compara con frecuencia objetivo
    if (fabsf(f - TARGET_FREQ) <= TOLERANCE){
        send_string("CUERDA AFINADA ✓\r\n");
    }
    else if (f < TARGET_FREQ){
        float diff = TARGET_FREQ - f;  // Cuánto falta subir
        ftoa_simple(diff, out);
        send_string("TENSAR (+");
        send_string(out);
        send_string(" Hz)\r\n");
    }
    else {
        float diff = f - TARGET_FREQ;  // Cuánto falta bajar
        ftoa_simple(diff, out);
        send_string("DESTENSAR (-");
        send_string(out);
        send_string(" Hz)\r\n");
    }
}

/* ============================
   SISTICK COMO CRONÓMETRO DE ALTA PRECISIÓN
   ============================ */

// Inicializa SysTick como cronómetro (sin interrupciones)
void SysTick_InitCrono(void){
    SysTick->CTRL = 0;                  // Apaga SysTick
    SysTick->LOAD = SYSTICK_LOAD;       // Máximo valor de cuenta (16,777,215)
    SysTick->VAL  = 0;                  // Reinicia contador a CERO
    // NOTA: No habilita interrupciones - solo lo usa como reloj
}

// Arranca el cronómetro
void SysTick_Start(void){
    SysTick->VAL = 0;                   // Reinicia contador a CERO
    SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
}

// Detiene el cronómetro
void SysTick_Stop(void){
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

// Lee el valor actual del cronómetro (cuenta REGRESIVA)
uint32_t SysTick_Read(void){
    return SysTick->VAL; // Ej: si VAL=50,000, significa que pasaron (LOAD - 50,000) ticks
}

// Calcula cuántos ticks pasaron entre inicio y fin (maneja desbordamiento)
uint32_t SysTick_Elapsed(uint32_t inicio, uint32_t fin){
    if (fin <= inicio) {
        // Caso normal: sin desbordamiento
        // Ej: inicio=1,000,000, fin=500,000 -> pasaron 500,000 ticks
        return inicio - fin;
    } else {
        // Caso con desbordamiento: el contador dio vuelta
        // Ej: inicio=100,000, fin=16,000,000 -> pasaron (16,777,215-16,000,000) + 100,000 + 1
        return (SYSTICK_LOAD - fin) + inicio + 1;
    }
}

/* ============================
   BUFFER CIRCULAR - OPERACIONES
   ============================ */

// Agrega nueva muestra al buffer (solo guarda las 2 últimas)
static inline void buffer_put(uint16_t sample){
    buffer.samples[buffer.idx] = sample;     // Guarda en posición actual
    buffer.idx = (buffer.idx + 1) & 0x1;     // Avanza índice (0->1, 1->0)
}

// Obtiene la muestra ANTERIOR (n-1)
static inline uint16_t buffer_get_prev(void){
    return buffer.samples[buffer.idx];       // Índice actual apunta al más viejo
}

// Obtiene la muestra ACTUAL (n)
static inline uint16_t buffer_get_curr(void){
    return buffer.samples[(buffer.idx + 1) & 0x1];  // Siguiente posición es la más nueva
}

/* ============================
   DETECCIÓN DE CRUCE POR CERO
   ============================ */

// Detecta cuando la señal pasa de negativa a positiva
static inline int detectar_cruce_ascendente(void){
    uint16_t prev = buffer_get_prev();  // Muestra anterior
    uint16_t curr = buffer_get_curr();  // Muestra actual
    
    // Centrar señal: restar el offset (valor de reposo)
    int16_t p = (int16_t)(prev - offset);  // Señal anterior centrada
    int16_t c = (int16_t)(curr - offset);  // Señal actual centrada
    
    // Eliminar ruido: si ambas muestras están cerca del cero, ignorar
    if (abs(p) <= noise_th && abs(c) <= noise_th){
        return 0;  // No es un cruce real, es ruido
    }
    
    // Cruce ascendente: anterior era negativo/cero y actual es positivo
    return (p <= 0 && c > 0);
}

/* ============================
   CONFIGURACIÓN DEL ADC
   ============================ */

void cfg_ADC(void){
    // Configura pin P0.23 como entrada analógica (ADC0.0)
    PINSEL_CFG_Type pin;
    pin.Portnum   = 0;
    pin.Pinnum    = 23;
    pin.Funcnum   = 1;      // Función ADC
    pin.Pinmode   = PINSEL_PINMODE_TRISTATE;
    pin.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&pin);

    ADC_Init(LPC_ADC, 10000);          // Inicializa ADC a 10 KHz
    ADC_ChannelCmd(LPC_ADC, 0, ENABLE); // Habilita canal 0
    ADC_BurstCmd(LPC_ADC, ENABLE);     // Modo continuo (conversiones automáticas)
    LPC_ADC->ADINTEN = (1 << 0);       // Habilita interrupción del canal 0
    NVIC_EnableIRQ(ADC_IRQn);          // Habilita interrupción en el NVIC
}

/* ============================
   CONFIGURACIÓN DEL UART
   ============================ */

void cfg_UART(void){
    PINSEL_CFG_Type pinTX, pinRX;

    // Configura P0.2 como TXD0
    pinTX.Portnum = 0; pinTX.Pinnum = 2; pinTX.Funcnum = 1;
    pinTX.Pinmode = PINSEL_PINMODE_TRISTATE; pinTX.OpenDrain = PINSEL_PINMODE_NORMAL;

    // Configura P0.3 como RXD0  
    pinRX.Portnum = 0; pinRX.Pinnum = 3; pinRX.Funcnum = 1;
    pinRX.Pinmode = PINSEL_PINMODE_TRISTATE; pinRX.OpenDrain = PINSEL_PINMODE_NORMAL;

    PINSEL_ConfigPin(&pinTX);
    PINSEL_ConfigPin(&pinRX);

    UART_CFG_Type UARTConfig;
    UART_FIFO_CFG_Type FIFOConfig;

    UART_ConfigStructInit(&UARTConfig);      // Configuración por defecto
    UART_FIFOConfigStructInit(&FIFOConfig);  // Configuración FIFO por defecto

    UART_Init((LPC_UART_TypeDef *)LPC_UART0, &UARTConfig);
    UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART0, &FIFOConfig);
    UART_TxCmd((LPC_UART_TypeDef *)LPC_UART0, ENABLE);  // Habilita transmisión
}

/* ============================
   CALIBRACIÓN AUTOMÁTICA
   ============================ */

// Calcula el valor de offset (silencia) del micrófono
void calibrar_offset(void){
    send_string("Calibrando offset...\r\n");
    
    // Deshabilitar IRQ temporalmente para no interferir
    NVIC_DisableIRQ(ADC_IRQn);
    
    uint32_t suma = 0;
    const int num_muestras = 100;  // Número de muestras para promediar
    
    // Tomar muestras cuando no hay sonido
    for(int i = 0; i < num_muestras; i++){
        while(!(LPC_ADC->ADGDR & (1UL << 31)));  // Espera a que termine conversión
        uint16_t muestra = (LPC_ADC->ADGDR >> 4) & 0xFFF;  // Extrae valor ADC (12 bits)
        suma += muestra;  // Acumula para promedio
    }
    
    offset = suma / num_muestras;  // Calcula valor promedio (offset)
    
    // Rehabilitar IRQ
    NVIC_EnableIRQ(ADC_IRQn);
    
    char msg[50];
    itoa_simple(offset, msg);
    send_string("Offset calibrado: ");
    send_string(msg);
    send_string("\r\n");
}

/* ============================
   ADC IRQ — MEDICIÓN CON SISTICK
   ============================ */

// Interrupción que se ejecuta CADA VEZ que el ADC termina una conversión
void ADC_IRQHandler(void){
    if (ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)) {
        // Lee el valor convertido (12 bits)
        uint16_t sample = ADC_ChannelGetData(LPC_ADC, 0) & 0xFFF;
        
        // Agregar muestra al buffer circular (solo guarda 2 últimas)
        buffer_put(sample);
        
        // Verifica si hay cruce por cero ascendente
        if (detectar_cruce_ascendente()){
            cruce_count++;  // Incrementa contador de cruces

            switch(cruce_count){
                case 1:
                    // PRIMER CRUCE: inicia medición
                    SysTick_Start();      // Arranca cronómetro
                    t_ini = SysTick_Read();  // Guarda tiempo inicial
                    break;
                    
                case 2:
                    // SEGUNDO CRUCE: terminó un período completo
                    t_fin = SysTick_Read();  // Guarda tiempo final
                    
                    // Calcula ticks entre primer y segundo cruce
                    uint32_t ticks_periodo = SysTick_Elapsed(t_ini, t_fin);
                    
                    if (ticks_periodo > 0) {
                        // Calcula frecuencia: f = 100,000,000 / ticks_del_periodo
                        frecuencia = (float)CPU_CLOCK / (float)ticks_periodo;
                        medicion_completa = 1;  // Avisa que hay nueva medición
                    }
                    
                    // Reinicia para siguiente medición
                    cruce_count = 0;
                    SysTick_Stop();  // Detiene cronómetro
                    break;
            }
        }
    }
}

/* ============================
   MAIN - PROGRAMA PRINCIPAL
   ============================ */

int main(void){
    SystemInit();           // Configuración inicial del sistema
    SysTick_InitCrono();   // Inicializa SysTick como cronómetro
    cfg_UART();            // Configura comunicación serial
    cfg_ADC();             // Configura ADC para leer micrófono
    
    send_string("Afinador de Guitarra - SysTick Cronometro\r\n");
    send_string("Objetivo: Mi grave (E2) = 82.41 Hz\r\n");
    send_string("==============================\r\n");
    
    // Calibrar offset al inicio (cuando no hay sonido)
    calibrar_offset();

    // Loop infinito principal
    while (1){
        if (medicion_completa){
            medicion_completa = 0;  // Reinicia bandera
            
            // Filtra frecuencias fuera de rango razonable para guitarra
            if (frecuencia > 60.0f && frecuencia < 150.0f){
                reportarAfinacion(frecuencia);  // Muestra resultado
            } else {
                send_string("Señal fuera de rango - toque la cuerda Mi\r\n");
            }
        }
        
        // Pequeño delay para no saturar la UART con mensajes
        for(volatile int i = 0; i < 100000; i++);
    }
}
