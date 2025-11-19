/**
 * @file main.c
 * @brief Afinador de guitarra para LPC1769 (AutoTune-1769)
 *
 * Programa principal que configura periféricos y orquesta el funcionamiento
 * del afinador:
 *  - ADC (canal 0, P0.23) muestreado a 20 kHz disparado por Timer0 (Match1).
 *  - GPDMA con descriptores LLI para transferencia continua de muestras a SRAM.
 *  - Timer0 para generar los eventos de muestreo.
 *  - UART0 para enviar estado/telemetría hacia una GUI.
 *  - GPIO y EXTI para control de LEDs e interacción por botones (inicio/cambio de cuerda).
 *
 * Características principales:
 *  - Calibración automática del micrófono (offset y umbral de ruido) con descarte
 *    de outliers y verificación de estabilidad.
 *  - Procesado en tiempo real: filtrado HPF + LPF, detección de cruces por cero y
 *    estimación de frecuencia por periodo medio de cruces.
 *  - Promediado de frecuencias recientes para reducir jitter y decisión de estado
 *    (OK / TENSAR / DESTENSAR) indicada por LEDs y enviada por UART.
 *
 * Requisitos: LPC1769, micrófono (ej. MAX9814), circuito de botones y LEDs.
 * Fecha: Noviembre 2024
 */

#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_gpio.h"
#include <string.h>
#include <math.h>

// Estructura para la configuración de la LLI del DMA
typedef struct {
    uint32_t SrcAddr;
    uint32_t DstAddr;
    uint32_t NextLLI;
    uint32_t Control;
} myLLI_t;

/* MACRO PARA EL MUESTREO CONTINUO DEL MICROFONO*/
#define ADC_RATE 20000                                              // Frecuencia de muestreo del ADC (20 kHz)
#define NUM_SAMPLES 2048                                            // Número de muestras por buffer
#define LLI_SIZE (sizeof(myLLI_t))                                  // Tamaño de una LLI
#define BUFFER_SIZE (NUM_SAMPLES * sizeof(uint32_t))                // Tamaño del buffer de muestras
#define AHB_BASE_ADDR 0x2007C000UL                                  // Base real de la RAM AHB (32 KB)
#define AHB_BASE_ADDR2  (AHB_BASE_ADDR + LLI_SIZE + BUFFER_SIZE + 4) // Ajustar según NUM_SAMPLES

/* MACROS PARA LA CALIBRACION DEL MICROFONO */
#define NUM_SAMPLES_CALIBRATION 256                                 // Cantidad de muestras para calibración
#define SIGMA_THRESHOLD 76                                          // Umbral de sigma para considerar señal válida (~3 mV)
#define OUTLIER_THRESHOLD 50                                        // Umbral para descartar picos bruscos en la señal
#define DISCARD_SAMPLES 16

/* MACROS PARA LA ESTIMACION DE LA FRECUENCIA*/
#define ALPHA_SCALER 1000                                           // Escala para cálculos enteros
#define ALPHA_COEFF 990                                             // Coeficiente alpha = 0.99 (corte bajo ~100 Hz)
#define BETA_SCALER 1000                                            // Escala para cálculos enteros
#define BETA_COEFF 700                                              // Coeficiente Beta = 0.9 (Suavizado)
#define ONE_MINUS_BETA_COEFF 300                                    // Coeficiente 1 - Beta = 0.1
#define FREQUENCY_BUFFER_SIZE 50                                    // Tamaño del buffer circular de frecuencias

/* MACROS PARA LA COMPARACION DE FRECUENCIAS Y ESTADO DE LEDS */
#define STRINGS 6
#define FREQUENCY_THRESHOLD 10

/* MACROS DE LOS LEDS*/
#define LED_RED (1<<27)    // P0.27
#define LED_GREEN (1<<28)  // P0.28
#define LED_YELLOW (1<<13) // P1.13

/* Prototipos de funciones */
void cfgADC(void);
void cfgUART(void);
void cfgTimer(void);
void cfgDMA(void);
void cfgGPIO(void);
void cfgEINT(void);
void sendState(uint32_t frequency, uint8_t state, uint8_t string);
void itoaSimple(int, char*);
int calibrateMicrophone(void);
uint32_t estimateFrequency(uint32_t *samples);
void compareFrequency(uint32_t frequency);

myLLI_t *cfgLLI_A  = (myLLI_t *)AHB_BASE_ADDR;                       // Configuración LLI A
myLLI_t *cfgLLI_B  = (myLLI_t *)(AHB_BASE_ADDR + sizeof(myLLI_t));   // Configuración LLI B

/* Buffers */
volatile uint32_t bufferCalibration[NUM_SAMPLES_CALIBRATION]; // Buffer para calibración
volatile uint32_t *bufferADC_A  = (volatile uint32_t *)(AHB_BASE_ADDR + 2 * sizeof(myLLI_t)); // Buffer para datos ADC
volatile uint32_t *bufferADC_B  = (volatile uint32_t *)(AHB_BASE_ADDR2 + 2 * sizeof(myLLI_t) + NUM_SAMPLES * sizeof(uint32_t)); // Buffer para datos ADC

/* Variables para la calibracion */
volatile uint16_t calibration_offset = 0;               // Offset calculado en la calibración
volatile uint16_t noise_threshold = SIGMA_THRESHOLD;    // Umbral de ruido para detección
volatile uint16_t calibration_count = 0;                              // Contador de muestras para calibración
volatile uint8_t calibration_mode = 0;                      // Bandera para indicar modo calibración
volatile uint8_t calibrated = 0;                            // Bandera para indicar si ya se calibró

/* Flags de estado de buffers */
volatile uint8_t buffer_ready_dma = 0;           // Bandera para indicar que el buffer DMA está listo
volatile uint8_t buffer_ready_calibrate = 0;     // Bandera para indicar que el buffer de calibración está listo

/* Cuerdas */
static uint16_t strings[STRINGS] = {342, 280, 209, 165, 147, 202};  // Frecuencias objetivo de las cuerdas (Hz)
static uint8_t curr_string = 0;                                     // Índice de la cuerda actual

static uint8_t start = 0;       // Bandera para iniciar el sistema


/** -----------------  MAIN ------------------- */
/**
* @brief Función principal
* Configura periféricos y entra en loop principal
*/
int main(void) {
    SystemInit();       // Configuración inicial del sistema
    cfgUART();          // Configura UART
    cfgADC();           // Configura ADC
    cfgTimer();         // Configura Timer
    cfgDMA();           // Configura DMA
    cfgEINT();          // Configura EINT
    cfgGPIO();           // Configura GPIO

	static uint32_t freq_buffer[FREQUENCY_BUFFER_SIZE]; // Buffer circular para almacenar frecuencias recientes y promediar
	static uint8_t freq_idx = 0;                        // Índice del buffer circular
	static uint8_t freq_count = 0;                      // Cantidad de frecuencias almacenadas
	volatile uint32_t frequency = 0;                    // Frecuencia estimada

    while (1){
    	if(start){
        	// INICIO DEL SISTEMA -> DESCALIBRADO
            if(calibration_mode){
            	//Entra en modo de calibracion
                NVIC_EnableIRQ(ADC_IRQn);               // Habilita interrupción ADC para calibración
                if (buffer_ready_calibrate) {           // Si el buffer de calibración está listo
                    buffer_ready_calibrate = 0;         // Resetear bandera
                    if (calibrateMicrophone()) {       // Procesar calibración
                        calibrated = 1;                 // Marcar como calibrado
                        calibration_mode = 0;           // Salir de modo calibración
                        char msg_calibrated[] = "CONTROL:OK\r\n"; // Mensaje de calibración por UART
                    	UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)msg_calibrated, strlen(msg_calibrated), BLOCKING);
                        GPDMA_ChannelCmd(0, ENABLE);    // Habilitar canal DMA para muestreo continuo
                    } else {                            // Si la calibración falla, permanecer en modo calibración
                        char msg_calibration_failed[] = "CONTROL:FAIL\r\n"; // Mensaje de fallo de calibración por UART
                    	UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)msg_calibration_failed, strlen(msg_calibration_failed), BLOCKING);
                        calibration_mode = 1;
                    }
                }
            }
            // Una vez calibrado el sistema, comienza a llenarse el buffer A para su posterior proceso
    		if (calibrated){
    			volatile uint32_t *buffer = NULL;      // Puntero al buffer listo
    			if (buffer_ready_dma == 1){            // Buffer A listo
    			        buffer = bufferADC_A;
    			        buffer_ready_dma = 0;          // Resetear bandera
    			    } else if (buffer_ready_dma == 2){ // Buffer B listo
    			        buffer = bufferADC_B;
    			        buffer_ready_dma = 0;
    			    }
    			if(buffer != NULL){
    				frequency = estimateFrequency((uint32_t *)buffer); // Estima frecuencia con el buffer listo
    			}
    			// Filtrar frecuencias plausibles antes de almacenar
    			if(frequency > 80 && frequency < 400){
    				// Almacenar en buffer circular para posterior promediado
    				freq_buffer[freq_idx] = frequency;
    				freq_idx = (freq_idx + 1) % FREQUENCY_BUFFER_SIZE;
    				if (freq_count < FREQUENCY_BUFFER_SIZE) {
    					freq_count++;
    				}
    			}
    			if (freq_count > 0) {
    			// Calcular frecuencia promedio (reduce jitter)
    			uint64_t sum_freq = 0;
    			for (int i = 0; i < freq_count; i++) {
    				sum_freq += freq_buffer[i];
    			}
    			uint32_t avg_frequency = (uint32_t)(sum_freq / freq_count); // Frecuencia promedio
    			compareFrequency(avg_frequency); // Compara con frecuencia objetivo
    			}
            }
    	}
    }
}
/** ----------------- CONFIGURACIONES ------------------- */
/**
 * @brief Configuracion del modulo GPIO para los LEDs
 * - Led Rojo: P0.27
 * - Led Verde: P0.28
 * - Led Amarillo: P2.13
 */
void cfgGPIO(void){
	PINSEL_CFG_Type cfgLed = {0};
	cfgLed.Portnum = 0;
	cfgLed.Pinnum = 27;
	cfgLed.Funcnum = 0;
	cfgLed.Pinmode = PINSEL_PINMODE_TRISTATE;
	cfgLed.OpenDrain = PINSEL_PINMODE_NORMAL;

	// Configuracion del Led Rojo
	PINSEL_ConfigPin(&cfgLed);
	GPIO_SetDir(cfgLed.Portnum, (1<<27), 1);

	// Configuracion del Led Verde
	cfgLed.Pinnum = 28;
	PINSEL_ConfigPin(&cfgLed);
	GPIO_SetDir(cfgLed.Portnum, (1<<28), 1);

	// Configuracion del Led Amarillo
	cfgLed.Portnum = 2;
	cfgLed.Pinnum = 13;
	PINSEL_ConfigPin(&cfgLed);
	GPIO_SetDir(cfgLed.Portnum, (1<<13), 1);

    // Apagar todos los LEDs al inicio
    GPIO_ClearValue(0, LED_RED);
    GPIO_ClearValue(2, LED_YELLOW);
    GPIO_ClearValue(0, LED_GREEN);
}

/**
 * @brief Configuracion del ADC canal 0 disparado por Match1 del Timer0
 * El ADC se configura para muestrear el canal 0 (P0.23) cada vez que
 * el Timer0 genera un evento de Match1 (cada 50 us). Los datos se transfieren
 * automáticamente a través del módulo GPDMA a buffers en SRAM. Muestreo a 20 kHz.
 */
void cfgADC(void){
	PINSEL_CFG_Type cfgCh0 = {0};
	cfgCh0.Portnum = 0;
	cfgCh0.Pinnum = 23;
	cfgCh0.Funcnum = 1;
	cfgCh0.Pinmode = PINSEL_PINMODE_TRISTATE;
	cfgCh0.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&cfgCh0);

	ADC_Init(LPC_ADC, ADC_RATE);
	ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
	ADC_BurstCmd(LPC_ADC, DISABLE);

	// Iniciar conversion ADC cuando ocurra Match1 en Timer0
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
    ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_RISING);

	// Habilitar interrupción para usar DMA
	LPC_ADC->ADINTEN = (1 << 8);
    // Prioridad 2: ADC (alta frecuencia durante calibración)
    NVIC_SetPriority(ADC_IRQn, 2);
	NVIC_DisableIRQ(ADC_IRQn);
}

/**
 * @brief Configuracion del Timer0 para generar Match1 periódico
 * El Timer0 se configura para generar un evento de Match1 cada 50 us,
 * lo que dispara una conversión ADC a 20 kHz.
 */
void cfgTimer(void) {
    TIM_DeInit(LPC_TIM0);  // Asegura que se limpia todo antes de reconfigurar

    TIM_TIMERCFG_Type cfgTimer;
    cfgTimer.PrescaleOption = TIM_PRESCALE_USVAL;
    cfgTimer.PrescaleValue = 1; //ponerlo en 1 para que cuente en us

    TIM_MATCHCFG_Type cfgMatcher;
    cfgMatcher.MatchChannel = 1;  // usar MAT1
    cfgMatcher.IntOnMatch = DISABLE;
    cfgMatcher.ResetOnMatch = ENABLE;
    cfgMatcher.StopOnMatch = DISABLE;
    cfgMatcher.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
    cfgMatcher.MatchValue = 25;  // 100us --->ponerlo en 50 para que cuente 100us y tenga una frecuencia de 10kHz

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &cfgTimer);
    TIM_ConfigMatch(LPC_TIM0, &cfgMatcher);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

/**
 * @brief Configuracion de UART0 (TX P0.2, RX P0.3)
 * Configura UART0 para comunicación serial a 9600 baudios.
 */
void cfgUART() {
    PINSEL_CFG_Type cfgPinTXD0;
    PINSEL_CFG_Type cfgPinRXD0;

    cfgPinTXD0.Portnum = 0;
    cfgPinTXD0.Pinnum = 2;
    cfgPinTXD0.Funcnum = 1;
    cfgPinTXD0.Pinmode = PINSEL_PINMODE_NORMAL;
    cfgPinTXD0.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&cfgPinTXD0);

    cfgPinRXD0.Portnum = 0;
    cfgPinRXD0.Pinnum = 3;
    cfgPinRXD0.Funcnum = 1;
    cfgPinRXD0.Pinmode = PINSEL_PINMODE_NORMAL;
    cfgPinRXD0.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&cfgPinRXD0);

    UART_CFG_Type UARTConfig;
    UART_FIFO_CFG_Type FIFOConfig;

    UART_ConfigStructInit(&UARTConfig);
    UART_FIFOConfigStructInit(&FIFOConfig);

    UART_Init((LPC_UART_TypeDef *)LPC_UART0, &UARTConfig);
    UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART0, &FIFOConfig);
    UART_TxCmd((LPC_UART_TypeDef *)LPC_UART0, ENABLE);
}

/**
 * @brief Configuracion del modulo GPDMA
 * Configura el canal 0 del GPDMA para transferir datos del ADC
 * a buffers en SRAM usando descriptores LLI para muestreo continuo.
 */
void cfgDMA(void){
    GPDMA_Channel_CFG_Type cfgDMA;

    // Inicializa DMA (resetea lo necesario)
    GPDMA_Init();

    // Limpia flags globales antes de configurar
    LPC_GPDMA->DMACIntTCClear = 0xFF;
    LPC_GPDMA->DMACIntErrClr = 0xFF;

    cfgLLI_A->SrcAddr = (uint32_t)&LPC_ADC->ADGDR;
    cfgLLI_A->DstAddr = (uint32_t)bufferADC_A;
    cfgLLI_A->NextLLI = (uint32_t)cfgLLI_B; // apunta al propio descriptor
    cfgLLI_A->Control = (NUM_SAMPLES & 0xFFF)    // transfer size
                    | (2 << 18)                // src width = word (32 bits)
                    | (2 << 21)                // dst width = word
                    | (1 << 27)                // dst increment
                    | (1UL << 31);             // enable terminal-count interrupt

    cfgLLI_B->SrcAddr = (uint32_t)&LPC_ADC->ADGDR;
    cfgLLI_B->DstAddr = (uint32_t)bufferADC_B;
    cfgLLI_B->NextLLI = (uint32_t)cfgLLI_A; // apunta al propio descriptor
    cfgLLI_B->Control = (NUM_SAMPLES & 0xFFF)    // transfer size
                    | (2 << 18)                // src width = word (32 bits)
                    | (2 << 21)                // dst width = word
                    | (1 << 27)                // dst increment
                    | (1UL << 31);             // enable terminal-count interrupt

    cfgDMA.ChannelNum = 0;
    cfgDMA.TransferSize = NUM_SAMPLES;
    cfgDMA.TransferType = GPDMA_TRANSFERTYPE_P2M;
    cfgDMA.SrcMemAddr = 0;
    cfgDMA.DstMemAddr = (uint32_t)bufferADC_A;
    cfgDMA.SrcConn = GPDMA_CONN_ADC;
    cfgDMA.DstConn = 0;
    cfgDMA.DMALLI = (uint32_t)cfgLLI_A; // apunta al descriptor en AHB

    GPDMA_Setup(&cfgDMA);
    LPC_GPDMA->DMACIntTCClear = (1 << cfgDMA.ChannelNum);
    LPC_GPDMA->DMACIntErrClr = (1 << cfgDMA.ChannelNum);
    // Prioridad 1: DMA (la más alta para el flujo de datos)
    NVIC_SetPriority(DMA_IRQn, 1);
    NVIC_EnableIRQ(DMA_IRQn);
}

/**
 * @brief Configuracion de EINT0 para iniciar calibracion
 * Configura los pines y las interrupciones externas EINT0 y EINT1.
 * - EINT0 (P2.10) inicia eL sistema, desde el modo de calibracion.
 * - EINT1 (P2.11) cambia la cuerda actual a afinar.
 * Ambas interrupciones se disparan con flanco de bajada.
 */
void cfgEINT(void){
    PINSEL_CFG_Type pinEINT = {0};
    // Configura P2.10 como EINT0
    pinEINT.Portnum = 2;
    pinEINT.Pinnum = 10;
    pinEINT.Funcnum = 1;
    pinEINT.Pinmode = PINSEL_PINMODE_PULLUP;
    pinEINT.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&pinEINT);

    EXTI_SetMode(EXTI_EINT0, EXTI_MODE_EDGE_SENSITIVE);
    EXTI_SetPolarity(EXTI_EINT0, EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE); // Flanco de bajada
    EXTI_ClearEXTIFlag(EXTI_EINT0);

    // Configura P2.11 como EINT1
    pinEINT.Pinnum = 11;
    PINSEL_ConfigPin(&pinEINT);

    EXTI_SetMode(EXTI_EINT1, EXTI_MODE_EDGE_SENSITIVE);
    EXTI_SetPolarity(EXTI_EINT1, EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);
    EXTI_ClearEXTIFlag(EXTI_EINT1);

    // Prioridad 3: EINT0 y EINT1 (eventos de usuario)
    NVIC_SetPriority(EINT0_IRQn, 3);
    NVIC_SetPriority(EINT1_IRQn, 4);
    NVIC_EnableIRQ(EINT0_IRQn);
    NVIC_EnableIRQ(EINT1_IRQn);
}

/** -----------------  HANDLERS ------------------- */

/**
 * @brief Handler de DMA
 * Maneja las interrupciones del GPDMA para el canal 0.
 * Actualiza las banderas de buffer listo según el descriptor LLI usado.
 */
void DMA_IRQHandler(void){

    // Maneja errores de DMA
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, 0)) {
        char str[50];
        strcpy(str, "GPDMA ERROR (IRQ): err_stat=");
        uint32_t err_stat = LPC_GPDMA->DMACIntErrStat;
        char tmp[12];
        itoaSimple(err_stat, tmp);
        strcat(str, tmp);
        UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)str, strlen(str), BLOCKING);

        GPDMA_ClearIntPending(GPDMA_STATCLR_INTERR, 0);
        NVIC_ClearPendingIRQ(DMA_IRQn);
        return;
    }
    // Maneja finalización de transferencia
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)) {
        if(LPC_GPDMACH0->DMACCLLI == (uint32_t)cfgLLI_B){ // Buffer A listo
        	buffer_ready_dma = 1;
        }else if(LPC_GPDMACH0->DMACCLLI == (uint32_t)cfgLLI_A){ // Buffer B listo
        	buffer_ready_dma = 2;
        }

        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);
    }

    NVIC_ClearPendingIRQ(DMA_IRQn);
}

/**
 * @brief Handler de EINT0 para iniciar calibracion
 * Configura el sistema para iniciar la calibración del micrófono.
 */
void EINT0_IRQHandler(void){
	start ^= 1;             // Toggle start flag
    calibration_mode = 1;   // Entra en modo calibración
    calibrated = 0;        // Marca como descalibrado

    if(start){
        char msg_start[] = "CONTROL:INIT\r\n"; // Mensaje de inicio por UART
    	UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)msg_start, strlen(msg_start), BLOCKING);
    }else{
        char msg_stop[] = "CONTROL:STOP\r\n"; // Mensaje de parada por UART
    	UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)msg_stop, strlen(msg_stop), BLOCKING);
    }
    EXTI_ClearEXTIFlag(EXTI_EINT0);
    NVIC_ClearPendingIRQ(EINT0_IRQn);
}

/**
 * @brief Handler de EINT0 para iniciar calibracion
 * Cambia la cuerda actual a afinar.
 */
void EINT1_IRQHandler(void){
	curr_string = (curr_string + 1)%STRINGS;  // Cambia a la siguiente cuerda

    EXTI_ClearEXTIFlag(EXTI_EINT1);
    NVIC_ClearPendingIRQ(EINT1_IRQn);
}

/**
 * @brief Handler de interrupcion ADC
 * Maneja las interrupciones del ADC.
 * Durante la calibración, almacena muestras en el buffer de calibración.
 */
void ADC_IRQHandler(){
    if(ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)){
        uint32_t adc_value = ADC_ChannelGetData(LPC_ADC, 0);
        if(calibration_count < NUM_SAMPLES_CALIBRATION){
            bufferCalibration[calibration_count++] = adc_value;
            if(calibration_count >= NUM_SAMPLES_CALIBRATION){
                calibration_count = 0;
                NVIC_DisableIRQ(ADC_IRQn);
                buffer_ready_calibrate = 1; // Indica que la calibración está lista
            }
        }
    }
}

/* ------------------  CALIBRATION ------------------ */
/**
 * @brief Calibrar el offset del micrófono
 * Procesa las muestras almacenadas en bufferCalibration para calcular
 * el offset y el umbral de ruido (sigma). Descarta picos bruscos.
 */
int calibrateMicrophone(void) {
    uint32_t sum = 0;               // Suma de las muestras
    uint64_t sumsq = 0;             // Suma de los cuadrados de las muestras
    uint16_t sample;                // Muestra actual
    uint16_t prev = 0;              // Muestra previa para detección de outliers

    // Descartar primeras muestras
    for (int i = 0; i < DISCARD_SAMPLES; i++) {
        (void)bufferCalibration[i];
    }

    // Procesar muestras restantes
    for(int i = DISCARD_SAMPLES; i < NUM_SAMPLES_CALIBRATION; i++) {
        sample = bufferCalibration[i];
        if (i > DISCARD_SAMPLES && (sample > prev + OUTLIER_THRESHOLD || sample + OUTLIER_THRESHOLD < prev)) {
            // ignora picos bruscos
            continue;
        }
        sum += sample; // Suma de las muestras
        prev = sample; // Actualiza la muestra previa
    }

    // Calculo de media y sigma
    int valid_samples = NUM_SAMPLES_CALIBRATION - DISCARD_SAMPLES; // Muestras válidas consideradas
    uint64_t mean = (uint64_t)sum / valid_samples;                  // Media de las muestras
    uint64_t variance = 0;                                          // Varianza de las muestras

    // Reiniciar para calcular varianza
    for(int i = DISCARD_SAMPLES; i < NUM_SAMPLES_CALIBRATION; i++) {
        sample = bufferCalibration[i];
        if (i > DISCARD_SAMPLES && (sample > prev + OUTLIER_THRESHOLD || sample + OUTLIER_THRESHOLD < prev)) {
            // ignora picos bruscos
            continue;
        }
        int32_t diff = (int32_t)sample - (int32_t)mean; // Diferencia respecto a la media
        sumsq += ((uint64_t)diff * (uint64_t)diff);     // Suma de los cuadrados de las diferencias
        prev = sample;                                  // Actualiza la muestra previa
    }
    // Cálculo de varianza
    variance = sumsq / (valid_samples - 1);

    // Calculo de desviación estándar (sigma)
    uint64_t sigma = 0;
    sigma = sqrtf((uint64_t)variance);

    calibration_offset = (uint16_t)mean; // Guarda el offset calculado
    noise_threshold = (uint16_t)sigma;  // Guarda el umbral de ruido

    char out[50];                        // Mensaje de salida por UART
    if(sigma > SIGMA_THRESHOLD) {
        strcpy(out, "Calibracion fallida: señal inestable\r\n");
        UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)out, strlen(out), BLOCKING);
    } else {
        strcpy(out, "Calibracion exitosa\r\n");
        UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)out, strlen(out), BLOCKING);
    }
    return sigma <= SIGMA_THRESHOLD; // Retorna éxito o fallo de calibración
}

/* ------------------  ESTIMATE FREQUENCY  ------------------ */
/**
 * @brief Estima la frecuencia de la señal muestreada
 * @param samples Puntero al buffer de muestras ADC
 * @return Frecuencia estimada en Hz
 *
 * Procesa las muestras del buffer ADC para estimar la frecuencia
 * mediante la detección de cruces por cero en la señal filtrada.
 */
uint32_t estimateFrequency(uint32_t *samples){
	static int32_t prev_input = 0;			// Estado previo de entrada del HPF
	static int32_t prev_output = 0; 		// Estado previo de salida del HPF

	static uint32_t frequency = 0;          // Frecuencia estimada

	static int current = 0;					// Estado actual: 0 desconocido, 1 arriba, -1 abajo
	static int prev = 0;					// Estado previo (para detectar transiciones)

	uint32_t first_cross = 0;               // Almacena el índice del primer cruce
	uint32_t last_cross = 0;                // Almacena el índice del último cruce
	uint32_t total_crosses = 0;             // Contador de cruces detectados
	uint32_t N = 0;                         // Periodo estimado en muestras

	static int32_t prev_lpf_output = 0; 	// Estado previo del LPF (suavizado)

	for(int i=0; i<NUM_SAMPLES; i++){

		uint32_t adc_value = (samples[i]>>4)&0xFFF; // Extrae valor ADC de 12 bits

		// Centra la muestra respecto del offset calculado en la calibracion
		int32_t raw_sample_centered = (int32_t)adc_value - (int32_t)calibration_offset;

		// High-pass filter (HPF) - formato discreto
		// hpf_output = alpha * (prev_output + x[n] - x[n-1])
		int32_t hpf_output = (ALPHA_COEFF * (prev_output + raw_sample_centered - prev_input)) / ALPHA_SCALER;
		prev_input = raw_sample_centered; // actualiza x[n-1]
		prev_output = hpf_output;         // actualiza el estado del HPF

		// Low-pass filter (LPF) - exponencial aproximado con coeficientes enteros
		// current_output = (1-beta)*hpf_output + beta*prev_lpf_output
		int32_t current_output = (ONE_MINUS_BETA_COEFF * hpf_output + BETA_COEFF * prev_lpf_output) / BETA_SCALER;
		prev_lpf_output = current_output; // actualiza el estado del LPF

		// Decide si la señal esta por encima o por debajo del ruido
		if(current_output > noise_threshold){
			current = 1; // señal arriba
		}else if(current_output < -noise_threshold){
			current = -1; // señal abajo
		}else{
			// Dentro del rango de ruido -> mantener el estado previo para evitar rebotes
			current = prev;
		}

		// Detectar transicion de -1 a +1 (crece positivo) -> indica el periodo completo
		if(current == 1 && prev == -1){
			if(first_cross == 0){ // Guarda el índice del primer cruce
				first_cross = i; // primer cruce detectado
			}
			last_cross = i; // Actualiza el índice del último cruce
			total_crosses++; // Incrementa el contador de cruces
		}
		// Actualizar estado previo si hay un estado valido
		if(current != 0){
			prev = current; // Actualiza el estado previo
		}
	}

	if (total_crosses > 1) {
	    // Distancia total en muestras / Número de periodos completos
	    uint32_t total_samples = last_cross - first_cross;
	    // N (periodo promedio) = (Total de muestras / Total de periodos completos)
	    N = total_samples / (total_crosses - 1);
	} else {
	    N = 0; // Menos de un ciclo completo detectado
	}

	if(N > 0){
	    // Usar la tasa de muestreo adecuada (20000 o 40000)
	    frequency = ADC_RATE / N; // Calcula la frecuencia estimada
	} else {
	    frequency = 0; // Frecuencia no válida
	}

	return frequency; // Retorna la frecuencia estimada
}

/* ------------------  COMPARE FREQUENCY  ------------------ */
/**
 * @brief Compara la frecuencia estimada con la frecuencia objetivo
 * y enciende los LEDs correspondientes según el estado.
  Además, envía el estado por UART.
  - LED Verde: Afinado (OK)
  - LED Amarillo: Tensar
  - LED Rojo: Destensar
 * @param f Frecuencia estimada en Hz
 */
void compareFrequency(uint32_t f){
	uint16_t frequency = (uint16_t)strings[curr_string];

    // Compara con frecuencia objetivo
    if ((f > frequency - FREQUENCY_THRESHOLD) && (f < frequency + FREQUENCY_THRESHOLD)){
        sendState(f, 0, curr_string); // Enviar estado OK

        // Encender LED verde, apagar los demás
        GPIO_SetValue(0, LED_GREEN);
        GPIO_ClearValue(0, LED_RED);
        GPIO_ClearValue(2, LED_YELLOW);
    }
    else if (f < frequency + FREQUENCY_THRESHOLD){
        sendState(f, 1, curr_string); // Enviar estado TENSAR

        // Encender LED amarillo, apagar los demás
        GPIO_SetValue(2, LED_YELLOW);
        GPIO_ClearValue(0, LED_RED);
        GPIO_ClearValue(0, LED_GREEN);
    }
    else {
        sendState(f, 2, curr_string); // Enviar estado DESTENSAR

        // Encender LED rojo, apagar los demás
        GPIO_SetValue(0, LED_RED);
        GPIO_ClearValue(0, LED_GREEN);
        GPIO_ClearValue(2, LED_YELLOW);
    }
}

/** -----------------  UTILITIES ------------------- */
/**
 * @brief Convierte un entero a string (itoa simple)
 * @param n Entero a convertir
 * @param s Cadena donde se almacena el resultado
 */
void itoaSimple(int n, char s[]) {
    int i, sign;
    if ((sign = n) < 0)
        n = -n;
    i = 0;
    do {
        s[i++] = n % 10 + '0';
    } while ((n /= 10) > 0);
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';

    int j = 0;
    char temp;
    i--;
    while (j < i) {
        temp = s[j];
        s[j] = s[i];
        s[i] = temp;
        j++;
        i--;
    }
}

/**
 * @brief Enviar string por UART0
 * @param frequency Frecuencia estimada
 * @param state Estado de la cuerda (0: OK, 1: TENSAR, 2: DESTENSAR)
 * @param string Índice de la cuerda actual
 * Envía un mensaje formateado para la GUI por UART0 con la frecuencia, estado y cuerda.
 * Ejemplo: "freq=220;state=OK;string=2\r\n"
 */
void sendState(uint32_t frequency, uint8_t state, uint8_t string){
    char str[100];
    char freq_str[10];
    char string_str[10];
    char state_str[10];

    itoaSimple(frequency, freq_str);
    itoaSimple(string, string_str);

    switch(state){
        case 0:
            strcpy(state_str, "OK");
            break;
        case 1:
            strcpy(state_str, "TENSAR");
            break;
        case 2:
            strcpy(state_str, "DESTENSAR");
            break;
        default:
            break;
    }
    strcpy(str, "freq=");
    strcat(str, freq_str);
    strcat(str, ";state=");
    strcat(str, state_str);
    strcat(str, ";string=");
    strcat(str, string_str);
    strcat(str, "\r\n");

    UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t*)str, strlen(str), BLOCKING);
}
