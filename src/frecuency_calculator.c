/**
* @file main.c
* @brief Ejemplo completo: Muestreo de micrófono con ADC, DMA y Timer
* Configura el ADC para muestrear el canal 0, disparado por un timer que genera
* un evento cada 10 ms. Los datos del ADC se transfieren automáticamente a un buffer
* en RAM usando el módulo GPDMA. Se implementa una rutina de calibración para
* determinar el offset del micrófono. Una vez calibrado, se detectan los cruces
* por cero en la señal muestreada para calcular la frecuencia de la cuerda.
 */

#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_systick.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>


/* MACRO PARA EL MUESTREO CONTINUO DEL MICROFONO*/
#define ADC_RATE 10000
#define NUM_SAMPLES 1024 //tiene que er potencia de 2
#define AHB_BASE_ADDR 0x2007C000UL   // Base real de la RAM AHB (32 KB)

/* MACROS PARA LA CALIBRACION DEL MICROFONO */
#define NUM_SAMPLES_CALIBRATION 256
#define SIGMA_THRESHOLD 15 // en LSB (~3 mV para ADC de 12 bits y Vref=3.3V)
#define OUTLIER_THRESHOLD 50   // LSB - ignora saltos grandes en promedio
#define DISCARD_SAMPLES 16

// Estructura para la configuración de la LLI del DMA
typedef struct {
    uint32_t SrcAddr;
    uint32_t DstAddr;
    uint32_t NextLLI;
    uint32_t Control;
} myLLI_t;

/* Prototipos de funciones */
void cfgADC(void);
void cfgUART(void);
void cfgDMA(void);
void cfgTimer(void);
void send_string(char* str);
void itoa_simple(int, char*);
int calibrate_microphone(void);
void init_fft(void); // <<<--- NUEVO PROTOTIPO


myLLI_t *cfgLLI  = (myLLI_t *)AHB_BASE_ADDR;

volatile uint32_t bufferCalibration[NUM_SAMPLES_CALIBRATION]; // Buffer para calibración
volatile uint32_t *bufferADC  = (volatile uint32_t *)(AHB_BASE_ADDR + sizeof(myLLI_t)); // Buffer para datos ADC

volatile uint16_t calibration_offset = 0; // Offset calculado en la calibración
volatile uint16_t noise_threshold = SIGMA_THRESHOLD; // Umbral de ruido para detección

int calibration_count = 0; // Contador de muestras para calibración

volatile int buffer_ready = 0; // Bandera para indicar que el buffer DMA está listo
volatile int buffer_ready_calibrated = 0; // Bandera para indicar que el buffer DMA de calibración está listo
volatile int calibration_mode = 1; // Bandera para indicar modo calibración
volatile int calibrated = 0; // Bandera para indicar si ya se calibró

/** -----------------  MAIN ------------------- */
/**
* @brief Función principal
*/
int main(void) {
    SystemInit();
    cfgUART();
    cfgTimer();
    cfgADC();
    SYSTICK_InternalInit(100); // Inicializa SysTick para medir tiempos
    SYSTICK_Cmd(ENABLE);

    while (1){

        if(calibration_mode){
            NVIC_EnableIRQ(ADC_IRQn);

            if (buffer_ready_calibrated) {
                buffer_ready_calibrated = 0;
                if (calibrate_microphone()) {
                    calibrated = 1;
                    calibration_mode = 0;
                    cfgDMA();
                    GPDMA_ChannelCmd(0, ENABLE);
                } else {
                    calibration_mode = 1; // Mantener en modo calibración
                }
            }
        }
        if (calibrated && buffer_ready){
            buffer_ready = 0;
            GPDMA_ChannelCmd(0, DISABLE);
            static uint8_t count_crosses = 0;
            const uint16_t threshold_high = calibration_offset + noise_threshold;
            const uint16_t threshold_low = calibration_offset - noise_threshold;
            static uint32_t prev_timestamp = 0;
            static uint32_t curr_timestamp = 0;
            uint32_t frequency = 0;
            int current = 0; // 0 = desconocido, 1 = arriba, -1 = abajo
            int prev = 0;

            for (int i = 0; i < NUM_SAMPLES; i++) {
                uint16_t sample = (uint16_t)((bufferADC[i] >> 4) & 0xFFF);
                char out[40];
                if (sample > threshold_high) {
                    current = 1; // arriba
                } else if (sample < threshold_low) {
                    current = -1; // abajo
                } else {
                    current = prev; // dentro del ruido, mantener estado previo
                }

                if (current != 0 && prev != 0 && current != prev) {
                    count_crosses = (count_crosses + 1) % 2;
                }

                if (current != 0) {
                    prev = current;
                }

                if(count_crosses){
                    prev_timestamp = curr_timestamp;
                    curr_timestamp = SYSTICK_GetCurrentValue();
                    if(prev_timestamp>curr_timestamp){
                    	frequency = SystemCoreClock / (prev_timestamp-curr_timestamp);
                    }else{
                    	frequency = SystemCoreClock / (curr_timestamp-prev_timestamp); //A CHEQUEAR -> llevar a frecuencia en Hz
                }
                    char out[40];
                    itoa_simple((int)frequency, out);
                    send_string("FRECUENCIA: ");
                    send_string(out);
                    send_string(" Hz\r\n");
            }
            GPDMA_ChannelCmd(0, ENABLE);
        }
        }
}
}

/** ----------------- CONFIGURACIONES ------------------- */

/**
 * @brief Configuracion del ADC canal 0 disparado por Match1 del Timer0
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
	ADC_IntConfig(LPC_ADC,ADC_ADINTEN0, ENABLE);
	ADC_BurstCmd(LPC_ADC, DISABLE);

	// Iniciar conversion ADC cuando ocurra Match1 en Timer0
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);

	NVIC_DisableIRQ(ADC_IRQn);
}

/**
 * @brief Configuracion de UART0 (TX P0.2, RX P0.3)
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
 */
void cfgDMA(void){
    GPDMA_Channel_CFG_Type cfgDMA;

    // Inicializa DMA (resetea lo necesario)
    GPDMA_Init();

    // Limpia flags globales antes de configurar
    LPC_GPDMA->DMACIntTCClear = 0xFF;
    LPC_GPDMA->DMACIntErrClr = 0xFF;

    cfgLLI->SrcAddr = (uint32_t)&LPC_ADC->ADGDR;
    cfgLLI->DstAddr = (uint32_t)bufferADC;
    cfgLLI->NextLLI = (uint32_t)cfgLLI; // apunta al propio descriptor (no &cfgLLI)
    cfgLLI->Control = (NUM_SAMPLES & 0xFFF)    // transfer size
                    | (2 << 18)                // src width = word (32 bits)
                    | (2 << 21)                // dst width = word
                    | (1 << 27)                // dst increment
                    | (1UL << 31);             // enable terminal-count interrupt


    cfgDMA.ChannelNum = 0;
    cfgDMA.TransferSize = NUM_SAMPLES;
    cfgDMA.TransferType = GPDMA_TRANSFERTYPE_P2M;
    cfgDMA.SrcMemAddr = 0;
    cfgDMA.DstMemAddr = (uint32_t)bufferADC;
    cfgDMA.SrcConn = GPDMA_CONN_ADC;
    cfgDMA.DstConn = 0;
    cfgDMA.DMALLI = (uint32_t)cfgLLI; // apunta al descriptor en AHB

    // Setup y limpiar flags del canal
    GPDMA_Setup(&cfgDMA);
    LPC_GPDMA->DMACIntTCClear = (1 << cfgDMA.ChannelNum);
    LPC_GPDMA->DMACIntErrClr = (1 << cfgDMA.ChannelNum);


    // Enable IRQ
    NVIC_EnableIRQ(DMA_IRQn);
}

/**
 * @brief Configuracion del Timer0 para generar Match1 periódico
 */
void cfgTimer(void) {
    TIM_DeInit(LPC_TIM0);  // Asegura que se limpia todo antes de reconfigurar

    TIM_TIMERCFG_Type cfgTimer;
    cfgTimer.PrescaleOption = TIM_PRESCALE_USVAL;
    cfgTimer.PrescaleValue = 1;

    TIM_MATCHCFG_Type cfgMatcher;
    cfgMatcher.MatchChannel = 1;  // usar MAT1
    cfgMatcher.IntOnMatch = DISABLE;
    cfgMatcher.ResetOnMatch = ENABLE;
    cfgMatcher.StopOnMatch = DISABLE;
    cfgMatcher.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
    cfgMatcher.MatchValue = 10;  // 10ms

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &cfgTimer);
    TIM_ConfigMatch(LPC_TIM0, &cfgMatcher);
    TIM_Cmd(LPC_TIM0, ENABLE);
}


/** -----------------  HANDLERS ------------------- */

/**
 * @brief Handler de DMA
 */
void DMA_IRQHandler(void){

    if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, 0)) {
        send_string("GPDMA ERROR (IRQ): err_stat=");
        uint32_t err_stat = LPC_GPDMA->DMACIntErrStat;
        char tmp[12];
        itoa_simple(err_stat, tmp);
        send_string(tmp);
        send_string("\r\n");

        GPDMA_ClearIntPending(GPDMA_STATCLR_INTERR, 0);
        NVIC_ClearPendingIRQ(DMA_IRQn);
        return;
    }

    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)) {
        buffer_ready = 1;

        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);
    }
    NVIC_ClearPendingIRQ(DMA_IRQn);
}



void ADC_IRQHandler(){
    if(ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)){
        uint32_t adc_value = ADC_ChannelGetData(LPC_ADC, 0);
        if(calibration_count < NUM_SAMPLES_CALIBRATION){
            bufferCalibration[calibration_count++] = adc_value;
            if(calibration_count >= NUM_SAMPLES_CALIBRATION){
                calibration_count = 0;
                NVIC_DisableIRQ(ADC_IRQn);
                buffer_ready_calibrated = 1; // Indica que la calibración está lista
                //aca almaceno las muestras y levanto la bandera cuando este listo
            }
        }
    }
}

/* ------------------  CALIBRATION ------------------ */
/**
 * @brief Calibrar el offset del micrófono
 */
int calibrate_microphone(void) {
    uint32_t sum = 0; // Suma de las muestras
    uint64_t sumsq = 0; // Suma de los cuadrados de las muestras
    uint16_t sample; // Muestra actual
    uint16_t prev = 0; // Muestra previa para detección de outliers

    // --- Descartar primeras muestras
    for (int i = 0; i < DISCARD_SAMPLES; i++) {
        (void)bufferCalibration[i];
    }

    // --- Procesar muestras restantes
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
    uint64_t mean = (uint64_t)sum / valid_samples; // Media de las muestras
    uint64_t variance = 0;

    for(int i = DISCARD_SAMPLES; i < NUM_SAMPLES_CALIBRATION; i++) {
        sample = bufferCalibration[i];
        if (i > DISCARD_SAMPLES && (sample > prev + OUTLIER_THRESHOLD || sample + OUTLIER_THRESHOLD < prev)) {
            // ignora picos bruscos
            continue;
        }
        int32_t diff = (int32_t)sample - (int32_t)mean;
        sumsq += ((uint64_t)diff * (uint64_t)diff); // Suma de los cuadrados de las diferencias
        prev = sample; // Actualiza la muestra previa
    }
    variance = sumsq / (valid_samples - 1);
    // Calculo de desviación estándar (sigma)
    uint64_t sigma = 0;

    sigma = sqrtf((uint64_t)variance);

    calibration_offset = (uint16_t)mean; // Guarda el offset calculado
    noise_threshold = (uint16_t)sigma;

    if(sigma > SIGMA_THRESHOLD) {
        send_string("Calibracion fallida: señal inestable\r\n");
    } else {
        send_string("Calibracion exitosa\r\n");
    }
    return sigma <= SIGMA_THRESHOLD;
}

/** -----------------  UTILITIES ------------------- */
/**
 * @brief Convierte un entero a string (itoa simple)
 * @param n Entero a convertir
 * @param s Cadena donde se almacena el resultado
 */
void itoa_simple(int n, char s[]) {
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
 */
void send_string(char* str) {
    UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t*)str, strlen(str), BLOCKING);
}
