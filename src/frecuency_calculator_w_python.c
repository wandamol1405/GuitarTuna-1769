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
#include <string.h>
#include <stdlib.h>
#include <math.h>


/* MACRO PARA EL MUESTREO CONTINUO DEL MICROFONO*/
#define ADC_RATE 10000
#define NUM_SAMPLES 2048 // TODO: a chequear
#define AHB_BASE_ADDR 0x2007C000UL   // Base real de la RAM AHB (32 KB)

#define fmaxf(a,b) ((a) > (b) ? (a) : (b))

/* MACROS PARA LA CALIBRACION DEL MICROFONO */
#define NUM_SAMPLES_CALIBRATION 256
#define SIGMA_THRESHOLD 8 // en LSB (~3 mV para ADC de 12 bits y Vref=3.3V)
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
void cfgTimer(void);
void cfgDMA(void);
void send_string(char* str);
void itoa_simple(int, char*);
int calibrate_microphone(void);



myLLI_t *cfgLLI  = (myLLI_t *)AHB_BASE_ADDR;

volatile uint32_t bufferCalibration[NUM_SAMPLES_CALIBRATION]; // Buffer para calibración
volatile uint32_t *bufferADC  = (volatile uint32_t *)(AHB_BASE_ADDR + sizeof(myLLI_t)); // Buffer para datos ADC

volatile uint16_t calibration_offset = 0; // Offset calculado en la calibración
volatile uint16_t noise_threshold = SIGMA_THRESHOLD; // Umbral de ruido para detección

int calibration_count = 0; // Contador de muestras para calibración

volatile int buffer_ready = 0; // Bandera para indicar que el buffer DMA está listo
volatile int calibration_mode = 1; // Bandera para indicar modo calibración
volatile int calibrated = 0; // Bandera para indicar si ya se calibró
volatile int buffer_ready_calibrate = 0;

/** -----------------  MAIN ------------------- */
/**
* @brief Función principal
*/
int main(void) {
    SystemInit();
    cfgUART();
    cfgADC();
    cfgTimer();
    cfgDMA();
    while (1){
        if(calibration_mode){
            NVIC_EnableIRQ(ADC_IRQn);

            if (buffer_ready_calibrate) {
            	buffer_ready_calibrate = 0;
                if (calibrate_microphone()) {
                    calibrated = 1;
                    calibration_mode = 0;
                    GPDMA_ChannelCmd(0, ENABLE);
                } else {
                    calibration_mode = 1; // Mantener en modo calibración
                }
            }
        }
		if (calibrated && buffer_ready){
        buffer_ready = 0;

        // 1. Deshabilitamos la IRQ de DMA para que no nos pise el buffer
        //    mientras lo estamos enviando.
        NVIC_DisableIRQ(DMA_IRQn);
        
        for(int i = 0; i < NUM_SAMPLES; i++) {
            // Extraemos los 12 bits de datos (0-4095)
            uint16_t sample = (uint16_t)((bufferADC[i] >> 4) & 0xFFF);
            
            // Enviamos esos 2 bytes en crudo
            UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)&sample, 2, BLOCKING);
        }
        
        // 2. Reactivamos la IRQ de DMA para la próxima captura
        NVIC_EnableIRQ(DMA_IRQn);
        /*for(int i = 0; i < NUM_SAMPLES; i++) {
            char temp[12];
            itoa_simple((int)((bufferADC[i] >> 4) & 0xFFF), temp);
            send_string(temp);
            send_string("\r\n");
        }*/
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
	ADC_BurstCmd(LPC_ADC, DISABLE);

	// Iniciar conversion ADC cuando ocurra Match1 en Timer0
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
    ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_RISING);

	// Habilitar interrupción para usar DMA
	LPC_ADC->ADINTEN = (1 << 8);
	NVIC_DisableIRQ(ADC_IRQn);
}

/**
 * @brief Configuracion del Timer0 para generar Match1 periódico
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
    cfgMatcher.MatchValue = 100;  // 10ms --->ponerlo en 100 para que cuente 100us y tenga una frecuencia de 10kHz

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &cfgTimer);
    TIM_ConfigMatch(LPC_TIM0, &cfgMatcher);
    TIM_Cmd(LPC_TIM0, ENABLE);
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
    UARTConfig.Baud_rate = 921600; // Configurar baud rate a 921600 oara que tarde menos que el DMA
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
                buffer_ready_calibrate = 1; // Indica que la calibración está lista
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
    const int max_iterations = 3;
    uint32_t valid_count;
    float mean = 0, sigma = 0;
    uint16_t sample;

    // Descartar las primeras muestras (ruido inicial)
    for (int i = 0; i < DISCARD_SAMPLES; i++) {
        (void)bufferCalibration[i];
    }

    // Paso 1: cálculo inicial de media y varianza
    for (int i = DISCARD_SAMPLES; i < NUM_SAMPLES_CALIBRATION; i++) {
        mean += bufferCalibration[i];
    }
    mean /= (NUM_SAMPLES_CALIBRATION - DISCARD_SAMPLES);

    for (int i = DISCARD_SAMPLES; i < NUM_SAMPLES_CALIBRATION; i++) {
        float diff = bufferCalibration[i] - mean;
        sigma += diff * diff;
    }
    sigma = sqrtf(sigma / (NUM_SAMPLES_CALIBRATION - DISCARD_SAMPLES));

    // Paso 2: refinamiento iterativo descartando outliers
    for (int iter = 0; iter < max_iterations; iter++) {
        float new_mean = 0;
        float new_sigma = 0;
        valid_count = 0;

        for (int i = DISCARD_SAMPLES; i < NUM_SAMPLES_CALIBRATION; i++) {
            sample = bufferCalibration[i];
            if (fabsf(sample - mean) <= 2.5f * sigma) { // solo muestras dentro de ±2.5σ
                new_mean += sample;
                valid_count++;
            }
        }

        if (valid_count == 0) break;

        new_mean /= valid_count;
        for (int i = DISCARD_SAMPLES; i < NUM_SAMPLES_CALIBRATION; i++) {
            sample = bufferCalibration[i];
            if (fabsf(sample - mean) <= 2.5f * sigma)
                new_sigma += (sample - new_mean) * (sample - new_mean);
        }
        new_sigma = sqrtf(new_sigma / valid_count);

        // Si el cambio en el promedio es pequeño, terminamos
        if (fabsf(new_mean - mean) < 1.0f && fabsf(new_sigma - sigma) < 0.5f) {
            mean = new_mean;
            sigma = new_sigma;
            break;
        }

        mean = new_mean;
        sigma = new_sigma;
    }

    calibration_offset = (uint16_t)roundf(mean);
    noise_threshold = (uint16_t)roundf(sigma);

    // Evaluar resultado final
    if (sigma > SIGMA_THRESHOLD) {
        send_string("⚠️ Calibracion inestable: demasiado ruido\r\n");
        char s1[20];
        itoa_simple((int)mean, s1);
        send_string("Media: "); send_string(s1); send_string("\r\n");
        char s2[20];
        itoa_simple((int)sigma, s2);
        send_string("Sigma: "); send_string(s2); send_string("\r\n");
        return 0;
    } else {
        send_string("✅ Calibracion exitosa\r\n");
        char s1[20];
        itoa_simple((int)mean, s1);
        send_string("Offset calculado: "); send_string(s1); send_string("\r\n");
        char s2[20];
        itoa_simple((int)sigma, s2);
        send_string("Sigma final: "); send_string(s2); send_string("\r\n");
        return 1;
    }
}


/* ------------------  ESTIMATE FREQUENCY  ------------------ */
/**
 * @brief Estima la frecuencia usando autocorrelación simple
 * @param buf32 buffer de muestras ADC (32 bits con datos en bits [15:4])
 * @param N cantidad de muestras
 * @param fs frecuencia de muestreo (Hz)
 * @param offset offset DC del micrófono
 * @return frecuencia estimada (Hz)
 */
/*float estimate_frequency_autocorr(uint32_t *buf32, int N, float fs, uint16_t offset) {
    // Convertir a float centrado
    static float samples[NUM_SAMPLES];
    for (int i = 0; i < N; i++) {
        samples[i] = ((float)(((buf32[i] >> 4) & 0xFFF) - offset));
    }

    // Calcular autocorrelación parcial hasta la mitad del buffer
    int max_lag = N / 2;
    float best_corr = 0.0f;
    int best_lag = 0;

    for (int lag = 10; lag < max_lag; lag++) { // ignorar lags muy chicos (<10 muestras)
        float sum = 0.0f;
        for (int i = 0; i < N - lag; i++) {
            sum += samples[i] * samples[i + lag];
        }
        if (sum > best_corr) {
            best_corr = sum;
            best_lag = lag;
        }
    }

    if (best_lag == 0) return 0.0f;

    float period = (float)best_lag / fs;
    return 1.0f / period;
}*/



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
