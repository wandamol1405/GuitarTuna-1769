/** 
* @file calibration.c
* @brief Calibración de micrófono usando ADC y UART
* Configura el ADC para muestrear el canal 0, disparado por un timer que genera
* un evento cada 10 ms. Se recopilan 256 muestras para calcular el offset del ADC.
*/
#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_pinsel.h"
#include <string.h>

#define ADC_RATE 200000
#define NUM_SAMPLES_CALIBRATION 256
#define SIGMA_THRESHOLD 8 // en LSB (~3 mV para ADC de 12 bits y Vref=3.3V)
#define OUTLIER_THRESHOLD 50   // LSB - ignora saltos grandes en promedio
#define DISCARD_SAMPLES 16

static uint16_t calibration_offset = 0;

void cfgADC(void);
void cfgUART(void);
void cfgTimer(void);
void send_string(char* str);
void itoa_simple(int, char*);
void calibrate_microphone(void);

volatile uint32_t bufferCalibration[NUM_SAMPLES_CALIBRATION];
int calibration_count = 0;

int main(void) {
    SystemInit();
    cfgUART();
    cfgADC();
    cfgTimer();
    while (1){
	};
}


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

	// Habilitar interrupción para usar DMA
	LPC_ADC->ADINTEN = (1 << 8);
	NVIC_EnableIRQ(ADC_IRQn);
}

/**
 * @brief Configuracion del Timer0 para generar Match1 periódico
 */
void cfgTimer(void) {
    TIM_DeInit(LPC_TIM0);  // Asegura que se limpia todo antes de reconfigurar

    TIM_TIMERCFG_Type cfgTimer;
    cfgTimer.PrescaleOption = TIM_PRESCALE_USVAL;
    cfgTimer.PrescaleValue = 1000;

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
 * @brief Handler de ADC
 */
void ADC_IRQHandler(){
    if(ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)){
        uint32_t adc_value = ADC_ChannelGetData(LPC_ADC, 0);
        if(calibration_count < NUM_SAMPLES_CALIBRATION){
            bufferCalibration[calibration_count++] = adc_value;
            if(calibration_count == NUM_SAMPLES_CALIBRATION){
                calibrate_microphone();
                calibration_count = 0;
            }
        }
    }
    NVIC_ClearPendingIRQ(ADC_IRQn);
}


/**
 * @brief Convierte un entero a string (itoa simple)
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
