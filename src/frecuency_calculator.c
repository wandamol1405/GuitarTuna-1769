/**
 * @file ky038-test-driver.c
 * @brief Test driver para el sensor KY-038 con ADC y UART
 *
 * Configura el ADC para muestrear en el canal 0, disparado por un timer que genera
 * un evento cada 100 us. Cada vez que se completa una conversión ADC, se envía
 * el valor leído por UART0.
 *
 */
#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_systick.h"
#include <string.h>
#include <math.h>

#define ADC_RATE 20000

#define ALPHA_SCALER 1000 // Escala para cálculos enteros
#define ALPHA_COEFF 990 // Coeficiente alpha = 0.99 (corte bajo ~100 Hz)

#define BETA_SCALER 1000 // Escala para cálculos enteros
#define BETA_COEFF 900   // Coeficiente Beta = 0.9 (Suavizado)
#define ONE_MINUS_BETA_COEFF 100 // Coeficiente 1 - Beta = 0.1

/* MACROS PARA LA CALIBRACION DEL MICROFONO */
#define NUM_SAMPLES_CALIBRATION 256
#define SIGMA_THRESHOLD 76 // en LSB (~3 mV para ADC de 12 bits y Vref=3.3V)
#define OUTLIER_THRESHOLD 50   // LSB - ignora saltos grandes en promedio
#define DISCARD_SAMPLES 16
#define LOAD 9999999
#define FREQUENCY_BUFFER_SIZE 20
#define UART_SEND_INTERVAL 8000

void cfgADC(void);
void cfgUART(void);
void cfgTimer(void);
void send_string(char* str);
void itoa_simple(int n, char s[]);
int calibrate_microphone(void);

volatile uint32_t bufferCalibration[NUM_SAMPLES_CALIBRATION]; // Buffer para calibración
volatile int buffer_ready_calibrated = 0; // Bandera para indicar que el buffer DMA de calibración está listo
volatile uint16_t calibration_offset = 0; // Offset calculado en la calibración
volatile uint16_t noise_threshold = SIGMA_THRESHOLD; // Umbral de ruido para detección

volatile int calibrated = 0; // Bandera para indicar si ya se calibró
int calibration_count = 0; // Contador de muestras para calibración

int main(void) {
    SystemInit();
    cfgUART();          // Primero UART
    cfgADC();
    cfgTimer();
    SYSTICK_InternalInit(100); // Inicializa SysTick para medir tiempos
    SYSTICK_Cmd(ENABLE);
    while (1);
}


/**
 * @brief Configuracion del ADC canal 0 disparado por Match1 del Timer0
 */
void cfgADC(void){
	PINSEL_CFG_Type cfgCh0;
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

	// Habilitar interrupción del canal 0
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
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
    cfgMatcher.MatchValue = 25;  // 100us

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
 * @brief Handler de interrupcion ADC
 */
void ADC_IRQHandler(void) {
	if(ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)){
	        uint32_t adc_value = ADC_ChannelGetData(LPC_ADC, 0);
	        //NO ESTA CALIBRADO
	        if(!calibrated){
				if(calibration_count < NUM_SAMPLES_CALIBRATION){
					bufferCalibration[calibration_count++] = adc_value;
				}else{
					calibration_count = 0;
					buffer_ready_calibrated = 1; // Indica que la calibración está lista
					//aca almaceno las muestras y levanto la bandera cuando este listo
				}
				if (buffer_ready_calibrated) {
					buffer_ready_calibrated = 0;
					if (calibrate_microphone()) {
						calibrated = 1;
					}
				}

	        }
	        //YA ESTA CALIBRADO
	        if(calibrated){
	        	//send_string("ENTRE");
	            static uint32_t prev_timestamp = 0;
	            static uint32_t curr_timestamp = LOAD;
	            static int32_t prev_input = 0;
	            static int32_t prev_output = 0;
	            char out[40];
	            uint32_t frequency = 0;
	            static int current = 0; // 0 = desconocido, 1 = arriba, -1 = abajo
	            static int prev = 0;
	            static uint32_t uart_counter = 0;
	            // Dentro de ADC_IRQHandler, en la sección de variables estáticas:
	            static int32_t prev_lpf_output = 0; // Estado LPF

				static uint32_t freq_buffer[FREQUENCY_BUFFER_SIZE];
				static uint8_t freq_idx = 0;
				static uint8_t freq_count = 0;

	            int32_t raw_sample_centered = (int32_t)adc_value - (int32_t)calibration_offset;
	            int32_t hpf_output = (ALPHA_COEFF * (prev_output + raw_sample_centered - prev_input)) / ALPHA_SCALER;
	            prev_input = raw_sample_centered;
	            prev_output = hpf_output;

	            int32_t current_output = (ONE_MINUS_BETA_COEFF * hpf_output + BETA_COEFF * prev_lpf_output) / BETA_SCALER;
	            prev_lpf_output = current_output;

				if (current_output > noise_threshold) {
					//send_string("arriba");
					//send_string("\r\n");
					current = 1; // arriba
				} else if (current_output < -noise_threshold) {
					current = -1; // abajo
					//send_string("abajo");
					//send_string("\r\n");
				} else {
					current = prev; // dentro del ruido, mantener estado previo
				}

				if (current==1&&prev==-1) {
					//itoa_simple(count_crosses, out);
					//send_string(out);
					//send_string("\r\n");
					prev_timestamp = curr_timestamp;
					curr_timestamp = SYSTICK_GetCurrentValue();
					if(prev_timestamp>curr_timestamp){
						frequency = SystemCoreClock / (prev_timestamp-curr_timestamp);
						if(frequency >80 && frequency < 400){
							freq_buffer[freq_idx] = frequency;
							freq_idx = (freq_idx + 1) % FREQUENCY_BUFFER_SIZE;
							if (freq_count < FREQUENCY_BUFFER_SIZE) {
								freq_count++;
							}
						}
					}

					//itoa_simple((int)frequency, out);
					//send_string("FRECUENCIA: ");
					//send_string(out);
					//send_string(" Hz\r\n");
				}

				if (current != 0) {
					prev = current;
				}

				uart_counter++;
				if (uart_counter >= UART_SEND_INTERVAL) {
					uart_counter = 0; // Reinicia el contador

					if (freq_count > 0) {
						// Calcular la frecuencia promedio SOLO cuando se va a enviar
						uint64_t sum_freq = 0;
						for (int i = 0; i < freq_count; i++) {
							sum_freq += freq_buffer[i];
						}
						uint32_t avg_frequency = (uint32_t)(sum_freq / freq_count);

						itoa_simple((int)avg_frequency, out);
						send_string("FRECUENCIA PROMEDIO: ");
						send_string(out);
						send_string(" Hz\r\n");
					} else {
						send_string("Sin señal valida...\r\n");
					}


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

