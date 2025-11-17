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
#include <math.h>

// Estructura para la configuración de la LLI del DMA
typedef struct {
    uint32_t SrcAddr;
    uint32_t DstAddr;
    uint32_t NextLLI;
    uint32_t Control;
} myLLI_t;

/* MACRO PARA EL MUESTREO CONTINUO DEL MICROFONO*/
#define ADC_RATE 20000
#define NUM_SAMPLES 2048 // TODO: a chequear
#define AHB_BASE_ADDR 0x2007C000UL   // Base real de la RAM AHB (32 KB)
#define LLI_SIZE (sizeof(myLLI_t))
#define BUFFER_SIZE (NUM_SAMPLES * sizeof(uint32_t))
#define AHB_BASE_ADDR2  (AHB_BASE_ADDR + LLI_SIZE + BUFFER_SIZE + 4) // Ajustar según NUM_SAMPLES

/* MACROS PARA LA CALIBRACION DEL MICROFONO */
#define NUM_SAMPLES_CALIBRATION 256
#define SIGMA_THRESHOLD 76 // en LSB (~3 mV para ADC de 12 bits y Vref=3.3V)
#define OUTLIER_THRESHOLD 50   // LSB - ignora saltos grandes en promedio
#define DISCARD_SAMPLES 16

/* MACROS PARA LA ESTIMACION DE LA FRECUENCIA*/
#define LOAD 9999999
#define ALPHA_SCALER 1000 // Escala para cálculos enteros
#define ALPHA_COEFF 990 // Coeficiente alpha = 0.99 (corte bajo ~100 Hz)
#define BETA_SCALER 1000 // Escala para cálculos enteros
#define BETA_COEFF 700   // Coeficiente Beta = 0.9 (Suavizado)
#define ONE_MINUS_BETA_COEFF 300 // Coeficiente 1 - Beta = 0.1
#define FREQUENCY_BUFFER_SIZE 25
#define UART_SEND_INTERVAL 8000


/* Prototipos de funciones */
void cfgADC(void);
void cfgUART(void);
void cfgTimer(void);
void cfgDMA(void);
void cfgEINT(void);
void send_string(char* str);
void itoa_simple(int, char*);
int calibrate_microphone(void);
uint32_t estimate_frequency(uint32_t *samples);


myLLI_t *cfgLLI_A  = (myLLI_t *)AHB_BASE_ADDR;
myLLI_t *cfgLLI_B  = (myLLI_t *)(AHB_BASE_ADDR + sizeof(myLLI_t));

volatile uint32_t bufferCalibration[NUM_SAMPLES_CALIBRATION]; // Buffer para calibración
volatile uint32_t *bufferADC_A  = (volatile uint32_t *)(AHB_BASE_ADDR + 2 * sizeof(myLLI_t)); // Buffer para datos ADC
volatile uint32_t *bufferADC_B  = (volatile uint32_t *)(AHB_BASE_ADDR2 + 2 * sizeof(myLLI_t) + NUM_SAMPLES * sizeof(uint32_t)); // Buffer para datos ADC

volatile uint16_t calibration_offset = 0; // Offset calculado en la calibración
volatile uint16_t noise_threshold = SIGMA_THRESHOLD; // Umbral de ruido para detección

int calibration_count = 0; // Contador de muestras para calibración

volatile int buffer_ready_dma = 0; // Bandera para indicar que el buffer DMA está listo
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
    cfgEINT();
	static uint32_t freq_buffer[FREQUENCY_BUFFER_SIZE];
	static uint8_t freq_idx = 0;
	static uint8_t freq_count = 0;
	volatile uint32_t frequency = 0;

    while (1){
    	// INICIO DEL SISTEMA -> DESCALIBRADO
        if(calibration_mode){
        	//Entra en modo de calibracion
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
		if (calibrated){
			volatile uint32_t *buffer = NULL;
			if (buffer_ready_dma == 1){
			        buffer = bufferADC_A;
			        buffer_ready_dma = 0; // Resetear bandera
			    } else if (buffer_ready_dma == 2){
			        buffer = bufferADC_B;
			        buffer_ready_dma = 0; // Resetear bandera
			    }
			if(buffer != NULL){
				frequency = estimate_frequency((uint32_t *)buffer);
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
			// Calcular frecuencia promedio SOLO cuando se va a enviar (reduce jitter)
			uint64_t sum_freq = 0;
			for (int i = 0; i < freq_count; i++) {
				sum_freq += freq_buffer[i];
			}
			uint32_t avg_frequency = (uint32_t)(sum_freq / freq_count);

			// Enviar por UART la frecuencia promedio
			char out[40];
			itoa_simple((int)avg_frequency, out);
			send_string("FRECUENCIA PROMEDIO: ");
			send_string(out);
			send_string(" Hz\r\n");
		} else {
			// No se detectaron cruces válidos recientemente
			send_string("Sin señal valida...\r\n");
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
    cfgMatcher.MatchValue = 25;  // 100us --->ponerlo en 50 para que cuente 100us y tenga una frecuencia de 10kHz

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
 * @brief Configuracion del modulo GPDMA
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
    cfgLLI_A->NextLLI = (uint32_t)cfgLLI_B; // apunta al propio descriptor (no &cfgLLI)
    cfgLLI_A->Control = (NUM_SAMPLES & 0xFFF)    // transfer size
                    | (2 << 18)                // src width = word (32 bits)
                    | (2 << 21)                // dst width = word
                    | (1 << 27)                // dst increment
                    | (1UL << 31);             // enable terminal-count interrupt

    cfgLLI_B->SrcAddr = (uint32_t)&LPC_ADC->ADGDR;
    cfgLLI_B->DstAddr = (uint32_t)bufferADC_B;
    cfgLLI_B->NextLLI = (uint32_t)cfgLLI_A; // apunta al propio descriptor (no &cfgLLI)
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
    NVIC_EnableIRQ(DMA_IRQn);
}

/**
 * @brief Configuracion de EINT0 para iniciar calibracion
 */
void cfgEINT(void){
    PINSEL_CFG_Type pinEINT = {0};
    pinEINT.Portnum = 2;
    pinEINT.Pinnum = 10;
    pinEINT.Funcnum = 1;
    pinEINT.Pinmode = PINSEL_PINMODE_PULLUP;
    pinEINT.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&pinEINT);

    EXTI_SetMode(EXTI_EINT0, EXTI_MODE_EDGE_SENSITIVE);
    //EXTI_SetPolarity(EXTI_EINT0, EXTI_POLARITY_LOW_ACTIVE); // Flanco de bajada
    EXTI_ClearEXTIFlag(EXTI_EINT0);

    NVIC_EnableIRQ(EINT0_IRQn);
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
        if(LPC_GPDMACH0->DMACCLLI == (uint32_t)cfgLLI_B){
        	buffer_ready_dma = 1;
        }else if(LPC_GPDMACH0->DMACCLLI == (uint32_t)cfgLLI_A){
        	buffer_ready_dma = 2;
        }

        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);
    }

    NVIC_ClearPendingIRQ(DMA_IRQn);
}

/**
 * @brief Handler de EINT0 para iniciar calibracion
 */
void EINT0_IRQHandler(void){
    calibration_mode = 1;
    calibrated = 0;
    EXTI_ClearEXTIFlag(EXTI_EINT0);
    NVIC_ClearPendingIRQ(EINT0_IRQn);
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
/* ------------------  ESTIMATE FREQUENCY  ------------------ */
/**
 * @brief Determina la frecuencia de la señal en bufferADC usando detección de cruces por cero
 * @return Número de cruces por cero detectados en el buffer
 */
uint32_t estimate_frequency(uint32_t *samples){
	static int32_t prev_input = 0;			// Estado previo de entrada del HPF
	static int32_t prev_output = 0; 		// Estado previo de salida del HPF

	static uint32_t frequency = 0;

	static int current = 0;					// Estado actual: 0 desconocido, 1 arriba, -1 abajo
	static int prev = 0;					// Estado previo (para detectar transiciones)

	static uint32_t crosses = 0;
	static uint32_t cross_prev = 0;
	static uint32_t N = 0;

	/* Estado estatico para filtros */
	static int32_t prev_lpf_output = 0; 	// Estado previo del LPF (suavizado)

	for(int i=0; i<NUM_SAMPLES; i++){

		uint32_t adc_value = (samples[i]>>4)&0xFFF;
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
			crosses++;
			if(cross_prev != 0 || crosses > 1)
			{
				uint32_t cross_curr = i;
				if(cross_prev != 0){
								N = cross_curr - cross_prev;
							}
							cross_prev = cross_curr;
			}
		}
		// Actualizar estado previo si hay un estado valido
		if(current != 0){
			prev = current;
		}

	}
	if(N>0){
		frequency = ADC_RATE / N;
	}

	return frequency;
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
