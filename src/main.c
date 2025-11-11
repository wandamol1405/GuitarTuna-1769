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

/* MACRO PARA EL MUESTREO CONTINUO DEL MICROFONO*/
#define ADC_RATE 200000
#define NUM_SAMPLES 128 // TODO: a chequear
#define AHB_BASE_ADDR 0x2007C000UL   // Base real de la RAM AHB (32 KB)

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
void cfgEINT(void);
void send_string(char* str);
void itoa_simple(int, char*);
bool calibrate_microphone(void);
uint32_t zero_crossing_detection(void);


myLLI_t *cfgLLI  = (myLLI_t *)AHB_BASE_ADDR;

volatile uint32_t bufferCalibration[NUM_SAMPLES_CALIBRATION]; // Buffer para calibración
volatile uint32_t *bufferADC  = (volatile uint32_t *)(AHB_BASE_ADDR + sizeof(myLLI_t)); // Buffer para datos ADC

volatile uint16_t calibration_offset = 0; // Offset calculado en la calibración
volatile uint16_t noise_threshold = SIGMA_THRESHOLD; // Umbral de ruido para detección

int calibration_count = 0; // Contador de muestras para calibración

volatile int buffer_ready = 0; // Bandera para indicar que el buffer DMA está listo
volatile int calibration_mode = 0; // Bandera para indicar modo calibración
volatile int calibrated = 0; // Bandera para indicar si ya se calibró

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
    while (1){
        
        if(calibration_mode){
            NVIC_EnableIRQ(ADC_IRQn);
            if(calibrate_microphone()){
                calibrated = 1;
                NVIC_DisableIRQ(ADC_IRQn);
                calibration_mode = 0;
            }
        }
        
		if(calibrated && buffer_ready){
			uint32_t crosses = zero_crossing_detection();
            float window_time = (float)(NUM_SAMPLES * 10) / 1000.0f; // en segundos
            float frequency = (float)(crosses / 2) / window_time;
            char out[40];
            itoa_simple((int)frequency, out);
            send_string("FRECUENCIA: ");
            send_string(out);
            send_string(" Hz\r\n");

            buffer_ready = 0;
		}
	};
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

    // Habilitar canal
    GPDMA_ChannelCmd(cfgDMA.ChannelNum, ENABLE);

    // Enable IRQ
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
    EXTI_SetPolarity(EXTI_EINT0, EXTI_POLARITY_LOW_ACTIVE); // Flanco de bajada 
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
        buffer_ready = 1;

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
            if(calibration_count == NUM_SAMPLES_CALIBRATION){
                calibrate_microphone();
                calibration_count = 0;
            }
        }
    }
    NVIC_ClearPendingIRQ(ADC_IRQn);
}

/* ------------------  CALIBRATION ------------------ */
/**
 * @brief Calibrar el offset del micrófono
 */
bool calibrate_microphone(void) {
    uint32_t sum = 0; // Suma de las muestras
    uint32_t sumsq = 0; // Suma de los cuadrados de las muestras
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
        sumsq += ((uint32_t)sample) * ((uint32_t)sample); // Suma de los cuadrados de las muestras
        prev = sample; // Actualiza la muestra previa
    }

    // Calculo de media y sigma
    int valid_samples = NUM_SAMPLES_CALIBRATION - DISCARD_SAMPLES; // Muestras válidas consideradas
    float mean = (float)sum / valid_samples; // Media de las muestras
    float variance = ((float)sumsq / valid_samples) - (mean * mean); // Varianza de las muestras

    // Calculo de desviación estándar (sigma)
    float sigma = 0; 
    // Raíz cuadrada usando método de Newton-Raphson
    if (variance > 0) {
        float x = variance;
        for (int i = 0; i < 6; i++)
            x = 0.5f * (x + variance / x);
        sigma = x;
    }

    calibration_offset = (uint16_t)mean; // Guarda el offset calculado
    noise_threshold = (uint16_t)sigma;

    if(sigma > SIGMA_THRESHOLD) {
        send_string("Calibracion fallida: señal inestable\r\n");
    } else {
        send_string("Calibracion exitosa\r\n");
        char offset_str[20];
        itoa_simple(calibration_offset, offset_str);
        send_string("Offset calculado: ");
        send_string(offset_str);
        send_string("\r\n");
    }
    return sigma <= SIGMA_THRESHOLD;
}

/* ------------------  ZERO CROSSING DETECTION ------------------ */
/**
 * @brief Detección de cruces por cero y envío de resultados por UART
 */
uint32_t zero_crossing_detection() {
    uint32_t crosses = 0;
    uint16_t value;

    // Definir la zona muerta (logica del umbral)
    uint16_t upper_threshold = calibration_offset + noise_threshold; // Umbral superior
    uint16_t lower_threshold = calibration_offset - noise_threshold; // Umbral inferior

    // Lógica de estado anterior (memoria)
    // Estados: 1 (arriba), -1 (abajo), 0 (zona muerta/inicio)
    int previous_state = 0;
    int current_state = 0;

    // Encontrar el primer estado válido
    uint32_t i = 1;
    while(i < NUM_SAMPLES) {
        value = (bufferADC[i] >> 4) & 0xFFF; // Limpiar el dato

        if (value > upper_threshold) {
            previous_state = 1; // Empezamos "arriba"
            break; 
        } else if (value < lower_threshold) {
            previous_state = -1; // Empezamos "abajo"
            break;
        }
        i++; // Seguir buscando si está en la zona muerta
    }

    // Recorrer el resto del buffer
    for (; i < NUM_SAMPLES; i++) {
        // Limpiar el dato que trajo el DMA
        value = (bufferADC[i] >> 4) & 0xFFF;

        // Determinar el estado actual
        if (value > upper_threshold) {
            current_state = 1;
        } else if (value < lower_threshold) {
            current_state = -1;
        } else {
            // Está en la zona muerta, así que "finge" ser el estado anterior
            current_state = previous_state; 
        }

        // Verificar si hubo un cruce por cero
        if (current_state != previous_state) {
            if ((previous_state == 1 && current_state == -1) || 
                (previous_state == -1 && current_state == 1)) {
                crosses++; // Incrementar contador de cruces
            }
            previous_state = current_state; // Actualizar estado anterior
        }
    }

    return crosses  ;
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