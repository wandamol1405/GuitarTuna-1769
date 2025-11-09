/**
 * @file dma-tm-adc.c
 * @brief Muestreo ADC con DMA disparado por Timer y envío por UART
 *
 * Configura el ADC para muestrear en el canal 0, disparado por un timer que genera
 * un evento cada 10 ms. Utiliza DMA para transferir 128 muestras a memoria cada vez
 * que se completa la transferencia. Calcula el promedio de las muestras y lo envía
 * por UART0.
 *
 */
#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_pinsel.h"
#include <string.h>

#define ADC_RATE 200000
#define NUM_SAMPLES 128
#define AHB_BASE_ADDR 0x20080000UL

typedef struct {
    uint32_t SrcAddr;
    uint32_t DstAddr;
    uint32_t NextLLI;
    uint32_t Control;
} myLLI_t;

void cfgADC(void);
void cfgUART(void);
void cfgTimer(void);
void cfgDMA(void);
void send_string(char* str);
void itoa_simple(int, char*);

myLLI_t *cfgLLI = (myLLI_t *)AHB_BASE_ADDR;
volatile uint32_t *bufferADC = (volatile uint32_t *)(AHB_BASE_ADDR + sizeof(cfgLLI));
volatile uint16_t average = 0;

int main(void) {
    SystemInit();
    cfgUART();          // Primero UART
    cfgADC();
    cfgTimer();
    cfgDMA();
    while (1){
		__WFI();
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
	ADC_IntConfig(LPC_ADC, ADC_ADINT0, ENABLE);
	ADC_BurstCmd(LPC_ADC, DISABLE);
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
