#include "LPC17xx.h"
#include "LPC17xxgpdma.h"


#define MEMORY_ADDR 0x2007C000 // Dirección base de la memoria RAM
#define SIZE_BUFFER 4095 // Tamaño del buffer de datos del ADC a memoria 
volatile uint32_t *bufferADC = (uint32_t *) MEMORY_ADDR; // Buffer para almacenar los datos del ADC


void gpdma_config(void) {
    GPDMA_Channel_CFG_Type gpdma_cfg;
    GPDMA_LLI_Type LLI_cfg;

    GPDMA_Init();
    
    // Configurar la estructura de configuración del canal GPDMA
    gpdma_cfg.ChannelNum = 0; // Canal 0
    gpdma_cfg.TransferSize = SIZE_BUFFER; // Tamaño del buffer
    gpdma_cfg.TransferType = GPDMA_TRANSFERTYPE_P2M; // Transferencia de periférico a memoria
    gpdma_cfg.SrsMemAddr = 0;
    gpdma_cfg.DestMemAddr = (uint32_t)bufferADC; // Dirección del buffer de destino
    gpdma_cfg.SrcConn = GPDMA_CONN_ADC; // Fuente: ADC
    gpdma_cfg.DestConn = 0; // Destino: memoria

    gpdma_cfg.DMALLI = (uint32_t)&LLI_cfg; // Configuración de la lista de enlaces
    LLI_cfg.SrcAddr = (uint32_t)&LPC_ADC->ADDR; // Dirección del registro de datos del ADC
    LLI_cfg.DstAddr = (uint32_t)bufferADC; // Dirección del buffer de destino
    LLI_cfg.NextLLI = 0; // No hay siguiente LLI
    LLI_cfg.Control = (2<<18) | (2<<21) | (0<<26) | (1<<27) | (1<<31); // Configuración del control de la transferencia

    // Inicializar el canal GPDMA con la configuración
    
    GPDMA_Setup(&gpdma_cfg);
    GPDMA_ChannelCmd(0, ENABLE); // Habilitar el canal GPDMA 0

}

void configADC (){
    ADC_Init(LPC_ADC, 10000); // Inicializar el ADC con una frecuencia de 10khz
    ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_RISING);
    ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS); // Iniciar el ADC en modo rafaga
    ADC_BurstCmd(LPC_ADC, ENABLE); // Habilitar el modo rafaga
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE); // Habilitar el canal 0
}

/**
 * @brief Configuracion del ADC, canal 0 mediante
 */
void cfgADC(){
    
	PINSEL_CFG_Type cfgCh0;
	cfgCh0.Portnum = 0;
	cfgCh0.Pinnum = 23;
	cfgCh0.Funcnum = 1;
	cfgCh0.Pinmode = PINSEL_PINMODE_TRISTATE;
	cfgCh0.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&cfgCh0);

	ADC_Init(LPC_ADC, ADC_RATE);
	ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_RISING);
	ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
	ADC_IntConfig(LPC_ADC, ADC_ADINT0, ENABLE);
	ADC_BurstCmd(LPC_ADC, DISABLE);
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
	NVIC_EnableIRQ(ADC_IRQn);

}
/*
 * @brief Configuracion del Timer
 */
void cfgTimer(){
	TIM_TIMERCFG_Type cfgTimer;
	cfgTimer.PrescaleOption = TIM_PRESCALE_USVAL;
	cfgTimer.PrescaleValue = 1000;

	TIM_MATCHCFG_Type cfgMatcher;
	cfgMatcher.MatchChannel = 0;
	cfgMatcher.IntOnMatch = DISABLE;
	cfgMatcher.ResetOnMatch = ENABLE;
	cfgMatcher.StopOnMatch = DISABLE;
	cfgMatcher.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
	cfgMatcher.MatcherValue = 500;

	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &cfgTimer);
	TIM_ConfigMatch(LPC_TIM0, &cfgMatcher);
	TIM_Cmd(LPC_TIM0, ENABLE);
}
