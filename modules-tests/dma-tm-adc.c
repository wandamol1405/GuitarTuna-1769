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
myLLI_t *cfgLLI2 = (myLLI_t *)AHB_BASE_ADDR;
volatile uint32_t *bufferADC2 = (volatile uint32_t *)(AHB_BASE_ADDR + NUM_SAMPLES + sizeof(cfgLLI));
volatile uint16_t average = 0;

int main(void) {
    SystemInit();
    cfgUART();
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
    cfgLLI->NextLLI = (uint32_t)&cfgLLI2; // no encadenado (o apuntar a sí mismo para circular)
    cfgLLI->Control = (NUM_SAMPLES & 0xFFF)    // transfer size
                    | (2 << 18)                // src width = word (32 bits)
                    | (2 << 21)                // dst width = word
                    | (1 << 27)                // dst increment
                    | (1UL << 31);             // enable terminal-count interrupt

    cfgLLI2->SrcAddr = (uint32_t)&LPC_ADC->ADGDR;
    cfgLLI2->DstAddr = (uint32_t)bufferADC2;
    cfgLLI2->NextLLI = (uint32_t)&cfgLLI; // no encadenado (o apuntar a sí mismo para circular)
    cfgLLI2->Control = (NUM_SAMPLES & 0xFFF)    // transfer size
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
    cfgDMA.DMALLI = (uint32_t)&cfgLLI; // apunta al descriptor en AHB

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
 * @brief Handler de DMA
 */
void DMA_IRQHandler(void){
    uint32_t tc_stat = LPC_GPDMA->DMACIntTCStat;
    uint32_t err_stat = LPC_GPDMA->DMACIntErrStat;

    if (err_stat & (1<<0)) {
        send_string("GPDMA ERROR (IRQ): err_stat=");
        char tmp[12];
        itoa_simple(err_stat, tmp);
        send_string(tmp);
        send_string("\r\n");

        GPDMA_ClearIntPending(GPDMA_STATCLR_INTERR, 0);
        return;
    }

    if (tc_stat & (1<<0)) {
        // calculo promedio (extraer 12 bits como hacías)
        uint32_t sum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            uint32_t raw = bufferADC[i];
            raw &= 0x0000FFF0;
            uint16_t value12 = raw >> 4;
            sum += value12;
        }
        average = (uint16_t)(sum / NUM_SAMPLES);
        char out[20];
        itoa_simple(average, out);
        send_string("PROMEDIO DMA: ");
        send_string(out);
        send_string("\r\n");

        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);
    }
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

