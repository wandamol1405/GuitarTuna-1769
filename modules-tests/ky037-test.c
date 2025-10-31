#include "LPC17xx.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"

void cfgGPIO(){
	PINSEL_CFG_Type pinGPIO = {0};
	pinGPIO.Portnum = 0;
	pinGPIO.Pinnum = 22;
	pinGPIO.Funcnum = 0;
	pinGPIO.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinGPIO.OpenDrain = PINSEL_PINMODE_NORMAL;
    GPIO_SetDir(0,22, 1);
	LPC_GPIO0->FIOSET |= (1<<22);
	LPC_GPIO1->FIOSET |= (1<<25);
	LPC_GPIO1->FIOSET |= (1<<26);
	PINSEL_ConfigPin(&pinGPIO);
}

void cfgADC(){
	PINSEL_CFG_Type pinADC = {0};
	pinADC.Portnum = 0;
	pinADC.Pinnum = 23;
	pinADC.Funcnum = 1;
	pinADC.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinADC.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&pinADC);

	ADC_Init(LPC_ADC, 200000);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	ADC_IntConfig(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
	ADC_BurstCmd(LPC_ADC, ENABLE);
	//ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
}


int main(){
	cfgGPIO();
	cfgADC();
	while(1);
}

void ADC_IRQHandler(){
	uint16_t value = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
	if(value > 0x800){
		LPC_GPIO0->FIOSET |= (1<<22);
	}else{
		LPC_GPIO0->FIOCLR |= (1<<22);
	}
	NVIC_ClearPendingIRQ(ADC_IRQn);
}
