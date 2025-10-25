#include "LPC17xx.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"

void cfgGPIO(){
	PINSEL_CFG_Type pinGPIO;
	pinGPIO.Portnum = PINSEL_PORT_0;
	pinGPIO.Pinnum = PINSEL_PIN_22;
	pinGPIO.Funcnum = PINSEL_FUNC_0;
	pinGPIO.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinGPIO.OpenDrain = PINSEL_PINMODE_NORMAL;
    GPIO_SetDir(0, (1<<22), 1);
	LPC_GPIO0->FIOCLR |= (1<<22);
	LPC_GPIO1->FIOSET |= (1<<25);
	LPC_GPIO1->FIOSET |= (1<<26);
	PINSEL_ConfigPin(&pinGPIO);
}

void cfgADC(){
	PINSEL_CFG_Type pinADC;
	pinADC.Portnum = PINSEL_PORT_0;
	pinADC.Pinnum = PINSEL_PIN_23;
	pinADC.Funcnum = PINSEL_FUNC_1;
	pinADC.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinADC.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&pinADC);

	ADC_Init(LPC_ADC, 200000);
	ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_RISING);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	ADC_IntConfig(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
	ADC_BurstCmd(LPC_ADC, DISABLE);
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
}

void cfgTimer(){
	TIM_TIMERCFG_Type timer;
	timer.PrescaleOption = TIM_PRESCALE_USVAL;
	timer.PrescaleValue = 1000000;

	TIM_MATCHCFG_Type match;
	match.MatchChannel = 1;
	match.IntOnMatch = DISABLE;
	match.ResetOnMatch = ENABLE;
	match.StopOnMatch = DISABLE;
	match.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
	match.MatchValue = 60;

	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timer);
	TIM_ConfigMatch(LPC_TIM0, &match);
	TIM_Cmd(LPC_TIM0, ENABLE);
}

int main(){
	cfgGPIO();
	cfgTimer();
	cfgADC();
	while(1);
}

void ADC_IRQHandler(){
	LPC_GPIO0->FIOPIN ^= (1<<22);
//	uint16_t value = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
//	if(value > 0x800){
//		LPC_GPIO0->FIOSET |= (1<<6);
//	}else{
//		LPC_GPIO0->FIOCLR |= (1<<6);
//	}
	NVIC_ClearPendingIRQ(ADC_IRQn);
}
