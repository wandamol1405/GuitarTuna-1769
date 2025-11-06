/*
	Prueba de ADC en modo Burst con interrupciones.
	Enciende el led integrado si el valor del ADC es mayor a 0x800. De lo contrario
	apaga el led. 
	Utilizar el pin P0.23 como entrada analógica (ADC0) y un potenciómetro para variar
	el valor de tensión.
*/
#include "LPC17xx.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"

#define MIDDLE_VALUE 0x800

/**
* @brief Configuración del led integrado a la placa (P0.22)
*/
void cfgGPIO(){
	PINSEL_CFG_Type pinGPIO = {0};
	pinGPIO.Portnum = 0;
	pinGPIO.Pinnum = 22;
	pinGPIO.Funcnum = 0;
	pinGPIO.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinGPIO.OpenDrain = PINSEL_PINMODE_NORMAL;
    GPIO_SetDir(0,22, 1);
	LPC_GPIO0->FIOSET |= (1<<22); // Apagar led rojo (activo bajo)
	LPC_GPIO3->FIOSET |= (1<<25); // Apagar led verde (activo bajo)
	LPC_GPIO3->FIOSET |= (1<<26); // Apagar led azul (activo bajo)
	PINSEL_ConfigPin(&pinGPIO);
}

/**
* @brief Configuración del ADC (P0.23)
*/
void cfgADC(){
	PINSEL_CFG_Type pinADC = {0};
	pinADC.Portnum = 0;
	pinADC.Pinnum = 23;
	pinADC.Funcnum = 1;
	pinADC.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinADC.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&pinADC);

	ADC_Init(LPC_ADC, 200000); // ADC a 200kHz
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE); // Habilitar canal 0
	ADC_IntConfig(LPC_ADC, ADC_CHANNEL_0, ENABLE); // Habilitar interrupción para canal 0
	NVIC_EnableIRQ(ADC_IRQn); // Habilitar interrupción en NVIC
	ADC_BurstCmd(LPC_ADC, ENABLE); // Habilitar modo Burst
}

/**
* @brief Función principal
*/
int main(){
	cfgGPIO();
	cfgADC();
	while(1);
}

/**
* @brief Handler de la interrupción del ADC
*/
void ADC_IRQHandler(){
	uint16_t value = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
	if(value > MIDDLE_VALUE){ // Si el valor es mayor a la mitad del rango (4095)
		LPC_GPIO0->FIOSET |= (1<<22); // Encender led rojo (activo bajo)
	}else{
		LPC_GPIO0->FIOCLR |= (1<<22); // Apagar led rojo
	}
	NVIC_ClearPendingIRQ(ADC_IRQn);
}
