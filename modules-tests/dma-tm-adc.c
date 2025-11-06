#include "LPC17xx.h"
#include "LPC17xxgpdma.h"


#define MEMORY_ADDR 0x2007C000 // Dirección base de la memoria RAM
#define SIZE_BUFFER 4095 // Tamaño del buffer de datos del ADC a memoria 
volatile uint32_t *bufferADC = (uint32_t *) MEMORY_ADDR; // Buffer para almacenar los datos del ADC

void init_uart0(uint32_t baudrate) {
    // 1. Encender periférico (Power Control)
    LPC_SC->PCONP |= (1 << 3); // PCUART0 = 1

    // 2. Configurar pines (Pin Select)
    LPC_PINCON->PINSEL0 |= (1 << 4); // P0.2 como TXD0
    LPC_PINCON->PINSEL0 |= (1 << 6); // P0.3 como RXD0

    // 3. Configurar formato (Line Control Register)
    LPC_UART0->LCR = 0x83; // 8-N-1, DLAB = 1

    // 4. Configurar Baudrate
    uint32_t PCLK_UART = 25000000; // Asumir PCLK = 25MHz
    uint32_t Fdiv = (PCLK_UART) / (16 * baudrate);
    LPC_UART0->DLL = Fdiv & 0xFF;
    LPC_UART0->DLM = (Fdiv >> 8) & 0xFF;

    // 5. Deshabilitar DLAB y habilitar FIFO
    LPC_UART0->LCR = 0x03; // DLAB = 0
    LPC_UART0->FCR = 0x07; // Habilitar y resetear FIFOs
}

/**
 * @brief Configura el ADC para el canal 0...
 */
void init_adc(void) {
    // 1. Encender periférico
    LPC_SC->PCONP |= (1 << 12); // PCADC = 1

    // 2. Configurar pin P0.23 como AD0.0
    LPC_PINCON->PINSEL1 |= (1 << 14);

    // 3. Deshabilitar pull-up/pull-down
    LPC_PINCON->PINMODE1 |= (2 << 14);

    // 4. Configurar el ADC
    LPC_ADC->ADCR = (1 << 0)    // SEL: Usar canal 0
                  | (4 << 8)    // CLKDIV: (25MHz / (4+1)) = 5MHz
                  | (0 << 16)   // BURST: 0
                  | (1 << 21)   // PDN: Power-On
                  | (0 << 24);  // START: 0

    // 5. Deshabilitar interrupciones
    LPC_ADC->ADINTEN = 0x00;
}

//envia un string por uart
void UART0_EnviarString(char* str) {
    while (*str != '\0') {
        while (!(LPC_UART0->LSR & 0x20)); // Esperar hasta que THR esté vacío
        LPC_UART0->THR = *str; // Enviar caracter
        str++; // Siguiente caracter
    }
}

//funcion de espera
void simple_delay(volatile uint32_t loops) {
    while(loops > 0) { 
        loops--;
    }
}

 //entero a string
void itoa_simple(int n, char s[]) {
    int i, sign;

    if ((sign = n) < 0)  // Guarda el signo
        n = -n;          // Lo hace positivo
    i = 0;
    do {       // Genera dígitos en orden reverso
        s[i++] = n % 10 + '0';   // Obtiene el dígito
    } while ((n /= 10) > 0);     // Lo elimina

    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0'; // Caracter nulo final

    // Invierte el string (porque se generó al revés)
    int j = 0;
    char temp;
    i--; // Apunta al último caracter, no al '\0'
    while (j < i) {
        temp = s[j];
        s[j] = s[i];
        s[i] = temp;
        j++;
        i--;
    }
}

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
	ADC_BurstCmd(LPC_ADC, ENABLE);
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

int main(){
	cfgTimer();
	cfgADC();
	gpdma_config();

    // Un buffer para guardar el texto
    char buffer_texto[50];
    uint16_t valor_adc;

    SystemInit();

    // Llamamos a las funciones de inicialización
    init_uart0(9600);
    init_adc();

    UART0_EnviarString("--- Medicion del micrófono ---\r\n");

    while(1) {

        // 1. Iniciar una conversión del ADC
        LPC_ADC->ADCR |= (1 << 24);

        // 2. Esperar a que la conversión termine
        while ( !(LPC_ADC->ADDR0 & (1U << 31)) );

        // 3. Leer el valor (12 bits)
        valor_adc = (LPC_ADC->ADDR0 >> 4) & 0xFFF;

        // 4. Formatear el valor en un string y enviarlo por UART
        UART0_EnviarString("Valor ADC: ");
        itoa_simple(valor_adc, buffer_texto); // Convierte el número a texto
        UART0_EnviarString(buffer_texto);     // Envía el texto
        UART0_EnviarString("\r\n");            // Envía el salto de línea

        // 5. Esperar un poco
        simple_delay(2000000);
    }
}


void DMA_IRQHandler(DMA_IRQHandler){
	ADC_BurstCmd(LPC_ADC, DISABLE);
	GPDMA_ClearIntChannel(0);
	procesarMuestras(bufferADC, SIZE_BUFFER);
	gpdma_config();
	//al finalizar el handler enable de nuevo del adc
	ADC_BurstCmd(LPC_ADC, ENABLE);
}

void procesarMuestras(bufferADC, SIZE_BUFFER) {

    uint32_t cruces = 0;
    uint32_t anterior = bufferADC[0];

    // Nivel de referencia (mitad del rango del ADC de 12 bits)
    uint32_t nivelCero = 2450;       
    uint32_t margenRuido = 10;       

    for (uint32_t i = 1; i < SIZE_BUFFER; i++) {
        uint32_t actual = bufferADC[i];

        // Detectar cruce por el cero
        if ((anterior < (nivelCero - margenRuido) && actual > (nivelCero + margenRuido)) ||
            (anterior > (nivelCero + margenRuido) && actual < (nivelCero - margenRuido))) {
            cruces++;
        }

        anterior = actual;
    }

    float tiempoMuestra_s = 0.001f; //tiempo entre 2 muestras

    // Tiempo total del bloque
    float tiempoTotal = tiempoMuestra_s * SIZE_BUFFER; //tiempo q tarda en dma en llenar el buffer

    // Cada ciclo completo tiene 2 cruces por cero
    float frecuencia = (cruces / 2.0f) / tiempoTotal; //regla de extremos y medios

    // Mostrar resultado
    compararFrecuencia(frecuencia);
}

void compararFrecuencia(float frecuencia) {
    char texto[64];
    float frecuenciaIdeal = 82.41f;  // Ejemplo: cuerda E de guitarra
    float tolerancia = 1.0f;  // +-1 Hz

    if (frecuencia < (frecuenciaIdeal - tolerancia)) {
        snprintf(texto, sizeof(texto), "Tensar cuerda: %.2f Hz\r\n", frecuencia);
    } else if (frecuencia > (frecuenciaIdeal + tolerancia)) {
        snprintf(texto, sizeof(texto), "Aflojar cuerda: %.2f Hz\r\n", frecuencia);
    } else {
        snprintf(texto, sizeof(texto), "Afinada: %.2f Hz\r\n", frecuencia);
    }

    UART_send(texto); // enviar por puerto serie
}

