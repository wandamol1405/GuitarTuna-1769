#include "LPC17xx.h"

/**
 * @brief Configura UART0 para transmitir...
 */
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

/**
 * @brief Envía un string por UART0...
 */
void UART0_EnviarString(char* str) {
    while (*str != '\0') {
        while (!(LPC_UART0->LSR & 0x20)); // Esperar hasta que THR esté vacío
        LPC_UART0->THR = *str; // Enviar caracter
        str++; // Siguiente caracter
    }
}

/**
 * @brief Una función de espera simple
 */
void simple_delay(volatile uint32_t loops) {
    while(loops > 0) { 
        loops--;
    }
}

/**
 * @brief Convierte un entero a string (itoa simple)
 */
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


/** 
 * @brief Función principal
 */

int main(void) {

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
