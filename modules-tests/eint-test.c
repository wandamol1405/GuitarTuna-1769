#include <LPC17xx.h>
// Se asume que el proyecto incluye una implementación de la función 'delay' y los archivos header estándar.

// Definiciones de conveniencia
#define EINT0_BIT_POS 0 // EINT0 es el bit 0 en EXTINT, EXTMODE, EXTPOLAR
#define P2_10_PIN_BIT 10
#define P2_10_PINSEL_BITS (P2_10_PIN_BIT * 2) // Bits 21:20 en PINSEL4 [1]

// --- PIN LED DE PRUEBA (P0.22) ---
#define LED_PIN_BIT 22
#define LED_PIN_MASK (1 << LED_PIN_BIT)

// Protagonista de la interrupción: EINT0_IRQHandler [2]
void EINT0_IRQHandler(void) {
    // 1. Tarea de la interrupción: Conmutar el estado del LED (P0.22)
    // Utilizamos FIOPIN para conmutar directamente el pin de salida [3].
    LPC_GPIO0->FIOPIN ^= LED_PIN_MASK;

    // 2. Limpiar el flag de interrupción de EINT0
    // Se limpia escribiendo un 1 en el bit correspondiente del registro EXTINT [4-6].
    // Si no se limpia, no se detectarán eventos futuros [6, 7].
    LPC_SC->EXTINT = (1 << EINT0_BIT_POS);
}

// Función de inicialización
void Config_EINT0_Button(void) {
    // A. CONFIGURACIÓN DEL PIN P2.10 (EINT0)

    // A.1. Seleccionar Función EINT0 (Función 01 para P2.10 en PINSEL4)
    // PINSEL4 controla los pines de P2[15:0] [8, 9]. P2.10 utiliza bits 21:20 [1].
    // Se configuran los bits 21:20 a 01b (EINT0) [10].
    LPC_PINCON->PINSEL4 &= ~(3 << P2_10_PINSEL_BITS); // Limpiamos bits 21:20
    LPC_PINCON->PINSEL4 |= (1 << P2_10_PINSEL_BITS);  // Establecemos 01b (Función EINT0) [1]

    // A.2. Configurar Resistencia Interna para P2.10 (Se asume Pull-up, valor por defecto 00b)
    // El pin debe ser configurado como Pull-up (00b) o Pull-down (11b) [11, 12].
    // Si asumimos que el botón conecta a GND, se recomienda Pull-up (00b).
    // P2.10 está en PINMODE4, bits 21:20 [13].
    LPC_PINCON->PINMODE4 &= ~(3 << P2_10_PINSEL_BITS); // Aseguramos 00b (Pull-up) [11].

    // B. CONFIGURACIÓN DE LA INTERRUPCIÓN EXTERNA EINT0

    // B.1. Configurar EINT0 como sensible al flanco (Edge-sensitive)
    // EXTMODE: Bit 0 para EINT0. Valor 1 = Flanco [4, 14].
    LPC_SC->EXTMODE |= (1 << EINT0_BIT_POS);

    // B.2. Configurar EINT0 para Flanco de Bajada (Falling Edge)
    // EXTPOLAR: Bit 0 para EINT0. Valor 0 = Flanco de bajada [15, 16].
    LPC_SC->EXTPOLAR &= ~(1 << EINT0_BIT_POS);

    // C. CONFIGURACIÓN DEL PIN DE SALIDA P0.22 (LED)

    // C.1. Configurar P0.22 como GPIO (Función 00b por defecto) [17].
    // C.2. Configurar la dirección del pin como salida (Bit 22 en FIODIR0 = 1) [18, 19].
    LPC_GPIO0->FIODIR |= LED_PIN_MASK;
    LPC_GPIO0->FIOCLR = LED_PIN_MASK; // Aseguramos que el LED inicie apagado (LOW) [20].

    // D. Habilitar la interrupción EINT0 en el NVIC
    // EINT0 corresponde a la IRQn 18 [21]. Se utiliza la función CMSIS `NVIC_EnableIRQ` [22].
    NVIC_EnableIRQ(EINT0_IRQn);
}

// Programa principal
int main(void) {
    // Inicialización de hardware
    Config_EINT0_Button();

    // Bucle infinito: el programa pasa la mayor parte del tiempo esperando
    // a que el hardware externo (el botón) genere una interrupción.
    while (1) {
        // Aquí se pueden implementar funcionalidades de bajo consumo si es necesario,
        // o simplemente esperar.
        __asm volatile ("wfi"); // Esperar por interrupción (Wait For Interrupt)
    }
}
