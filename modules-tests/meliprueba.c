#include "LPC17xx.h"
#include "LPC17xxgpdma.h"


#define MEMORY_ADDR 0x2007C000 // Dirección base de la memoria RAM
#define SIZE_BUFFER 4095 // Tamaño del buffer de datos del ADC a memoria
#define ADC_RATE 10000 // Frecuencia de muestreo del ADC

#define VALOR_MEDIO_SILENCIO 2452 // mi cero lógico 
#define UMBRAL_RUIDO 10 // "zona muerta" +-10

volatile uint32_t *bufferADC = (uint32_t *) MEMORY_ADDR; // Buffer para almacenar los datos del ADC
//bandera global para avisar en el main()
volatile uint8_t buffer_listo = 0;


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
    LLI_cfg.SrcAddr = (uint32_t)&LPC_ADC->ADGDR; // Dirección del registro de datos del ADC
    LLI_cfg.DstAddr = (uint32_t)bufferADC; // Dirección del buffer de destino
    LLI_cfg.NextLLI = 0; // No hay siguiente LLI
    LLI_cfg.Control = (SIZE_BUFFER) | (2<<18) | (2<<21) | (0<<26) | (1<<27) | (1<<31);

    // Inicializar el canal GPDMA con la configuración
    
    GPDMA_Setup(&gpdma_cfg);
    //GPDMA_ChannelCmd(0, ENABLE); // Habilitar el canal GPDMA 0
	
    //lo habilitamos en el main
}

/**
 * @brief Configuracion del ADC, canal 0 mediante
 */
// ¡Esta es la configuración clave!

void cfgADC_con_DMA_y_TIMER(){
    
    // (Configuración del Pin P0.23... todo eso está igual)
    PINSEL_CFG_Type cfgCh0;
    cfgCh0.Portnum = 0;
    cfgCh0.Pinnum = 23;
    cfgCh0.Funcnum = 1;
    cfgCh0.Pinmode = PINSEL_PINMODE_TRISTATE;
    cfgCh0.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&cfgCh0);

    // Inicializamos el ADC (la tasa es solo el reloj, no el trigger)
    ADC_Init(LPC_ADC, 200000); // 200kHz está bien como reloj
    
    ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINT0, DISABLE); // ¡Correcto!
    ADC_DMACmd(LPC_ADC, ENABLE);                // ¡Correcto!

    // --- LA PARTE MÁS IMPORTANTE ---
    
    // 1. APAGAMOS EL BURST
    ADC_BurstCmd(LPC_ADC, DISABLE); 
    
    // 2. LE DECIMOS QUE OBEDEZCA AL TIMER
    // "Inicia la conversión en el borde de subida del MAT0.1"
    ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
}

// Vamos a reconfigurar esto para 10kHz
void cfgTimer_para_ADC(){
    
    TIM_TIMERCFG_Type cfgTimer;
    cfgTimer.PrescaleOption = TIM_PRESCALE_TICKVAL; // Usar ticks de PCLK
    cfgTimer.PrescaleValue = 1; // PCLK_TIMER / 1 = 25MHz

    TIM_MATCHCFG_Type cfgMatcher;
    cfgMatcher.MatchChannel = 1; // ¡Usaremos el Match 1!
    cfgMatcher.IntOnMatch = DISABLE;
    cfgMatcher.ResetOnMatch = ENABLE; // Se resetea cada 100us
    cfgMatcher.StopOnMatch = DISABLE;
    cfgMatcher.ExtMatchOutputType = TIM_EXTMATCH_NOTHING; // Sin salida externa
    
    // (PCLK_TIMER / Frecuencia) - 1
    // (25,000,000 / 10,000) - 1 = 2500 - 1 = 2499
    cfgMatcher.MatcherValue = 2499;

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &cfgTimer);
    TIM_ConfigMatch(LPC_TIM0, &cfgMatcher);
    TIM_Cmd(LPC_TIM0, ENABLE);
}


/**
 * @brief ¡¡FUNCIÓN COMPLETA!!
 * Procesa el buffer lleno y cuenta los cruces por el valor medio.
 * Implementa un "Disparador Schmitt" (histerésis) por software.
 */
uint32_t calcularCrucesPorCero(volatile uint32_t* buffer, uint32_t tamano) {
    
    uint32_t cruces = 0;
    uint16_t valor_limpio; // El valor 0-4095
    
    // --- 1. Definir la Zona Muerta (Lógica del Umbral) ---
    // (Usa los #defines globales)
    const uint16_t limite_superior = VALOR_MEDIO_SILENCIO + UMBRAL_RUIDO;
    const uint16_t limite_inferior = VALOR_MEDIO_SILENCIO - UMBRAL_RUIDO;

    // --- 2. Lógica de "estado_anterior" (La Memoria) ---
    // Estados: 1 (Arriba), -1 (Abajo), 0 (En zona muerta/inicio)
    int estado_anterior = 0;
    int estado_actual = 0;
    
    // --- 3. Encontrar el primer estado válido ---
    // (Ignoramos el silencio inicial hasta que la señal "salga" de la zona muerta)
    uint32_t i = 1;
    while(i < tamano) {
        valor_limpio = (buffer[i] >> 4) & 0xFFF; // Limpiar el primer dato
        
        if (valor_limpio > limite_superior) {
            estado_anterior = 1; // Empezamos "Arriba"
            break; // ¡Encontramos el primer estado!
        } else if (valor_limpio < limite_inferior) {
            estado_anterior = -1; // Empezamos "Abajo"
            break; // ¡Encontramos el primer estado!
        }
        i++; // Seguir buscando si está en la zona muerta
    }

    // --- 4. Recorrer el resto del buffer ---
    // (i continúa desde donde la dejamos)
    for (; i < tamano; i++) {
        
        // a. Limpiar el dato que trajo el DMA
        valor_limpio = (buffer[i] >> 4) & 0xFFF;
        
        // b. Determinar el estado actual (Arriba, Abajo, o Zona Muerta)
        if (valor_limpio > limite_superior) {
            estado_actual = 1;
        } else if (valor_limpio < limite_inferior) {
            estado_actual = -1;
        } else {
            // Está en la zona muerta, así que "finge" ser el estado anterior
            estado_actual = estado_anterior; 
        }

        // c. Comprobar si hubo un cruce REAL
        // (Un cruce real es cuando el estado cambia de 1 a -1 o de -1 a 1)
        if (estado_actual != estado_anterior) {
            cruces++; // ¡Contamos un cruce!
            estado_anterior = estado_actual; // Actualizamos la memoria para el próximo ciclo
        }
        
        // Si no cambió (ej. 1 -> 1 o -1 -> -1 o estaba en zona muerta)
        // no hacemos nada y la señal es ignorada.
    }
    
    return cruces;
}

// Handler que se llama cuando el DMA termina
void DMA_IRQHandler(void) {
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)) { // Si fue IRQ del Canal 0
        
        // 1. Detener el DMA (ya hizo su trabajo)
        GPDMA_ChannelCmd(0, DISABLE);
        // (El Timer sigue corriendo, pero el ADC ya no dispara el DMA)

        // 2. Avisar al main() que el buffer está listo
        buffer_listo = 1; 
            
        // 3. Limpiar la bandera de interrupción
        GPDMA_ClearIntPending(GPDMA_STAT_INTTC, 0);
    }
}


int main(void) {
    SystemInit();
    
    // (init_uart0() para depurar)
    
    // 1. Configurar todo el hardware
    gpdma_config();
    cfgADC_con_DMA_y_TIMER();
    cfgTimer_para_ADC(); // Inicia el timer

    // 2. Habilitar la interrupción del DMA en el CPU
    NVIC_EnableIRQ(DMA_IRQn);

    // 3. ¡INICIO! Empezar la primera captura
    GPDMA_ChannelCmd(0, ENABLE); 

    while(1) {
        
        __WFI(); // Dormir hasta la interrupción del DMA

        if (buffer_listo) {
            
            // 1. Procesar el buffer
            uint32_t N_cruces = calcularCrucesPorCero(bufferADC, SIZE_BUFFER);
            
            // 2. Calcular la frecuencia (¡OJO CON EL TIEMPO!)
            // Tiempo = 4095 muestras / 10000 muestras/seg = 0.4095 seg
            float tiempo_ventana = (float)SIZE_BUFFER / (float)ADC_RATE;
            float frecuencia = ( (float)N_cruces / 2.0f ) / tiempo_ventana;
            
            // 3. (Enviar 'frecuencia' por UART...)
            
            // 4. Reiniciar para la próxima captura
            buffer_listo = 0;
            GPDMA_ChannelCmd(0, ENABLE); // Reactivar el canal DMA
        }
    }
}

/**
 * @brief Handler de Interrupción del DMA.
 * Se llama UNA SOLA VEZ cuando el buffer (4095 muestras) se llena.
 */
void DMA_IRQHandler(void) {
    
    // Primero, verificar que la interrupción es la que esperamos
    // (Canal 0, Terminal Count - o sea, "trabajo terminado")
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)) { 
        
        // Tarea 1: Detener el canal DMA para que no escriba más.
        GPDMA_ChannelCmd(0, DISABLE);

        // Tarea 2: Avisar al main() que el buffer está listo.
        buffer_listo = 1; 
            
        // Tarea 3: Limpiar la bandera de interrupción. ¡Obligatorio!
        GPDMA_ClearIntPending(GPDMA_STAT_INTTC, 0);
    }
}