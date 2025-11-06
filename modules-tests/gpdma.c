#include "LPC17xx.h"
#include "LPC17xxgpdma.h"

void gpdma_config(void) {
    GPDMA_Channel_CFG_Type gpdma_cfg;
    GPDMA_LLI_Type cfgADC_LLI;

    GPDMA_Init();

    // Configurar la estructura de configuración del canal GPDMA
    gpdma_cfg.ChannelNum = 0; // Canal 0
    gpdma_cfg.TransferSize = 

    // Inicializar el canal GPDMA con la configuración
    
    GPDMA_Setup(&gpdma_cfg);
}