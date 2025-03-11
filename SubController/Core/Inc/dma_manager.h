#pragma once
#include "main.h"
// #include "transducer.h"

# define DMABaseNum (1024*6515) 

# if EnableCarrier

# define CarrierFrequency 200U
# define DMA_Buffer_Resolution ((uint32_t)((double)DMABaseNum/CarrierFrequency + 0.5))
# define MainWaveLengthInBuffer (DMABaseNum/(Transducer_Base_Freq))
# define DMA_Buffer_Update UpdateDMA_BufferWith_Duty_AM()

# else

# define DMA_Buffer_Resolution ((uint32_t)(DMABaseNum/Transducer_Base_Freq))
# define MainWaveLengthInBuffer (DMA_Buffer_Resolution)
# define DMA_Buffer_Update UpdateDMA_Buffer()
# endif

const float GPIO_Group_Output_Offset[5] = {0U, 3, 6.5, 9, 11.5};

static DMA_HandleTypeDef* DMA_Stream_Handles[5];

static uint16_t DMA_Buffer[5][DMA_Buffer_Resolution] __attribute__((section(".dma")));

uint8_t DMA_Enable_Flag = 0;

void DMA_Init();
void StartDMAs();
void UpdateDMA_Buffer();
void UpdateDMA_BufferWith_Duty_AM();
void CleanDMABuffer();
void EnableDMAs();
void DisableDMAs();

#define TOGGLE_PIN(port, pin) ((port)->ODR ^= (pin))
#define SET_PIN(port, pin, value) ((value)?((port)->BSRR = (pin)):((port)->BSRR = (pin) << 16))
