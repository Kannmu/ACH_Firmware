#pragma once
#include "main.h"
#include "transducer.h"

#define _USE_MATH_DEFINES
# define DMABaseNum (1024*6515) 

# if EnableDutyAM

# define CarrierFrequency 200U
# define DMA_Buffer_Resolution ((uint32_t)((double)DMABaseNum/CarrierFrequency + 0.5))
# define MainWaveLengthInBuffer (DMABaseNum/(Transducer_Base_Freq))
# define UPDATE_DMA_BUFFER UpdateAllDMABuffer()

# else

# define DMA_Buffer_Resolution ((uint32_t)(DMABaseNum/Transducer_Base_Freq))
# define MainWaveLengthInBuffer (DMA_Buffer_Resolution)
# define UPDATE_DMA_BUFFER UpdateAllDMABuffer()
# endif

# define TimeGapPerDMABufferBit ((long double)(1.0/(Transducer_Base_Freq*DMA_Buffer_Resolution)))
// # define BufferGapPerMicroseconds ((float)(1e-6)/TimeGapPerDMABufferBit)

// dma_manager.h
extern const float GPIO_Group_Output_Offset[5];
extern uint8_t DMA_Enable_Flag;

extern DMA_HandleTypeDef* DMA_Stream_Handles[5];

__ALIGNED(32) extern uint16_t DMA_Buffer[5][DMA_Buffer_Resolution] __attribute__((section(".dma")));

extern const uint32_t BufferResolution;
// extern long double TimeGapPerDMABufferBit;
extern const uint16_t BufferGapPerMicroseconds;

void DMA_Init();
void StartDMAs();
void UpdateAllDMABuffer();
void UpdateSingleDMABuffer(Transducer *, enum ShootMode);
void CleanDMABuffer();
void EnableDMAs();
void DisableDMAs();

#define TOGGLE_PIN(port, pin) ((port)->ODR ^= (pin))
#define SET_PIN(port, pin, value) ((value)?((port)->BSRR = (pin)):((port)->BSRR = (pin) << 16))
