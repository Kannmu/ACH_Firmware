# pragma once
# include "main.h"
# include "transducer.h"

# define DMA_CHANNELS 5
# define _USE_MATH_DEFINES
# define DMABaseNum (1024 * 7850)

# define DMA_Buffer_Resolution ((uint32_t)(DMABaseNum/Transducer_Base_Freq))
# define MainWaveLengthInBuffer (DMA_Buffer_Resolution)

# define TimeGapPerDMABufferBit ((long double)(1.0/(Transducer_Base_Freq * DMA_Buffer_Resolution)))
// # define BufferGapPerMicroseconds ((float)(1e-6)/TimeGapPerDMABufferBit)

// dma_manager.h
extern const float GPIO_Group_Output_Offset[DMA_CHANNELS];

extern DMA_HandleTypeDef* DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32) extern uint16_t DMA_Buffer[DMA_CHANNELS][DMA_Buffer_Resolution] __attribute__((section(".dma")));

extern const uint32_t BufferResolution;

// extern long double TimeGapPerDMABufferBit;
extern const uint16_t BufferGapPerMicroseconds;

void DMA_Init();
void Start_DMAs();
void Update_All_DMABuffer();
void Update_Single_DMABuffer(Transducer *);
void Clean_DMABuffer();
