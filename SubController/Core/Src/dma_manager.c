#include "dma_manager.h"

void DMA_Init()
{
    DMA_Stream_Handles[0] = &hdma_memtomem_dma1_stream0;
    DMA_Stream_Handles[1] = &hdma_memtomem_dma1_stream1;
    DMA_Stream_Handles[2] = &hdma_memtomem_dma1_stream2;
    DMA_Stream_Handles[3] = &hdma_memtomem_dma2_stream0;
    DMA_Stream_Handles[4] = &hdma_memtomem_dma2_stream1;
    CleanDMABuffer();
    StartDMAs();
}

void StartDMAs()
{
    HAL_DMA_Start(&hdma_memtomem_dma1_stream0, (uint32_t)(DMA_Buffer[0]), (uint32_t)(&(GPIOA->ODR)), sizeof(DMA_Buffer[0]) / sizeof(DMA_Buffer[0][0]));
    HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)(DMA_Buffer[1]), (uint32_t)(&(GPIOB->ODR)), sizeof(DMA_Buffer[1]) / sizeof(DMA_Buffer[1][0]));
    HAL_DMA_Start(&hdma_memtomem_dma1_stream2, (uint32_t)(DMA_Buffer[2]), (uint32_t)(&(GPIOC->ODR)), sizeof(DMA_Buffer[2]) / sizeof(DMA_Buffer[2][0]));
    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)(DMA_Buffer[3]), (uint32_t)(&(GPIOD->ODR)), sizeof(DMA_Buffer[3]) / sizeof(DMA_Buffer[3][0]));
    HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)(DMA_Buffer[4]), (uint32_t)(&(GPIOE->ODR)), sizeof(DMA_Buffer[4]) / sizeof(DMA_Buffer[4][0]));
    DMA_Enable_Flag = 1;
}

void EnableDMAs()
{
    for (int i = 0; i < 5; i++)
    {
        __HAL_DMA_ENABLE(DMA_Stream_Handles[i]);
    }
    DMA_Enable_Flag = 1;
}

void DisableDMAs()
{
    for (int i = 0; i < 5; i++)
    {
        __HAL_DMA_DISABLE(DMA_Stream_Handles[i]);
    }
    DMA_Enable_Flag = 0;
}

void UpdateDMA_Buffer()
{
    // CleanDMABuffer();
    for (int i = 0; i < NumTransducer; i++)
    {
        Transducer *TempT = TransducerArray[i];
        for (int j = 0; j < DMA_Buffer_Resolution; j++)
        {
            if (((uint8_t)((j + TempT->calib + TempT->shift_buffer_bits + (GPIO_Group_Output_Offset[TempT->port_num] * BufferGapPerMicroseconds)) / (DMA_Buffer_Resolution / 2)) % 2 == 0))
            {
                DMA_Buffer[TempT->port_num][j] |= (TempT->pin); // Set to High Level
            }
            else
            {
                DMA_Buffer[TempT->port_num][j] &= ~(TempT->pin); // Set to Low Level
            }
        }
    }
}

void CleanDMABuffer()
{
    memset(DMA_Buffer, 0x0000, sizeof(DMA_Buffer));
}
