#define _USE_MATH_DEFINES
#include "dma_manager.h"

const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 3, 6.5, 9, 11.5};

const uint32_t BufferResolution = DMA_Buffer_Resolution;

const uint16_t half_period = DMA_Buffer_Resolution / 2;

const uint16_t BufferGapPerMicroseconds = ((float)(1e-6)/TimeGapPerDMABufferBit);;

DMA_HandleTypeDef* DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32) uint16_t DMA_Buffer[DMA_CHANNELS][DMA_Buffer_Resolution] __attribute__((section(".dma")));

void DMA_Init()
{
    DMA_Stream_Handles[0] = &hdma_memtomem_dma1_stream0;
    DMA_Stream_Handles[1] = &hdma_memtomem_dma1_stream1;
    DMA_Stream_Handles[2] = &hdma_memtomem_dma1_stream2;
    DMA_Stream_Handles[3] = &hdma_memtomem_dma2_stream0;
    DMA_Stream_Handles[4] = &hdma_memtomem_dma2_stream1;
    Clean_DMABuffer();
    Start_DMAs();
}

void Start_DMAs()
{
    HAL_DMA_Start(&hdma_memtomem_dma1_stream0, (uint32_t)(DMA_Buffer[0]), (uint32_t)(&(GPIOA->ODR)), sizeof(DMA_Buffer[0]) / sizeof(DMA_Buffer[0][0]));
    HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)(DMA_Buffer[1]), (uint32_t)(&(GPIOB->ODR)), sizeof(DMA_Buffer[1]) / sizeof(DMA_Buffer[1][0]));
    HAL_DMA_Start(&hdma_memtomem_dma1_stream2, (uint32_t)(DMA_Buffer[2]), (uint32_t)(&(GPIOC->ODR)), sizeof(DMA_Buffer[2]) / sizeof(DMA_Buffer[2][0]));
    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)(DMA_Buffer[3]), (uint32_t)(&(GPIOD->ODR)), sizeof(DMA_Buffer[3]) / sizeof(DMA_Buffer[3][0]));
    HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)(DMA_Buffer[4]), (uint32_t)(&(GPIOE->ODR)), sizeof(DMA_Buffer[4]) / sizeof(DMA_Buffer[4][0]));
}

void Enable_DMAs()
{
    __disable_irq();
    for (int i = 0; i < 5; i++)
    {
        __HAL_DMA_ENABLE(DMA_Stream_Handles[i]);
    }
    __enable_irq();
}

void Disable_DMAs()
{
    __disable_irq();
    for (int i = 0; i < 5; i++)
    {
        __HAL_DMA_DISABLE(DMA_Stream_Handles[i]);
    }
    __enable_irq();
}

void Update_All_DMABuffer(enum ShootMode mode)
{
    for (size_t i = 0; i < NumTransducer; i++)
    {
        Update_Single_DMABuffer(&TransducerArray[i], mode);
    }
}

void Update_Single_DMABuffer(Transducer *currentTransducer, enum ShootMode mode)
{
    const uint8_t port_num = currentTransducer->port_num;
    const uint16_t pin = currentTransducer->pin;
    
    // 预计算常量部分
    // const uint16_t offset = (mode==Raw)? 0 :currentTransducer->calib + (mode==Raw)? 0 : currentTransducer->shift_buffer_bits + (GPIO_Group_Output_Offset[port_num] * BufferGapPerMicroseconds);

    uint16_t phase_offset = 0;
    if (mode != Raw) {
        // 这才是清晰的写法，而不是那堆垃圾三元运算符
        phase_offset = currentTransducer->calib + currentTransducer->shift_buffer_bits;
    }
    phase_offset += (uint16_t)(GPIO_Group_Output_Offset[port_num] * BufferGapPerMicroseconds);
    
    for (size_t j = 0; j < BufferResolution; j++)
    {
        if ((uint8_t)((j + phase_offset) / half_period) % 2 == 0)
        {
            DMA_Buffer[port_num][j] |= (pin); // Set to High Level
        }
        else
        {
            DMA_Buffer[port_num][j] &= ~(pin); // Set to Low Level
        }
    }
}

void Clean_DMABuffer()
{
    memset(DMA_Buffer, 0x0000, sizeof(DMA_Buffer));
}
