#define _USE_MATH_DEFINES
#include "dma_manager.h"
#include "utiles.h"

const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 3, 6.5, 9, 11.5};
// const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 0U, 0U, 0U, 0U};

const uint32_t BufferResolution = DMA_Buffer_Resolution;

const uint16_t half_period = DMA_Buffer_Resolution / 2;

const uint16_t BufferGapPerMicroseconds = ((float)(1e-6)/TimeGapPerDMABufferBit);;

const uint16_t preserved_pins_mask = KEY0_Pin | KEY1_Pin | LED2_Pin | LED1_Pin | LED0_Pin;

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

void Update_All_DMABuffer()
{
    uint32_t start_cyc = DWT->CYCCNT;
    Clean_DMABuffer();
    for (size_t i = 0; i < NumTransducer; i++)
    {
        Update_Single_DMABuffer(&TransducerArray[i]);
    }
    uint32_t end_cyc = DWT->CYCCNT;
    updateDMABufferDeltaTime =  (double)((double)(end_cyc - start_cyc) / (double)SystemCoreClock);
}

void Update_Single_DMABuffer(Transducer *currentTransducer)
{
    const uint8_t port_num = currentTransducer->port_num;
    const uint16_t pin = currentTransducer->pin;
    
    uint16_t phase_offset = currentTransducer->calib + currentTransducer->shift_buffer_bits;

    phase_offset += (uint16_t)(GPIO_Group_Output_Offset[port_num] * BufferGapPerMicroseconds);

    phase_offset %= BufferResolution;
    
    uint32_t start_idx = (BufferResolution - phase_offset) % BufferResolution;
    uint32_t end_idx = start_idx + half_period;
    uint16_t* pBuffer = DMA_Buffer[port_num];

    if (end_idx <= BufferResolution)
    {
        // No wrap-around
        for (size_t j = start_idx; j < end_idx; j++)
        {
            pBuffer[j] |= pin;
        }
    }
    else
    {
        // Wrap-around: Fill to end, then start from beginning
        for (size_t j = start_idx; j < BufferResolution; j++)
        {
            pBuffer[j] |= pin;
        }
        size_t remaining = end_idx - BufferResolution;
        for (size_t j = 0; j < remaining; j++)
        {
            pBuffer[j] |= pin;
        }
    }
}

void Clean_DMABuffer()
{
    memset(DMA_Buffer, 0x0000, sizeof(DMA_Buffer));
    Restore_LED_State();
}
