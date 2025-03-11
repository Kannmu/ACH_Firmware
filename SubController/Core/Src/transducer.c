
#include "transducer.h"
#include "calibration.h"
#include "dma_manager.h"

// Transducer Array

void Transducer_Init(void)
{
    for (int i = 0; i < NumTransducer; i++)
    {
        TransducerArray[i] = (Transducer *)malloc(sizeof(Transducer));
        TransducerArray[i]->Index = i;
        TransducerArray[i]->port = map_pin_name_to_gpio_port(transducer_pins[i]);
        TransducerArray[i]->port_num = map_pin_name_to_gpio_port_num(transducer_pins[i]);
        TransducerArray[i]->pin = map_pin_name_to_pin_number(transducer_pins[i]);
        TransducerArray[i]->calib = Transducer_Calibration_Array[i] * BufferGapPerMicroseconds;
        TransducerArray[i]->row = i / ArraySize;
        TransducerArray[i]->column = i % ArraySize;
        TransducerArray[i]->coordinate[0] = (TransducerArray[i]->row - (ArraySize / 2.0) + 0.5) * TransducerGap;    // X
        TransducerArray[i]->coordinate[1] = (TransducerArray[i]->column - (ArraySize / 2.0) + 0.5) * TransducerGap; // Y
        TransducerArray[i]->coordinate[2] = 0;                                                                      // Z
        TransducerArray[i]->phase = 0;
        TransducerArray[i]->duty = 0.5;
        TransducerArray[i]->shift_buffer_bits = 0;
    }
}

void SingleFire(Transducer *TempT, uint8_t Clean)
{
    if (Clean == 1U)
        CleanDMABuffer();
    for (int j = 0; j < DMA_Buffer_Resolution; j++)
    {
        if (((uint16_t)((j + TempT->calib + TempT->shift_buffer_bits + (GPIO_Group_Output_Offset[TempT->port_num] * BufferGapPerMicroseconds)) / (DMA_Buffer_Resolution / 2)) % 2 == 0))
        {
            DMA_Buffer[TempT->port_num][j] &= ~(TempT->pin); // Set to Low Level
        }
        else
        {
            DMA_Buffer[TempT->port_num][j] |= (TempT->pin); // Set to High Level
        }
    }
}

void FullFire(Transducer *T[])
{
    CleanDMABuffer();
    for (int i = 0; i < NumTransducer; i++)
    {
        Transducer *TempT = TransducerArray[i];
        for (int j = 0; j < DMA_Buffer_Resolution; j++)
        {
            if (((uint16_t)((j + TempT->calib + (GPIO_Group_Output_Offset[TempT->port_num] * BufferGapPerMicroseconds)) / (DMA_Buffer_Resolution / 2)) % 2 == 0))
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

// Update Simulation to Transducers Parameters
void Transducer_UpdateAllPhase(Point P)
{
    for (int i = 0; i < NumTransducer; i++)
    {
        // Distance Calculation
        TransducerArray[i]->distance = EulerDistance(TransducerArray[i]->coordinate, P.position);

        // Distance to Phase
        TransducerArray[i]->phase = (2 * pi) - (fmod((TransducerArray[i]->distance * Wave_K), (2.0 * pi)));

        if (TwinTrap == 1)
            TransducerArray[i]->phase += (TransducerArray[i]->column > 3) ? pi / 2 : 2.5 * pi;
        // Phase to Gap Ticks
        TransducerArray[i]->shift_buffer_bits = (TransducerArray[i]->phase / (2 * pi * Transducer_Base_Freq)) / TimeGapPerDMABufferBit;
    }

    DMA_Buffer_Update;
}

GPIO_TypeDef *map_pin_name_to_gpio_port(const char *pin_name)
{
    if (pin_name == NULL)
        return NULL;

    switch (pin_name[1])
    {
    case 'A':
        return GPIOA;
    case 'B':
        return GPIOB;
    case 'C':
        return GPIOC;
    case 'D':
        return GPIOD;
    case 'E':
        return GPIOE;
    case 'F':
        return GPIOF;
    default:
        return GPIOA;
    }
}

uint8_t map_pin_name_to_gpio_port_num(const char *pin_name)
{
    if (pin_name == NULL)
        return 0;

    switch (pin_name[1])
    {
    case 'A':
        return 0U;
    case 'B':
        return 1U;
    case 'C':
        return 2U;
    case 'D':
        return 3U;
    case 'E':
        return 4U;
    case 'F':
        return 5U;
    default:
        // Error_Handler();
        return 0U;
    }
}

uint16_t map_pin_name_to_pin_number(const char *pin_name)
{

    if (pin_name == NULL)
        return 0;

    char pin_number_str[4];
    strncpy(pin_number_str, &pin_name[2], 3);
    pin_number_str[3] = '\0';

    int pin_number = atoi(pin_number_str);
    switch (pin_number)
    {
    case 0:
        return GPIO_PIN_0;
    case 1:
        return GPIO_PIN_1;
    case 2:
        return GPIO_PIN_2;
    case 3:
        return GPIO_PIN_3;
    case 4:
        return GPIO_PIN_4;
    case 5:
        return GPIO_PIN_5;
    case 6:
        return GPIO_PIN_6;
    case 7:
        return GPIO_PIN_7;
    case 8:
        return GPIO_PIN_8;
    case 9:
        return GPIO_PIN_9;
    case 10:
        return GPIO_PIN_10;
    case 11:
        return GPIO_PIN_11;
    case 12:
        return GPIO_PIN_12;
    case 13:
        return GPIO_PIN_13;
    case 14:
        return GPIO_PIN_14;
    case 15:
        return GPIO_PIN_15;
    default:
        return GPIO_PIN_0;
    }
}
