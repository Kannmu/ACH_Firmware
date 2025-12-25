# define _USE_MATH_DEFINES
# include "transducer.h"
# include "calibration.h"
# include "dma_manager.h"
# include "custom_math.h"

float Wave_K = ((2.0*M_PI*Transducer_Base_Freq)/SoundSpeed);

// Transducer Array
const char *TransducerPins[] =
    {
        "PC11", "PD2", "PD4", "PD6", "PB4", "PB6", "PB8", "PE0",

        "PC12", "PD3", "PD5", "PD7", "PB5", "PB7", "PB9", "PE1",

        "PA3", "PA1", "PC3", "PC1", "PC15", "PC13", "PE5", "PE3",

        "PA2", "PA0", "PC2", "PC0", "PC14", "PE6", "PE4", "PE2",

        "PC9", "PC7", "PD15", "PD13", "PD11", "PD9", "PB15", "PB13",

        "PC8", "PC6", "PD14", "PD12", "PD10", "PD8", "PB14", "PB12",

        "PE15", "PE13", "PE11", "PE9", "PE7", "PB1", "PC5", "PA7",

        "PE14", "PE12", "PE10", "PE8", "PB2", "PB0", "PC4", "PA6",

        "PC10"};

Transducer TransducerArray[NumTransducer];

void Transducer_Init(void)
{
    for (size_t i = 0; i < NumTransducer; i++)
    {
        // TransducerArray[i] = (Transducer *)malloc(sizeof(Transducer));
        TransducerArray[i].index = i;
        TransducerArray[i].port = map_pin_name_to_gpio_port(TransducerPins[i]);
        TransducerArray[i].port_num = map_pin_name_to_gpio_port_num(TransducerPins[i]);
        TransducerArray[i].pin = map_pin_name_to_pin_number(TransducerPins[i]);
        TransducerArray[i].calib = Transducer_Calibration_Array[i] * BufferGapPerMicroseconds;
        TransducerArray[i].row = i / ArraySize;
        TransducerArray[i].column = i % ArraySize;
        TransducerArray[i].position3D[0] = (TransducerArray[i].row - (ArraySize / 2.0) + 0.5) * TransducerGap;    // X
        TransducerArray[i].position3D[1] = (TransducerArray[i].column - (ArraySize / 2.0) + 0.5) * TransducerGap; // Y
        TransducerArray[i].position3D[2] = 0;                                                                      // Z
        TransducerArray[i].distance = 0;
        TransducerArray[i].phase = 0;
        TransducerArray[i].duty = 0.5;
        TransducerArray[i].shift_buffer_bits = 0;
    }
}

void Clean_Transducers_Calib()
{
    for (int i = 0; i < NumTransducer-1; i++)
    {
        TransducerArray[i].calib = 0;
    }
}

void Set_Transducers_Calib()
{
    for (int i = 0; i < NumTransducer-1; i++)
    {
        TransducerArray[i].calib = Transducer_Calibration_Array[i] * BufferGapPerMicroseconds;
    }
}

void Set_Plane_Wave()
{
    for (int i = 0; i < NumTransducer-1; i++)
    {
        TransducerArray[i].phase = 0;
        TransducerArray[i].shift_buffer_bits = 0;
    }
}

// Update Simulation to Transducers Parameters
void Set_Focus_Point(float *position)
{
    for (int i = 0; i < NumTransducer-1; i++)
    {
        // Distance Calculation
        TransducerArray[i].distance = Euler_Distance(TransducerArray[i].position3D, position);

        // Distance to Phase
        TransducerArray[i].phase = Distance_to_Phase(TransducerArray[i].distance);

        // Phase to Gap Ticks
        TransducerArray[i].shift_buffer_bits = Phase_to_Gap_Ticks(TransducerArray[i].phase);
    }
}

// Set Phases to Transducers
void Set_Phases(float phases[])
{
    for (int i = 0; i < NumTransducer-1; i++)
    {
        TransducerArray[i].phase = phases[i];
        TransducerArray[i].shift_buffer_bits = Phase_to_Gap_Ticks(TransducerArray[i].phase);
    }
}

float Distance_to_Phase(float distance)
{
    return (2.0 * M_PI) - (fmod((distance * Wave_K), (2.0 * M_PI)));
}

float Phase_to_Gap_Ticks(float phase)
{
    return (phase / (2.0 * M_PI * Transducer_Base_Freq)) / TimeGapPerDMABufferBit;
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

static const uint16_t port_num_map[] = {
    ['A']=0, ['B']=1, ['C']=2, ['D']=3, ['E']=4
};

uint8_t map_pin_name_to_gpio_port_num(const char *pin) {
    return port_num_map[(int)pin[1]];
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
