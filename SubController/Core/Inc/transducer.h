#pragma once


const char *transducer_pins[] =
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

# define ArraySize 8
# define TransducerGap (16.602 * (1e-3))
# define NumTransducer (sizeof(transducer_pins) / sizeof(transducer_pins[0]))

# define SoundSpeed 343.2

# define Transducer_Base_Freq 40000UL

# define Wave_K ((2*pi*Transducer_Base_Freq)/SoundSpeed)

# define TimeGapPerDMABufferBit ((long double)(1.0/(Transducer_Base_Freq*DMA_Buffer_Resolution)))

# define BufferGapPerMicroseconds ((float)(1e-6)/TimeGapPerDMABufferBit)

# define TransducerPeriod  ((long double)(1.0 / Transducer_Base_Freq))

# define WaveLength (TransducerPeriod*SoundSpeed)

#include "main.h"
#include "simulation.h"

// Transducer Class
typedef struct Transducer
{
    uint8_t Index;
    uint8_t row;
    uint8_t column;
    float coordinate[3];

    GPIO_TypeDef *port;
    uint8_t port_num;
    uint16_t pin;
    uint16_t calib;

    float distance;
    float phase;
    uint16_t shift_buffer_bits;

    float duty;
} Transducer;

struct Transducer *TransducerArray[NumTransducer];

void Transducer_Init(void);
void Transducer_UpdateAllPhase(Point);

void FullFire(Transducer *[]);
void SingleFire(Transducer *, uint8_t);
GPIO_TypeDef *map_pin_name_to_gpio_port(const char *);
uint8_t map_pin_name_to_gpio_port_num(const char *);
uint16_t map_pin_name_to_pin_number(const char *);

