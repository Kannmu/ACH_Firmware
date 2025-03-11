#pragma once

# define ArraySize 8
# define TransducerGap (16.602 * (1e-3))

# define NumTransducer 64U

# define SoundSpeed 343.2

# define Transducer_Base_Freq 40000UL

# define Wave_K ((2*pi*Transducer_Base_Freq)/SoundSpeed)

# define TimeGapPerDMABufferBit ((long double)(1.0/(Transducer_Base_Freq*DMA_Buffer_Resolution)))

# define BufferGapPerMicroseconds ((float)(1e-6)/TimeGapPerDMABufferBit)

# define TransducerPeriod  ((long double)(1.0 / Transducer_Base_Freq))

# define WaveLength (TransducerPeriod*SoundSpeed)

#include "main.h"
#include "simulation.h"


// transducer.h
extern const char *transducer_pins[];


extern struct Transducer *TransducerArray[NumTransducer];


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


void Transducer_Init(void);
void Transducer_UpdateAllPhase(Point);

void FullFire(Transducer *[]);
void SingleFire(Transducer *, uint8_t);
GPIO_TypeDef *map_pin_name_to_gpio_port(const char *);
uint8_t map_pin_name_to_gpio_port_num(const char *);
uint16_t map_pin_name_to_pin_number(const char *);

