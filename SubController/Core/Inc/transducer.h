#pragma once
#include "main.h"
#include "simulation.h"

# define ArraySize 8
# define TransducerGap (16.602 * (1e-3))

# define NumTransducer 64U

# define SoundSpeed 343.2

# define Transducer_Base_Freq 40000UL

# define TransducerPeriod  ((long double)(1.0 / Transducer_Base_Freq))

# define WaveLength (TransducerPeriod*SoundSpeed)


typedef struct Point Point;

// Transducer Class
typedef struct Transducer
{
    uint8_t Index;
    uint8_t row;
    uint8_t column;
    float position3D[3];

    GPIO_TypeDef *port;
    uint8_t port_num;
    uint16_t pin;
    uint16_t calib;

    float distance;
    float phase;
    uint16_t shift_buffer_bits;

    float duty;
} Transducer;

typedef enum ShootMode
{
    Raw=0,Calib=1
}ShootMode;

extern const char *TransducerPins[];
extern Transducer *TransducerArray[NumTransducer];

extern float Wave_K;

void Transducer_Init(void);
void UpdateFocusPoint(Point *P);

GPIO_TypeDef *map_pin_name_to_gpio_port(const char *);
uint8_t map_pin_name_to_gpio_port_num(const char *);
uint16_t map_pin_name_to_pin_number(const char *);

