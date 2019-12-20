// Wrappers for gpio created in EECS149 lab

#pragma once

#include "nrf.h"
#include "stdbool.h"

typedef struct {
uint32_t OUT;
uint32_t OUTSET;
uint32_t OUTCLR;
uint32_t IN;
uint32_t GPIO_DIR;
uint32_t DIRSET;
uint32_t DIRCLR;
uint32_t LATCH;
uint32_t DETECTMODE;
uint32_t BUFFER[118];
uint32_t PIN_CNF[32];
} gpio_register_t;


typedef enum {
    INPUT = 0,
    OUTPUT,
} gpio_direction_t;

// Inputs:
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir);

// Inputs:
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num);

// Inputs:
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num);

// Inputs:
//  gpio_num - gpio number 0-31
// Returns:
//  current state of the specified gpio pin
bool gpio_read(uint8_t gpio_num);