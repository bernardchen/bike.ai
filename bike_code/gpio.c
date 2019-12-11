#include "gpio.h"

gpio_register_t * regs = 0x50000504;

// Inputs:
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir) {
  regs->PIN_CNF[gpio_num] = dir;
}

// Set gpio_num high
// Inputs:
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num) {
  regs->OUT |= (1 << gpio_num);
}

// Set gpio_num low
// Inputs:
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num) {
  regs->OUT &= ~(1 << gpio_num);
}

// Inputs:
//  gpio_num - gpio number 0-31
bool gpio_read(uint8_t gpio_num) {
  return (bool)((regs->IN >> gpio_num) & 1);
}