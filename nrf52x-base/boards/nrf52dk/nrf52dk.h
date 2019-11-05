// Pin definitions for nRF52 development kit (PCA10040)

#pragma once

#include "nrf_gpio.h"

#define NRF52DK_LED1 NRF_GPIO_PIN_MAP(0,17)
#define NRF52DK_LED2 NRF_GPIO_PIN_MAP(0,18)
#define NRF52DK_LED3 NRF_GPIO_PIN_MAP(0,19)
#define NRF52DK_LED4 NRF_GPIO_PIN_MAP(0,20)

#define NRF52DK_BUTTON1 NRF_GPIO_PIN_MAP(0,13)
#define NRF52DK_BUTTON2 NRF_GPIO_PIN_MAP(0,14)
#define NRF52DK_BUTTON3 NRF_GPIO_PIN_MAP(0,15)
#define NRF52DK_BUTTON4 NRF_GPIO_PIN_MAP(0,16)

#define NRF52DK_UART_TXD NRF_GPIO_PIN_MAP(0,6)
#define NRF52DK_UART_RXD NRF_GPIO_PIN_MAP(0,8)

