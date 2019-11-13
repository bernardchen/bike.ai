

#include "ultrasonic_ranger.h"


ultrasonic_state_t ranger_state = SWITCH_TO_OUTPUT;
		// state of the ultrasonic ranger
long range = 0; // represents the calculated range
uint32_t duration = 0; // used to measure the length input is high
uint32_t timer_val = 0; // used to measure time for
						// output timing and input timeout

void init_ultrasonic_ranger(buckler_port_t ranger_port)
{
	// A1/D1 is not used by this sensor
	switch (ranger_port)
	{
		case A:
		{
			nrf_gpio_cfg_input(BUCKLER_GROVE_A0,NRF_GPIO_PIN_NOPULL);
			break;
		}
		case D:
		{
			nrf_gpio_cfg_input(BUCKLER_GROVE_D0,NRF_GPIO_PIN_NOPULL);
			break;
		}
	}
}

long ultrasonic_ranger_loop_call()
{
	// get timer value


	switch (ranger_state)
	{
		case SWITCH_TO_OUTPUT:
		{
			printf("SWITCH_TO_OUTPUT\n");
			ranger_state = FIRST_LOW;
			break;
		}

		case FIRST_LOW:
		{
			printf("FIRST_LOW\n");
			ranger_state = SEND_OUT_SIGNAL;
			break;
		}

		case SEND_OUT_SIGNAL:
		{
			printf("SEND_OUT_SIGNAL\n");
			ranger_state = SET_LOW_AND_SWITCH_TO_INPUT;
			break;
		}

		case SET_LOW_AND_SWITCH_TO_INPUT:
		{
			printf("SET_LOW_AND_SWITCH_TO_INPUT\n");
			ranger_state = WAIT_FOR_PREV_END;
			break;
		}

		case WAIT_FOR_PREV_END:
		{
			printf("WAIT_FOR_PREV_END\n");
			ranger_state = WAIT_FOR_START;
			break;
		}

		case WAIT_FOR_START:
		{
			printf("WAIT_FOR_START\n");
			ranger_state = WAIT_FOR_END;
			break;
		}

		case WAIT_FOR_END:
		{
			printf("WAIT_FOR_END\n");
			ranger_state = CALCULATE_DISTANCE;
			break;
		}

		case CALCULATE_DISTANCE:
		{
			printf("CALCULATE_DISTANCE\n");
			ranger_state = SWITCH_TO_OUTPUT;
			range += 1;
			break;
		}
	}

	return range;
}