

#include "app_error.h"
#include "app_timer.h"


#include "ultrasonic_ranger.h"


/************* GLOBAL CONSTS USED BY RANGER *************/
ultrasonic_state_t ranger_state = SWITCH_TO_OUTPUT;
		// state of the ultrasonic ranger
long range = 600; // represents the calculated range
uint32_t duration = 0; // used to measure the length input is high
uint32_t timer_start_rtc = 0; // rtc counter value when timer started
uint32_t timer_val = 0; // used to measure time for
						// output timing and input timeout
// represents which buckler port we're using
buckler_port_t ranger_port = D; // the enum
uint32_t ranger_port_pin_num = BUCKLER_GROVE_D0; // the actual pin number


/************* TIMER STUFF *************/
// always set the timer to 1000 so if it's ever called, something went wrong
// and we want to restart measurement.
// otherwise for normal switching, we should do it before the timer ends
#define RANGER_TIMEOUT (1000) // 1 second

// gets called when timer runs out and doesn't transition states as expected.
// will move to switch_to_output
static void ranger_timeout_handler(void * p_context)
{
	// if timer actually times out, then we reset and go back to output mode
	ranger_state = SWITCH_TO_OUTPUT;
	// also set range to 0
	range = 600;
}

APP_TIMER_DEF(ranger_timer);
// this only needs to be called once
void ranger_timer_init()
{
  ret_code_t err_code;

    // Create timers
  err_code = app_timer_create(&ranger_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                ranger_timeout_handler); // don't do anything to avoid interrupts
  APP_ERROR_CHECK(err_code);
  printf("\n\nUltrasonic Ranger timer created!\n\n");
}

// used to convert tick values to microseconds
uint32_t app_timer_ticks_to_usec(uint32_t ticks)
{
  return ((uint32_t)ROUNDED_DIV(
                  (ticks) * 1000000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1),
                  (uint64_t)APP_TIMER_CLOCK_FREQ));
}

/**** timer wrapper functions ****/
void ranger_timer_start()
{
  ret_code_t error_code = app_timer_start(ranger_timer, APP_TIMER_TICKS(RANGER_TIMEOUT), NULL);
  timer_start_rtc = app_timer_cnt_get();
  APP_ERROR_CHECK(error_code);
}
void ranger_timer_stop()
{
  ret_code_t error_code = app_timer_stop(ranger_timer);
  APP_ERROR_CHECK(error_code);
}
// timer_start_rtc is already set by ranger_timer_start()
uint32_t ranger_get_time_usec()
{
	// in the case of overflow
	uint32_t curr_rtc_val = app_timer_cnt_get();
	if (curr_rtc_val < timer_start_rtc)
	{
		return app_timer_ticks_to_usec((APP_TIMER_MAX_CNT_VAL - timer_start_rtc) + curr_rtc_val);
	}
	else
	{
		return app_timer_ticks_to_usec(curr_rtc_val - timer_start_rtc);
	}
}

/************* RANGER HELPER FUNCS *************/
/**** set input/output ****/
void set_ranger_to_input()
{
	nrf_gpio_cfg_input(ranger_port_pin_num,NRF_GPIO_PIN_NOPULL);			
}
void set_ranger_to_output()
{
	nrf_gpio_cfg_output(ranger_port_pin_num);
}

/**** change outputs ****/
void ranger_disable_output()
{
	nrf_gpio_pin_clear(ranger_port_pin_num);
}
void ranger_enable_output()
{
	nrf_gpio_pin_set(ranger_port_pin_num);
}

// get input value
uint32_t ranger_get_input()
{
	return nrf_gpio_pin_read(ranger_port_pin_num);
}



void init_ultrasonic_ranger(buckler_port_t port)
{
	ranger_port = port;
	switch (ranger_port)
	{
		case A:
		{
			ranger_port_pin_num = BUCKLER_GROVE_A0;
			break;
		}
		case D:
		{
			ranger_port_pin_num = BUCKLER_GROVE_D0;
			break;
		}
	}

	// A1/D1 is not used by this sensor
	set_ranger_to_output();

	ranger_timer_init();
}


// THE loop cool that gets called every loop in main
long ultrasonic_ranger_loop_call()
{
	// get timer value


	switch (ranger_state)
	{
		case SWITCH_TO_OUTPUT:
		{
			set_ranger_to_output();

			// set to low and start timer
			ranger_disable_output();
			ranger_timer_start();

			ranger_state = FIRST_LOW;
			break;
		}

		case FIRST_LOW:
		{
			// wait until timer is 2
			timer_val = ranger_get_time_usec();
			if (timer_val >= 2)
			{
				ranger_timer_stop();

				// set to high and start timer
				ranger_enable_output();
				ranger_timer_start();

				ranger_state = SEND_OUT_SIGNAL;	
			}
			break;
		}

		case SEND_OUT_SIGNAL:
		{
			timer_val = ranger_get_time_usec();
			if (timer_val >= 5)
			{
				// set to low and swich to input
				ranger_timer_stop();
				ranger_disable_output();

				set_ranger_to_input();
				ranger_timer_start();

				ranger_state = WAIT_FOR_PREV_END;
			}
			break;
		}

		case WAIT_FOR_PREV_END:
		{
			// wait unti low
			if (!ranger_get_input())
			{
				ranger_timer_stop();
				ranger_state = WAIT_FOR_START;	
				ranger_timer_start();
			}
			break;
		}

		case WAIT_FOR_START:
		{
			// wait until high
			if (ranger_get_input())
			{
				ranger_timer_stop();
				ranger_state = WAIT_FOR_END;
				// start time is automatically set with ranger_timer_start()
				ranger_timer_start();
			}
			break;
		}

		case WAIT_FOR_END:
		{
			// wait until low
			if (!ranger_get_input())
			{
				duration = ranger_get_time_usec();
				ranger_timer_stop();
				ranger_state = CALCULATE_DISTANCE;
			}
			break;
		}

		case CALCULATE_DISTANCE:
		{
			ranger_state = SWITCH_TO_OUTPUT;
			range = duration / 29 / 2;

			break;
		}
	}

	return range;
}