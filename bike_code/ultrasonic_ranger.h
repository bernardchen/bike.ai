// Wrapper for the Grove Ultrasonic Ranger
//
// Uses a state machine to avoid blocking that would
// occur in a while loop
#pragma once

#include <stdio.h>
#include <math.h>

#include "nrf.h"
#include "nrfx_gpiote.h"

#include "buckler.h"


// the different states for the ultrasonic ranger
typedef enum {
	SWITCH_TO_OUTPUT,
	FIRST_LOW,
	SEND_OUT_SIGNAL,
	// SET_LOW_AND_SWITCH_TO_INPUT, // no longer need because do this on transition
	WAIT_FOR_PREV_END,
	WAIT_FOR_START,
	WAIT_FOR_END,
	CALCULATE_DISTANCE,
	ERROR
} ultrasonic_state_t;


// enum to determine which Buckler port is used
// for the ultrasonic ranger
typedef enum {
	A,
	D,
	UNUSED
} buckler_port_t;

void init_ultrasonic_ranger(buckler_port_t left_port, buckler_port_t right_port, uint32_t initLEDs);

// These functions will return the distance measured by the ultrasonic ranger.
// It will go through the entire process of switching it to output, and
// then switching to input, and do all the calculations.
// Will take 12 millisecond max to run per side. Timeout set to that to avoid taking too long to poll
// This means in standard polling of both sides, the delay will be about 30 seconds.
// Distance can be measured up to a max of 200cm.
// Any range greater than that will be considered 600 (timeout)
// It may also be 600 if for some reason, there is a timeout
// when trying to obtain the distance.
long ultrasonic_get_left_distance_cm();
long ultrasonic_get_right_distance_cm();


/************ PROXIMITY WARNING LED WRAPPERS ************/
// functions to turn on and off the leds for right side
// (wrapper used to hard code the LEDs used in ultrasonic_ranger.c)
void turn_on_right_proxi_led();
void turn_off_right_proxi_led();

// functions to turn on and off the leds for left side
// (wrapper used to hard code the LEDs used in ultrasonic_ranger.c)
void turn_on_left_proxi_led();
void turn_off_left_proxi_led();
