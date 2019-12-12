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
	D
} buckler_port_t;

void init_ultrasonic_ranger(buckler_port_t port, uint32_t initLEDs);

// This function will return the distance measured by the ultrasonic ranger.
// It will go through the entire process of switching it to output, and
// then switching to input, and do all the calculations.
// May take about a millisecond or two to fully run.
// It may also be 600 if for some reason, there is a timeout
// when trying to obtain the distance.
long ultrasonic_ranger_get_distance_cm();


/************ PROXIMITY WARNING LED WRAPPERS ************/
// functions to turn on and off the leds for right side
// (wrapper used to hard code the LEDs used in ultrasonic_ranger.c)
void turn_on_right_proxi_led();
void turn_off_right_proxi_led();

// functions to turn on and off the leds for left side
// (wrapper used to hard code the LEDs used in ultrasonic_ranger.c)
void turn_on_left_proxi_led();
void turn_off_left_proxi_led();
