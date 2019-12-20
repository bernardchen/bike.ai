// Wrapper for the Grove Ultrasonic Ranger
//
// Uses a state machine to step through the process 
// presented by Grove to read the distance.
// Example codes can be seen on the official website:
// http://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/
#pragma once

#include <stdio.h>
#include <math.h>

#include "nrf.h"
#include "nrfx_gpiote.h"

#include "buckler.h"


// The different states for the ultrasonic ranger when measuring.
// Not that relevant for the uesr.
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
// for the ultrasonic ranger.
// Set to UNUSED in init_ultrasonic_ranger if the corresponding ranger is not being used.
	// Always will use GroveD0 or GroveA0, not 1.
typedef enum {S
	A,
	D,
	UNUSED
} buckler_port_t;

// Call the function to initialize the ultrasonic ranger.
// Input:
// left_port is the port for the left ultrasonic ranger
// right_port is the port for the right ultrasonic ranger
	// use the UNUSED enum if the ranger is not b eing used
// initLEDs is whether or not to init the hardcoded LEDs too
	// 0 is don't init, any other value is
void init_ultrasonic_ranger(buckler_port_t left_port, buckler_port_t right_port, uint32_t initLEDs);

// These functions will return the distance measured by the ultrasonic ranger.
// It will go through the entire process of switching it to output, and
// then switching to input, and do all the calculations.
// Will take 12 millisecond max to run per side. Timeout set to that to avoid taking too long to poll
// This means in standard polling of both sides, the delay will be about 30 milliseconds (about 15 per side).
// Distance can be measured up to a max of 200cm.
// Any range greater than that will be considered 600 (timeout)
// It may also be 600 if for some reason, there is a timeout
// when trying to obtain the distance.
long ultrasonic_get_left_distance_cm();
long ultrasonic_get_right_distance_cm();


/************ PROXIMITY WARNING LED WRAPPERS ************/
// functions to turn on and off the leds for right side
// (wrapper used to hard code the LEDs used in ultrasonic_ranger.c.
// To change which GPIO port to use, change the macro at the bottom of ultrasonic_ranger.c
void turn_on_right_proxi_led();
void turn_off_right_proxi_led();

// functions to turn on and off the leds for left side
// (wrapper used to hard code the LEDs used in ultrasonic_ranger.c.
// To change which GPIO port to use, change the macro at the bottom of ultrasonic_ranger.c
void turn_on_left_proxi_led();
void turn_off_left_proxi_led();
