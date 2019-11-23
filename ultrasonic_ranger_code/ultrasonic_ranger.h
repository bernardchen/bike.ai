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
} ultrasonic_state_t;


// enum to determine which Buckler port is used
// for the ultrasonic ranger
typedef enum {
	A,
	D
} buckler_port_t;

void init_ultrasonic_ranger(buckler_port_t port);

// The function to call inside the while loop.
// This needs to be repeatedly called to
// actually process and go through the states
// to obtain the distance from the Grove Ultrasonic Ranger.
// The return value will be either the previous distance
// or a newly obtained distance.
// It may also be 0 if for some reason, there is a timeout
// when trying to obtain the distance.
long ultrasonic_ranger_loop_call();
