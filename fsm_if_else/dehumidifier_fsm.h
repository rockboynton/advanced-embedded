// dehumidifier_fsm.h
#ifndef DEHUMIDIFIER_FSM_H_
#define DEHUMIDIFIER_FSM_H_

#include "msp.h"

typedef enum
{
	OFF,
	NORMAL_OPERATION,
	DEFROST,
    NUM_STATES
} State;

typedef enum
{
	HUMIDITY_RISE,
	HUMIDITY_FALL,
    ICE_SENSED,
    IDLE,
    NUM_EVENTS
} Event;

typedef void (*function_ptr)(void);

typedef struct {
    State next_state;
    function_ptr action;
} state_element;

// function prototypes for state actions

/**
 * @brief Turn the fan and compressor on
 *
 */
void operate_normally(void);

/**
 * @brief Turn the fan and compressor off
 *
 */
void turn_off(void);

/**
 * @brief Turn the fan on and the compressor off
 *
 */
void defrost_coils(void);

/**
 * @brief Wraps the `__no_operation` for use in the state table
 *
 */
void no_op(void);

/**
 * @brief Run the appropriate state's action and return the next state
 *
 * @param current_state - current state of the system with action to run
 * @param input - current input event
 * @return State - next state
 */
State update_state(State current_state, Event input);

#endif // DEHUMIDIFIER_FSM_H_
