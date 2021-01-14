/*
 * template_fsm_header.h
 *
 *  Created on: Mar 26, 2017
 *      Author: widder
 *      blah, blah, blah
 */

#ifndef DEHUMIDIFIER_FSM_H_
#define DEHUMIDIFIER_FSM_H_

typedef enum
{
	S0,
	S1,
	NUM_STATES
} State;

typedef enum
{
	IN0,
	IN1,
    NUM_EVENTS
} Event;

typedef void (*function_ptr)(void);

typedef struct {
    State next_state;
    function_ptr action;
} state_element;

// create state table array
state_element state_table[NUM_STATES][NUM_EVENTS] = {
    {{}}
};

State update_state(State current_state, Event input)
{
    state_element current = state_table[current_state][input];

    // run the proper action function
    (*current.action)();

    // return next state info
    return current.next_state;
}

#endif /* DEHUMIDIFIER_FSM_H_ */
