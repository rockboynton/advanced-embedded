// dehumidifier_fsm.c
#include "dehumidifier_fsm.h"

/**
 * @brief Turn the fan and compressor on
 *
 */
void operate_normally(void)
{

}

/**
 * @brief Turn the fan and compressor off
 *
 */
void turn_off(void)
{

}

/**
 * @brief Turn the fan on and the compressor off
 *
 */
void defrost_coils(void)
{

}


void no_op(void)
{
    __no_operation();
}

// create state table array
state_element state_table[NUM_STATES][NUM_EVENTS] = {
    {{NORMAL_OPERATION, operate_normally}, {OFF, no_op}, {DEFROST, defrost_coils}, {OFF, no_op}},
    {{NORMAL_OPERATION, no_op}, {OFF, turn_off}, {DEFROST, defrost_coils}, {NORMAL_OPERATION, no_op}},
    {{DEFROST, no_op}, {DEFROST, no_op}, {DEFROST, no_op}, {NORMAL_OPERATION, operate_normally}}
};

State update_state(State current_state, Event input)
{
    state_element current = state_table[current_state][input];

    // run the proper action function
    (*current.action)();

    // return next state info
    return current.next_state;
}
