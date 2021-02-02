// dehumidifier_fsm.c
#include "dehumidifier_fsm.h"

/**
 * @brief Turn the fan and compressor on
 *
 */
void operate_normally(void)
{
    P5->OUT |= BIT0;
    P5->OUT |= BIT2;
}

/**
 * @brief Turn the fan and compressor off
 *
 */
void turn_off(void)
{
    P5->OUT &= ~BIT0;
    P5->OUT &= ~BIT2;
}

/**
 * @brief Turn the fan on and the compressor off
 *
 */
void defrost_coils(void)
{
    P5->OUT |= BIT0;
    P5->OUT &= ~BIT2;
}


void no_op(void)
{
    __no_operation();
}

State update_state(State current_state, Event input)
{
    State next_state;

    if (current_state == OFF)
    {
        if (input == HUMIDITY_RISE)
        {
            next_state = NORMAL_OPERATION;
            operate_normally();
        }
        else if (input == HUMIDITY_FALL)
        {
            next_state = OFF;
            no_op();
        }
        else if (input == ICE_SENSED)
        {
            next_state = DEFROST;
            defrost_coils();
        }
        else if (input == IDLE)
        {
            next_state = OFF;
            no_op();
        }
    }
    else if (current_state == NORMAL_OPERATION)
    {
        if (input == HUMIDITY_RISE)
        {
            next_state = NORMAL_OPERATION;
            no_op();
        }
        else if (input == HUMIDITY_FALL)
        {
            next_state = OFF;
            turn_off();
        }
        else if (input == ICE_SENSED)
        {
            next_state = DEFROST;
            defrost_coils();
        }
        else if (input == IDLE)
        {
            next_state = NORMAL_OPERATION;
            no_op();
        }
    }
    else if (current_state == DEFROST)
    {
        if (input == HUMIDITY_RISE)
        {
            next_state = NORMAL_OPERATION;
            operate_normally();
        }
        else if (input == HUMIDITY_FALL)
        {
            next_state = OFF;
            turn_off();
        }
        else if (input == ICE_SENSED)
        {
            next_state = DEFROST;
            no_op();
        }
        else if (input == IDLE)
        {
            next_state = OFF;
            turn_off();
        }
    }

    // return next state info
    return next_state;
}
