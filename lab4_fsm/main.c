/*****************************************************************************
MSP432 main.c

Rock Boynton
01/21/2020
EE4930 Lab 4

Description:

*********   Bourns 3352T-1-103LF-10K potentiometer reference   ***************
Signal (Bourns 3352T)  LaunchPad pin
GND    (CCW,   pin 1)  ground
Wiper  (Wiper, pin 2)  connected to P4.7
VCC    (CW,    pin 3)  power

*********   Nokia LCD interface reference   **********************************

Red SparkFun Nokia 5110 (LCD-10168)
-----------------------------------
Signal        (Nokia 5110) LaunchPad pin
3.3V          (VCC, pin 1) power
Ground        (GND, pin 2) ground
UCA3STE       (SCE, pin 3) connected to P9.4
Reset         (RST, pin 4) connected to P9.3
Data/Command  (D/C, pin 5) connected to P9.2
UCA3SIMO      (DN,  pin 6) connected to P9.7
UCA3CLK       (SCLK, pin 7) connected to P9.5
back light    (LED, pin 8) not connected
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "msp.h"

#include "msoe_lib_clk.h"
#include "msoe_lib_lcd.h"

#include "dehumidifier_fsm.h"

// Defines
#define INT_ADC14_BIT (1 << 24)
#define PWM_PER 46903 // TOP value for timer at 4 Hz
#define ADC_RANGE 256 // range of a 8-bit adc
#define TEN_HZ_PSC 18750

// global inputs
int room_temperature;
int humidity;
int humidity_setpoint;
bool ice_sensed;

// global outputs
bool fan_on;
bool compressor_on;

/**
 * @brief Initialize GPIOs
 *
 * Set unused pins to pullup/down enabled to avoid floating inputs,
 * initialize port P5.0 GPIO OUT to determine program runtime
 *
 */
void init_gpio(void)
{
    // set unused pins to pullup/down enabled to avoid floating inputs
    P1->REN |= 0xFF;
    P2->REN |= 0xFF;
    P3->REN |= 0xFF;
    P4->REN |= 0xFF;
    P5->REN |= 0xFF;
    P6->REN |= 0xFF;
    P7->REN |= 0xFF;
    P8->REN |= 0xFF;
    P9->REN |= 0xFF;
    P10->REN |= 0xFF;

    // pin P5.0
    P5->DIR |= BIT0;   // configure as output
    P5->OUT &= ~BIT0;  // set output low to start
}


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer



	State current_state = OFF; // start in OFF state
	Event input_event;

    while (1)
	{
		// read inputs
	    if (!ice_sensed && humidity >= humidity_setpoint + 5) {
            input_event = HUMIDITY_RISE;
        } else if (!ice_sensed && humidity >= humidity_setpoint + 5 && !ice_sensed) {
            input_event = HUMIDITY_FALL;
        } else if (ice_sensed) {
            input_event = ICE_SENSED;
        } else if (!ice_sensed) {
            input_event = IDLE;
        }

		current_state = update_state(current_state, input_event);
        //__sleep(); // ? add delay ?
	}
}
