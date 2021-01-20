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
#include <stdint.h>
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

typedef struct Input
{
    uint8_t val;
    const uint8_t lcd_x;
    const uint8_t lcd_y;
    bool changed;
} Input;

Input temp = {0, 6, 0, false};
Input humidity = {0, 10, 1, false};
Input humidity_setpoint = {0, 10, 2, false};
Input ice_sensed = {0, 9, 3, false};

// global inputs
// uint8_t temperature;
// uint8_t humidity;
// uint8_t humidity_setpoint;
// bool ice_sensed;

// global outputs
bool fan_on;
bool compressor_on;

// global flags
// bool humidity_changed;
// bool temperature_changed;
// bool setpoint_changed;
// bool defrost_changed;

/**
 * @brief Initialize the LCD
 *
 */
void init_lcd(void)
{
	LCD_Config();
	LCD_clear();
	LCD_home();
	LCD_contrast(10);
}

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

void update_display(Input *input)
{
    if (input->changed)
    {
        LCD_goto_xy(input->lcd_x, input->lcd_y);
        LCD_print_udec3(input->val);

        input->changed = false;
    }
}


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
    Clock_Init_48MHz(); // run system at 48MHz (default is 3MHz)

    // setup
	init_gpio();
//	init_adc();
//	init_pwm();
//	init_periodic_timer();
	init_lcd();

	__enable_interrupts(); // global interrupt enable

    LCD_print_str("Temp:");

	LCD_goto_xy(0, 1);
	LCD_print_str("Setpt: ");

	LCD_goto_xy(0, 2);
	LCD_print_str("Humidity:");

    LCD_goto_xy(0, 3);
	LCD_print_str("Defrost:");

	State current_state = OFF; // start in OFF state
	Event input_event;

    while (1)
	{
        __disable_interrupts(); // entering critical section
        update_display(&temp);
        update_display(&humidity);
        update_display(&humidity_setpoint);
        update_display(&ice_sensed);
        __enable_interrupts(); // leaving critical section

		// read inputs
	    if (!ice_sensed.val && humidity.val >= humidity_setpoint.val + 5) {
            input_event = HUMIDITY_RISE;
        } else if (!ice_sensed.val && humidity.val >= humidity_setpoint.val + 5) {
            input_event = HUMIDITY_FALL;
        } else if (ice_sensed.val) {
            input_event = ICE_SENSED;
        } else if (!ice_sensed.val) {
            input_event = IDLE;
        }

		current_state = update_state(current_state, input_event);
        //__sleep(); // ? add delay ?
	}
}
