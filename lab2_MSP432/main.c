/*****************************************************************************
MSP432 main.c

Rock Boynton
12/07/2020
EE4930 Lab 1

// TODO description
 *
 *

*********   Bourns 3352T-1-103LF-10K potentiometer reference   ***************
Signal (Bourns 3352T)  LaunchPad pin
GND    (CCW,   pin 1)  ground
Wiper  (Wiper, pin 2)  power
VCC    (CW,    pin 3)  connected to P4.7

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

#include <stdint.h>
#include <stdbool.h>
#include "msp.h"
#include "msoe_lib_clk.h"
#include "msoe_lib_lcd.h"

	/**
 * @brief Initialize GPIOs
 *
 * Set unused pins to pullup/down enabled to avoid floating inputs,
 * initialize port P1.1 GPIO IN for pushbutton, and
 * initialize port 1.0 GPIO OUT for led
 *
 */
void init_gpio(void);

/**
 * @brief Initialize analog-to-digital converter (ADC)
 *
 */
void init_adc(void);

/**
 * @brief Initialize pulse-width modulation (PWM)
 *
 */
void init_pwm(void);

/**
 * @brief Initialize the LCD
 *
 */
void init_lcd(void);

/**
 * @brief Clears the specified row
 *
 * @param row - the index of the row to clear
 */
void LCD_clear_row(uint8_t row);

// Global variables
bool pushbutton_is_pressed = false;
uint8_t adc_reading = 0;
uint8_t pwm_duty_cycle = 0;

int main(void)
{
	WDT_A->CTL = WDTCTL = WDTPW | WDTHOLD;; // stop watchdog timer
	Clock_Init_48MHz();							// run system at 48MHz (default is 3MHz)

	// setup
	// TODO enable global interrupts
	init_gpio();
	init_adc();
	init_pwm();
	init_lcd();

	LCD_print_str("ADC reading:");
	LCD_goto_xy(2, 0);
	LCD_print_str("Duty Cycle: ");
	while (1)
	{
		if (pushbutton_is_pressed)
		{
			LCD_clear_row(1);
			LCD_goto_xy(1, 2);
			LCD_print_dec3(adc_reading);

			LCD_clear_row(3);
			LCD_goto_xy(3, 2);
			LCD_print_dec3(pwm_duty_cycle);
			LCD_goto_xy(3, pwm_duty_cycle < 10 ? 3 : 4);
			LCD_print_str("%");
		}
	}
}

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

	// P1.1 is pushbutton S1
	P1->DIR &= ~BIT1; // make input
	P1->OUT |= BIT1;  // set as pull up
	// TODO enable pushbutton interrupt

	// TODO setup ADC input

	// TODO setup a timer

	// TODO init PWM output
}

void init_adc(void)
{
	// TODO
}

void init_lcd(void)
{
	LCD_Config();
	LCD_clear();
	LCD_home();
	LCD_contrast(10);

	LCD_print_str("EE4930");
}

void LCD_clear_row(uint8_t row)
{
	LCD_goto_xy(row, 0);
	LCD_print_str("            ");
}
