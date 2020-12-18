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
#include <stdio.h>
#include "msp.h"
#include "msoe_lib_clk.h"
#include "msoe_lib_lcd.h"
#include "msoe_lib_delay.h"

// Defines
#define INT_ADC14_BIT (1 << 24)
#define PWM_PER 46903 // TOP value for timer at 4 Hz
#define ADC_RANGE 256 // range of a 8-bit adc
#define TEN_HZ_PSC (uint16_t) 18750

// Global variables
bool pushbutton_is_pressed = false;
uint16_t adc_reading = 0;
uint8_t pwm_duty_cycle = 0;

/**
 * @brief Initialize GPIOs
 *
 * Set unused pins to pullup/down enabled to avoid floating inputs,
 * initialize port P1.1 GPIO IN for pushbutton, and
 * initialize port 1.0 GPIO OUT for led
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

	// P1.1 is pushbutton S1
	P1->DIR &= ~BIT1; // make input
	P1->OUT |= BIT1;  // set as pull up
	P1->IE |= BIT1; // enable interrupt
	P1->IES |= BIT0; // falling edge
	NVIC->ISER[1] |= BIT3; // enable interrupt in NVIC

	// TODO setup ADC input

	// TODO setup a timer

	// TODO init PWM output
	// set pin 2.5 to TimA0
	P2->SEL0 |= BIT5;
	P2->SEL1 &= ~BIT5;
	P2->DIR |= BIT5; // set pin 2.5 to output mode

	P4->SEL1 |= BIT7;  // give A/D control of pin - A6
	P4->SEL0 |= BIT7;
	P4->DIR |= BIT0;  // P4.0 will be toggled by 'hand' in TA0 interrupt
	P4->OUT &= ~BIT0;

	// TODO remove
	// P1.0 is LED1
	P1->DIR |=  BIT0; // make output
	P1->OUT &= ~BIT0; // set as pull down
}

/**
 * @brief Initialize analog-to-digital converter (ADC)
 *
 */
void init_adc(void)
{
	// Sampling time, S&H=96, ADC14 on, SMCLK, single input, single conv.
	ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_4 | ADC14_CTL0_ON;
	ADC14->CTL1 &= ~(ADC14_CTL1_RES_2 | ADC14_CTL1_RES_1); // 8-bit conversion
	ADC14->CTL1 |= (4 << ADC14_CTL1_CSTARTADD_OFS); // use MEM[4]
	ADC14->MCTL[4] |= ADC14_MCTLN_INCH_6;			// input on A6
	ADC14->IER0 |= ADC14_IER0_IE4;					// enable interrupt
	ADC14->CTL0 |= ADC14_CTL0_ENC;

	NVIC->ISER[0] |= INT_ADC14_BIT; // enable ADC interrupt in NVIC
}

/**
 * @brief Initialize pulse-width modulation (PWM)
 *
 */
void init_pwm(void)
{
	// use SMCLK, UP mode, interrupt enabled, prescale 8 x 8 = 64
	TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP |
					 TIMER_A_CTL_CLR | TIMER_A_CTL_IE | TIMER_A_CTL_ID__8;
	TIMER_A0->CCTL[2] |= TIMER_A_CCTLN_OUTMOD_7; // Reset/set output mode
	TIMER_A0->EX0 |= TIMER_A_EX0_IDEX__8;	 // factor of 8
	P2->SEL0 |= BIT5;						 // give timer control
	P2->DIR |= BIT5;						 // make output
	NVIC->ISER[0] |= BIT9;				 // enable TA0_N interrupt
	P4->DIR |= BIT0;					 // P4.0 will be toggled by 'hand' in TA0 interrupt
	P4->OUT &= ~BIT0;
}

/**
 * @brief initialize timer for periodic interrupts
 *
 * Triggers the ADC to start a conversion
 */
void init_periodic_timer(void)
{
    // Configure TimA1 to SMCLOCK (12 MHz using 48MHz system clock), divide by 8,  UP mode, Interrupt Enable
    TIMER_A1->CTL |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__8 | TIMER_A_CTL_MC__UP
                     | TIMER_A_CTL_IE | TIMER_A_CTL_CLR;
    TIMER_A1->EX0 |= TIMER_A_EX0_IDEX__8; // divide by 8 again, giving a timer freq of 187.5KHz
    TIMER_A1->CCR[0] = TEN_HZ_PSC;  // set frequency as 10 Hz
    NVIC->ISER[0] |= BITB;  // enable TA1_N interrupt
}

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
 * @brief Clears the specified row
 *
 * @param row - the index of the row to clear
 */
void LCD_clear_row(uint8_t row)
{
	LCD_goto_xy(0, row);
	LCD_print_str("            ");
}

/**
 * @brief Pushbutton interrupt handler
 *
 */
void PORT1_IRQHandler(void)
{
	uint16_t dummy = P1->IV; // clear flag
	pushbutton_is_pressed = true;
}

/**
 * @brief PWM timer interrupt handler
 */
void TA0_N_IRQHandler(void)
{
    uint16_t dummy = TIMER_A0->IV; // clear flag
}

/**
 * @brief Periodic timer interrupt
 *
 * Triggers ADC conversion
 *
 */
void TA1_N_IRQHandler(void)
{
	uint16_t dummy = TIMER_A1->IV; // clear flag
	ADC14->CTL0 |= ADC14_CTL0_SC; // start a new ADC conversion
    P1->OUT ^= BIT0; // led on
}

/**
 * @brief ADC interrupt handler
 *
 * Reads the value and updates the duty cycle
 *
 */
void ADC14_IRQHandler(void)
{
	adc_reading = ADC14->MEM[4];
	pwm_duty_cycle = 100 - (uint8_t) ((((float) adc_reading) / ADC_RANGE) / 100); // multiply first to avoid int math error
	TIMER_A0->CCR[2] = (PWM_PER * pwm_duty_cycle) / 100; // invert duty cycle for output
}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;; // stop watchdog timer
	Clock_Init_48MHz();	// run system at 48MHz (default is 3MHz)

	// setup
	init_gpio();
	init_adc();
	init_pwm();
	init_periodic_timer();
	init_lcd();

	__enable_interrupts(); // global interrupt enables

	LCD_print_str("ADC reading:");

	LCD_goto_xy(0, 2);
	LCD_print_str("Duty Cycle: ");

	uint8_t count = 0;
	while (1)
	{
		if (pushbutton_is_pressed)
		{
		    LCD_clear();
		    LCD_home();
		    LCD_print_str("ADC reading:");
		    LCD_goto_xy(0, 1);
		    LCD_print_dec5(adc_reading);
		    LCD_goto_xy(0, 2);
		    LCD_print_str("Duty Cycle: ");
		    LCD_print_dec5(pwm_duty_cycle);
		    LCD_goto_xy(pwm_duty_cycle < 10 ? 3 : 4, 3);
		    LCD_print_str("%");
		    pushbutton_is_pressed = false;

//		    //P1->OUT |= BIT0; // led on
////		    printf("TimerA0: %d\n", TIMER_A0->R);
////		    printf("TimerA1: %d\n", TIMER_A1->R);
//
//
//
//		    //__disable_interrupts(); // entering critical section
//
//			LCD_clear_row(1);
//			LCD_goto_xy(0, 1);
//			LCD_print_dec3(adc_reading);
//			// LCD_print_dec3(count); // TODO remove
//
//			LCD_clear_row(3);
//			LCD_goto_xy(0, 3);
//
//			LCD_print_dec3(pwm_duty_cycle);
//			// LCD_print_dec3(count); // TODO remove
//			LCD_goto_xy(pwm_duty_cycle < 10 ? 3 : 4, 3);
//			LCD_print_str("%");
//
//			count++; // TODO remove
//
//			pushbutton_is_pressed = false;
//
//			//__enable_interrupts(); // leaving critical section
//            //P1->OUT &= ~BIT0; // turn led off
		}
	}
}
