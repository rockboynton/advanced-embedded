/*****************************************************************************
MSP432 main.c

Rock Boynton
01/28/2020
EE4930 Lab 5 - Low Power

Description:
    // TODO

*********   Analog Devices TMP36 temperature sensor   ***********************
Signal    (TMP36)      LaunchPad pin
+Vs       (pin 1)      power
Vout      (pin 2)      connected to P4.7
GND       (pin 3)      ground

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
#include "msoe_lib_delay.h"

// Defines
#define TWELVE_BIT_ADC_RANGE 4096
#define TEN_HZ_PSC 18750

typedef struct Input
{
    float val;
    const uint8_t lcd_x;
    const uint8_t lcd_y;
    bool changed;
} Input;

Input temp = {0, 8, 0, false};

/**
 * @brief Initialize the LCD
 *
 * Sets it up to display temp
 *
 */
void init_lcd(void)
{
	LCD_Config();
	LCD_clear();
	LCD_home();
	LCD_contrast(10);

    LCD_print_str("Temp:");
}

/**
 * @brief Initialize input pins
 *
 * Temp Sensor - // TODO
 *
 */
void init_inputs(void)
{
   // TODO
}

/**
 * @brief Initialize GPIOs
 *
 * Set unused pins to pullup/down enabled to avoid floating inputs
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

    init_inputs();
}

void update_display(Input *input)
{
    static char val_str[16];
    if (input->changed)
    {
        LCD_goto_xy(input->lcd_x, input->lcd_y);
        sprintf(val_str, "%.1f", input->val);
        LCD_print_str(val_str);
        input->changed = false;
    }
}

/**
 * @brief Initialize analog-to-digital converters (ADC)
 *
 * Initializes the temperature (P4.7)
 */
void init_adc(void)
{
    // Sampling time, S&H=96, ADC14 on, SMCLK, single input, single conv.
	ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_4 | ADC14_CTL0_ON;
	ADC14->CTL1 &= ~(ADC14_CTL1_RES_2 | ADC14_CTL1_RES_1); // clear res bits
    ADC14->CTL1 |= ADC14_CTL1_RES__12BIT; // set 12 bit resolution
    ADC14->CTL1 |= (4 << ADC14_CTL1_CSTARTADD_OFS); // use MEM[4]
	ADC14->MCTL[4] |= ADC14_MCTLN_INCH_6;			// input on A6
	ADC14->IER0 |= ADC14_IER0_IE4;					// enable interrupt
	ADC14->CTL0 |= ADC14_CTL0_ENC;
	NVIC->ISER[0] |= ADC14_IER0_IE24; // enable ADC interrupt in NVIC

    P4->SEL1 |= BIT7;  // give A/D control of pin - A6
    P4->SEL0 |= BIT7;
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
 * @brief ADC interrupt handler
 *
 * Reads the value and updates the temperature
 *
 */
void ADC14_IRQHandler(void)
{
    uint32_t adc_reading = ADC14->MEM[4];
    printf("reading (0-4096): %d\t", adc_reading);
    uint32_t adc_voltage = adc_reading * (3300.0 / TWELVE_BIT_ADC_RANGE);
    printf("voltage (v): %d\t", adc_voltage);
    float temp_c = (adc_voltage - 500) / 10.0; // 10mV/C and 500mV offset
    printf("temp (c): %f\t", temp_c);
    temp.val = temp_c * 9.0 / 5.0 + 32;
    printf("temp (f): %f\n", temp.val);
    temp.changed = true;
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
}

void main(void)

{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
    // Clock_Init_48MHz(); // run system at 48MHz (default is 3MHz)

    // setup
	init_gpio();
	init_adc();
    init_periodic_timer();
    init_lcd();

    __enable_interrupts(); // global interrupt enable

    while (1)
	{
        __disable_interrupts(); // entering critical section

        update_display(&temp);

        __enable_interrupts(); // leaving critical section
	}
}
