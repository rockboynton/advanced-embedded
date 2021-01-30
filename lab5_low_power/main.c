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
#include "msoe_lib_misc.h"

// Defines
#define TWELVE_BIT_ADC_RANGE 4096
#define TEN_HZ_PSC 65000// 18750
#define SHORT 5000

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
 * Output pin for  indicating temp reading is available
 *
 */
void init_outputs(void)
{
    // pin P4.5
    P4->DIR |= BIT5;  // configure as output
    P4->OUT &= ~BIT5; // set output low to start
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

    init_outputs();
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
    P4->SEL1 |= BIT7;  // give A/D control of pin - A6
    P4->SEL0 |= BIT7;

    // Sampling time, S&H=96, ADC14 on, SMCLK, single input, single conv.
	ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_4 | ADC14_CTL0_ON;
	ADC14->CTL1 &= ~(ADC14_CTL1_RES_2 | ADC14_CTL1_RES_1); // clear res bits
    ADC14->CTL1 |= ADC14_CTL1_RES__12BIT; // set 12 bit resolution
    ADC14->CTL1 |= (4 << ADC14_CTL1_CSTARTADD_OFS); // use MEM[4]
	ADC14->MCTL[4] |= ADC14_MCTLN_INCH_6;			// input on A6
	ADC14->IER0 |= ADC14_IER0_IE4;					// enable interrupt
	ADC14->CTL0 |= ADC14_CTL0_ENC;
	NVIC->ISER[0] |= ADC14_IER0_IE24; // enable ADC interrupt in NVIC

}

/**
 * @brief initialize timer for periodic interrupts
 *
 * Triggers the ADC to start a conversion
 */
void init_periodic_timers(void)
{
    // Configure TimA1 to SMCLOCK (12 MHz using 48MHz system clock), divide by 8,  UP mode, Interrupt Enable
    TIMER_A1->CTL |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__8 | TIMER_A_CTL_MC__UP
                     | TIMER_A_CTL_IE | TIMER_A_CTL_CLR;
    TIMER_A1->EX0 |= TIMER_A_EX0_IDEX__8; // divide by 8 again, giving a timer freq of 187.5KHz
    TIMER_A1->CCR[0] = TEN_HZ_PSC;  // set frequency as 10 Hz
    NVIC->ISER[0] |= BITB;  // enable TA1_N interrupt

    // Configure TimA0 to SMCLOCK (12 MHz using 48MHz system clock), divide by 8,  UP mode, Interrupt Enable
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__8 | TIMER_A_CTL_MC__CONTINUOUS | TIMER_A_CTL_IE | TIMER_A_CTL_CLR;
    TIMER_A0->EX0 |= TIMER_A_EX0_IDEX__8; // divide by 8 again, giving a timer freq of 187.5KHz
    TIMER_A0->CCR[0] = 0;             // set frequency as 10 Hz
    NVIC->ISER[0] |= BIT9;                // enable TA1_N interrupt
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
    uint32_t adc_voltage = adc_reading * (3300.0 / TWELVE_BIT_ADC_RANGE);
    float temp_c = (adc_voltage - 500) / 10.0; // 10mV/C and 500mV offset
    temp.val = temp_c * 9.0 / 5.0 + 32;
    // temp.changed = true;
    P4->OUT |= BIT5; // indicate temp reading is available
    TIMER_A0->CCR[0] = SHORT; // start timer for short period indicating temp ready
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

/**
 * @brief Timer for result ready
 */
void TA0_N_IRQHandler(void)
{
    uint16_t dummy = TIMER_A0->IV; // clear flag
    P4->OUT &= ~BIT5; // end indication
    TIMER_A0->CCR[0] = 0;
}

void main(void)

{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
    // Clock_Init_48MHz(); // run system at 48MHz (default is 3MHz)

    // setup
	init_gpio();
	Set_ports_to_out();
	init_adc();
    init_periodic_timers();
    // init_lcd();

    __enable_interrupts(); // global interrupt enable

    // lpm0 mode
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_CPM__LPM0_LF_VCORE0;
//    PCM-> CTL1 = PCM_CTL1_FORCE_LPM_ENTRY
    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // sleep on exit from ISR


    __DSB();

    while (1)
	{
        // __disable_interrupts(); // entering critical section
        // update_display(&temp);
        // __enable_interrupts(); // leaving critical section
        __sleep();
        __no_operation();
    }
}

// ! **********************************************************************************************************

// /**
//  * main.c
//  */

// #include "msp.h"
// #include <stdint.h>
// #include <string.h>
// #include <stdio.h>
// #include "msoe_lib_clk.h"
// #include "msoe_lib_lcd.h"
// #include "msoe_lib_delay.h"

// #define ADC_RANGE 4096 // for 12 bit conversions

// void initPCM();

// void initADC();

// void initWDA();

// void initGPIOOutput();

// //Global Variables
// uint8_t temperature = 0; //temperature reading
// uint8_t flip = 1;        // boolean value for running the WDT_A interrupt every other time ~8min intervals

// void main(void)
// {

//     WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

//     // setup lcd
//     //LCD_Config();
//     //LCD_clear();
//     //LCD_home();
//     //LCD_contrast(10);

//     // init ports
//     initGPIOOutput();
//     initADC();
//     initWDA();
//     initPCM();

//     while (1)
//     {
//         //Empty main loop. Everything is interrupt controlled.
//     }
// }

// void initWDA()
// {
//     //Configure WDT_A
//     WDT_A->CTL = 0x5A5D;       // unlock WDT_A, vloclk src, interval mode, reset count, ~250ms initial timer
//     NVIC->ISER[0] |= (1 << 3); // enable WDT_A interrupt in NVIC
// }

// void initPCM()
// {
//     // setup clock sources
//     CS->KEY = 0x0000695A;     //unlock
//     CS->CLKEN |= 1 << 8;      // turn VLOCLK on;
//     CS->CTL1 &= ~(0b1110111); // clear SM/HSMCLK, and MCLK sources
//     CS->CTL1 |= 0b0010001;    // set SMCLK/HSMCLK, MCLK to VLOCLK
//     CS->KEY = 1;              // lock

//     // Configure for LPM3
//     SCB->SCR |= (1 << 2);   // Set Sleep Deep bit
//     PCM->CTL0 = 0x695A0008; // unlock PCM/PMR, request LMP3, LP_AM_LDO_VCORE0
//     SCB->SCR |= (0b10);     // Set Sleep On Exit
//     PCM->CTL1 = PCM_CTL0_KEY_VAL | PCM_CTL1_FORCE_LPM_ENTRY;
// }

// // Initializes Outputs for signaling a new temperature is completed
// // initializes P2.4 for signaling
// void initGPIOOutput()
// {
//     // P2.4
//     P2->SEL0 &= ~(1 << 4); // use GPIO function
//     P2->SEL1 &= ~(1 << 4);
//     P2->DIR |= (1 << 4);  // make output
//     P2->OUT &= ~(1 << 4); // setup output low to start
// }

// //Initialize ADC6 on pin 4.7
// //Interrupt upon completed conversion
// void initADC()
// {
//     //setup pins 4.7 in analog mode
//     P4->SEL0 |= 0b1 << 7;
//     P4->SEL1 |= 0b1 << 7;
//     // start sampling on SC bit,
//     // source a timer,
//     // use SMCLK,
//     // 96 sample/hold time
//     // turn core on
//     ADC14->CTL0 &= 0x0;
//     ADC14->CTL0 |= (1 << 26) | (1 << 21) | (0b101 << 12) | (0b101 << 8) | (1 << 4);
//     // 12 bit resolution and use memory location 4 to start
//     ADC14->CTL1 &= 0xF0000000;
//     ADC14->CTL1 |= (0b10 << 4) | (4 << 16);
//     ADC14->MCTL[4] |= 0x6; // input on A6
// }

// // Interrupt Handler for the watchdog timer
// void WDT_A_IRQHandler(void)
// {
//     if (flip)
//     {                        // run the interrupt every other time it is triggered
//         WDT_A->CTL = 0x5A5A; // unlock WDT_A, ~4min timer (up from 250ms)
//         ADC14->CTL0 |= 0b10; // enable ADC
//         ADC14->CTL0 |= 1;    // start ADC conversions
//         while (((ADC14->CTL0 & (1 << 16)) >> 16))
//             ; // wait for conversion to finish
//         temperature = (uint8_t)((float)ADC14->MEM[4] * 0.1498535 - 64);
//         P2->OUT |= 1 << 4; // signal temperature is ready
//         //LCD_goto_xy(0,0);
//         //LCD_print_str("Temp: ");
//         //LCD_print_udec5(temperature);
//         ADC14->CTL0 &= ~(0b10010); // disable and turn off ADC
//         int i;
//         for (i = 0; i < 100; ++i);
//         P2->OUT &= ~(1 << 4);
//         for (i = 0; i < 100; ++i);
//         flip = 0;
//     }
//     else
//     {
//         flip = 1;
//     }
// }

// ! ************************************************************************************************************

// /* --COPYRIGHT--,BSD_EX
//  * Copyright (c) 2013, Texas Instruments Incorporated
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * *  Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  *
//  * *  Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in the
//  *    documentation and/or other materials provided with the distribution.
//  *
//  * *  Neither the name of Texas Instruments Incorporated nor the names of
//  *    its contributors may be used to endorse or promote products derived
//  *    from this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
//  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  *******************************************************************************
//  *
//  *                       MSP432 CODE EXAMPLE DISCLAIMER
//  *
//  * MSP432 code examples are self-contained low-level programs that typically
//  * demonstrate a single peripheral function or device feature in a highly
//  * concise manner. For this the code may rely on the device's power-on default
//  * register values and settings such as the clock configuration and care must
//  * be taken when combining code from several examples to avoid potential side
//  * effects. Also see http://www.ti.com/tool/mspdriverlib for an API functional
//  * library & https://dev.ti.com/pinmux/ for a GUI approach to peripheral configuration.
//  *
//  * --/COPYRIGHT--*/
// //******************************************************************************
// //  MSP432P401 Demo - Software Port Interrupt Service on P1.1 from LPM4
// //
// //  Description: MSP432 device is configured to enter LPM4 mode with GPIOs
// //  properly terminated. P1.1 is configured as an input. Pressing the button
// //  connect to P1.1 results in device waking up and servicing the Port1 ISR.
// //  LPM3 current can be measured when P1.0 is output low (e.g. LED off).
// //
// //  ACLK = 32kHz, MCLK = SMCLK = default DCO
// //
// //
// //               MSP432P401x
// //            -----------------
// //        /|\|                 |
// //         | |                 |
// //         --|RST              |
// //     /|\   |                 |
// //      --o--|P1.1         P1.0|-->LED
// //     \|/
// //
// //   Dung Dang
// //   Texas Instruments Inc.
// //   Oct 2016 (updated) | November 2013 (created)
// //   Built with CCSv6.1, IAR, Keil, GCC
// //******************************************************************************

// #include "msp.h"
// int main(void)
// {
//     // Hold the watchdog

//     WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

//     // Configuring P1.0 as output and P1.1 (switch) as input with pull-up
//     // resistor. Rest of pins are configured as output low.
//     // Notice intentional '=' assignment since all P1 pins are being
//     // deliberately configured
//     P1->DIR = ~(uint8_t)BIT1;
//     P1->OUT = BIT1;
//     P1->REN = BIT1; // Enable pull-up resistor (P1.1 output high)
//     P1->SEL0 = 0;
//     P1->SEL1 = 0;
//     P1->IES = BIT1; // Interrupt on high-to-low transition
//     P1->IFG = 0;    // Clear all P1 interrupt flags
//     P1->IE = BIT1;  // Enable interrupt for P1.1

//     // Enable Port 1 interrupt on the NVIC
//     NVIC->ISER[1] = 1 << ((PORT1_IRQn)&31);

//     // Terminate all remaining pins on the device
//     P2->DIR |= 0xFF;
//     P2->OUT = 0;
//     P3->DIR |= 0xFF;
//     P3->OUT = 0;
//     P4->DIR |= 0xFF;
//     P4->OUT = 0;
//     P5->DIR |= 0xFF;
//     P5->OUT = 0;
//     P6->DIR |= 0xFF;
//     P6->OUT = 0;
//     P7->DIR |= 0xFF;
//     P7->OUT = 0;
//     P8->DIR |= 0xFF;
//     P8->OUT = 0;
//     P9->DIR |= 0xFF;
//     P9->OUT = 0;
//     P10->DIR |= 0xFF;
//     P10->OUT = 0;

//     // Configure Port J
//     PJ->DIR |= (BIT0 | BIT1 | BIT2 | BIT3);
//     PJ->OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);

//     // Enable PCM rude mode, which allows to device to enter LPM3 without waiting for peripherals
//     PCM->CTL1 = PCM_CTL0_KEY_VAL | PCM_CTL1_FORCE_LPM_ENTRY;

//     // Enable global interrupt
//     __enable_irq();

//     // Setting the sleep deep bit
//     SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);

//     // Do not wake up on exit from ISR
//     SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

//     // Ensures SLEEPONEXIT takes effect immediately
//     __DSB();

//     // Go to LPM4
//     __sleep();
// }

// /* Port1 ISR */
// void PORT1_IRQHandler(void)
// {
//     volatile uint32_t i;

//     // Toggling the output on the LED
//     if (P1->IFG & BIT1)
//         P1->OUT ^= BIT0;

//     // Delay for switch debounce
//     for (i = 0; i < 10000; i++)

//         P1->IFG &= ~BIT1;
// }
