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
#define TEN_HZ_PSC 65000 // 18750
#define SHORT 5000

typedef struct Input
{
    float val;
    const uint8_t lcd_x;
    const uint8_t lcd_y;
    bool changed;
} Input;

volatile Input temp = {0, 8, 0, false};

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

void initWDA()
{
    //Configure WDT_A
    WDT_A->CTL = 0x5A5D;       // unlock WDT_A, vloclk src, interval mode, reset count, ~250ms initial timer
    NVIC->ISER[0] |= (1 << 3); // enable WDT_A interrupt in NVIC

    WDT_A->CTL = WDT_A_CTL_PW |
}

void init_cs()
{
    CS->KEY = CS_KEY_VAL;     //unlock
    CS->CLKEN |= CS_CLKEN_VLO_EN;      // turn VLOCLK on;
    CS->CTL1 &= ~(0b1110111); // clear SM/HSMCLK, and MCLK sources
    CS->CTL1 |= 0b0010001;    // set SMCLK/HSMCLK, MCLK to VLOCLK
    CS->KEY = 0;              // lock
}

int initPCM()
{
    // Configure for LPM3
    //    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;   // Set Sleep Deep bit
    //    PCM->CTL0 = 0x695A0008; // unlock PCM/PMR, request LMP3, LP_AM_LDO_VCORE0
    //    SCB->SCR |= (0b10);     // Set Sleep On Exit

    // Power mode cannot be changed until the Power Control Module (PCM)
    // is not active. Status is held in PCMCTL1 register bit 8 (PMR_BUSY)
    // Power mode can be changed when this bit is zero (idle)
    // Wait for  PMR to be idle
    uint32_t PMR_fail = 100000;
    uint32_t AM_LDO_VCORE1_fail = 500000;
    while (PCM->CTL1 & PCM_CTL1_PMR_BUSY)
    {
        PMR_fail--;
        if (PMR_fail == 0) // Attempt Failed - no changes made - return 1
            return -1;
    } // end while

    // Simultaneously set the PCM to LPM3
    PCM->CTL0 = (PCM->CTL0 & ~0xFFFF000F) | PCM_CTL0_KEY_VAL | PCM_CTL0_LPMR__LPM3;
    //      Force 0's in key       force 1's in key   force mode to 0

    // There is a flag to indicate transition to active mode is NOT valid
    // PCMIFG bit 2 (AM_INVALID_TR_IFG)
    if (PCM->IFG & 0x00000004)
    {
        PCM->CLRIFG = 0x00000004; // Attempt failed - clear flag and return 2
        return -2;
    } // end if

    // Check the actual mode, and wait for LPM3 to be set
    while ((PCM->CTL0 & PCM_CTL0_CPM_MASK) != PCM_CTL0_CPM__LPM3)
    {
        AM_LDO_VCORE1_fail--;
        if (AM_LDO_VCORE1_fail == 0)
            return -3; // Attempt failed - return 3
    }                  // end while
                       // Power mode change is complete
    // Clear the change key to prevent unintended changes
    PCM->CTL1 &= ~PCM_CTL1_KEY_MASK;

    // Clocks cannot be changed until the Power Control Module (PCM)
    // is not active. Status is held in PCMCTL1 register bit 8 (PMR_BUSY)
    // Clocks can be changed when this bit is zero (idle)
    // Wait for  PMR to be idle
    while (PCM->CTL1 & PCM_CTL1_PMR_BUSY)
    {
        PMR_fail--;
        if (PMR_fail == 0) // Attempt Failed - return 4
            return -4;
    } // end while

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;   // Set Sleep Deep bit
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk; // Set Sleep On Exit
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
    ADC14->CTL1 &= ~(ADC14_CTL1_RES_2 | ADC14_CTL1_RES_1);     // clear res bits
    ADC14->CTL1 |= ADC14_CTL1_RES__12BIT | ADC14_CTL1_PWRMD_2; // set 12 bit resolution, low power
    ADC14->CTL1 |= (4 << ADC14_CTL1_CSTARTADD_OFS);            // use MEM[4]
    ADC14->MCTL[4] |= ADC14_MCTLN_INCH_6;                      // input on A6
    ADC14->IER0 |= ADC14_IER0_IE4;                             // enable interrupt
    ADC14->CTL0 |= ADC14_CTL0_ENC;
    NVIC->ISER[0] |= ADC14_IER0_IE24; // enable ADC interrupt in NVIC

    P4->SEL1 |= BIT7; // give A/D control of pin - A6
    P4->SEL0 |= BIT7;
}

/**
 * @brief initialize timer for periodic interrupts
 *
 * Triggers the ADC to start a conversion
 */
void init_periodic_timers(void)
{
    // Configure TimA1 to SMCLOCK (12 MHz using 48MHz system clock), divide by 8,  UP mode, Interrupt Enable
    TIMER_A1->CTL |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__8 | TIMER_A_CTL_MC__UP | TIMER_A_CTL_IE | TIMER_A_CTL_CLR;
    TIMER_A1->EX0 |= TIMER_A_EX0_IDEX__8; // divide by 8 again, giving a timer freq of 187.5KHz
    TIMER_A1->CCR[0] = TEN_HZ_PSC;        // set frequency as 10 Hz
    NVIC->ISER[0] |= BITB;                // enable TA1_N interrupt

    // Configure TimA0 to SMCLOCK (12 MHz using 48MHz system clock), divide by 8,  UP mode, Interrupt Enable
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__8 | TIMER_A_CTL_MC__UP | TIMER_A_CTL_IE | TIMER_A_CTL_CLR;
    TIMER_A0->EX0 |= TIMER_A_EX0_IDEX__8; // divide by 8 again, giving a timer freq of 187.5KHz
    TIMER_A0->CCR[0] = 0;                 // set frequency as 10 Hz
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
    uint32_t adc_voltage = adc_reading * (3300.0f / TWELVE_BIT_ADC_RANGE);
    float temp_c = (adc_voltage - 500) / 10.0f; // 10mV/C and 500mV offset
    temp.val = temp_c * 9.0f / 5.0f + 32;
    // temp.changed = true;
    P4->OUT |= BIT5;          // indicate temp reading is available
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
    ADC14->CTL0 |= ADC14_CTL0_SC;  // start a new ADC conversion
}

/**
 * @brief Timer for result ready
 */
void TA0_N_IRQHandler(void)
{
    uint16_t dummy = TIMER_A0->IV; // clear flag
    P4->OUT &= ~BIT5;              // end indication
    TIMER_A0->CCR[0] = 0;
}

void main(void)

{
    // setup
    init_gpio();
    init_wdt()
    init_cs();
    init_pcm();
    init_adc();
    init_periodic_timers();
    // init_lcd();

    __enable_interrupts(); // global interrupt enable

    while (1)
    {
        // __disable_interrupts(); // entering critical section
        // update_display(&temp);
        // __enable_interrupts(); // leaving critical section
        __sleep();
    }
}
