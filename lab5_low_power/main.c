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

//#include "msoe_lib_lcd.h"
#include "msoe_lib_misc.h"

// Defines
#define TWELVE_BIT_ADC_RANGE 4096

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
//void init_lcd(void)
//{
//    LCD_Config();
//    LCD_clear();
//    LCD_home();
//    LCD_contrast(10);
//
//    LCD_print_str("Temp:");
//}

/**
 * @brief Initialize input pins
 *
 * Output pin P4.5 for indicating temp reading is available
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

//void update_display(Input *input)
//{
//    static char val_str[16];
//    if (input->changed)
//    {
//        LCD_goto_xy(input->lcd_x, input->lcd_y);
//        sprintf(val_str, "%.1f", input->val);
//        LCD_print_str(val_str);
//        input->changed = false;
//    }
//}

void init_wdt()
{
    //Configure WDT_A
    WDT_A->CTL = WDT_A_CTL_PW // use password
            | WDT_A_CTL_SSEL__VLOCLK // use VLOCLK
            | WDT_A_CTL_TMSEL // interval timer mode
            | WDT_A_CTL_CNTCL // clear the count
            | WDT_A_CTL_IS_5; // ~250 ms initial interval
    NVIC->ISER[0] |= WDT_A_IRQn;
}

void init_cs()
{
    CS->KEY = CS_KEY_VAL;     //unlock
    CS->CLKEN |= CS_CLKEN_VLO_EN;      // turn VLOCLK on;
    CS->CTL1 &= ~(CS_CTL1_SELM_MASK | CS_CTL1_SELM_MASK); // clear SM/HSMCLK, and MCLK sources
    CS->CTL1 |= CS_CTL1_SELM__VLOCLK | CS_CTL1_SELS__VLOCLK;    // set SMCLK/HSMCLK, MCLK to VLOCLK
    CS->KEY = 0;              // lock
}

// followed similar format of Clock_init_48MHz()
int init_pcm()
{
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_LPMR__LPM3 | PCM_CTL0_AMR__AM_LF_VCORE0; // unlock PCM/PMR, request LMP3, LP_AM_LDO_VCORE0
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk   // Set Sleep Deep bit
                | SCB_SCR_SLEEPONEXIT_Msk; // Set Sleep On Exit
    return 0;
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
    temp.changed = true;
    P4->OUT |= BIT5;          // indicate temp reading is available
    ADC14->CTL0 &= ~(ADC14_CTL0_ON | ADC14_CTL0_ENC); // turn off ADC
    WDT_A->CTL = WDT_A_CTL_PW // use password
                 | WDT_A_CTL_CNTCL        // clear the count
                 | WDT_A_CTL_IS_5;        // ~250ms mins interval to indicate ready
    __deep_sleep();
}

// Interrupt Handler for the watchdog timer
void WDT_A_IRQHandler(void)
{
    if (temp.changed)
    {
        P4->OUT &= ~BIT5;            // end indication
        temp.changed = false;
        WDT_A->CTL = WDT_A_CTL_PW      // use password
                   | WDT_A_CTL_CNTCL // clear the count
                   | WDT_A_CTL_IS_2; // change to ~4 mins intervals
        __deep_sleep();
    }
    else
    {
        init_adc();
        ADC14->CTL0 |= ADC14_CTL0_SC; // start a new ADC conversion
        __sleep();
    }
}

void main(void)

{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;   // Stop watchdog timer

    // setup
    init_gpio();
    Set_ports_to_out();
    init_cs();
    init_wdt();
    int code;
    if ((code = init_pcm()) != 0)
    {
       printf("Error code %d\n", code);
       return;
    }
    init_adc();
    // init_lcd();

    __enable_interrupts(); // global interrupt enable
    __deep_sleep();

    while (1)
    {
        // __disable_interrupts(); // entering critical section
        // update_display(&temp);
        // __enable_interrupts(); // leaving critical section
    }
}
