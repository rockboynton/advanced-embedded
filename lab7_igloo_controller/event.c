/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== event.c ========
MSP432 event.c

Rock Boynton
02/23/2021
EE4930 Final

Description:
    Controller for an igloo. Provides heating and fan control.

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
 */

#define __MSP432P401R__
#include "msp.h"

#include <stdio.h>
#include <string.h>

#include "msoe_lib_clk.h"
#include "msoe_lib_lcd.h"
#include "msoe_lib_delay.h"

/* XDC module Headers */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>

/* BIOS module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>

#include <ti/drivers/Board.h>

/* TI Drivers */
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>

#define CLOCK_PERIOD 10 // number of clock ticks per adc trigger \
                      // period of ~10 ms

#define TASK_STACK_SIZE 1024
#define ADC_INTERRUPT 40 // interrupt number for the ADC
#define ADC_RANGE 256  // for 8 bit conversions

Task_Struct check_for_input_task_struct, update_lcd_task_struct, get_temp_i2c_task_struct, heater_task_struct;
Char check_for_input_task_stack[TASK_STACK_SIZE], update_lcd_task_stack[TASK_STACK_SIZE], get_temp_i2c_task_stack[TASK_STACK_SIZE], heater_task_stack[TASK_STACK_SIZE];

Clock_Struct clock_struct;
Clock_Handle clock_handle;

Event_Struct reading_available_event_struct, lcd_event_struct;
Event_Handle reading_available_event_handle, lcd_event_handle;

Swi_Struct adc_swi_struct;
Swi_Handle adc_swi_handle;

Hwi_Struct adc_hwi_struct;
Hwi_Handle adc_hwi_handle;

uint8_t setpoint = 0;
uint16_t temperature = 0;
Bool heating = false;

/**
 * @brief Initialize the LCD
 *
 * Sets it up to display setpoint set point
 *
 */
void init_lcd(void)
{
    LCD_Config();
    LCD_clear();
    LCD_home();
    LCD_contrast(10);

    LCD_print_str("Setpt: xx F");
    LCD_goto_xy(0, 1);
    LCD_print_str("Temp:  xx F");
    LCD_goto_xy(0, 2);
    LCD_print_str("Heat:  OFF");
}

void init_adc()
{
    // Sampling time, S&H=96, ADC14 on, SMCLK, single input, single conv.
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_4 | ADC14_CTL0_ON;
    ADC14->CTL1 &= ~(ADC14_CTL1_RES_2 | ADC14_CTL1_RES_1); // 8-bit conversion
    ADC14->CTL1 |= (4 << ADC14_CTL1_CSTARTADD_OFS);        // use MEM[4]
    ADC14->MCTL[4] |= ADC14_MCTLN_INCH_6;                  // input on A6
    ADC14->IER0 |= ADC14_IER0_IE4;                         // enable interrupt
    ADC14->CTL0 |= ADC14_CTL0_ENC;

    P4->SEL1 |= BIT7; // give A/D control of pin - A6
    P4->SEL0 |= BIT7;
}

/*
 * Clock function triggers an ADC conversion
 */
Void start_conversion()
{
    ADC14->CTL0 |= 1; // start ADC conversions
}

Void get_temp_i2c()
{
    uint16_t tmp_data;
    uint8_t rxBuffer[2];

    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    I2C_Handle i2cHandle = I2C_open(0, &params);
    I2C_Transaction transaction;
    transaction.slaveAddress = 0x48;
    transaction.readBuf = rxBuffer;
    transaction.readCount = sizeof(rxBuffer);
    transaction.writeCount = 0;

    /* Take samples forever */
    while (1)
    {
        //get a new sample the same time the adc conversions finnish. ~0.5 sec
        UInt posted = Event_pend(reading_available_event_handle,
                                Event_Id_00,
                                Event_Id_NONE,
                                BIOS_WAIT_FOREVER);

        if (posted == 0)
        {
            System_printf("Timeout expired for Event_pend()\n");
            break;
        }

        // signal to LCD task if the new temperature value is different than whats currently displayed
        if (posted & Event_Id_00)
        {
            if (I2C_transfer(i2cHandle, &transaction))
            {
                tmp_data = (rxBuffer[0] << 4) | (rxBuffer[1] >> 4); // combine data result before multiplication
                temperature = (tmp_data * 0.0625);                      // convert to reading to celsius
                temperature = ((((float)temperature) * 9) / 5) + 32;    // convert celsius to farenheight
            }
            else
            {
                System_printf("I2C bus fault.\n");
            }
        }
    }

    I2C_close(i2cHandle);
    BIOS_exit(0);
}

Void heater()
{
    UInt posted;

    //setup pwm
    PWM_Handle pwm;
    PWM_Params pwmParams;
    uint32_t dutyValue;
    // Initialize the PWM parameters
    PWM_Params_init(&pwmParams);
    pwmParams.idleLevel = PWM_IDLE_LOW;
    pwmParams.periodUnits = PWM_PERIOD_HZ;
    pwmParams.periodValue = 13;              // 1MHz
    pwmParams.dutyUnits = PWM_DUTY_FRACTION; // fractional percentage
    pwmParams.dutyValue = 0;                 // 0% initial duty cycle
    // get PWM instance
    pwm = PWM_open(0, &pwmParams);
    if (pwm == NULL)
    {
        // PWM_open() failed
        while (1){}
    }
    dutyValue = (uint32_t)(((uint64_t)PWM_DUTY_FRACTION_MAX * 50) / 100);
    PWM_setDuty(pwm, dutyValue); // set duty cycle to 40%

    while (1)
    {
        //update output the same time the adc conversion finishes.
        posted = Event_pend(reading_available_event_handle,
                            Event_Id_00,
                            Event_Id_NONE,
                            BIOS_WAIT_FOREVER);

        if (posted == 0)
        {
            System_printf("Timeout expired for Event_pend()\n");
            break;
        }
        if (posted & Event_Id_00)
        {
            if ((temperature - 1) >= setpoint)
            {
                //turn heater off
                P2->OUT &= ~(BIT4 | BIT6);
                PWM_stop(pwm);
                heating = false;
            }
            else if ((temperature + 1) <= setpoint)
            {
                //turn heater on
                P2->OUT |= BIT6;
                PWM_start(pwm);
                heating = true;
            }
        }
    }

    BIOS_exit(0); //should never get here
}

init_gpio()
{
    // pin P2.4 -- fan
    P2->DIR |= BIT4;  // configure as output
    P2->OUT &= ~BIT4; // set output low to start

    // pin P2.6 -- heater
    P2->DIR |= BIT6;  // configure as output
    P2->OUT &= ~BIT6; // set output low to start
}

/*
 * Pends on an LCD update event to update the LCD with a new setpoint temperature
 */
Void update_lcd()
{
    UInt posted;

    while (1)
    {
        //wait for event
        posted = Event_pend(lcd_event_handle,
                            Event_Id_00,
                            Event_Id_NONE,
                            BIOS_WAIT_FOREVER);
        if (posted == 0)
        {
            System_printf("Timeout expired for Event_pend()\n");
            break;
        }
        // update the LCD if the correct event trigger was posted.
        if (posted & Event_Id_00)
        {
            LCD_goto_xy(6, 0);
            LCD_print_udec3(setpoint);
            LCD_goto_xy(6, 1);
            LCD_print_udec3(temperature);
            LCD_goto_xy(7, 2);
            LCD_print_str(heating ? "ON " : "OFF");
        }
    }
}

/*
 * Pends on a new reading to be available from the ADC. Conditionally updates the LCD if the value
 * has changed.
 */
Void check_for_input()
{
    UInt posted;
    uint8_t old_setpt = 0;
    uint16_t old_temperature = 0;

    while (1)
    {
        // waits for a reading to be available
        posted = Event_pend(reading_available_event_handle,
                            Event_Id_00,
                            Event_Id_NONE,
                            BIOS_WAIT_FOREVER);

        if (posted == 0)
        {
            System_printf("Timeout expired for Event_pend()\n");
            break;
        }

        // signal to LCD task if the new setpoint value or temperature is different than whats currently displayed
        if (posted & Event_Id_00)
        {
            if (setpoint != old_setpt)
            {
                old_setpt = setpoint;
                Event_post(lcd_event_handle, Event_Id_00);
            }
            else if (temperature != old_temperature)
            {
                old_temperature = temperature;
                Event_post(lcd_event_handle, Event_Id_00);
            }
        }
    }
    BIOS_exit(0);
}

/*
 * Hardware Interrupt handler
 * Called when a new ADC value is ready.
 * Posts to a software interrupt to read ADC and convert to setpoint
 */
Void adc_hwi()
{
    uint32_t adc_val = ADC14->MEM[4]; //clear the interrupt by reading the value
    Swi_post(adc_swi_handle); // signal to the SWI that a new conversion is ready
}

/*
 * Software interrupt handler
 * Reads the ADC value and converts it to the setpoint range 50 - 90 F
 * Posts event check_for_input that a new value is available
 */
Void adc_swi()
{
    setpoint = (((float)ADC14->MEM[4]) / ADC_RANGE) * (90 - 50 + 1) + 50;
    Event_post(reading_available_event_handle, Event_Id_00);
}

int main()
{

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    init_lcd();
    Hwi_enable();
    init_adc();
    init_gpio();

    /* Construct BIOS Objects */
    Task_Params task_params;
    Clock_Params clock_params;
    Swi_Params adc_swi_params;
    Hwi_Params adc_hwi_params;

    /* Call driver init functions */
    Board_init();
    I2C_init();
    PWM_init();

    /* Construct Task threads */
    Task_Params_init(&task_params);
    task_params.stackSize = TASK_STACK_SIZE;
    // updating the internal setpoint is less important than handling the HW interrupts
    // but more important than updating the LCD display
    task_params.priority = 4;
    task_params.stack = &check_for_input_task_stack;
    Task_construct(&check_for_input_task_struct, (Task_FuncPtr)check_for_input, &task_params, NULL);

    // lcd update is not super important
    task_params.priority = 5;
    task_params.stack = &update_lcd_task_stack;
    Task_construct(&update_lcd_task_struct, (Task_FuncPtr)update_lcd, &task_params, NULL);

    // getting temperature reading is most important
    task_params.priority = 3;
    task_params.stack = &get_temp_i2c_task_stack;
    Task_construct(&get_temp_i2c_task_struct, (Task_FuncPtr)get_temp_i2c, &task_params, NULL);

    // setting heater is least important
    task_params.priority = 6;
    task_params.stack = &heater_task_stack;
    Task_construct(&heater_task_struct, (Task_FuncPtr)heater, &task_params, NULL);

    /* Obtain event handlers */
    Event_construct(&reading_available_event_struct, NULL);
    reading_available_event_handle = Event_handle(&reading_available_event_struct);

    Event_construct(&lcd_event_struct, NULL);
    lcd_event_handle = Event_handle(&lcd_event_struct);

    /* setup clock */
    Clock_Params_init(&clock_params);
    clock_params.startFlag = TRUE;
    Clock_construct(&clock_struct, (Clock_FuncPtr)start_conversion, CLOCK_PERIOD * 2, &clock_params);
    clock_handle = Clock_handle(&clock_struct);
    Clock_setPeriod(clock_handle, CLOCK_PERIOD);

    /* setup SWI */
    Swi_Params_init(&adc_swi_params);
    adc_swi_params.priority = 2; // priority after HWI
    adc_swi_params.trigger = 0;

    Swi_construct(&adc_swi_struct, (Swi_FuncPtr)adc_swi, &adc_swi_params, NULL);
    adc_swi_handle = Swi_handle(&adc_swi_struct);

    /* setup HWI */
    Hwi_Params_init(&adc_hwi_params);
    adc_hwi_params.eventId = ADC_INTERRUPT;
    adc_hwi_params.priority = 1; // top priority
    Hwi_construct(&adc_hwi_struct, ADC_INTERRUPT, (Hwi_FuncPtr)adc_hwi, &adc_hwi_params, NULL);
    adc_hwi_handle = Hwi_handle(&adc_hwi_struct);

    /* start the BIOS */
    BIOS_start();
    return (0);
}
