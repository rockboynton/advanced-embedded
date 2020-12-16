# LAB2 – MSP432

Rock Boynton

EE 4930/011

12/17/2020

## Objectives

The objective of this lab is to become familiar and gain some experience with
more of the MSP32's peripherals including a timer, an ADC (reading from a
potentiometer), and PWM signal generation. Then building upon what we learned
last week, display the values on an LCD screen.

This lab also gets us familiar with using interrupts for different functions
including capturing a pushbutton press, periodically trigger an ADC conversion
with a timer, and actually capture the A/D conversion value using an interrupt.

## Description

This laboratory consists of interfacing the MSP432 to the LCD, an analog input
signal, and generating a PWM output.

Specifications:

* Connect a potentiometer to 3.3V and ground, and connect its wiper terminal to
  an analog input on the MSP432.

* Perform Analog-to-Digital (A/D) conversions on this analog input with the
  following specs:

  * ‘single-channel single-conversion’ mode
  * 8-bit resolution.
  * A timer and its interrupt to periodically initiate a new A/D conversion.
    * The period used was **TODO PERIOD USED AND WHY**
  * A/D interrupt to handle getting the new conversion value when it is done.

* A/D reading is treated as a percentage. A timer generates a PWM signal with a
  duty cycle corresponding to this percentage but inverted, i.e., if the input
  is at 20%, the duty cycle is 80%, if at 70%, the duty cycle is 30%, etc.

* On the LCD, the A/D reading and the calculated duty cycle of the PWM signal
  are displayed. The LCD only updates with new values when SW1 is pressed. Use an
  interrupt with P1.1 to initiate this action. Do not call the LCD functions
  within the ISR.

### TODO Scope Measurements to verify signal

## Conclusions

## Source Code

```c

```

## Schematic
![Schematic](lab2_schem.png)

## Wiring Diagram

![Wiring diagram](lab2_bb.png)
