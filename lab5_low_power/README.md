# LAB4 – Low Power

Rock Boynton

EE 4930/011

01/28/2020

## Objectives

The objective of this lab is to write code for a low power application and to use tools to measure
and analyze code performance in terms of power and energy.

## Description

This lab implements a remote temperature measurement system for an eagle's nest as part of a study
to monitor their habits. Eagles must maintain a temperature of about 105 deg. F in the eggs. Eggs normally take 35 days to incubate. Nests typically contain 1 – 3 eggs. Eggs are laid three days apart. The system will need to run a minimum of 70 days to allow time to get it installed well ahead of eggs being laid, time for all eggs to hatch, and uncertainty as to exactly when the eggs will be laid.

Specifications:

* The code must use the A/D to measure temperature in degrees F (resolution of 0.1 deg. F min.) using a TMP36 sensor. The temperature reading must be stored in a global variable that the wireless radio can access, and the radio must be notified that a new value is available for transmission (e.g., set an output high for some short time interval, then back low).
* Temperature readings must be taken about every ten minutes and transmitted to the base station.
* The battery is a CR2032 lithium battery with a capacity of about 210 mAhr.
* Use the Energy Trace tool to analyze your code in terms of its time spent in various modes.
* Use a series resistor (e.g., 10 Ohms) in the power connection to measure the actual power supply current of the system on an oscilloscope.
* Calculate the estimated battery life of your system using both the Energy Trace data and the scope data.
* For code demo, set the time between readings to something more reasonable, like 10 seconds.


## Conclusion

This was a very challenging lab. Getting the main functionality of the system wasn't too difficult,
though. I originally implemented it using two timers, TimerA0 and TimerA1. TimerA0 was used to
periodically kick off the 

## Source Code

```c

```

## Schematic & Energy Trace Output

![Temp Monitor Schematic](Temp_Monitor_Schematic.png)

![Energy Trace Output](Energy_Trace_Output.png)
