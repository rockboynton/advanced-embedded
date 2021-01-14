# LAB3 – Compiler Optimization

Rock Boynton

EE 4930/011

01/05/2020

## Objectives

The objective of this lab is to become familiar and gain some experience with
different optimization settings in a compiler. In particular, we want to observe and analyze the
effects of a variety of optimization levels on code size produced and speed.

Also, it is important to learn what code constructs are *optimize-able*, and implement them
so the effects of the compiler are more obvious.

## Description

This laboratory consists of writing heavily inefficient code to run on the MSP432 implementing
constructs that are typically able to be optimized by a compiler depending on the optimization level
and preference for speed vs. size.

Here is a list of some of those constructs I implemented:

* Including unnecessary header files
* Nested small function blocks
* Declaring unused variables
* Performing complex calculations that should be pre-calculated
* Using powers of two for multiplication and division
* Calling functions in a loop
* Nested if/else constructs
* Using recursive functions

See comments in the code for more

The code is measured for time accurately by toggling an output pin in each loop of the main program
and measuring the time period with an oscilloscope, so 1/2 the period of the pin is the runtime of
an iteration of the loop

## Conclusions

Optimization is always a topic I found interesting, but haven't been able to explore much in my
previous coursework. Seeing how much faster the code ran (~96% faster at level 5) after pumping up
the optimization level was really cool to see. I hope I can utilize what I have learned when writing
more complex code that actually does something useful!

Writing intentionally inefficient code was interesting, but good to make the ideas more concrete and
understand what not to do when writing code for efficiency.

I also hadn't thought of using a GPIO pin to accurately measure time using something in the C
standard library, but I can see how that can be useful, especially in embedded environments.

## Source Code

```c
/*****************************************************************************
MSP432 main.c

Rock Boynton
01/05/2020
EE4930 Lab 3

Description:
    This is a code written in a very inefficient manner in the hopes of observing
    the effects of the TI compiler optimization settings. There are a variety of
    intentional inefficient areas of the code, which provide opportunities for
    optimization.

    The code runtime is measured by toggling an output pin (P5.0) and viewing the period
    in an oscilloscope, where half the period would be one loop of the program.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "msp.h"

// including unused header files allows the compiler to optimize space
// include random unused c standard modules
#include <complex.h>
#include <fenv.h>
#include <locale.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <wchar.h>

// include all unused msoe library
#include "msoe_lib_all.h"

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

/**
 * Switch statement may be faster than many if-elses
 * Using many nested if-else may provide opportunity for optimization
 * Should put most common use case first
 */
int nested_if_else_constructs(int num)
{
    if (num < 1) {
        return 1;
    } else if (num < 10) {
        return 10;
    } else if (num < 100) {
        return 100;
    } else {
        if (num % 100) {
            return 100;
        } else if (num % 4) {
            if (num % 2) {
                return 2;
            }
            return 4;
        } else {
            return -1;
        }
    }
}

/**
 * Small function can be inlined by the compiler
 */
int add(int x, int y)
{
    return x + y;
}

/**
 * This requires a lot of implicit casting between int and double, slowing the code down.
 * Obviously this implements multiplication with addition which is slower than using the
 * multiplication hardware,
 * maybe the compiler will optimize that
 */
int multiply(double x, double y)
{
    double res = 0;
    int i;
    for (i = 0; i < y; ++i) {
        res = add(res, x);
    }
    return res;
}

/**
 * Use slower recursive power function instead of faster iterative
 *  Also uses slower multiply method above
 */
int power(int x, unsigned int exp)
{
    if (exp == 0)
        return 1;
    else if (exp % 2 == 0)
        return multiply(power(x, exp/2), power(x, exp/2));
    else
        return x * multiply(power(x, exp/2), power(x, exp/2));
}

double complex_calculation() {
    // do a long string of calculations using powers of 2 inside of local variables for them
    int two = 2;
    int four = 4;
    int eight = 8;
    int sixteen = 16;

    // calculate 8 using unnecessary means
    // using powers of 2 the compiler should be able to optimize this by bit shifting
    // this can also be simplified/solved before putting into code
    int num = two * two / two * four * eight / eight * sixteen / four * eight / sixteen / sixteen;
    return power(num, num);
}

// Nested functions may be unrolled for smaller number of iterations
long long* inner_func(int iters)
{
    // waste space using long long when only int needed
    long long* buff = malloc(sizeof(long long) * iters);
    int i;
    for (i = 0; i < iters; ++i) {
        // repeatedly declaring variable that's never used in loop, compiler can optimize
        int j = complex_calculation();
        nested_if_else_constructs(1002);
        buff[i] = (long long) j;
    }
    return buff;
}

void medium_func(int iters)
{
    int i;
    for (i = 0; i < iters; ++i) {
        int j, k, l, m; // unused variables
        inner_func(iters); // allocated memory from inner_func isn't freed
    }
}

void outer_func(int iters)
{
    int i;
    for (i = 0; i < iters; ++i) {
        medium_func(iters);
    }
}

/**
 * Run some very inefficient code in a loop.
 * Period of the code is 1/2 the period of the output pin 5.0
 *
 */
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer
    init_gpio();
    while (1) {
        P5->OUT ^= BIT0;  // toggle pin
        outer_func(10);
    }
}
```

## Lab Results Table

| EE4930               |                    |                  |                |                 |                |                     |                     |                      |                     |                 |                |
| -------------------- | ------------------ | ---------------- | -------------- | --------------- | -------------- | ------------------- | ------------------- | -------------------- | ------------------- | --------------- | -------------- |
| Lab 3 Data           |                    |                  |                |                 |                |                     |                     |                      |                     |                 |                |
| Name:                | Rock Boynton       |                  |                |                 |                |                     |                     |                      |                     |                 |
| Date:                | 1/5/2021           |                  |                |                 |                |                     |                     |                      |                     |                 |                |
|                      |                    |                  |                |                 |                |                     |                     |                      |                     |                 |                |
| Optimization setting | size/speed setting | Module code size | Percent change | Total code size | Percent change | Module ro data size | Total  ro data size | Module  rw data size | Total  rw data size | run time \[ms\] | Percent change |
| OFF                  | 0                  | 1666             |                | 2774            |                | 228                 | 261                 | 8                    | 3112                | 419.2           |                |
| 0                    | 0                  | 1254             | \-24.7%        | 2362            | \-14.9%        | 228                 | 261                 | 8                    | 3112                | 365.4           | \-12.8%        |
| 2                    | 0                  | 1030             | \-38.2%        | 2710            | \-2.3%         | 228                 | 261                 | 8                    | 3112                | 331.6           | \-20.9%        |
| 4                    | 0                  | 458              | \-72.5%        | 2134            | \-23.1%        | 228                 | 255                 | 0                    | 3104                | 17.1            | \-95.9%        |
| 0                    | 2                  | 1354             | \-18.7%        | 2462            | \-11.2%        | 228                 | 261                 | 8                    | 3112                | 364.9           | \-13.0%        |
| 2                    | 2                  | 1106             | \-33.6%        | 2786            | 0.4%           | 228                 | 261                 | 8                    | 3112                | 332.6           | \-20.7%        |
| 4                    | 2                  | 530              | \-68.2%        | 2222            | \-19.9%        | 228                 | 255                 | 0                    | 3104                | 17.6            | \-95.8%        |
| 0                    | 3                  | 1570             | \-5.8%         | 2678            | \-3.5%         | 228                 | 261                 | 8                    | 3112                | 365.1           | \-12.9%        |
| 2                    | 3                  | 1278             | \-23.3%        | 2958            | 6.6%           | 228                 | 261                 | 8                    | 3112                | 332.2           | \-20.8%        |
| 4                    | 3                  | 688              | \-58.7%        | 2380            | \-14.2%        | 228                 | 255                 | 0                    | 3104                | 17.3            | \-95.9%        |
| 0                    | 5                  | 1688             | 1.3%           | 2796            | 0.8%           | 228                 | 261                 | 8                    | 3112                | 365.1           | \-12.9%        |
| 2                    | 5                  | 1396             | \-16.2%        | 3076            | 10.9%          | 228                 | 261                 | 8                    | 3112                | 332.2           | \-20.8%        |
| 4                    | 5                  | 688              | \-58.7%        | 2372            | \-14.5%        | 228                 | 255                 | 0                    | 3104                | 17.5            | \-95.8%        |
