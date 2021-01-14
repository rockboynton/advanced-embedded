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
 * Obviously this implements multiplication with addition which is slower than using the multiplication hardware,
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

    // calculate 8 using unecessary means
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
        int j = complex_calculation(); // repeatedly declaring variable that's never used in loop, compiler can optimize
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
        inner_func(iters);
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
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	init_gpio();
	while (1) {
	    P5->OUT ^= BIT0;  // toggle pin
	    outer_func(10);
	}
}
