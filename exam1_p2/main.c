//*****************************************************************************
//
// EE4930_exam1_p2.c
// 1/22/2021
//
// Optimize the following code for speed.
// Reference the line numbers of the code you change in each section of your solution
// Include comments to indicate what you did and how it improves execution speed
//
// Modified by
//****************************************************************************
#include "msp.h"

void init_A2D(void);
void main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // setup port pin for scope timing
    P3->DIR |= BIT6;
    P3->OUT &= ~BIT6;
    init_A2D();

    // remove unecessary variables
    int arr[4], arr1[100], alpha[4], omega[4];
    int i;
    int y, k, beta, gamma, sum = 0;


    while (1)
    {
        beta = ADC14->MEM[0];
        for (i = 0; i < 4; i++)
        {
            // Do stuff in the same loop rather than seperate loops
            alpha[i] = beta * i;
            arr[i] = gamma + (1 << i); // power with base 2 is left shift operation
            omega[i] = alpha[i] + arr[3 - i];
        }

        gamma = beta * beta * beta * beta; // unroll multiplication rather than use power function

    // perform static calculation by hand, saves multiplication
    // x = 2;
    // y = x * 31; // 62
    y = 62;
    // z = (y + 3) * 4 - (x + 12); // 246
    // x = y + z; // 308
    // let k = 33 * x = 10164, pulling out calculation from loop so only performing once
    k = 10164;
    for (i = 0; i < 100; i += 10) // partially unroll large loop...could unroll further
    {
        // replace mod with equivalent power-of-2 bitwise op (x % 2^n == x & (2^n-1))
        // replace unchanging variables with calculated literals
        arr1[i] = k + (y * i) - omega[i & 3];
        arr1[i + 1] = k + (y * (i + 1)) - omega[(i + 1) & 3];
        arr1[i + 2] = k + (y * (i + 2)) - omega[(i + 2) & 3];
        arr1[i + 3] = k + (y * (i + 3)) - omega[(i + 3) & 3];
        arr1[i + 4] = k + (y * (i + 4)) - omega[(i + 4) & 3];
        arr1[i + 5] = k + (y * (i + 5)) - omega[(i + 5) & 3];
        arr1[i + 6] = k + (y * (i + 6)) - omega[(i + 6) & 3];
        arr1[i + 7] = k + (y * (i + 7)) - omega[(i + 7) & 3];
        arr1[i + 8] = k + (y * (i + 8)) - omega[(i + 8) & 3];
        arr1[i + 9] = k + (y * (i + 9)) - omega[(i + 9) & 3];
        }

        // unroll mod 9 loop into single expression, no loop needed
        sum += arr1[0] + arr1[9] + arr1[18] + arr1[27] + arr1[36] + arr1[45] + arr1[54] + arr1[63]
               + arr1[72] + arr1[81] + arr1[90] + arr1[99];

        P3->OUT ^= BIT6; //toggle for timing measurement
    }
}

// remove power function (not needed)

void init_A2D(void)
{
    // Sampling time, S&H=96, ADC14 on, ACLK
    // * CHANGED Set all bits in a single statement, saves a register read-modify-write
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_3 | ADC14_CTL0_CONSEQ_2 | ADC14_CTL0_ON | ADC14_CTL0_MSC;
    // ADC14->CTL0 |= ADC14_CTL0_CONSEQ_2 | ADC14_CTL0_ON | ADC14_CTL0_MSC;
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_6; // input on A6

    P4->SEL0 |= 0x80; // use A/D
    P4->SEL1 |= 0x80;
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;

    return;
}
