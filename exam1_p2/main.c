//*****************************************************************************
//
// EE4930_exam1_p2.c
// 1/22/2021
//
// Optimize the following code for speed.
// Reference the line numbers of the code you change in each section of your solution
// Include comments to indicate what you did and how it improves execution speed
//
//****************************************************************************
#include "msp.h"

int power(int x, int y);
void init_A2D(void);
void main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // setup port pin for scope timing
    P3->DIR |= BIT6;
    P3->OUT &= ~BIT6;
    init_A2D();

    int arr[4], arr1[100], alpha[4], omega[4];
    int i, j, k;
    int x, y, z, beta, gamma, sum = 0;

    while (1)
    {
        beta = ADC14->MEM[0];

        for (j = 0; j < 4; j++)
        {
            alpha[j] = beta * j;
        }

        gamma = power(beta, 4);

        for (i = 0; i < 4; i++)
        {
            x = 2;
            arr[i] = gamma + power(x, i);
        }

        for (k = 0; k < 4; k++)
        {
            omega[k] = alpha[k] + arr[3 - k];
        }

        y = x * 31;
        z = (y + 3) * 4 - (x + 12);

        for (i = 0; i < 100; i++)
        {
            x = y + z;
            arr1[i] = (33 * x) + (y * i) - omega[i % 4];
        }

        for (i = 0; i < 100; i++)
        {
            if ((i % 9) == 0)
                sum += arr1[i];
        }
        P3->OUT ^= BIT6; //toggle for timing measurement
    }
}

int power(int x, int y)
{
    int i, z = 1;
    for (i = 0; i < y; i++)
    {
        z = z * x;
    }
    return z;
}

void init_A2D(void)
{
    // Sampling time, S&H=96, ADC14 on, ACLK
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_3;
    ADC14->CTL0 |= ADC14_CTL0_CONSEQ_2 | ADC14_CTL0_ON | ADC14_CTL0_MSC;
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_6; // input on A6

    P4->SEL0 |= 0x80;                  // use A/D
    P4->SEL1 |= 0x80;
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;

    return;
}
