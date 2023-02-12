#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>
#include "PID_PWM.h"
#include "serial.h"

int main(void)
{
	Init_PWM();
	Init_Interrupt();
	Init_Start_PID();
	Start_PWM();
    while (1) 
    {
    }
}

