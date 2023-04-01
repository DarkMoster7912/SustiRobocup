/*
 * provausbasp.c
 *
 * Created: 14/08/2015 10:29:34
 *  Author: Supervisor
 */ 


#define F_CPU 16000000ul

#include <avr/io.h>

#include <util/delay.h>
int main(void)
{
	DDRB=0xff;
    while(1)
    {

   PORTB=0;   
   _delay_ms(2000);
   PORTB=255;
   _delay_ms(2000);
        //TODO:: Please write your application code 
    }
}