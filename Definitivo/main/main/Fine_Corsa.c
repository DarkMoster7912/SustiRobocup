#define  F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Pid_PWM.h"
#include "I2C.h"
#include "serial.h"

volatile uint8_t n = 0x0F;	//0xFF
volatile int s=0;
volatile int d=0;
volatile int c=0;
volatile int N=0;
//volatile int a=0;


void PCINT_Init(){
	PCICR = 1<<PCIE2;
	PCIFR = 1<<PCIF2;
	PCMSK2 = (1<<PCINT16) | (1<<PCINT17);
	sei();
	DDRK=0;
}



ISR(PCINT2_vect){			//finecorsa
	uint8_t changedbits;

	changedbits = PINK ^ n;
	n = PINK;
	
	s++;
	d++;
	c++;
	
	//_delay_ms(100);
	
	if(changedbits & (1 << PK0))
	{
		if(c==1){
			stop_tutto();
			_delay_ms(1000);
			avanti();
			_delay_ms(100);
			seg_enc_a_zero(0);
			stop_tutto();
			n = 0xFF;
		}
		else{
			if(s==1){
				stop_tutto();
				_delay_ms(1000);
				indietro();
				_delay_ms(100);
				stop_tutto();
				seg_enc_a_zero(0);
				n = 0xFF;
			}
			else if(s>1){			//10 uF per fitro alti rimbalzo
				s=0;
				//n=0xFF;
			}
		}
	}

	if(changedbits & (1 << PK1))
	{
		if(c==1){
			stop_tutto();
			_delay_ms(1000);
			indietro();
			_delay_ms(100);
			stop_tutto();
			seg_enc_a_zero(0);
			n = 0xFF;
		}
		else{
			if(d==1){
				stop_tutto();
				_delay_ms(1000);
				avanti();
				_delay_ms(100);
				stop_tutto();
				seg_enc_a_zero(0);
				n = 0xFF;
			}
			else if(d>1){
				d=0;
				//n=0xFF;
			}
		}	
	}
}