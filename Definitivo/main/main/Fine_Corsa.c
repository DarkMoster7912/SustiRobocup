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



/*ISR(PCINT2_vect){			//finecorsa
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
			_delay_ms(300);
			stop_tutto();
			n = 0xFF;
		}
		else{
			if(s==1){
				stop_tutto();
				_delay_ms(1000);
				indietro();
				_delay_ms(300);
				stop_tutto();
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
			_delay_ms(300);
			stop_tutto();
			n = 0xFF;
		}
		else{
			if(d==1){
				stop_tutto();
				_delay_ms(1000);
				avanti();
				_delay_ms(300);
				stop_tutto();
				n = 0xFF;
			}
			else if(d>1){
				d=0;
				//n=0xFF;
			}
		}	
	}
}*/

ISR(PCINT2_vect){
	uint8_t changedbits;

	changedbits = PINK ^ n;
	n = PINK;
	
	s++;
	d++;
	c++;
	N++;
	
	//_delay_ms(100);
	
	Serial_Send(N); Serial_Send("\n");
	
	if(changedbits & (1 << PK0))
	{
		if(c==1){
			Serial_Send("17\n");
			_delay_ms(1000);
			Serial_Send("Ciao\n");
			_delay_ms(300);
			Serial_Send("17\n");
			n = 0xFF;
		}
		else{
			if(s==1){
				Serial_Send("Stop\n");
				_delay_ms(1000);
				Serial_Send("Indietro\n");
				_delay_ms(300);
				Serial_Send("Stop\n");
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
			Serial_Send("Stop\n");
			_delay_ms(1000);
			Serial_Send("Indietro\n");
			_delay_ms(300);
			Serial_Send("Stop\n");
			n = 0xFF;
		}
		else{
			if(d==1){
				Serial_Send("17\n");
				_delay_ms(1000);
				Serial_Send("Ciao\n");
				_delay_ms(300);
				Serial_Send("17\n");
				n = 0xFF;
			}
			else if(d>1){
				d=0;
				//n=0xFF;
			}
		}
		
	}
	if(changedbits & (1<<PK4)){
		if(N==1){
			Serial_Send("Nero fermati\n");Serial_Send(N); Serial_Send("\n");
			_delay_ms(1000);
			n = 0xFF;
		}
		else if(N>1){
			N=0;
			Serial_Send("Niente\n");
		}
	}
}