#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "laser.h"
#include "I2C.h"
#include "Pid_PWM.h"


/*
void laser1(){
	stop_tutto();
	_delay_ms(1500);
	do{
		sinistra();
	} while(data != 0x27);
	stop_tutto();
	_delay_ms(1500);
	if((data != 0x11) |
		(data != 0x12) |
		(data != 0x13) |
		(data != 0x14) |
		(data != 0x15) |
		(data != 0x16) ){
			
			avanti();
		}else {
			do{
				destra();
			}while(data != 0x26);
			stop_tutto();
			_delay_ms(1500);
			
			if((data != 0x11) |
			(data != 0x12) |
			(data != 0x13) |
			(data != 0x14) |
			(data != 0x15) |
			(data != 0x16) ){
				avanti();
			}else{
				do{
					destra();
				}while(data != 0x26);
				stop_tutto();
				_delay_ms(1500);
				avanti();
			}
		}
}
*/
void laser2(){
	stop_tutto();
	Set_Velocita(200.0);
	_delay_ms(500);
	indietro();
	_delay_ms(450);
	sinistra();
	_delay_ms(100);
	stop_tutto();
	_delay_ms(500);
	avanti();
}

