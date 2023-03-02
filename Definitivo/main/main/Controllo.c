#define  F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Pid_PWM.h"
#include "I2C.h"
#include "serial.h"


void controllo(int data){
	switch(data){
		case 0x01:
			stop_tutto();
			break;
		case 0x02:
			avanti();
			_delay_ms(800);
			stop_tutto();
			_delay_ms(1000);
			break;
		
		//Laser
		
		
		//Giroscopio
		case 0x27:				//sinistra
			sinistra();
			break;
		case 0x26:				//destra
			destra();
			break;
		
		
		//Telecamere
		case 0x20:
			_delay_ms(400);
			stop_tutto();
			_delay_ms(1000);
			cubetto();			//definire funzione
			break;
		default:
			stop_tutto();
			break;
	}
}


void controllo_1(int data){
	if(data == 0x14){
		do{
			destra();
		}while(data != 0x26);
	}
	if(data == 0x15){
		do{
			sinistra();
		}while(data != 0x27);
	}
	if(data == 0x17){
		avanti();
		_delay_ms(1600);
		stop_tutto();
		_delay_ms(1000);
	}
}