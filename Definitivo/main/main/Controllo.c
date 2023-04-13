#define  F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Pid_PWM.h"
#include "I2C.h"
#include "serial.h"






// volatile int data_ret = 0;
// volatile int seg_enc=0;
// 
// void delay_movimento( int c){
// 	if(c==1){			//avanti
// 		while (seg_enc != 1390);
// 		indietro();
// 		_delay_ms(15);
// 		stop_tutto();
// 		_delay_ms(300);
// 		interrupt_rasp();
// 		seg_enc=0;
// 	}
// 	else if(c==2){		//stop da girata sinistra
// 		//stop_tutto();
// 		destra();
// 		_delay_ms(15);
// 		stop_tutto();
// 		_delay_ms(300);
// 		interrupt_rasp();
// 		seg_enc=0;
// 	}
// 	else if(c==3){		//stop da girata destra
// 		//stop_tutto();
// 		sinistra_lento();
// 		_delay_ms(15);
// 		stop_tutto();
// 		_delay_ms(300);
// 		interrupt_rasp();
// 		seg_enc=0;
// 	}
// }
// 
// void controllo_movimento(int data){
// 	switch(data){
// 		
// 		//Fermo Da controllo avanti e girata
// 		case 0x01:
// 			delay_movimento(2);		//da sinistra
// 			data_ret = 0x04;
// 			break;
// 		case 0x02:
// 			avanti();
// 			delay_movimento(1);		//avanti
// 			/*_delay_ms(700);		//800
// 			stop_tutto();*/
// 			_delay_ms(500);
// 			break;
// 		case 0x03:
// 			delay_movimento(3);		//da destra
// 			data_ret = 0x04;
// 			break;
// 		case 0x04:
// 			stop_tutto();		//fermo
// 			break;
// 		
// 		
// 		//Giroscopio
// 		case 0x27:				//sinistra
// 			sinistra();
// 			break;
// 		case 0x26:				//destra
// 			destra();
// 			break;
// 		case 0x28:
// 			destra_lento();
// 			break;
// 		case 0x29:
// 			sinistra_lento();
// 			break;
// 		
// 		default:
// 			stop_tutto();
// 			break;
// 	}
// }
// 
// int return_data(){
// 	return data_ret;
// }
// 
// 
// void adc_init(){
// 	ADMUX = (1<<REFS0);
// 	ADCSRB = 0;
// 	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);
// }
// 
// int adc_value(){
// 	int k;
// 	ADCSRA = ADCSRA | (1<<ADSC);
// 	while((ADCSRA&(1<<ADSC))!=0);
// 	k = ADC;
// 	return k;
// }
// 
// 
// void adc_control( int k ){		//definire delay
// 	if(k==400){
// 		stop_tutto();
// 		_delay_ms(300);
// 		indietro();
// 		_delay_ms(600);
// 	}
// 	else;
// }
// 
// void controllo_telecamera( int data){
// 	int i;
// 	if(data == 0x36){
// 		Serial_Send("croce\n");
// 		/*avanti();
// 		_delay_ms(200);
// 		stop_tutto();
// 		_delay_ms(500);
// 		cubetto();*/
// 		for(i=0; i<5; i++){
// 			PORTF = 1;			//lampeggia led
// 			_delay_ms(450);
// 			PORTF = 0;				//lampeggia led
// 			_delay_ms(450);
// 		}
// 	}
// 	else;
// }
// 
// 
// void interrupt_rasp(){
// 	PORTF = 2;
// 	_delay_ms(10);
// 	PORTF = 0;
// }