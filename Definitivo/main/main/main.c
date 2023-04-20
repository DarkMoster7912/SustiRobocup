#define  F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Pid_PWM.h"
#include "I2C.h"
#include "serial.h"
//#include "Controllo.h"
#include "Fine_Corsa.h"

volatile int data = 0;
volatile int data_att = 0;
void I2C_received(uint8_t received_data)		//ricevo da master
{
	data = received_data;
	//data_att = data;
}

void I2C_requested()						//trasmetto a master
{
	I2C_transmitByte(data);
}

int data_return(int d){
	return d;
}

volatile int data_ret = 0;
volatile int seg_enc=0;
volatile int fn =0;


void delay_movimento( int c){
	if(c==1){			//avanti
		do{
			seg_enc = return_seg_enc();
		}while ((seg_enc != 655) && (data!=0x34) && (fn != 2));
		if((data == 0x34) || (fn == 2)){
			if (data == 0x34){	//Nero
				indietro();
				_delay_ms(700);
				stop_tutto();
				_delay_ms(200);
				destra(800.0);
				_delay_ms(650);
				stop_tutto();
				_delay_ms(200);
				avanti();
				_delay_ms(300);
				stop_tutto();
				_delay_ms(500);
			}
			else{
				indietro();//Finecorsa
				_delay_ms(200);
				stop_tutto();
				_delay_ms(500);
			}
			interrupt_rasp();
			seg_enc_a_zero(0);
			seg_enc=0;
			fn=0;
		}
		else{
			indietro();
			_delay_ms(15);
			stop_tutto();
			_delay_ms(500);
			interrupt_rasp();
			seg_enc_a_zero(0);
			seg_enc=0;
		}
		
	}
	else if(c==2){		//stop da girata sinistra
		//stop_tutto();
		destra(500.0);
		_delay_ms(15);
		stop_tutto();
		_delay_ms(500);
		//interrupt_rasp();
		seg_enc_a_zero(0);
		seg_enc=0;
	}
	else if(c==3){		//stop da girata destra
		//stop_tutto();
		sinistra(500.0);
		_delay_ms(15);
		stop_tutto();
		_delay_ms(500);
		//interrupt_rasp();
		seg_enc_a_zero(0);
		seg_enc=0;
	}
}
volatile int i=0;
void controllo_movimento(){
	switch(data){
		
		//Fermo Da controllo avanti e girata
		case 0x01:
			delay_movimento(2);		//da sinistra
			//data = 0x04;
			seg_enc_a_zero(0);
			
			break;
		case 0x02:
			avanti();
			//data = 0x04;
			delay_movimento(1);		//avanti
			
			PORTF = 1<<PF0;
			_delay_ms(500);
			PORTF = 0;
			/*_delay_ms(700);		//800
			stop_tutto();*/
			break;
		case 0x03:
			delay_movimento(3);		//da destra
			//data = 0x04;
			seg_enc_a_zero(0);
			break;
		case 0x04:
			stop_tutto();		//fermo
			seg_enc_a_zero(0);
			break;
		
		
		//Giroscopio
		case 0x27:				//sinistra
			sinistra(800.0);
			break;
		case 0x26:				//destra
			destra(800.0);
			break;
		case 0x28:
			destra(500.0);
			if(return_seg_enc() > 1500){
				data = 0x02;
				_delay_ms(10);
				data = 0x04;
				_delay_ms(500);
				interrupt_rasp();
			}
			break;
		case 0x29:
			sinistra(500.0);
			if(return_seg_enc() > 1500){
				data = 0x02;
				_delay_ms(10);
				data = 0x04;
				_delay_ms(500);
				interrupt_rasp();
			}
			break;
			
		case 0x30:
			sinistra(250.0);
			break;
		case 0x31:
			destra(250.0);
			break;
		case 0x32:
			destra(250.0);
			_delay_ms(15);
			stop_tutto();
			interrupt_rasp();
			break;
		case 0x33:
			sinistra(250.0);
			_delay_ms(15);
			stop_tutto();
			interrupt_rasp();
			break;
			
		
		case 0x36:
			//int i;
			for(i=0; i<5; i++){
				PORTF = 1;			//lampeggia led
				_delay_ms(450);
				PORTF = 0;				//lampeggia led
				_delay_ms(450);
			}
			cubetto();
			//avanti();
			//delay_movimento(1);
			interrupt_rasp();
			//controllo_movimento();
			break;
		
		default:
			stop_tutto();
			seg_enc_a_zero(0);
			break;
	}
}

/*
void controllo_telecamera(void){
	int i;
	if(data == 0x36){
		Serial_Send("croce\n");
		for(i=0; i<5; i++){
			PORTF = 1;			//lampeggia led
			_delay_ms(450);
			PORTF = 0;				//lampeggia led
			_delay_ms(450);
		}
		avanti();
		delay_movimento(1);
	}
	else;
}*/


void interrupt_rasp(){
	PORTF = 1<<PF1;
	_delay_ms(10);
	PORTF = 0;
}

int main(void)
{
	Serial_Init();
	
	//I2C
	I2C_init(0x10);
	I2C_setCallbacks(I2C_received, I2C_requested);
	
	//Finecorsa
	PCINT_Init();
	
	//Foto-resistenza
	//adc_init();
	
	//Servo
	//Set_Servo(150);
	//_delay_ms(3000);
	
	//Motori
    PID();
	PWM();
	
	
	
	//Led riconoscimento
	DDRF = 3;
	_delay_ms(2000);
	interrupt_rasp();
	
    while (1) 
    {
		//main1:
		controllo_movimento();
		
		//data = data_return(0);
		//.
		//controllo_telecamera();
		
		
// 		if(data == 0x36){
// 			delay_movimento(1);
// 			_delay_ms(2000);
// 			data = 0;
// 		}
		
    }
}

volatile uint8_t n = 0x0F;	//0xFF
volatile int s=0;
volatile int d=0;
volatile int c=0;
volatile int N=0;
volatile int a=0;



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
// 			stop_tutto();
// 			_delay_ms(50)
// 			avanti();
// 			_delay_ms(100);
// 			seg_enc_a_zero(0);
// 			stop_tutto();
// 			_delay_ms(1000);
// 			//data_return(0);
// 			interrupt_rasp();
			 fn = 1;
			n = 0xFF;
		}
		else{
			if(s==1){
// 				stop_tutto();
// 				_delay_ms(1000);
// 				indietro();
// 				_delay_ms(100);
// 				stop_tutto();
// 				seg_enc_a_zero(0);
// 				_delay_ms(1000);
// 				//data_return(0);
// 				interrupt_rasp();
				n = 0xFF;
				fn = 2;
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
// 			stop_tutto();
// 			_delay_ms(1000);
// 			indietro();
// 			_delay_ms(100);
// 			stop_tutto();
// 			seg_enc_a_zero(0);
// 			_delay_ms(1000);
// 			//data_return(0);
// 			interrupt_rasp();
			n = 0xFF;
			fn = 2;
		}
		else{
			if(d==1){
// 				stop_tutto();
// 				_delay_ms(1000);
// 				avanti();
// 				_delay_ms(100);
// 				stop_tutto();
// 				seg_enc_a_zero(0);
// 				_delay_ms(1000);
// 				//data_return(0);
// 				interrupt_rasp();
				n = 0xFF;
				fn = 1;
			}
			else if(d>1){
				d=0;
				//n=0xFF;
			}
		}
	}
}