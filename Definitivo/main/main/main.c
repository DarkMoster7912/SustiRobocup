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
void I2C_received(uint8_t received_data)		//ricevo da master
{
	data = received_data;
}

void I2C_requested()						//trasmetto a master
{
	I2C_transmitByte(data);
}

volatile int data_ret = 0;
volatile int seg_enc=0;

void delay_movimento( int c){
	if(c==1){			//avanti
		do{
			seg_enc = return_seg_enc();
		}while ((seg_enc != 1300) && (data!=0x34));
		if(data == 0x34){
			indietro();
			_delay_ms(15);
			stop_tutto();
			_delay_ms(300);
			interrupt_rasp();
			seg_enc_a_zero(0);
			seg_enc=0;
			Serial_Send("Nero");
		}
		else{
			indietro();
			_delay_ms(15);
			stop_tutto();
			_delay_ms(300);
			interrupt_rasp();
			seg_enc_a_zero(0);
			seg_enc=0;
		}
		
	}
	else if(c==2){		//stop da girata sinistra
		//stop_tutto();
		destra();
		_delay_ms(15);
		stop_tutto();
		_delay_ms(300);
		interrupt_rasp();
		seg_enc_a_zero(0);
		seg_enc=0;
	}
	else if(c==3){		//stop da girata destra
		//stop_tutto();
		sinistra_lento();
		_delay_ms(15);
		stop_tutto();
		_delay_ms(300);
		interrupt_rasp();
		seg_enc_a_zero(0);
		seg_enc=0;
	}
}

void controllo_movimento(){
	switch(data){
		
		//Fermo Da controllo avanti e girata
		case 0x01:
			delay_movimento(2);		//da sinistra
			seg_enc_a_zero(0);
			break;
		case 0x02:
			avanti();
			delay_movimento(1);		//avanti
			/*_delay_ms(700);		//800
			stop_tutto();*/
			_delay_ms(500);
			break;
		case 0x03:
			delay_movimento(3);		//da destra
			data_ret = 0x04;
			break;
		case 0x04:
			stop_tutto();		//fermo
			seg_enc_a_zero(0);
			break;
		
		
		//Giroscopio
		case 0x27:				//sinistra
			sinistra();
			break;
		case 0x26:				//destra
			destra();
			break;
		case 0x28:
			destra_lento();
			break;
		case 0x29:
			sinistra_lento();
			break;
		
		default:
			stop_tutto();
			seg_enc_a_zero(0);
			break;
	}
}

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
	}
	else;
}


void interrupt_rasp(){
	PORTF = 2;
	_delay_ms(10);
	PORTF = 0;
}

int main(void)
{
	Serial_Init();
	
	//I2C
	I2C_init(0x10);
	I2C_setCallbacks(I2C_received, I2C_requested);
	
	
	//Foto-resistenza
	//adc_init();
	
	//Servo
	//Set_Servo(150);
	//_delay_ms(3000);
	
	//Motori
    PID();
	PWM();
	
	//Finecorsa
	PCINT_Init();
	
	//Led riconoscimento
	DDRF = 3;
	
	
    while (1) 
    {
		
		controllo_movimento();
		
		//data = return_data();
		controllo_telecamera();
		
		
// 		if(data == 0x36){
// 			delay_movimento(1);
// 			_delay_ms(2000);
// 			data = 0;
// 		}
		
    }
}





