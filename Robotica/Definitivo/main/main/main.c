#define  F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Pid_PWM.h"
#include "I2C.h"
#include "serial.h"

volatile int data = 0;
void I2C_received(uint8_t received_data)
{
	data = received_data;
}

void I2C_requested()
{
	I2C_transmitByte(data);
}

int main(void)
{
	I2C_init(0x10);
	I2C_setCallbacks(I2C_received, I2C_requested);
	_delay_ms(1500);
    //PID();
	//PWM();
	_delay_ms(2000);
    while (1) 
    {
		//avanti();
		switch(data){
			//Laser
			case 0x11:			//davanti
				//laser1();
				stop_tutto();
				_delay_ms(1500);
				sinistra();
			break;
			case 0x12:			//destro davanti
				/*stop_tutto();
				Set_Velocita(200.0);
				_delay_ms(500);
				indietro();
				_delay_ms(450);
				sinistra();
				_delay_ms(100);
				stop_tutto();
				_delay_ms(500);
				avanti();*/
				laser2();
			break;
			case 0x13:			//destro dietro
				stop_tutto();
				Set_Velocita(200);
				_delay_ms(500);
				avanti();
			break;
			case 0x14:			//sinitra davanti
				stop_tutto();
			break;
			case 0x15:			//sinistra dietro
				stop_tutto();
			break;
			case 0x16:			//dietro
				stop_tutto();
				_delay_ms(1000);
				sinistra();
			break;
			
			
			//Giroscopio
			case 0x27:				//sinistra
				stop_tutto();
				_delay_ms(1000);
				if(data != 0x11){
					avanti();
				}else{
					destra();
				}
			break;
			case 0x26:				//destra
				stop_tutto();
				_delay_ms(1000);
				if(data != 0x11){
					avanti();
				}else{
					sinistra();
				}
			break;
			case 0x25:				//avanti
				stop_tutto();
				_delay_ms(1000);
				if( data != 0x11){
					avanti();
				} else{
					indietro();
				}
			break;
			//Telecamere
			case 0x20:
				_delay_ms(750);
				stop_tutto();
			break;
			//altro?
		}
    }
}

