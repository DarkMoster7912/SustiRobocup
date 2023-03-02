#define  F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Pid_PWM.h"
#include "I2C.h"
#include "serial.h"
#include "Controllo.h"

volatile int data = 0;
void I2C_received(uint8_t received_data)		//ricevo da master
{
	data = received_data;
}

void I2C_requested()						//trasmetto a master
{
	I2C_transmitByte(data);
}

int main(void)
{
	Serial_Init();
	I2C_init(0x10);
	I2C_setCallbacks(I2C_received, I2C_requested);
	_delay_ms(3000);
	
	//Set_Servo(30);
    PID();
	PWM();
	/*_delay_ms(500);
	avanti();
	_delay_ms(1500);
	stop_tutto();
	_delay_ms(1000);*/
    while (1) 
    {
		controllo(data);
    }
}

