#define F_CPU 16000000UL
#define KTIMER_256 1000000UL/62500UL //durata tick timer con N = 256 --> 64us
#define K_ENCODER1 990UL //n� impulsi in un giro Motore 1
#define K_ENCODER2 620UL //n� impulsu Motore 2
#define K_ENCODER3 990UL //Motore 3
#define K_ENCODER4 990UL //MOtore 4

#define SET_POINT_VELOCITA 1500.0
#define SET_START_PWM 1000

#define I_MAX 1023
#define I_MIN -0
#define KI 0.457//0.152// da settare 
#define KP 4.28//da settare
#define KD 0.0//da settare

#include <avr/io.h>
#include <time.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include "serial.h"
#include "I2C.h"


volatile double e_1=0, e_old_1;//errore pid1
volatile double p_1=0, i_1=0, d_1=0, i_old_1=0, pid1=0;//variabili pid1
 
 volatile double e_2=0, e_old_2;//errore pid2
 volatile double p_2=0, i_2=0, d_2=0, i_old_2=0, pid2=0;//variabili pid2

volatile double e_3=0, e_old_3;//errore pid3
volatile double p_3=0, i_3=0, d_3=0, i_old_3=0, pid3=0;//variabili pid3

volatile double e_4=0, e_old_4;//errore pid4
volatile double p_4=0, i_4=0, d_4=0, i_old_4=0, pid4=0;//variabili pid4

 
volatile int t_1=0, t_old_1=0;
volatile int deltat_1, deltat_old_1;
volatile int velocita_1;

volatile int t_2=0, t_old_2=0;
volatile int deltat_2, deltat_old_2;
volatile int velocita_2;

volatile int t_3=0, t_old_3=0;
volatile int deltat_3, deltat_old_3;
volatile int velocita_3;

volatile int t_4=0, t_old_4=0;
volatile int deltat_4, deltat_old_4;
volatile int velocita_4;


volatile int seg_enc=0;

#define DATA_LENGHT 5
volatile int data = 0;//[DATA_LENGHT];
volatile int k=0;
void I2C_received(uint8_t received_data)
{
	data = received_data;
	/*if(k==6){
		k=0;
		data [k] = received_data;
		}else{
		data[k] = received_data;
		k++;
	}*/
}

void I2C_requested()
{
	I2C_transmitByte(data);
}



ISR(INT2_vect){//interrupt encoder1
	t_1 = (int32_t) TCNT5;
	deltat_1 = (t_1-t_old_1) * KTIMER_256;
	velocita_1 = 1000000000UL/(K_ENCODER1 * deltat_1); //giri al secondo
	t_old_1 = t_1;
	/*seg_enc++;
	Serial_Send(seg_enc); Serial_Send("\n");*/
}

ISR(INT3_vect){//interrupt encoder2
	t_2 = (int32_t) TCNT5;
	deltat_2 = (t_2-t_old_2) * KTIMER_256;
	velocita_2 = 1000000000UL/(K_ENCODER2 * deltat_2); //giri al secondo
	t_old_2 = t_2;
}

ISR(INT4_vect){//interrupt encoder3
	t_3 = (int32_t) TCNT5;
	deltat_3 = (t_3-t_old_3) * KTIMER_256;
	velocita_3 = 1000000000UL/(K_ENCODER3 * deltat_3); //giri al secondo
	t_old_3 = t_3;
	/*seg_enc++;
	Serial_Send(seg_enc); Serial_Send("\n");*/
}

ISR(INT5_vect){//interrupt encoder4
	t_4 = (int32_t) TCNT5;
	deltat_4 = (t_4-t_old_4) * KTIMER_256;
	velocita_4 = 1000000000UL/(K_ENCODER4 * deltat_4); //giri al secondo
	t_old_4 = t_4;
	/*seg_enc++;
	Serial_Send(seg_enc); Serial_Send("\n");*/
}

void Init_PWM123(){//Motore 1/2/3
	TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<COM1C1) | (1<<WGM11) | (1<<WGM10);
	DDRB = (1<<PB5) | (1<<PB6) | (1<<PB7); //| (1<<PB4);//Uscita PWM motori e Servo
}

void Init_PWM4(){//Motore 4
	TCCR3A = (1<<COM3A1) | (1<<WGM31) | (1<<WGM30);
	DDRE = (1<<PE3);
}

/*void Init_PWM_Servo(){
	TCCR2A = (1<<COM2A1) | (1<<WGM21) | (1<<WGM20);
	//ddrb � nel init pwm123
}*/

void Start_PWM123(){//Motore 1/2/3
	TCCR1B = (1<<WGM12) | (1<<CS12);
}

void Start_PWM4(){//Motore 4
	TCCR3B = (1<<WGM32) | (1<<CS32);
}

/*void Start_PWM_Servo(){//	Servo
	TCCR2B = (0<<FOC2A) | (0<<FOC2B) | (1<<CS22) | (1<<CS20);
}*/

void Set_PWM1(int duty_1){//Motore 1
	OCR1A = duty_1;
}

void Set_PWM2(int duty_2){//Motore 2
	OCR1B = duty_2;
}

void Set_PWM3(int duty_3){//Motore 3
	OCR1C = duty_3;
}

void Set_PWM4(int duty_4){//Motore 4
	OCR3A = duty_4;
}

void Set_Servo(int duty_5){
	OCR4B = duty_5;
}

void PID1(){//pid motore 1
	e_1 = (double) (SET_POINT_VELOCITA - velocita_1);
	p_1 = e_1*KP;
	i_1= i_old_1 + (KI*e_1);
	d_1 = (e_1 - e_old_1) * KD;
	if(i_1>I_MAX){
		i_1 = I_MAX;
	}
	if(i_1<I_MIN){
		i_1=I_MIN;
	}
	i_old_1 = i_1;
	e_old_1 = e_1;
	pid1 = p_1+i_1+d_1;
	if(pid1<0){
		pid1 = 0;
	}
	if(pid1>1023){
		pid1 = 1023;
	}
	Set_PWM1((int) pid1);
}

void PID2(){//pid motore 2
	e_2 = (double) (SET_POINT_VELOCITA - velocita_2);
	p_2 = e_2*KP;
	i_2= i_old_2 + (KI*e_2);
	d_2 = (e_2 - e_old_2) * KD;
	if(i_2>I_MAX){
		i_2 = I_MAX;
	}
	if(i_2<I_MIN){
		i_2=I_MIN;
	}
	i_old_2 = i_2;
	e_old_2 = e_2;
	pid2 = p_2+i_2+d_2;
	if(pid2<0){
		pid2 = 0;
	}
	if(pid2>1023){
		pid2 = 1023;
	}
	Set_PWM2((int) pid2);
}

void PID3(){//pid motore 3
	e_3= (double) (SET_POINT_VELOCITA - velocita_3);
	p_3 = e_3*KP;
	i_3= i_old_3 + (KI*e_3);
	d_3 = (e_3 - e_old_3) * KD;
	if(i_3>I_MAX){
		i_3 = I_MAX;
	}
	if(i_3<I_MIN){
		i_3=I_MIN;
	}
	i_old_3 = i_3;
	e_old_3 = e_3;
	pid3 = p_3+i_3+d_3;
	if(pid3<0){
		pid3 = 0;
	}
	if(pid3>1023){
		pid3 = 1023;
	}
	Set_PWM3((int) pid3);
}

void PID4(){//pid motore 4
	e_4 = (double) (SET_POINT_VELOCITA - velocita_4);
	p_4 = e_4*KP;
	i_4= i_old_4 + (KI*e_4);
	d_4 = (e_4 - e_old_4) * KD;
	if(i_4>I_MAX){
		i_4 = I_MAX;
	}
	if(i_4<I_MIN){
		i_4=I_MIN;
	}
	i_old_4 = i_4;
	e_old_4 = e_4;
	pid4 = p_4+i_4+d_4;
	if(pid4<0){
		pid4 = 0;
	}
	if(pid4>1023){
		pid4 = 1023;
	}
	Set_PWM4((int) pid4);
}

void fermo(){
	OCR1A = 0;
	OCR1B = 0;
	OCR1C = 0;
	OCR3A = 0;
}

void sinistra(){
	PORTA = (1<<PA0) | (1<<PA2) | (1<<PA4) | (1<<PA6);
	Set_PWM1(SET_POINT_VELOCITA);
	Set_PWM2(SET_POINT_VELOCITA);
	Set_PWM3(SET_POINT_VELOCITA);
	Set_PWM4(SET_POINT_VELOCITA);
}

void avanti(){
	PORTA = (1<<PA0) | (1<<PA3) | (1<<PA4)| (1<<PA7);//Verso motore
	Set_PWM1(SET_START_PWM);
	Set_PWM2(SET_START_PWM);
	Set_PWM3(SET_START_PWM);
	Set_PWM4(SET_START_PWM);
}

ISR(TIMER4_COMPA_vect){
	PID1();
	PID2();
	PID3();
	PID4();
	TCNT4 = 0;
}



int main(void)
{
	Serial_Init();
	//PID
	//_delay_ms(1500);
	EICRA = (1<<ISC20) | (1<<ISC21) | (1<<ISC30) | (1<<ISC31);//interrupt rising edge
	EICRB = (1<<ISC40) | (1<<ISC41) | (1<<ISC50) | (1<<ISC51);
	EIMSK = 1<<INT2 | 1<<INT3 | 1<<INT4 |1<<INT5;//attivamento interrupt esterni
	sei();//attivamento ogni interrupt
	
	DDRA = 0xFF;
	PORTA = (1<<PA0) | (1<<PA3) | (1<<PA4)| (1<<PA7);//Verso motore
	
	TCCR5A = 0;
	TCCR5B = 1<<CS52;
	
	//fa partire interrupt ogni millisecondo --> attiva pid
// 	TCCR4A = 0;
// 	TCCR4B = (1<<WGM42) | (1<<CS41) | (1<<CS40); //f = 250KHz
// 	TIMSK4=1<<OCIE4A;
// 	OCR4A = 250;
	
	TCCR4A = (1<<COM1B1) | (1<<WGM41) | (1<<WGM40);//inizializza il servo
	TCCR4B = (1<<WGM42) | (1<<CS42);//N=256
	TIMSK4 = 1<<OCIE4A;
	OCR4A = 62;
	DDRH = 1<<PH4;
	
	Init_PWM123();
	Init_PWM4();
	//Init_PWM_Servo();
	
	Start_PWM123();
	Start_PWM4();
	//Start_PWM_Servo();
	
	Set_PWM1(SET_START_PWM);
	Set_PWM2(SET_START_PWM);
	Set_PWM3(SET_START_PWM);
	Set_PWM4(SET_START_PWM);
	//Set_Servo(100);
	
	//ADC
	//int k;
	/*ADMUX = (1<<REFS0);
	ADCSRB = 0;
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);*/
	
	
	int i;
	/*init_interrupt();
	init_TWI();*/
	
	
	I2C_init(0x10);
	I2C_setCallbacks(I2C_received, I2C_requested);
	
	
	/*int i;
	for (i=0; i<DATA_LENGHT; i++){
		data[i] = 0;
	}*/
	
	
	DDRF = 1;
    while (1) 
    {
		//Serial_Send("e= "); Serial_Send(e); Serial_Send("\n");
		//Serial_Send("p= "); Serial_Send(p); Serial_Send("\n");
		//Serial_Send("i= "); Serial_Send(i); Serial_Send("\n");
		//Serial_Send("d= "); Serial_Send(d); Serial_Send("\n");
		//Serial_Send("pid= "); Serial_Send(pid1); Serial_Send("\n");
		/*Serial_Send("velocita1= "); Serial_Send(velocita_1); Serial_Send("\n");
		Serial_Send("velocita2= "); Serial_Send(velocita_2); Serial_Send("\n");
		Serial_Send("velocita3= "); Serial_Send(velocita_3); Serial_Send("\n");
		Serial_Send("velocita4= "); Serial_Send(velocita_4); Serial_Send("\n");
		Serial_Send("\n");*/
		//_delay_ms(250);
		//Serial_Send("OCR3A= "); Serial_Send(OCR3A); Serial_Send("\n");
		//Serial_Send("deltat= "); Serial_Send(deltat_4); Serial_Send("\n");
		//_delay_ms(500);
		
		/*ADCSRA = ADCSRA | (1<<ADSC);
		while((ADCSRA&(1<<ADSC))!=0);
		k = ADC;
		Serial_Send(k);
		Serial_Send("\n");*/
		
		
		//Serial_Send(OCR3A); Serial_Send("\n");
		
		
		/*Set_Servo(30);
		_delay_ms(1500);
		Set_Servo(200);
		_delay_ms(1500);*/
		
		
		//s = conversione();
		//Serial_Send(s); Serial_Send("\n");
		
		//init_TWI();
		/*avanti();
		_delay_ms(1000);
		fermo();
		_delay_ms(1000);*/
		//init_interrupt();
		/*init_TWI();
		do{
			s = conversione();
			Serial_Send(s); Serial_Send("\n");
		} while ((s != 0x27));
		Serial_Send("fermo dopo giro");
		_delay_ms(10000);
		TWI_Stop();*/
		
		
		/*_delay_ms(2500);
		fermo();
		_delay_ms(1000);
		sinistra();
		while(data != 2);
		fermo();
		_delay_ms(1000);
		avanti();*/
		
		
		/*s = Serial_Rx();
		if(s == 10){
			PORTF = 1;
		}*/
		
		
/*
		int i;
		for(i=0; i<5; i++){
			Serial_Send(data[i]); Serial_Send("\t\t");
		}
		Serial_Send("\n");
*/
		_delay_ms(1000);
		
		Serial_Send(data);/ *Serial_Send("\t\t");Serial_Send(data);* /Serial_Send("\n");
		if(data == 0x50){
			fermo();
			for(i=0; i<5; i++){
				PORTF = 1;
				_delay_ms(500);
				PORTF = 0;
				_delay_ms(500);
			}
			avanti();
		}
		
		
		if(data == 0x11){
			fermo();
			_delay_ms(1000);
			do{
				sinistra();
			}while(data != 0x27);
			avanti();
		}
		
		
    }
}

