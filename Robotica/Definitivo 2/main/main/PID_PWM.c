#define F_CPU 16000000UL
#define KTIMER_256 1000000UL/62500UL //durata tick timer con N = 256 --> 64us
#define K_ENCODER1 990UL //n? impulsi in un giro Motore 1
#define K_ENCODER2 990UL //n? impulsu Motore 2
#define K_ENCODER3 990UL //Motore 3
#define K_ENCODER4 990UL //MOtore 4

#define SET_POINT_VELOCITA 1500.0
#define SET_INIT_PWM 1000.0

#define I_MAX 1023
#define I_MIN -0
#define KI 1// da settare
#define KP 6//da settare
#define KD 1//da settare

#define n_mot 4


#include <avr/io.h>
#include <time.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "serial.h"


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

volatile int driver[n_mot][4];

//volatile int seg_enc=0;


ISR(INT0_vect){//interrupt encoder1
	t_1 = (int32_t) TCNT4;
	deltat_1 = (t_1-t_old_1) * KTIMER_256;
	velocita_1 = 1000000000UL/(K_ENCODER1 * deltat_1); //giri al secondo
	t_old_1 = t_1;
	/*seg_enc++;
	Serial_Send(seg_enc); Serial_Send("\n");*/
}

ISR(INT1_vect){//interrupt encoder2
	t_2 = (int32_t) TCNT4;
	deltat_2 = (t_2-t_old_2) * KTIMER_256;
	velocita_2 = 1000000000UL/(K_ENCODER2 * deltat_2); //giri al secondo
	t_old_2 = t_2;
}

ISR(INT2_vect){//interrupt encoder3
	t_3 = (int32_t) TCNT4;
	deltat_3 = (t_3-t_old_3) * KTIMER_256;
	velocita_3 = 1000000000UL/(K_ENCODER3 * deltat_3); //giri al secondo
	t_old_3 = t_3;
	/*seg_enc++;
	Serial_Send(seg_enc); Serial_Send("\n");*/
}

ISR(INT3_vect){//interrupt encoder4
	t_4 = (int32_t) TCNT4;
	deltat_4 = (t_4-t_old_4) * KTIMER_256;
	velocita_4 = 1000000000UL/(K_ENCODER4 * deltat_4); //giri al secondo
	t_old_4 = t_4;
	/*seg_enc++;
	Serial_Send(seg_enc); Serial_Send("\n");*/
}

void Set_PWM1(int duty_1){//Motore 1
	OCR1A = duty_1;
}

void Set_PWM2(int duty_2){//Motore 2
	OCR1B = duty_2;
}

void Set_PWM3(int duty_3){//Motore 3
	OCR3A = duty_3;
}

void Set_PWM4(int duty_4){//Motore 4
	OCR3B = duty_4;
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

ISR(TIMER0_COMPA_vect){
	PID1();
	PID2();
	PID3();
	PID4();
	driver_set();
	TCNT0 = 0;
}

void Init_Start_PID(){
	TCCR4A |= 0x00;
	TCCR4B |= (1<<CS41);
	TCCR4C |= 0x00;
	
	TCCR0A = (1<<COM0A0)|(0<<COM0A1);
	TCCR0B = (1<<CS02)|(0<<CS01)|(0<<CS00);
	TIMSK0 = (1<<OCIE0A);
	OCR0A = 156;
}

void Init_PWM(){
	DDRA = 0xF8;		//Motori A --> A1 = avanti destro, A2 = dietro destro
	TCCR1A |= (1<<WGM10)|(1<<WGM11);		//pwm 10 bit phase corrected con t/c1 e t/c3
	TCCR1B |= (0<<WGM12);
	TCCR1A |= (0<<COM1A0)|(1<<COM1A1);
	TCCR1A |= (0<<COM1B0)|(1<<COM1B1);
	
	DDRJ = 0x8F;		//Motori B --> B1 = avanti sinistro, B2 = dietro sinistro
	TCCR3A |= (1<<WGM30)|(1<<WGM31);		//pwm 10 bit phase corrected con t/c1 e t/c3
	TCCR3B |= (0<<WGM32);
	TCCR3A |= (0<<COM3A0)|(1<<COM3A1);
	TCCR3A |= (0<<COM3B0)|(1<<COM3B1);
}

void Start_PWM(){		//prescaler /1 per t/c1 e t/c3 + enable driver
	PORTA |= 0x20;
	PORTJ |= 0x04;

	TCCR1B |= (0<<CS12)|(0<<CS11)|(1<<CS10);
	TCCR3B |= (0<<CS32)|(0<<CS31)|(1<<CS30);
	
	Set_PWM1(SET_INIT_PWM);
	Set_PWM2(SET_INIT_PWM);
	Set_PWM3(SET_INIT_PWM);
	Set_PWM4(SET_INIT_PWM);
}

void Init_Interrupt(){
	EICRA = 0xFF;
	EIMSK = (1<<INT0)|(1<<INT1)|(1<<INT2)|(1<<INT3);
	sei();
}

void driver_set(){

	if(driver[0][0]<0){
		PORTJ &= 0b111;
		PORTJ |= 0b10000;
		OCR1B = (driver[0][0] * -1);
		}else{
		PORTJ &= 0b111;
		PORTJ |= 0b1000;
		OCR1B = driver[0][0];
	}
	
	if(driver[1][0]<0){
		PORTJ &= 0b11100;
		PORTJ |= 0b1;
		OCR1A = (driver[1][0] * -1);
		}else{
		PORTJ &= 0b11100;
		PORTJ |= 0b10;
		OCR1A = driver[1][0];
	}
	
	if(driver[2][0]<0){
		PORTA &= 0b11100000;
		PORTA |= 0b01000;
		OCR3B = (driver[2][0] * -1);
		}else{
		PORTA &= 0b11100000;
		PORTA |= 0b10000;
		OCR3B = driver[2][0];
	}
	
	if(driver[3][0]<0){
		PORTA &= 0b111000;
		PORTA |= 0b10000000;
		OCR3A = (driver[3][0] * -1);
		}else{
		PORTA &= 0b111000;
		PORTA |= 0b01000000;
		OCR3A = driver[3][0];
	}

}