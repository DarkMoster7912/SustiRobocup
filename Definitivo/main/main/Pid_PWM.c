#define F_CPU 16000000UL
#define KTIMER_256 1000000UL/62500UL //durata tick timer con N = 256 --> 64us
#define K_ENCODER1 1000UL //n? impulsi in un giro Motore 1
#define K_ENCODER2 460UL //n? impulsu Motore 2
#define K_ENCODER3 630UL //Motore 3
#define K_ENCODER4 640UL //MOtore 4

//#define SET_POINT_VELOCITA 1500.0
#define SET_INT_PWM 500
#define SET_START_PWM 500

#define I_MAX 1023
#define I_MIN -0
#define KI 0.000957//0.120//0.457// da settare
#define KP 62.95//4.07//4.28//da settare
#define KD 0.000000256//da settare

#include <avr/io.h>
#include <time.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "serial.h"
#include "Pid_PWM.h"
#include "Controllo.h"



//Motori
volatile double SET_POINT_VELOCITA = 500.0;
volatile double SET_POINT_VELOCITA_1 = 500.0;

void Set_Velocita(double k){
	SET_POINT_VELOCITA = k;
}


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


volatile int seg_enc_0 = 0;


int return_seg_enc(){
	return seg_enc_0;
}

void seg_enc_a_zero(int c){
	seg_enc_0 = c;
}

ISR(INT2_vect){//interrupt encoder1
	t_1 = (int32_t) TCNT5;
	deltat_1 = (t_1-t_old_1) * KTIMER_256;
	velocita_1 = 1000000000UL/(K_ENCODER1 * deltat_1); //giri al secondo
	t_old_1 = t_1;
	//seg_enc_0++;
	//Serial_Send(seg_enc_0); Serial_Send("\n");
}

ISR(INT3_vect){//interrupt encoder2
	t_2 = (int32_t) TCNT5;
	deltat_2 = (t_2-t_old_2) * KTIMER_256;
	velocita_2 = 1000000000UL/(K_ENCODER2 * deltat_2); //giri al secondo
	t_old_2 = t_2;
	seg_enc_0++;
	Serial_Send(seg_enc_0); Serial_Send("\n");
}

ISR(INT4_vect){//interrupt encoder3
	t_3 = (int32_t) TCNT5;
	deltat_3 = (t_3-t_old_3) * KTIMER_256;
	velocita_3 = 1000000000UL/(K_ENCODER3 * deltat_3); //giri al secondo
	t_old_3 = t_3;
	//seg_enc_0++;
	//Serial_Send(seg_enc_0); Serial_Send("\n");
}

ISR(INT5_vect){//interrupt encoder4
	t_4 = (int32_t) TCNT5;
	deltat_4 = (t_4-t_old_4) * KTIMER_256;
	velocita_4 = 1000000000UL/(K_ENCODER4 * deltat_4); //giri al secondo
	t_old_4 = t_4;
	//seg_enc_0++;
	//Serial_Send(seg_enc_0); Serial_Send("\n");
}

void Init_PWM123(){//Motore 1/2/3
	TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<COM1C1) | (1<<WGM11) | (1<<WGM10);
	DDRB = (1<<PB5) | (1<<PB6) | (1<<PB7); //| (1<<PB4);//Uscita PWM motori e Servo
}

void Init_PWM4(){//Motore 4
	TCCR3A = (1<<COM3A1) | (1<<WGM31) | (1<<WGM30);
	DDRE = (1<<PE3);
}

void Start_PWM123(){//Motore 1/2/3
	TCCR1B = (1<<WGM12) | (1<<CS12);
}

void Start_PWM4(){//Motore 4
	TCCR3B = (1<<WGM32) | (1<<CS32);
}

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
	Set_PWM4((int) pid1);
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
	Set_PWM1((int) pid3);
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
	Set_PWM3((int) pid4);
}

ISR(TIMER4_COMPA_vect){
	PID1();
	PID2();
	PID3();
	PID4();
	TCNT4 = 0;
}

void PWM(){
	
	TCCR5A = 0;
	TCCR5B = 1<<CS52;
	
	DDRH = 1<<PH4;
	DDRA = 0xFF;
	DDRF = 3;
	DDRC = (1<<PC6) | (1<<PC7);
	PORTC = 0;
	
	Init_PWM123();
	Init_PWM4();
	
	Start_PWM123();
	Start_PWM4();
	
	Set_PWM1(SET_INT_PWM);
	Set_PWM2(SET_INT_PWM);
	Set_PWM3(SET_INT_PWM);
	Set_PWM4(SET_INT_PWM);
}

void PID(){
	TCCR4A = (1<<COM1B1) | (1<<WGM41) | (1<<WGM40);//inizializza il servo
	TCCR4B = (1<<WGM42) | (1<<CS42);//N=256
	TIMSK4 = 1<<OCIE4A;
	OCR4A = 125;
	
	EICRA = (1<<ISC20) | (1<<ISC21) | (1<<ISC30) | (1<<ISC31);//interrupt rising edge
	EICRB = (1<<ISC40) | (1<<ISC41) | (1<<ISC50) | (1<<ISC51);
	EIMSK = 1<<INT2 | 1<<INT3 | 1<<INT4 |1<<INT5;//attivamento interrupt esterni
	sei();//attivamento ogni interrupt
}

void avanti(){
	SET_POINT_VELOCITA = 800.0;
	//PORTA = (1<<PA1) | (1<<PA2) | (1<<PA5)| (1<<PA6);//Verso motore
	
	PORTA = (1<<PA1) | (1<<PA2) | (1<<PA4)| (1<<PA7);//Verso motore
	//PA2-PA3 Motore 4		PA2 AVANTI
	//PA6-PA7 Motore 3		PA6 AVANTI
	//PA0-PA1 Motore 2		PA1 AVANTI
	//PA4-PA5 Motore 1		PA4 AVANTI
	//Set_PWM4(SET_POINT_VELOCITA);
	//_delay_ms(50);
	
	//Set_PWM1(SET_POINT_VELOCITA);
	//Set_PWM2(SET_POINT_VELOCITA);
	//Set_PWM3(SET_POINT_VELOCITA);
	
}

void indietro(){
	SET_POINT_VELOCITA = 500.0;
	PORTA = (1<<PA0) | (1<<PA3) | (1<<PA5)| (1<<PA6);//Verso motore
	//Set_PWM1(SET_POINT_VELOCITA);
	//Set_PWM2(SET_POINT_VELOCITA);
	//Set_PWM3(SET_POINT_VELOCITA);
	//Set_PWM4(SET_POINT_VELOCITA);
}

void destra( double v){
	SET_POINT_VELOCITA = v;
	//SET_POINT_VELOCITA_1 = 300.0;
	PORTA = (1<<PA4) | (1<<PA7) | (1<<PA3)| (1<<PA0);
	//Set_PWM1(SET_POINT_VELOCITA);
	//Set_PWM2(SET_POINT_VELOCITA_1);
	//Set_PWM3(SET_POINT_VELOCITA);
	//Set_PWM4(SET_POINT_VELOCITA_1);
}

// void destra_lento(){
// 	SET_POINT_VELOCITA = 500.0;
// 	//SET_POINT_VELOCITA_1 = 175.0;
// 	PORTA = (1<<PA4) | (1<<PA7) | (1<<PA3)| (1<<PA0);
// 	Set_PWM1(SET_POINT_VELOCITA);
// 	Set_PWM2(SET_POINT_VELOCITA_1);
// 	Set_PWM3(SET_POINT_VELOCITA);
// 	Set_PWM4(SET_POINT_VELOCITA_1);
//}

void sinistra(double v){
	SET_POINT_VELOCITA = v;
	//SET_POINT_VELOCITA_1 = 300.0;
	PORTA = (1<<PA5) | (1<<PA6) | (1<<PA2)| (1<<PA1);
// 	Set_PWM1(SET_POINT_VELOCITA_1);
// 	Set_PWM2(SET_POINT_VELOCITA);
// 	Set_PWM3(SET_POINT_VELOCITA_1);
// 	Set_PWM4(SET_POINT_VELOCITA);
}

// void sinistra_lento(){
// 	SET_POINT_VELOCITA = 500.0;
// 	//SET_POINT_VELOCITA_1 = 175.0;
// 	PORTA = (1<<PA5) | (1<<PA6) | (1<<PA2)| (1<<PA1);
// 	//Set_PWM1(SET_POINT_VELOCITA_1);
// 	//Set_PWM2(SET_POINT_VELOCITA);
// 	//Set_PWM3(SET_POINT_VELOCITA_1);
// 	//Set_PWM4(SET_POINT_VELOCITA);
//}

void stop_tutto(){
	SET_POINT_VELOCITA = 0;
	//Set_PWM2(SET_POINT_VELOCITA);
	//Set_PWM3(SET_POINT_VELOCITA);
	
	//_delay_ms(250);
	//Set_PWM1(SET_POINT_VELOCITA);
	//Set_PWM4(SET_POINT_VELOCITA);
}



//Servo
void Set_Servo(int duty_5){
	OCR4B = duty_5;
}

volatile int gir=0;
volatile int cub = 0;
ISR(PCINT1_vect){
	gir++;
}



void cubetto(){
	PORTC = 1<<PC6;
	while(gir!=3000);
	PORTC = 0;	//		PWM--> 30 : 200
	cub++;
}

void reset_cub(){
	PORTC = 1<<PC7;
	while((gir!=(3000*cub)));
	PORTC = 0;
	cub=0;
}