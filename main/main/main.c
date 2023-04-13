#define F_CPU 16000000UL
#define KTIMER_256 1000000UL/62500UL //durata tick timer con N = 256 --> 64us
#define K_ENCODER1 990UL //n� impulsi in un giro Motore 1
#define K_ENCODER2 990UL //n� impulsu Motore 2   620_UL
#define K_ENCODER3 990UL //Motore 3
#define K_ENCODER4 990UL //MOtore 4

volatile double SET_POINT_VELOCITA = 500.0;
#define SET_START_PWM 500

#define I_MAX 1023
#define I_MIN -0
#define KI 0.130//0.457// da settare
#define KP 4.16//4.28//da settare
#define KD 0.0246//da settare

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



ISR(INT2_vect){//interrupt encoder1		oc1b
	t_1 = (int32_t) TCNT5;
	deltat_1 = (t_1-t_old_1) * KTIMER_256;
	velocita_1 = 1000000000UL/(K_ENCODER1 * deltat_1); //giri al secondo
	t_old_1 = t_1;
	//seg_enc++;
	//Serial_Send(seg_enc); Serial_Send("\n");
}

ISR(INT3_vect){//interrupt encoder3		oc3a
	t_2 = (int32_t) TCNT5;
	deltat_2 = (t_2-t_old_2) * KTIMER_256;
	velocita_2 = 1000000000UL/(K_ENCODER2 * deltat_2); //giri al secondo
	t_old_2 = t_2;
	//seg_enc++;
	//Serial_Send(seg_enc); Serial_Send("\n");
}

ISR(INT4_vect){//interrupt encoder2		oc1c
	t_3 = (int32_t) TCNT5;
	deltat_3 = (t_3-t_old_3) * KTIMER_256;
	velocita_3 = 1000000000UL/(K_ENCODER3 * deltat_3); //giri al secondo
	t_old_3 = t_3;
	seg_enc++;
	Serial_Send(seg_enc); Serial_Send("\n");
}

ISR(INT5_vect){//interrupt encoder4		oc1a
	t_4 = (int32_t) TCNT5;
	deltat_4 = (t_4-t_old_4) * KTIMER_256;
	velocita_4 = 1000000000UL/(K_ENCODER4 * deltat_4); //giri al secondo
	t_old_4 = t_4;
	//seg_enc++;
	//Serial_Send(seg_enc); Serial_Send("\n");
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

void fermo(){
	SET_POINT_VELOCITA = 0;
	Set_PWM1(SET_POINT_VELOCITA);
	Set_PWM2(SET_POINT_VELOCITA);
	Set_PWM3(SET_POINT_VELOCITA);
	Set_PWM4(SET_POINT_VELOCITA);
	
}

void sinistra(){
	SET_POINT_VELOCITA = 150.0;
	PORTA = (1<<PA1) | (1<<PA3) | (1<<PA5)| (1<<PA7);
	Set_PWM1(SET_POINT_VELOCITA);
	Set_PWM2(SET_POINT_VELOCITA);
	Set_PWM3(SET_POINT_VELOCITA);
	Set_PWM4(SET_POINT_VELOCITA);
}

void avanti(){
	SET_POINT_VELOCITA = 600.0;
	PORTA = (1<<PA1) | (1<<PA2) | (1<<PA4)| (1<<PA6);//Verso motore
	Set_PWM4(SET_START_PWM);
	_delay_ms(200);
	Set_PWM1(SET_START_PWM);
	Set_PWM2(SET_START_PWM);
	Set_PWM3(SET_START_PWM);
}

ISR(TIMER4_COMPA_vect){
	PID1();		//PWM4
	PID2();		//PW2
	PID3();		//PW1
	PID4();		//PWM3
	//PIDC();
	TCNT4 = 0;
}


volatile uint8_t n = 0x0F;	//0xFF
volatile int s = 0, c=0;

volatile int tc=0, delta_tc=0, tc_old=0, velocita_c=0;
volatile double ec=0, ec_old;//errore pidc
volatile double pc=0, ic=0, dc=0, ic_old=0, pidc=0;//variabili pidc

volatile int set_point_cubetti = 500, num_cub=0;

ISR(PCINT1_vect){
	uint8_t changedbits;
	
	changedbits = PINJ ^ n;
	n = PINJ;
	if(changedbits & (1<<PJ1)){
		s++;
		if((s%2)==0){
			Serial_Send(s);
			Serial_Send("\n");
			tc = (int32_t) TCNT5;
			delta_tc = (t_1-tc_old) * KTIMER_256;
			velocita_c = 1000000000UL/(K_ENCODER1 * deltat_1); //giri al secondo		cambiare KENCODER
			tc_old = tc;
		}
		n=0x0F;
	}
}


void PIDC(){
	if(c==1){
		ec = (double) (set_point_cubetti - velocita_c);
		pc = ec*KP;
		ic= ic_old + (KI*ec);
		dc = (ec - ec_old) * KD;
		if(ic>I_MAX){
			ic = I_MAX;
		}
		if(ic<I_MIN){
			ic=I_MIN;
		}
		ic_old = ic;
		ec_old = ec;
		pidc = pc+ic+dc;
		if(pidc<0){
			pidc = 0;
		}
		if(pidc>1023){
			pidc = 1023;
		}
		if(pidc > 700){
			Set_Servo(0);
			num_cub++;
		}
		else{
			Set_Servo((int) pidc);
		}
	}
}

int main(void)
{
	Serial_Init();
	//PID
	//_delay_ms(1500);
	EICRA = (1<<ISC20) | (1<<ISC21) | (1<<ISC30) | (1<<ISC31);//interrupt rising edge
	EICRB = (1<<ISC40) | (1<<ISC41) | (1<<ISC50) | (1<<ISC51);
	EIMSK = 1<<INT2 | 1<<INT3 | 1<<INT4 |1<<INT5;//attivamento interrupt esterni
	PCICR = 1<<PCIE1;
	PCIFR = 1<<PCIF1;
	PCMSK1 = 1<<PCINT10;
	DDRJ = 0;
	PORTJ = 1<<PJ1;
	sei();//attivamento ogni interrupt
	
	DDRA = 0xFF;
	PORTA = (1<<PA1) | (1<<PA2) | (1<<PA4)| (1<<PA6);
	DDRC = (1<<PC6) | (1<<PC7);
	PORTC = 1<<PC6;
	DDRG = 1<<PG0;
	PORTG = 1<<PG0;
	//PA2-PA3 Motore 4		PA2 AVANTI
	//PA6-PA7 Motore 3		PA6 AVANTI
	//PA0-PA1 Motore 2		PA1 AVANTI
	//PA4-PA5 Motore 1		PA4 AVANTI
	
	TCCR5A = 0;
	TCCR5B = 1<<CS52;
	
	//fa partire interrupt ogni millisecondo --> attiva pid
// 	TCCR4A = 0;
// 	TCCR4B = (1<<WGM42) | (1<<CS41) | (1<<CS40); //f = 250KHz
// 	TIMSK4=1<<OCIE4A;
// 	OCR4A = 250;
	
	
	DDRH = 1<<PH4;
	PORTH = 1<<PH4;
	TCCR4A = (1<<COM1B1) | (1<<WGM41) | (1<<WGM40);//inizializza il servo
	TCCR4B = (1<<WGM42) | (1<<CS42);//N=256
	TIMSK4 = 1<<OCIE4A;
	OCR4A = 125;			//PID ogni 2 ms
	
	
	/*Set_Servo(150);
	_delay_ms(1500);
	Set_Servo(50);
	_delay_ms(1500);*/
	
	Init_PWM123();
	Init_PWM4();
	//Init_PWM_Servo();
	_delay_ms(2000);
	Start_PWM123();
	Start_PWM4();
	//Start_PWM_Servo();
	
	//Set_PWM1(SET_START_PWM);
	//Set_PWM2(SET_START_PWM);
	//Set_PWM3(SET_START_PWM);
	//Set_PWM4(SET_START_PWM);
	//Set_Servo(100);
	
	//ADC
	//int k;
	/*ADMUX = (1<<REFS0);
	ADCSRB = 0;
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);*/
	
	
	//int i;
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
		
		avanti();
		_delay_ms(2000);
		fermo();
		_delay_ms(1000);
		
		//Serial_Send(OCR3A); Serial_Send("\n");
	}
}


// ISR(INT2_vect){//interrupt motore 1
// 	t_1 = (int32_t) TCNT5;
// 	deltat_1 = (t_1-t_old_1) * KTIMER_256;
// 	velocita_1 = 1000000000UL/(K_ENCODER1 * deltat_1); //giri al secondo
// 	t_old_1 = t_1;
// 	//seg_enc++;
// 	//Serial_Send(seg_enc); Serial_Send("\n");
// }
// 
// ISR(INT3_vect){//interrupt motore 2
// 	t_2 = (int32_t) TCNT5;
// 	deltat_2 = (t_2-t_old_2) * KTIMER_256;
// 	velocita_2 = 1000000000UL/(K_ENCODER2 * deltat_2); //giri al secondo
// 	t_old_2 = t_2;
// 	seg_enc++;
// 	Serial_Send(seg_enc); Serial_Send("\n");
// }
// 
// ISR(INT4_vect){//interrupt motore 3
// 	t_3 = (int32_t) TCNT5;
// 	deltat_3 = (t_3-t_old_3) * KTIMER_256;
// 	velocita_3 = 1000000000UL/(K_ENCODER3 * deltat_3); //giri al secondo
// 	t_old_3 = t_3;
// 	//seg_enc++;
// 	//Serial_Send(seg_enc); Serial_Send("\n");
// }
// 
// ISR(INT5_vect){//interrupt motore 4
// 	t_4 = (int32_t) TCNT5;
// 	deltat_4 = (t_4-t_old_4) * KTIMER_256;
// 	velocita_4 = 1000000000UL/(K_ENCODER4 * deltat_4); //giri al secondo
// 	t_old_4 = t_4;
// 	/*seg_enc++;
// 	Serial_Send(seg_enc); Serial_Send("\n");*/
// }
// 
// void Init_PWM123(){//Motore 1/2/3
// 	TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<COM1C1) | (1<<WGM11) | (1<<WGM10);
// 	DDRB = (1<<PB5) | (1<<PB6) | (1<<PB7); //| (1<<PB4);//Uscita PWM motori e Servo
// }
// 
// void Init_PWM4(){//Motore 4
// 	TCCR3A = (1<<COM3A1) | (1<<WGM31) | (1<<WGM30);
// 	DDRE = (1<<PE3);
// }
// 
// /*void Init_PWM_Servo(){
// 	TCCR2A = (1<<COM2A1) | (1<<WGM21) | (1<<WGM20);
// 	//ddrb � nel init pwm123
// }*/
// 
// void Start_PWM123(){//Motore 1/2/3
// 	TCCR1B = (1<<WGM12) | (1<<CS12);
// }
// 
// void Start_PWM4(){//Motore 4
// 	TCCR3B = (1<<WGM32) | (1<<CS32);
// }
// 
// /*void Start_PWM_Servo(){//	Servo
// 	TCCR2B = (0<<FOC2A) | (0<<FOC2B) | (1<<CS22) | (1<<CS20);
// }*/
// 
// void Set_PWM1(int duty_1){//Motore 1
// 	OCR1A = duty_1;
// }
// 
// void Set_PWM2(int duty_2){//Motore 2
// 	OCR1B = duty_2;
// }
// 
// void Set_PWM3(int duty_3){//Motore 3
// 	OCR1C = duty_3;
// }
// 
// void Set_PWM4(int duty_4){//Motore 4
// 	OCR3A = duty_4;
// }
// 
// 
// 
// void PID1(){
// 	e_1 = (double) (SET_POINT_VELOCITA - velocita_1);
// 	p_1 = e_1*KP;
// 	i_1= i_old_1 + (KI*e_1);
// 	d_1 = (e_1 - e_old_1) * KD;
// 	if(i_1>I_MAX){
// 		i_1 = I_MAX;
// 	}
// 	if(i_1<I_MIN){
// 		i_1=I_MIN;
// 	}
// 	i_old_1 = i_1;
// 	e_old_1 = e_1;
// 	pid1 = p_1+i_1+d_1;
// 	if(pid1<0){
// 		pid1 = 0;
// 	}
// 	if(pid1>1023){
// 		pid1 = 1023;
// 	}
// 	Set_PWM4((int) pid1);
// }
// 
// void PID2(){//pid motore 2
// 	e_2 = (double) (SET_POINT_VELOCITA - velocita_2);
// 	p_2 = e_2*KP;
// 	i_2= i_old_2 + (KI*e_2);
// 	d_2 = (e_2 - e_old_2) * KD;
// 	if(i_2>I_MAX){
// 		i_2 = I_MAX;
// 	}
// 	if(i_2<I_MIN){
// 		i_2=I_MIN;
// 	}
// 	i_old_2 = i_2;
// 	e_old_2 = e_2;
// 	pid2 = p_2+i_2+d_2;
// 	if(pid2<0){
// 		pid2 = 0;
// 	}
// 	if(pid2>1023){
// 		pid2 = 1023;
// 	}
// 	Set_PWM4((int) pid2);
// }
// 
// void PID3(){//pid motore 3
// 	e_3= (double) (SET_POINT_VELOCITA - velocita_3);
// 	p_3 = e_3*KP;
// 	i_3= i_old_3 + (KI*e_3);
// 	d_3 = (e_3 - e_old_3) * KD;
// 	if(i_3>I_MAX){
// 		i_3 = I_MAX;
// 	}
// 	if(i_3<I_MIN){
// 		i_3=I_MIN;
// 	}
// 	i_old_3 = i_3;
// 	e_old_3 = e_3;
// 	pid3 = p_3+i_3+d_3;
// 	if(pid3<0){
// 		pid3 = 0;
// 	}
// 	if(pid3>1023){
// 		pid3 = 1023;
// 	}
// 	Set_PWM4((int) pid3);
// }
// 
// void PID4(){//pid motore 4
// 	e_4 = (double) (SET_POINT_VELOCITA - velocita_4);
// 	p_4 = e_4*KP;
// 	i_4= i_old_4 + (KI*e_4);
// 	d_4 = (e_4 - e_old_4) * KD;
// 	if(i_4>I_MAX){
// 		i_4 = I_MAX;
// 	}
// 	if(i_4<I_MIN){
// 		i_4=I_MIN;
// 	}
// 	i_old_4 = i_4;
// 	e_old_4 = e_4;
// 	pid4 = p_4+i_4+d_4;
// 	if(pid4<0){
// 		pid4 = 0;
// 	}
// 	if(pid4>1023){
// 		pid4 = 1023;
// 	}
// 	Set_PWM4((int) pid4);
// }
// 
// ISR(TIMER4_COMPA_vect){
// 	//PID1();
// 	//PID2();
// 	PID3();
// 	//PID4();
// 	TCNT4 = 0;
// }
// 
// int main(){
// 	Serial_Init();
// 	//PID
// 	_delay_ms(1500);
// 	EICRA = (1<<ISC20) | (1<<ISC21) | (1<<ISC30) | (1<<ISC31);//interrupt rising edge
// 	EICRB = (1<<ISC40) | (1<<ISC41) | (1<<ISC50) | (1<<ISC51);
// 	EIMSK = 1<<INT2 | 1<<INT3 | 1<<INT4 |1<<INT5;//attivamento interrupt esterni
// 	sei();//attivamento ogni interrupt
// 	
// 	
// 	
// 	TCCR5A = 0;
// 	TCCR5B = 1<<CS52;
// 	
// // 	fa partire interrupt ogni millisecondo --> attiva pid
// 	TCCR4A = 0;
// 	TCCR4B = (1<<WGM42) | (1<<CS41) | (1<<CS40); //f = 250KHz
// 	TIMSK4=1<<OCIE4A;
// 	OCR4A = 250;
// 	
// 	
// 	DDRH = 1<<PH4;
// 	PORTA = 1<<PH4;
// // 	TCCR4A = (1<<COM1B1) | (1<<WGM41) | (1<<WGM40);//inizializza il servo
// // 	TCCR4B = (1<<WGM42) | (1<<CS42);//N=256
// // 	TIMSK4 = 1<<OCIE4A;
// // 	OCR4A = 125;			//PID ogni 2 ms
// 	
// 	
// 	/*Set_Servo(150);
// 	_delay_ms(1500);
// 	Set_Servo(50);
// 	_delay_ms(1500);*/
// 	
// 	Init_PWM123();
// 	Init_PWM4();
// 	//Init_PWM_Servo();
// 	//_delay_ms(2000);
// 	Start_PWM123();
// 	Start_PWM4();
// 	//Start_PWM_Servo();
// 	
// 	//Set_PWM1(SET_START_PWM);		//Motore 4
// 	//Set_PWM2(SET_START_PWM);		//Motore 1
// 	//Set_PWM3(SET_START_PWM);		//Motore 2
// 	Set_PWM4(200);		//Motore 3
// 	
// 	DDRC = 0xFF;
// 	PORTC = 255;//(1<<PC0) | (1<<PC1);
// 	
// 	DDRA = 0xFF;
// 	PORTA = (1<<PA0) | (1<<PA2) | (1<<PA4)| (1<<PA6);//Verso motore
// 	//PA2-PA3 Motore 2		PA2 AVANTI
// 	//PA6-PA7 Motore 3		PA6 AVANTI
// 	//PA0-PA1 Motore 4		PA0 AVANTI
// 	//PA4-PA5 Motore 1		PA4 AVANTI
// 	//PORTA = 1<<PA4;
// 	
// 	while(1){
// 		//Serial_Send(seg_enc);
// 	}
// }


