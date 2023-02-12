//
#define F_CPU 16000000ul
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <util/delay.h>

#include "serial.h"

long BAUD0 = (F_CPU/16/USART0_BAUDRATE)-1;
//long BAUD1 = (F_CPU/16/USART1_BAUDRATE)-1;
//long BAUD2 = (F_CPU/16/USART2_BAUDRATE)-1;
unsigned char temp[3];


float ABS(double x){
	if(x<0) x*=-1;
	return x;
}

float limita(float x, float I, float F){
	if(F>I){
		if(x<I)x=I;
		if(x>F)x=F;
		}else{
		if(x>I)x=I;
		if(x<F)x=F;
	}
	return x;
}



void Serial_Init(){
 	UBRR0H = (unsigned char)(BAUD0>>8); 
 	UBRR0L = (unsigned char)BAUD0;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	UCSR0C = 0b00000110;
} 
void Serial_Tx(unsigned char data)
{
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0=data;
}
unsigned char Serial_Rx( void )
{
	while ( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}

void Serial_Send_Int(int64_t num)
{
	if(num<0)
	{
		Serial_Send_String("-");
		num=-num;
	}
	if(num==0){Serial_Tx('0');}
	else
	{
		char str[64];				// definisce una stringa sulla quale convertire il numero da trasmettere (max 10 cifre)
		char i;						// contatore ciclo
		for(i=0;i<32;i++) str[i]=0; // cancella la stringa
		i=31;
		while (num)
		{
			str[i]=num%10+'0';		// converte il numero da trasmettere in una stringa (dalla cifra meno significativa)
			num/=10;
			i--;
		}
		for (i=0;i<32;i++)			// invia la stringa un carattere alla volta
		if (str[i]) Serial_Tx(str[i]);
	}
}


void Serial_Send_String(char *string1)
{
	int i=0;
	for(i=0;i<strlen(string1);i++)
	{
		Serial_Tx(string1[i]);
	}
}
void SerialN()
{
	Serial_Tx(13);
	Serial_Tx(10);
}
void Serial_Send_Float(double data)
{
	if(data<0)
	{
		Serial_Send_String("-");
		data=-data;
	}
	long var=data;
	Serial_Send_Int(var);
	int k=0,j=0;
	double Num=data;
	
	
	while (fmod(Num,1.00)!=0)
	{
		Num*=10;
		k++;
	}
	double data1=Num;

		
	var=var*pow(10,k);

	data1=data1-var;
	
	long NuM=data1;
	while (data1>1){
		data1/=10.00;
		j++;
		
	}
	int diff=k-j;
	Serial_Send_String(".");
	
	for(int i=0;i<diff;i++)Serial_Tx('0');
	Serial_Send_Int(NuM);
}

double Serial_Recv_Num()
{
	int i=0;
	char num[64];
	for(i=0;i<32;i++) num[i]=0;
	i=0;
	char ce=Serial_Rx();
	while (ce!= 'e')
	{
		
		
		ce=Serial_Rx();
	}
	double Num=atof(num);				//converte la stringa in un float
	return Num;
}






void Serial_Tx1(int data) {
	
	if (data>99) {
		for (int k=0; k<3; k++){
			temp[k] = data%10;
			data = data / 10;
		}
		}else if(data>9) {
		for (int k=0; k<2; k++){
			temp[k] = data%10;
			data = data / 10;
		}
		temp[2] = 0;
		}else {
		temp[0] = data;
		temp[1] = 0;
		temp[2] = 0;
	}
	
	for (int k=0; k<3; k++){
		while ( !( UCSR0A & (1<<UDRE0)) );
		UDR0 = temp[k];
		_delay_us(1);
	}
	
	_delay_ms(1);
	
}