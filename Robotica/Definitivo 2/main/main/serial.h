
#define SERIAL
#include <avr/io.h>
#include <stdarg.h>
 
#define USART0_BAUDRATE 1000000
#define USART1_BAUDRATE 115200
#define USART2_BAUDRATE 115200
 
 
float ABS(double x);
float limita(float x, float I, float F);
 
void Serial_Init();//Init
void Serial_Tx( unsigned char data );//TRASMISSIONE
void Serial_Tx1( int data );
unsigned char Serial_Rx( void );//RICEVI
void Serial_Send_Int(int64_t num);//NUMERO
void Serial_Send_String(char *string1);//STRINGA
void Serial_Send_Float(double val);
double Serial_Recv_Num(void);
void SerialN();//NUOVA RIGA


#define Serial_Send(_1) _Generic(_1,int: Serial_Send_Int,char*: Serial_Send_String,float: Serial_Send_Float,char: Serial_Tx,double: Serial_Send_Float,unsigned char: Serial_Send_Int,unsigned long: Serial_Send_Int,unsigned int: Serial_Send_Int)(_1)