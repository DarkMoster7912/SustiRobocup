/*
 * Controllo.h
 *
 * Created: 27/02/2023 17:59:41
 *  Author: Enrico
 */ 


#ifndef CONTROLLO_H_
#define CONTROLLO_H_

extern void controllo_movimento( void );
extern void controllo_telecamera( void );
extern void adc_init( void );
extern int adc_value( void );
extern void adc_control( int k );
extern void interrupt_rasp( void );

extern int return_data( void );


#endif /* CONTROLLO_H_ */