#ifndef I2C_H_
#define I2C_H_

#ifndef SCL_FREQ
#define SCL_FREQ 1000000UL
#endif

#ifndef TWI_DATA_LENGTH
#define TWI_DATA_LENGTH 1
#endif

/*
extern void init_interrupt();
extern void init_TWI();
//extern void writeToSlave(uint8_t address, uint8_t* data);
//extern void readFromSlave(uint8_t address);
//extern void TWIWrite(uint8_t byte);
extern void TWIWaitUntilReady();
//extern void TWICheckCode(uint8_t code);
extern void TWI_Start();
extern void TWI_Stop();
extern void TWI_ACK();
extern void TWI_NACK();*/

extern int conversione();


#include <avr/interrupt.h>
#include <stdint.h>

void I2C_init(uint8_t address);
void I2C_stop(void);
void I2C_setCallbacks(void (*recv)(uint8_t), void (*req)());

inline void __attribute__((always_inline)) I2C_transmitByte(uint8_t data)
{
	TWDR = data;
}

ISR(TWI_vect);


#endif /* I2C_H_ */