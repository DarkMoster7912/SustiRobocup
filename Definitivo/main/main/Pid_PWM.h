/*
 * Pid_PWM.h
 *
 * Created: 26/01/2023 11:03:50
 *  Author: Enrico
 */ 


#ifndef PID_PWM_H_
#define PID_PWM_H_

extern void PID();
extern void PWM();
extern void Set_Velocita();
extern void avanti();
extern void indietro();
extern void sinistra();
extern void sinistra_lento();
extern void destra();
extern void destra_lento();
extern void stop_tutto();
extern void delay_movimento(int c);
extern void cubetto();
extern void Set_Servo( int duty_5);

extern int return_seg_enc(void);
void seg_enc_a_zero(int c);



#endif /* PID_PWM_H_ */