/*
 * init.h
 *
 *  Created on: 24.06.2022
 *      Author: Fabian
 */

#ifndef INIT_H_
#define INIT_H_

/*  Interrupts  */
__interrupt void ECAP1_ISR(void);               //Hall 1 ISR
__interrupt void ECAP2_ISR(void);               //Hall 2 ISR
__interrupt void ECAP3_ISR(void);               //Hall 3 ISR
__interrupt void Timer0_ISR_PI_update(void);    //PI-Berechnung const. sample time
__interrupt void Timer1_ISR_Sys_control(void);   //Systemfunktionen
  //Systemfunktionen
__interrupt void SHUTDOWN_ISR(void);            //Not-Aus ISR
__interrupt void SCIB_receive_ISR(void);
__interrupt void SCIB_transmit_ISR(void);

/*  Configs */
void SysCtrl(void);
void ConfigInterrupt(void);
void ConfigPwm(void);
void ConfigAdc(void);
void ConfigEcap(void);
void ConfigGpio(void);
void ConfigI2c(void);
void ConfigSPI(void);
void ConfigSCIB(void);
void PushAsm(void);     //call last
void InitAll(void);

#endif /* INIT_H_ */
