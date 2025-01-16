/*
 *  init.c
 *
 *  Created on: 11.04.2022
 *  Author: Fabian Kurth
 *  Description: initialization for different modules used
 */

#include "DSP28x_Project.h"
#include "init.h"
#include "functions.h"

float PWM_BASE_FREQ = 20000.0;   //10000 kHz
float f_TBCLK = 45000000.0;      //45 MHz
float PWM_BASE_DUTY = 0.52;      //50%
float PI_SAMPLE_TIME_US = 1000;  //Abtastzeit Regler
float SYS_SAMPLE_TIME_US = 100000;

unsigned long int PWM_TBPRD = 0;
unsigned int PWM_CMP = 0;

void SysCtrl(void)
{
    //memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
    EALLOW;
    DisableDog();
    InitSysCtrl();                                      //setzt u.A. die Frequnez auf 90Mhz
    InitFlash();                                      //Abarbeitung im RAM, nicht FLASH
    InitPieCtrl();                                      //Grundinitialisierung PIE-Gruppe
    InitPieVectTable();                                 //Aufbau Vektortabelle mit Basisfunktionen
    InitCpuTimers();                                    //Betriebsart Rückwärtszähler
    //InitAdc();                                          //Initialisierung ADC
    ConfigCpuTimer(&CpuTimer0, 90, PI_SAMPLE_TIME_US);  //Initialisierung Timer 0
    ConfigCpuTimer(&CpuTimer1, 90, SYS_SAMPLE_TIME_US); //Initialisierung Timer 1

    SysCtrlRegs.LOSPCP.bit.LSPCLK = 1;                  //LSPCLK = SYSCLKOUT/2
    CpuTimer0Regs.TCR.bit.TSS = 0;                      //Timer Start
    CpuTimer1Regs.TCR.bit.TSS = 0;                      //Timer Start
    SysCtrlRegs.LPMCR0.bit.LPM = 0;                     //0 = IDLE-Mode
    EDIS;
}

void ConfigInterrupt(void)
{
    EALLOW;
    /*  Interrupts  */
    PieVectTable.ECAP1_INT =& ECAP1_ISR;
    PieVectTable.ECAP2_INT =& ECAP2_ISR;
    PieVectTable.ECAP3_INT =& ECAP3_ISR;
    PieVectTable.XINT1 =& SHUTDOWN_ISR;
    PieVectTable.XINT2 =& SHUTDOWN_ISR;
    PieVectTable.TINT0 =& Timer0_ISR_PI_update;
    PieVectTable.TINT1 =& Timer1_ISR_Sys_control;
    PieVectTable.SCIRXINTB =& SCIB_receive_ISR;
    PieVectTable.SCITXINTB =& SCIB_transmit_ISR;

    IER = 9;        //Freigabe INT1/4 (Timer0, XInt1, ECAP1/2/3)
    IER |= 1 << 8;  //Freigabe INT9 (SCIRXINTB)
    IER |= 1<<12;   //Freigabe INT13 (Timer1, nicht durch PIE geschleift)

    PieCtrlRegs.PIEIER4.all |= 7;           //Freigabe ECAP1/2/3_INT
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      //Freigabe TINT0
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;      //Freigabe XInt1
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;      //Freigabe XInt2
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;      //Freigabe SCIRXINTB
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;      //Freigabe SCITXINTB

    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 8;   //falling edge GPIO8 als XINT1 Interrupt (Shutdown)
    GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 9;   //falling edge GPIO9 als XINT1 Interrupt (Shutdown)
    XIntruptRegs.XINT1CR.bit.ENABLE = 1;        //Int enable XINT1
    XIntruptRegs.XINT1CR.bit.POLARITY = 2;      //falling edge XINT1
    XIntruptRegs.XINT2CR.bit.ENABLE = 1;        //Int enable XINT2
    XIntruptRegs.XINT2CR.bit.POLARITY = 2;      //falling edge XINT2
    EDIS;
}

void ConfigGpio(void)
{
    EALLOW;
    /*  PIN Belegung    */
    //  PWM
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     //ePWM1A = S1
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;     //ePWM1B = S2
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;     //ePWM2A = S3
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;     //ePWM2B = S4
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;     //ePWM3A = S5
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;     //ePWM3B = S6

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;     //ePWM4A = ResetPeak(detection)

    //  ECAP
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;    //ECAP1 = Hall1Mot
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1;    //ECAP2 = Hall2Mot
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 1;    //ECAP3 = Hall3Mot

    //  GPIO
    //  OUTPUTS
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;    //GPIO55 = Shutdown
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 1;     //GPIO55 = output
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;    //GPIO13 = BuckOff
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;     //GPIO13 = output
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;    //GPIO17 = P2
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;     //GPIO17 = output
    GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;    //GPIO44 = P1
    GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1;     //GPIO44 = output
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;    //GPIO50 = P3
    GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1;     //GPIO50 = output
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;    //GPIO51 = P4
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;     //GPIO51 = output
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;    //GPIO52 = P5
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;     //GPIO52 = output
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;    //GPIO53 = P6
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1;     //GPIO53 = output

    //  INPUTS
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;     //GPIO7 = Taster
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 0;      //GPIO7 = input
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;     //GPIO8 = Taster
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 0;      //GPIO8 = input
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;     //GPIO9 = Taster
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;      //GPIO9 = input
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;    //GPIO10 = Taster
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;     //GPIO10 = input


    //  I2C, SPI
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;    //SDAA = SDA
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;    //SCLA = SCL
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2;    //SCITXDB = SCITXDB
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;    //SCIRXDB = SCIRXDB

    /*  Registerinitialisierungen   */
    ConfigPwm();
    ConfigAdc();
    ConfigEcap();
    ConfigI2c();
    ConfigSPI();

    EDIS;
}


void ConfigPwm(void)
{
    /*  Konfigurieren der PWM Module
        Ansteuerung von 3 Brücken
        Software Low-Force zu Beginn    */

    PWM_TBPRD = (long int)((f_TBCLK * (1 / PWM_BASE_FREQ)) - 1);   //Berechnung des Initial-TBPRD-Wertes zur Festlegung der Frequenz
    PWM_CMP = (int)(PWM_TBPRD * (1 - PWM_BASE_DUTY));       //Berechnung des Initial-CMPA/B-Wertes zur Festlegung des Duty Cycle

    /*  PWM 1   */
    EPwm1Regs.TBPRD = PWM_TBPRD; //100us base PWM Frequency
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;  //Up-Down-Count-Mode
    EPwm1Regs.AQCTLA.bit.ZRO = 0; //deaktiviert
    EPwm1Regs.AQCTLA.bit.PRD = 0; //deaktiviert
    //EPWM1A
    EPwm1Regs.AQCTLA.bit.CAD = 1; //clear ePWM1A low wenn Zähler CMPA erreicht (bei Zählrichtung abwärts)
    EPwm1Regs.AQCTLA.bit.CAU = 2; //set ePWM1A high wenn Zähler CMPA erreicht (bei Zählrichtung aufwärts)
    EPwm1Regs.CMPA.half.CMPA = PWM_TBPRD + 1; //Pulsbreite
    //EPWM1B
    //EPWM1B als Gegentaktsignal
    EPwm1Regs.DBRED=2000;  //rising edge delay (20us)
    EPwm1Regs.DBFED=2000;  //falling edge delay (20us)
    EPwm1Regs.DBCTL.bit.OUT_MODE=3; //S1 und S0
    EPwm1Regs.DBCTL.bit.POLSEL=2;   //S3 und S2
    EPwm1Regs.DBCTL.bit.IN_MODE=0;   //S5 und S4

    /*  PWM 2   */
    EPwm2Regs.TBPRD = PWM_TBPRD; //100us base PWM Frequency
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm2Regs.TBCTL.bit.CTRMODE = 2;  //Up-Down-Count-Mode
    EPwm2Regs.AQCTLA.bit.ZRO = 0;
    EPwm2Regs.AQCTLA.bit.PRD = 0;
    //EPWM2A
    EPwm2Regs.AQCTLA.bit.CAD = 1;
    EPwm2Regs.AQCTLA.bit.CAU = 2;
    EPwm2Regs.CMPA.half.CMPA = PWM_TBPRD + 1; //Pulsbreite
    //EPWM2B
    //EPWM2B als Gegentaktsignal
    EPwm2Regs.DBRED=2000;  //rising edge delay (20us)
    EPwm2Regs.DBFED=2000;  //falling edge delay (20us)
    EPwm2Regs.DBCTL.bit.OUT_MODE=3; //S1 und S0
    EPwm2Regs.DBCTL.bit.POLSEL=2;   //S3 und S2
    EPwm2Regs.DBCTL.bit.IN_MODE=0;   //S5 und S4

    /*  PWM 3   */
    EPwm3Regs.TBPRD = PWM_TBPRD; //100us base PWM Frequency
    EPwm3Regs.TBCTL.bit.CLKDIV = 0;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm3Regs.TBCTL.bit.CTRMODE = 2;  //Up-Down-Count-Mode
    EPwm3Regs.AQCTLA.bit.ZRO = 0;
    EPwm3Regs.AQCTLA.bit.PRD = 0;
    //EPWM3A
    EPwm3Regs.AQCTLA.bit.CAD = 1;
    EPwm3Regs.AQCTLA.bit.CAU = 2;
    EPwm3Regs.CMPA.half.CMPA = PWM_TBPRD + 1; //Pulsbreite
    //EPWM3B
    //EPWM3B als Gegentaktsignal
    EPwm3Regs.DBRED=2000;  //rising edge delay (10us)
    EPwm3Regs.DBFED=2000;  //falling edge delay (10us)
    EPwm3Regs.DBCTL.bit.OUT_MODE=3; //S1 und S0
    EPwm3Regs.DBCTL.bit.POLSEL=2;   //S3 und S2
    EPwm3Regs.DBCTL.bit.IN_MODE=0;   //S5 und S4

    EPwm4Regs.TBPRD = (long int)((f_TBCLK * (1 / 5)) - 1);
    EPwm4Regs.TBCTL.bit.CLKDIV = 5;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm4Regs.TBCTL.bit.CTRMODE = 2;  //Up-Down-Count-Mode
    EPwm4Regs.CMPA.half.CMPA = 0.02 * EPwm4Regs.TBPRD ;
    EPwm4Regs.AQCTLA.bit.CAD = 1;
    EPwm4Regs.AQCTLA.bit.CAU = 2;
}

void ConfigAdc(void)
{
    /*  Konfigurieren der ADC Module   */

    AdcRegs.ADCCTL2.bit.CLKDIV4EN = 1;      //Sysclock/4
    AdcRegs.ADCCTL2.bit.CLKDIV2EN = 1;

    /*  Potentiometer   */
    AdcRegs.ADCSAMPLEMODE.bit.SIMULEN0 = 0; //SOC0/1 Single Sample Mode
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 1;     //CPU-timer 0 als SOC0 Trigger
    AdcRegs.ADCSOC0CTL.bit.CHSEL = 0x5;     //ADCINA5 als Analog-Eingang
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 20;       //Abtastfenster auf 15 Takte

    /*  Strom in WR-Zweig 1 */
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 1;     //CPU-timer 0 als SOC1 Trigger
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 0x4;     //ADCINA4 als Analog-Eingang
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 63;       //Abtastfenster auf 7 Takte

    /*  Strom in WR-Zweig 2 */
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 1;     //CPU-timer 0 als SOC2 Trigger
    AdcRegs.ADCSOC2CTL.bit.CHSEL = 0xB;     //ADCINB3 als Analog-Eingang
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 63;       //Abtastfenster auf 7 Takte

    /*  Strom in WR-Zweig 3 */
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 1;     //CPU-timer 0 als SOC3 Trigger
    AdcRegs.ADCSOC3CTL.bit.CHSEL = 0x3;     //ADCINA3 als Analog-Eingang
    AdcRegs.ADCSOC3CTL.bit.ACQPS = 63;       //Abtastfenster auf 7 Takte

    /*  Generatorspannung   */
    AdcRegs.ADCSAMPLEMODE.bit.SIMULEN2 = 0; //SOC4/5 Single Sample Mode
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 1;     //CPU-timer 0 als SOC4 Trigger
    AdcRegs.ADCSOC4CTL.bit.CHSEL = 0xD;     //ADCINB5 als Analog-Eingang
    AdcRegs.ADCSOC4CTL.bit.ACQPS = 63;       //Abtastfenster auf 7 Takte
}

void ConfigEcap(void)
{
    /*  Konfigurieren der ECAP Module
        Es werden drei Motor-Hall-Sensoren genutzt, um die Rotorposition festzustellen.
        Dabei muss jeder Impuls so schnell wie möglich ausgewertet werden.  */

    ECap1Regs.ECCTL1.bit.PRESCALE = 0;  //Hall 1; Prescale = 1; Jede Flanke wird ausgewertet
    ECap1Regs.ECCTL1.bit.CAP1POL = 0;   //1. Event bei steigender Flanke
    ECap1Regs.ECCTL1.bit.CTRRST1 = 1;   //Counter Reset bei Erstem Event aktiviert
    ECap1Regs.ECCTL1.bit.CAP2POL = 1;   //2. Event bei fallender FLanke
    ECap1Regs.ECCTL1.bit.CTRRST2 = 1;   //Counter Reset bei Zweitem Event aktiviert
    ECap1Regs.ECCTL1.bit.CAP3POL = 0;   //3. Event bei steigender Flanke
    ECap1Regs.ECCTL1.bit.CTRRST3 = 1;   //Counter Reset bei Drittem Event aktiviert
    ECap1Regs.ECCTL1.bit.CAP4POL = 1;   //4. Event bei steigender Flanke
    ECap1Regs.ECCTL1.bit.CTRRST4 = 1;   //Counter Reset bei Viertem Event aktiviert
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;
    ECap1Regs.ECCTL2.bit.CAP_APWM = 0;  //CAPTURE Mode
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0;   //Continuous mode (was bedeutet das?)
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = 3; //Disable Sync-Out
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;  //Disable Sync-In
    ECap1Regs.ECCTL2.bit.STOP_WRAP = 3; //Wrap nach 4. Event
    ECap1Regs.ECCTL2.bit.REARM = 0;     //no rearm?
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;     //free running timer
    ECap1Regs.ECEINT.bit.CEVT1 = 1;     //Interrupt nach 1. Event
    ECap1Regs.ECEINT.bit.CEVT2 = 1;     //Interrupt nach 2. Event
    ECap1Regs.ECEINT.bit.CEVT3 = 1;     //Interrupt nach 3. Event
    ECap1Regs.ECEINT.bit.CEVT4 = 1;     //Interrupt nach 4. Event

    ECap2Regs.ECCTL1.bit.PRESCALE = 0;  //Hall 2; Prescale = 1; Jede Flanke wird ausgewertet
    ECap2Regs.ECCTL1.bit.CAP1POL = 0;   //1. Event bei steigender Flanke
    ECap2Regs.ECCTL1.bit.CTRRST1 = 1;   //Counter Reset bei Erstem Event aktiviert
    ECap2Regs.ECCTL1.bit.CAP2POL = 1;   //2. Event bei fallender FLanke
    ECap2Regs.ECCTL1.bit.CTRRST2 = 1;   //Counter Reset bei Zweitem Event aktiviert
    ECap2Regs.ECCTL1.bit.CAP3POL = 0;   //3. Event bei steigender Flanke
    ECap2Regs.ECCTL1.bit.CTRRST3 = 1;   //Counter Reset bei Drittem Event aktiviert
    ECap2Regs.ECCTL1.bit.CAP4POL = 1;   //4. Event bei steigender Flanke
    ECap2Regs.ECCTL1.bit.CTRRST4 = 1;   //Counter Reset bei Viertem Event aktiviert
    ECap2Regs.ECCTL1.bit.CAPLDEN = 1;   //CAP Load enable
    ECap2Regs.ECCTL2.bit.CAP_APWM = 0;  //CAPTURE Mode
    ECap2Regs.ECCTL2.bit.CONT_ONESHT = 0;   //Continuous mode (was bedeutet das?)
    ECap2Regs.ECCTL2.bit.SYNCO_SEL = 3; //Disable Sync-Out
    ECap2Regs.ECCTL2.bit.SYNCI_EN = 0;  //Disable Sync-In
    ECap2Regs.ECCTL2.bit.STOP_WRAP = 3; //Wrap nach 2. Event
    ECap2Regs.ECCTL2.bit.REARM = 0;     //no rearm?
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;     //free running timer
    ECap2Regs.ECEINT.bit.CEVT1 = 1;     //Interrupt nach 1. Event
    ECap2Regs.ECEINT.bit.CEVT2 = 1;     //Interrupt nach 2. Event
    ECap2Regs.ECEINT.bit.CEVT3 = 1;     //Interrupt nach 1. Event
    ECap2Regs.ECEINT.bit.CEVT4 = 1;     //Interrupt nach 2. Event

    ECap3Regs.ECCTL1.bit.PRESCALE = 0;  //Hall 3; Prescale = 1; Jede Flanke wird ausgewertet
    ECap3Regs.ECCTL1.bit.CAP1POL = 0;   //1. Event bei steigender Flanke
    ECap3Regs.ECCTL1.bit.CTRRST1 = 1;   //Counter Reset bei Erstem Event aktiviert
    ECap3Regs.ECCTL1.bit.CAP2POL = 1;   //2. Event bei fallender FLanke
    ECap3Regs.ECCTL1.bit.CTRRST2 = 1;   //Counter Reset bei Zweitem Event aktiviert
    ECap3Regs.ECCTL1.bit.CAP3POL = 0;   //3. Event bei steigender Flanke
    ECap3Regs.ECCTL1.bit.CTRRST3 = 1;   //Counter Reset bei Drittem Event aktiviert
    ECap3Regs.ECCTL1.bit.CAP4POL = 1;   //4. Event bei steigender Flanke
    ECap3Regs.ECCTL1.bit.CTRRST4 = 1;   //Counter Reset bei Viertem Event aktiviert
    ECap3Regs.ECCTL1.bit.CAPLDEN = 1;
    ECap3Regs.ECCTL2.bit.CAP_APWM = 0;  //CAPTURE Mode
    ECap3Regs.ECCTL2.bit.CONT_ONESHT = 0;   //Continuous mode (was bedeutet das?)
    ECap3Regs.ECCTL2.bit.SYNCO_SEL = 3; //Disable Sync-Out
    ECap3Regs.ECCTL2.bit.SYNCI_EN = 0;  //Disable Sync-In
    ECap3Regs.ECCTL2.bit.STOP_WRAP = 3; //Wrap nach 2. Event
    ECap3Regs.ECCTL2.bit.REARM = 0;     //no rearm?
    ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;     //free running timer
    ECap3Regs.ECEINT.bit.CEVT1 = 1;     //Interrupt nach 1. Event
    ECap3Regs.ECEINT.bit.CEVT2 = 1;     //Interrupt nach 2. Event
    ECap3Regs.ECEINT.bit.CEVT3 = 1;     //Interrupt nach 1. Event
    ECap3Regs.ECEINT.bit.CEVT4 = 1;     //Interrupt nach 2. Event
}


void ConfigI2c(void)
{
    /*  Konfigurieren des I2C Moduls   */


}

void ConfigSPI(void)
{
    /*  Konfigurieren des SPI Moduls   */

}

void ConfigSCIB(void)
{
    /*  Konfigurieren des SCIB Moduls. Zuständig
     *  für den Bluetooth Chip.
     *  (Vielleicht noch Transmitter Interrupt deak.)
     *  Optimierbar */

    ScibRegs.SCICTL1.bit.SWRESET = 0;       //Reset
    ScibRegs.SCICCR.bit.STOPBITS = 0;       //1 Stoppbit
    ScibRegs.SCICCR.bit.PARITYENA = 0;      //Parität DEaktiviert
    ScibRegs.SCICCR.bit.PARITY = 0;         //ungerade Parität
    ScibRegs.SCICCR.bit.SCICHAR = 7;        //8 bit pro Zeichen
    ScibRegs.SCICCR.bit.LOOPBKENA = 0;      //Loop back disabled
    ScibRegs.SCICCR.bit.ADDRIDLE_MODE = 0;  //Idle-Line Mode

    ScibRegs.SCICTL1.bit.RXENA = 1;         //receiver enabled
    ScibRegs.SCICTL1.bit.TXENA = 1;         //transmitter enabled
    ScibRegs.SCICTL1.bit.RXERRINTENA = 0;   //Receive error int disable
    ScibRegs.SCICTL1.bit.SLEEP = 0;         //sleep disabled
    ScibRegs.SCICTL1.bit.TXWAKE = 0;        //no transmitter wakeup mode
    ScibRegs.SCICTL2.bit.TXINTENA=0;        //disable transmitter interrupt

    ScibRegs.SCIHBAUD = 0;                  //Baudrate auf 115200 bit/s bei LSPCLK = 45MHz
    ScibRegs.SCILBAUD = 0x18;               //Baudrate auf 115200 bit/s bei LSPCLK = 45MHz

    /*  FIFO CONFIG */
    /*  Transmit    */
    ScibRegs.SCIFFTX.bit.SCIFFENA = 1;      //enable FIFO
    ScibRegs.SCIFFTX.bit.SCIRST = 1;        //unklar
    ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 1;  //FIFO Zeiger-Operation
    ScibRegs.SCIFFTX.bit.TXFFIENA = 0;      //(vorerst) kein Interrupt
    ScibRegs.SCIFFTX.bit.TXFFIL = 0;        //wenn FIFO leer -> Interrupt

    /*  Receive */
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;   //RXFIFO Zeiger-Operation
    ScibRegs.SCIFFRX.bit.RXFFIENA = 1;      //Interrupt
    ScibRegs.SCIFFRX.bit.RXFFIL = 1;        //wenn RXFIFO mit 1 Zeichen belegt -> Interrupt

    ScibRegs.SCICTL1.bit.SWRESET=1; //Reset
}

void PushAsm(void)
{
    asm(" PUSH IER");   //IER-Einstellung in den Stack schreiben
    asm(" POP DBGIER"); //DBGIER-Einstellung aus dem Stack
    asm(" CLRC INTM");  //Freigabe aller Interrupts
    asm(" CLRC DBGM");  //Freigabe von Interrupts im Debugmode
}

void InitAll(void)
{
    SysCtrl();
    ConfigInterrupt();
    ConfigPwm();
    ConfigAdc();
    ConfigEcap();
    ConfigGpio();
    ConfigI2c();
    ConfigSPI();
    ConfigSCIB();
    PushAsm();     //call last

}


