/*
 *  main.c
 *
 *  Created on: 11.04.2022
 *  Author: Fabian Kurth
 *  Description: main control file for a 3 phase generator system
 *  Features:
 */

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "DSP28x_Project.h"
#include "functions.h"
#include "init.h"
#include "PI_controller.h"

#define PI_KP 0.00007
#define PI_KI 0.001
#define PI_Ts 0.001 //10ms
#define PI_LIMMAX 1
#define PI_LIMMIN 0.05
/*  Global Variables    */

extern unsigned long int PWM_TBPRD;
extern unsigned int PWM_CMP;
extern float PWM_BASE_FREQ;

unsigned int sector = 1;
unsigned int last_sector = 2;
unsigned int direction = 1; //rechts = 1
unsigned int pwm_enable = 0;
unsigned int Timer0_ISR_cnt = 0;
unsigned int sector_open_loop = 6;
unsigned int open_loop_speed = 200;

unsigned int winkel_1 = 0;
float omega = 2;

unsigned int sci_allow_transmit = 1;
unsigned int sci_telegram_cnt = 0;
unsigned int read_sci_int_cnt = 0;
char sci_data_in[16];
char sci_data_out[25]; //0120(Speed_is) 0200 (Strom Zweig 1) 0200 0200 2300 (Generator)

char shutdown_char[] = "SHUTDOWN";

unsigned int system_mode = 0;
float speed_set = 0;
float speed_is = 0;

float current_S2 = 0;
float current_S4 = 0;
float current_S6 = 0;
float current_total = 0;
float rolling_avg_current_S2 = 0;
float rolling_avg_current_S4 = 0;
float rolling_avg_current_S6 = 0;
float rolling_avg_current_total = 0;
unsigned int overcurrent_triggered = 0;
float gen_voltage = 0;


/*  PI-Regler Struktur */
PI_controller PI_speed = {PI_KP, PI_KI, PI_LIMMAX, PI_LIMMIN, PI_Ts};
unsigned int Timer0_Interrupt_Count = 0;


void main(void)
{
    InitAll();          //initalisiert alles
    PI_Init(&PI_speed); //initialisiert PI-Regler
    pwm_enable = shutdown_motor();   //PWM-Modul deaktiviert

    while (1) {asm(" IDLE");}
}


__interrupt void Timer1_ISR_Sys_control(void)
{
    /*  Arbeitet Systemfunktionen ab    */
    char temp_str[4];
    float temp = 0;
    unsigned int i;
    unsigned int weight[] = {0, 0, 0, 0};
    unsigned int hold_speed_set = 0;


    strcpy(temp_str, sci_data_in);
    temp_str[3] = 0x00;

    /*  Startfreigabe Motor */
    if (!GpioDataRegs.GPADAT.bit.GPIO10 & !pwm_enable & (GpioDataRegs.GPADAT.bit.GPIO8 | GpioDataRegs.GPADAT.bit.GPIO9))
    {
        pwm_enable = release_shutdown(); //Freigabe des Shutdown wenn Kippschalter auf l oder r und S2 gedrückt
        update_PWM_sector_control(sector - 1, pwm_enable, PWM_CMP);
    }

    /*  Festlegung des System-Zustands
     *  Moduswechsel durch Taster S3 (GPIO7) oder durch Bluetooth Text  */
    if (!strcmp(temp_str, "MOD") && (sci_data_in[3] > 47) && (sci_data_in[3] < 58))
    {
        system_mode = sci_data_in[3] - 48;
    }
    if (GpioDataRegs.GPADAT.bit.GPIO7){system_mode++;}
    if (system_mode > 2) {system_mode = 0;}


    /*  Betriebsartausführung   */
    switch (system_mode)
    {
    case 0:
        /*  Keine Interaktion mit Bluetooth, Sollwertvorgabe (1/min) durch Potentiometer */
        GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;

        if (pwm_enable)
        {
            GpioDataRegs.GPBSET.bit.GPIO44 = 1;
            temp = (float)(AdcResult.ADCRESULT0);  //Normierung auf 0 bis ca 1
            speed_set = temp;
        }
        else
        {
            GpioDataRegs.GPBTOGGLE.bit.GPIO44 = 1;
        }
        break;

    case 1:
        /*  Interaktion mit Bluetooth, Sollwertvorage durch Bluetooth
         *  Rückgabe des Ist-Drehzahl-Wertes    */

        GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;

        if (pwm_enable)
        {
            GpioDataRegs.GPASET.bit.GPIO17 = 1;

            for (i = 0; i < 4; i++)
            {
                if ((sci_data_in[i] > 47) && (sci_data_in[i] < 58))
                {
                    weight[i] = 1;
                }
                else
                {
                    hold_speed_set = 1;
                    i = 4;
                }
            }

            if (!hold_speed_set) {speed_set = ((sci_data_in[0] - 48) * 1000 * weight[0] + (sci_data_in[1] - 48) * 100 * weight[1] + (sci_data_in[2] - 48) * 10 * weight[2]+ (sci_data_in[3] - 48)  *  weight[3]);}

            if (sci_allow_transmit)
            {
                create_transmit_data_block(&sci_data_out[0], speed_is, rolling_avg_current_S2, rolling_avg_current_S4, rolling_avg_current_S6, rolling_avg_current_total, gen_voltage);

                sci_telegram_cnt = 0;
                ScibRegs.SCIFFTX.bit.TXFFIENA = 1;
            }
        }
        else
        {
            GpioDataRegs.GPATOGGLE.bit.GPIO17 = 1;
        }

        break;

    case 2:
        /*  Wie case 0, allerdings mit Rückgabe des Ist-Drehzahl-Wertes */

        GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1;

        if (pwm_enable)
        {
            GpioDataRegs.GPBSET.bit.GPIO50 = 1;

            if (sci_allow_transmit)
            {
                create_transmit_data_block(&sci_data_out[0], speed_is, rolling_avg_current_S2, rolling_avg_current_S4, rolling_avg_current_S6, rolling_avg_current_total, gen_voltage);

                sci_telegram_cnt = 0;
                ScibRegs.SCIFFTX.bit.TXFFIENA = 1;
            }
        }
        else
        {
            GpioDataRegs.GPBTOGGLE.bit.GPIO50 = 1;
        }

        break;
    default:
        break;
    }
}


__interrupt void Timer0_ISR_PI_update(void)
{
    /*  zyklische Reglerabarbeitung */
    float winkel_rad = 0;
    unsigned int winkel_2;
    unsigned int winkel_3;
    float winkel_1_rad;
    float winkel_2_rad;
    float winkel_3_rad;

    Timer0_Interrupt_Count++;

    if (system_mode == 0 || system_mode == 2) {speed_set = (float)(AdcResult.ADCRESULT0);}

    if (pwm_enable)
    {
        winkel_1 += omega;
        winkel_2 = winkel_1 + 120;
        winkel_3 = winkel_1 + 240;

        winkel_1 = winkel_1 % 360;
        winkel_2 = winkel_2 % 360;
        winkel_3 = winkel_3 % 360;

        winkel_1_rad = (float)winkel_1 / 180 * 3.1415;
        winkel_2_rad = (float)winkel_2 / 180 * 3.1415;
        winkel_3_rad = (float)winkel_3 / 180 * 3.1415;

        EPwm1Regs.CMPA.half.CMPA = PWM_TBPRD * (0.6 + 0.3 * sin(winkel_1_rad));
        EPwm2Regs.CMPA.half.CMPA = PWM_TBPRD * (0.6 + 0.3 * sin(winkel_2_rad));
        EPwm3Regs.CMPA.half.CMPA = PWM_TBPRD * (0.6 + 0.3 * sin(winkel_3_rad));

        /*  Soft-Start fertig, closedloop
        controller_out = PI_Update(&PI_speed, speed_set, speed_is);
        PWM_CMP = (int)(PWM_TBPRD * (1 - controller_out));*/
    }

    current_S2 = (float)AdcResult.ADCRESULT1 * 0.700;   //Strom in mA
    current_S4 = (float)AdcResult.ADCRESULT2 * 0.700;   //Strom in mA
    current_S6 = (float)AdcResult.ADCRESULT3 * 0.700;   //Strom in mA

    rolling_avg_current_S2 = rolling_avg_current_S2 * 0.99 + 0.01 * current_S2;
    rolling_avg_current_S4 = rolling_avg_current_S4 * 0.99 + 0.01 * current_S4;
    rolling_avg_current_S6 = rolling_avg_current_S6 * 0.99 + 0.01 * current_S6;

    if (sector == 1 || sector == 2) { current_total = current_S6;}
    else if (sector == 3 || sector == 4) { current_total = current_S2;}
    else {current_total = current_S4;}

    rolling_avg_current_total =  rolling_avg_current_total * 0.99 + 0.01 * current_total;

    if (current_S2 > 2000 || current_S4 > 2000 || current_S6 > 2000) {overcurrent_triggered = 1;}

    gen_voltage = (float)AdcResult.ADCRESULT4 / 4096 * 3.3 * 13.2;
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}


__interrupt void SHUTDOWN_ISR(void)
{
    /*  Interrupt wenn Shutdown Switch betätigt wird    */
    pwm_enable = shutdown_motor();
    speed_is = 0;
    overcurrent_triggered = 0;
    omega = 2;

    if (sci_allow_transmit)
    {
        copy_string(&sci_data_out[0], &shutdown_char[0]);
        sci_data_out[14] = 0x0;
        sci_telegram_cnt = 0;
        ScibRegs.SCIFFTX.bit.TXFFIENA = 1;
    }


    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}


__interrupt void ECAP1_ISR(void)
{
    /*  Erfassung der Drehzahl und Aktualisierung des Sektors
     *  Das ECAP Signal ist invertiert zum eigentlichen Hall-Signal
     *  Es fehlt eine Drehrichtungslogik    */

    unsigned long ecap1_period = 0;

    if (ECap1Regs.ECFLG.bit.CEVT1 | ECap1Regs.ECFLG.bit.CEVT3) {sector = 5;}        //Flag 1 (rising edge)
    else if (ECap1Regs.ECFLG.bit.CEVT2 | ECap1Regs.ECFLG.bit.CEVT4) {sector = 2;}   //Flag 2 (falling edge)

    ecap1_period = ECap1Regs.CAP2 + ECap1Regs.CAP3;  //Auslesen
    speed_is = 2700000000 / ((float)ecap1_period);   //Drehzahl, Faktor = 90MHz * 0.5(polpaar) * 60

    ECap1Regs.ECCLR.all |= 31;      //Clear all flags
    PieCtrlRegs.PIEACK.bit.ACK4 = 1;    //Freigeben PIEIER4-Interrupts
}


__interrupt void ECAP2_ISR(void)
{

    if (ECap2Regs.ECFLG.bit.CEVT1 | ECap2Regs.ECFLG.bit.CEVT3) {sector = 1;}        //Flag 1 (rising edge)
    else if (ECap2Regs.ECFLG.bit.CEVT2 | ECap2Regs.ECFLG.bit.CEVT4) {sector = 4;}   //Flag 2 (falling edge)

    ECap2Regs.ECCLR.all |= 31;      //Clear all flags
    PieCtrlRegs.PIEACK.bit.ACK4 = 1;    //Freigeben PIEIER4-Interrupts*/
}


__interrupt void ECAP3_ISR(void)
{

    if (ECap3Regs.ECFLG.bit.CEVT1 | ECap3Regs.ECFLG.bit.CEVT3) {sector = 3;}        //Flag 1 (rising edge)
    else if (ECap3Regs.ECFLG.bit.CEVT2 | ECap3Regs.ECFLG.bit.CEVT4) {sector = 6;}   //Flag 2 (falling edge)

    ECap3Regs.ECCLR.all |= 31;      //Clear all flags
    PieCtrlRegs.PIEACK.bit.ACK4 = 1;    //Freigeben PIEIER4-Interrupts*/
}


__interrupt void SCIB_receive_ISR(void)
{
    /*  SCIB-receive ISR ausgelöst durch Bluetooth    */
    char *sci_data_in_ptr = &sci_data_in[0];

    read_sci_int_cnt = read_sci_data(sci_data_in_ptr, read_sci_int_cnt);

    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;

}

__interrupt void SCIB_transmit_ISR(void)
{
    /*  SCIB-send ISR ausgelöst durch leeren TXFIFO    */
    char *sci_data_out_ptr = &sci_data_out[4 * sci_telegram_cnt];
    unsigned int write_status_return = 0;
    unsigned int write_status = 0;
    unsigned int write_allow_transmit_return = 0;

    sci_telegram_cnt++;

    write_status_return = write_sci_data(sci_data_out_ptr, sci_allow_transmit);

    write_status = write_status_return & 1;
    write_allow_transmit_return = (write_status_return & 2) >> 1;

    sci_allow_transmit = write_allow_transmit_return;

    if (write_status){sci_telegram_cnt = 0;}

    ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;

}


