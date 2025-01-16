
/*
 * functions.c
 *
 *  Created on: 24.06.2022
 *      Author: Fabian
 */

#include "DSP28x_Project.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>


/*  Globale Variablen   */
unsigned int commutation_table[7][6] =          {{1,0,0,0,0,1},{0,0,1,0,0,1},{0,1,1,0,0,0},{0,1,0,0,1,0},{0,0,0,1,1,0},{1,0,0,1,0,0},{0,0,0,0,0,0}};
unsigned int commutation_table_inverted[6][6] = {{0,1,1,1,1,0},{1,1,0,1,1,0},{1,0,0,1,1,1},{1,0,1,1,0,1},{1,1,1,0,0,1},{0,1,1,0,1,1}};

extern unsigned long int PWM_TBPRD;
extern unsigned int PWM_CMP;


void update_PWM_sector_control(unsigned int rotor_position, unsigned int enable_bit, unsigned int pwm_cmp)
{
    /*  Legt die PWM-Sequenz abhängig von der Rotorfrequenz fest.
        Hat nichts mit der Regelung zu tun, sondern legt lediglich
        die übergeordnete Drehfrequenz fest.
        Parameter 1: Sektor 0-5 oder Shutdown 6  */

    if (enable_bit)
    {
        if (commutation_table[rotor_position][0])
        {
            EPwm1Regs.CMPA.half.CMPA = pwm_cmp;
            //EPwm1Regs.DBCTL.bit.OUT_MODE = 0; //S1 und S0 aktiv
        }
        else
        {
            EPwm1Regs.CMPA.half.CMPA = PWM_TBPRD + 1;
            //EPwm1Regs.DBCTL.bit.OUT_MODE = 0;
        }

        if (commutation_table[rotor_position][1])
        {
            EPwm1Regs.CMPB = 0;
            //EPwm1Regs.DBCTL.bit.OUT_MODE = 0;
        }
        else
        {
            EPwm1Regs.CMPB = PWM_TBPRD + 1;
        }

        if (commutation_table[rotor_position][2])
        {
            EPwm2Regs.CMPA.half.CMPA = pwm_cmp;
            //EPwm2Regs.DBCTL.bit.OUT_MODE = 0; //S1 und S0 aktiv
        }
        else
        {
            EPwm2Regs.CMPA.half.CMPA = PWM_TBPRD + 1;
            //EPwm2Regs.DBCTL.bit.OUT_MODE = 0;
        }

        if (commutation_table[rotor_position][3])
        {
            EPwm2Regs.CMPB = 0;
            //EPwm2Regs.DBCTL.bit.OUT_MODE = 0;
        }
        else
        {
            EPwm2Regs.CMPB = PWM_TBPRD + 1;
        }

        if (commutation_table[rotor_position][4])
        {
            EPwm3Regs.CMPA.half.CMPA = pwm_cmp;
            //EPwm3Regs.DBCTL.bit.OUT_MODE = 0; //S1 und S0 aktiv
        }
        else
        {
            EPwm3Regs.CMPA.half.CMPA = PWM_TBPRD + 1;
            //EPwm3Regs.DBCTL.bit.OUT_MODE = 0;
        }

        if (commutation_table[rotor_position][5])
        {
            EPwm3Regs.CMPB = 0;
            //EPwm3Regs.DBCTL.bit.OUT_MODE = 0;
        }
        else
        {
            EPwm3Regs.CMPB = PWM_TBPRD + 1;
        }
    }
}

unsigned int shutdown_motor(void)
{
    /*  PWM-Signale */
    update_PWM_sector_control(6, 1, PWM_CMP);
    return 0;
}


unsigned int release_shutdown(void)
{
    /*  PWM-Signale frei   */
    return 1;
}


void buck_off(void)
{
    /*  Deaktiviert 15V Buck-Converter  */
    GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;
}

void buck_on(void)
{
    /*  Aktiviert 15V Buck-Converter  */
    GpioDataRegs.GPASET.bit.GPIO13 = 1;
}


unsigned int read_sci_data(char *data_storage, unsigned int int_cnt)
{
    /*  Liest vollen SCIRX FIFO in den char-Array auf den der Pointer zeigt */
    unsigned int return_val = 0;
    unsigned int temp_buf = 0;
    unsigned int i;

    temp_buf = ScibRegs.SCIRXBUF.bit.RXDT;
    if (int_cnt < 15)
    {
        *(data_storage + int_cnt) = temp_buf;
        return_val = int_cnt + 1;
    }
    else
    {
        return_val = 0;
    }


    if ((temp_buf == 10) || (temp_buf == 0x00))
    {
        return_val = 0;
        for (i = int_cnt + 1; i < 16; i++) {*(data_storage + i) = 0x00;}
    }

    return return_val;
}

unsigned int write_sci_data(char *data_storage, unsigned int allow_transmit)
{
    /*  Sendet Zeichen aus *data_storage  */
    unsigned int i;
    unsigned int status = 0;

    for (i = 0; i < 4; i++)
    {
        if (*data_storage != 0x00)
        {
            ScibRegs.SCITXBUF = *data_storage;
            allow_transmit = 0;
            data_storage++;
        }
        else
        {
            status = 1; //1 = letztes Zeichen erreicht
            allow_transmit = 2;
            i = 4;
            ScibRegs.SCIFFTX.bit.TXFFIENA = 0;  //deaktiviert TXFIFOINT
        }
    }

    return allow_transmit + status;
}

unsigned int copy_string(char *data_storage, char *new_data_storage)
{
    /*  Ändert den String in *data_storage zu *new_data_storage
     *  Annahme sind gleichlange strings    */
    unsigned int i;
    unsigned int data_storage_length = strlen(new_data_storage);

    for (i = 0; i < (data_storage_length + 1); i++)
    {
        *(data_storage + i) = *(new_data_storage + i);
    }

    return 0;
}

void update_status_leds(unsigned int bit_pattern)
{
    /*  Ändert die Status-LEDs  */

    GpioDataRegs.GPASET.bit.GPIO17 = (bit_pattern & 2) >> 1;
    GpioDataRegs.GPBSET.bit.GPIO44 = bit_pattern & 1;
    GpioDataRegs.GPBSET.bit.GPIO50 = (bit_pattern & 4) >> 2;
    GpioDataRegs.GPBSET.bit.GPIO51 = (bit_pattern & 8) >> 3;
    GpioDataRegs.GPBSET.bit.GPIO52 = (bit_pattern & 16) >> 4;
    GpioDataRegs.GPBSET.bit.GPIO53 = (bit_pattern & 32) >> 5;

    GpioDataRegs.GPACLEAR.bit.GPIO17 = !((bit_pattern & 2) >> 1);
    GpioDataRegs.GPBCLEAR.bit.GPIO44 = !(bit_pattern & 1);
    GpioDataRegs.GPBCLEAR.bit.GPIO50 = !((bit_pattern & 4) >> 2);
    GpioDataRegs.GPBCLEAR.bit.GPIO51 = !((bit_pattern & 8) >> 3);
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = !((bit_pattern & 16) >> 4);
    GpioDataRegs.GPBCLEAR.bit.GPIO53 = !((bit_pattern & 32) >> 5);
}

void create_transmit_data_block(char *char_out, float w_is, float I_S2, float I_S4, float I_S6, float I_total, float V_Gen)
{
    unsigned int str_len = 0;
    unsigned int sprintf_return = 0;
    unsigned int i;
    char temp_str[4];
    char speed_is_char[4];
    char current_S2_char[4];
    char current_S4_char[4];
    char current_S6_char[4];
    char current_total_char[4];
    char gen_voltage_char[4];

    /*  Erzeugt den zyklischen Datenstream an den Bluetooth Chip    */
    sprintf_return = snprintf(speed_is_char, 5, "%d", (int)w_is);
    str_len = strlen(speed_is_char);
    copy_string(&temp_str[4-str_len], &speed_is_char[0]);
    for (i = 0; i < (4 - str_len); i++) {temp_str[i] = '0';}
    copy_string(&char_out[0], &temp_str[0]);

    sprintf_return = snprintf(current_S2_char, 5, "%d", (int)I_S2);
    str_len = strlen(current_S2_char);
    copy_string(&temp_str[4-str_len], &current_S2_char[0]);
    for (i = 0; i < (4 - str_len); i++) {temp_str[i] = '0';}
    copy_string(&char_out[4], &temp_str[0]);

    sprintf_return = snprintf(current_S4_char, 5, "%d", (int)I_S4);
    str_len = strlen(current_S4_char);
    copy_string(&temp_str[4-str_len], &current_S4_char[0]);
    for (i = 0; i < (4 - str_len); i++) {temp_str[i] = '0';}
    copy_string(&char_out[8], &temp_str[0]);

    sprintf_return = snprintf(current_S6_char, 5, "%d", (int)I_S6);
    str_len = strlen(current_S6_char);
    copy_string(&temp_str[4-str_len], &current_S6_char[0]);
    for (i = 0; i < (4 - str_len); i++) {temp_str[i] = '0';}
    copy_string(&char_out[12], &temp_str[0]);

    sprintf_return = snprintf(current_total_char, 5, "%d", (int)I_total);
    str_len = strlen(current_total_char);
    copy_string(&temp_str[4-str_len], &current_total_char[0]);
    for (i = 0; i < (4 - str_len); i++) {temp_str[i] = '0';}
    copy_string(&char_out[16], &temp_str[0]);

    sprintf_return = snprintf(gen_voltage_char, 5, "%d", (int)(V_Gen * 100));
    str_len = strlen(gen_voltage_char);
    copy_string(&temp_str[4-str_len], &gen_voltage_char[0]);
    for (i = 0; i < (4 - str_len); i++) {temp_str[i] = '0';}
    copy_string(&char_out[20], &temp_str[0]);
}
