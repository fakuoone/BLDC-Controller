/*
 * PI_controller.c
 *
 *  Created on: 27.06.2022
 *      Author: Fabian
 */

#include "PI_controller.h"
#include "DSP28x_Project.h"


void PI_Init(PI_controller *pi)
{
    /*  Initialisiert den Regler    */
    (*pi).ek_1 = 0.0f;  //=pi->ek_1 = 0.0f;
    (*pi).yk_1 = 0.0f;
    (*pi).yk = 0.0f;
}


float PI_Update(PI_controller *pi, float soll, float ist)
{
    float ek = soll - ist;

    /*  Berechnet den aktuellen Regler-Output   */
    pi->yk = (pi->Kp + pi->Ki * pi->Ts) * ek - pi->Kp * pi->ek_1 + pi->yk_1;

    /*  Beschränkung der Regler-Ausgangsgröße   */
    if (pi->yk > pi->limMax){pi->yk = pi->limMax;}
    if (pi->yk < pi->limMin){pi->yk = pi->limMin;}

    /*  Aktualisierung der Differenzenwerte */
    pi->ek_1 = ek;
    pi->yk_1 = pi->yk;

    return pi->yk;
}

