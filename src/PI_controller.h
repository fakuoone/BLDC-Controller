/*
 * PI_controller.h
 *
 *  Created on: 27.06.2022
 *      Author: Fabian
 */

#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

typedef struct
{
    float Kp;
    float Ki;

    float limMax;   //highest output
    float limMin;   //lowest output

    float Ts;        //sample Time

    float ek_1;     //previous error
    float yk_1;     //previous output

    float yk;       //output
} PI_controller;

void PI_Init(PI_controller *pi);
float PI_Update(PI_controller *pi, float soll, float ist);

#endif /* PI_CONTROLLER_H_ */
