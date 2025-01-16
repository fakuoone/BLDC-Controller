/*
 * functions.h
 *
 *  Created on: 24.06.2022
 *      Author: Fabian
 */
#include <stdbool.h>

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

/*  Prototypes  */
/*  Funktional  */
unsigned int shutdown_motor(void);
unsigned int release_shutdown(void);
void buck_off(void);
void buck_on(void);
unsigned int read_sci_data(char *data_storage, unsigned int length);
unsigned int write_sci_data(char *data_storage, unsigned int allow_transmit);
unsigned int copy_string(char *data_storage, char *new_data_storage);
void create_transmit_data_block(char *char_out, float speed_is, float I_S2, float I_S4, float I_S6, float I_total, float V_Gen);
void update_status_leds(unsigned int bit_pattern);

void update_PWM_sector_control(unsigned int rotor_position, unsigned int enable_bit, unsigned int pwm_cmp);

#endif /* FUNCTIONS_H_ */
