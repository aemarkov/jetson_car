#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>
#include <stdbool.h>

void Motors_Init(void);

void L_Motor_SetDirection(bool a, bool b);
void R_Motor_SetDirection(bool a, bool b);

/**
    \brief Control motor speed and direction
    <0 - backward
     0 - GND-stop
    >0 - forward
    \param[in] left  Left motor speed [-128; 127]
    \param[in] right Right motor speed [-128; 127]
*/
void Motors_Control(int8_t left, int8_t right);

#endif
