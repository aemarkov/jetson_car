#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>
#include <stdbool.h>

void Motors_Init(void);

void L_Motor_SetDirection(bool a, bool b);
void R_Motor_SetDirection(bool a, bool b);

/**
    \brief Управляет скоростью и направлением вращения моторов
    <0 - назад
     0 - GND-стоп
    >0 - вперед
    \param[in] left Скорость левого двигателя [-128; 127]
    \param[in] right Скорость правого двигателя [-128; 127]
*/
void Motors_Control(int8_t left, int8_t right);

#endif
