/**
 * @file    Motor.h
 * @brief   Motor header file
 * @details If you want to use Motor methods, Include Motor.h file.
*/
#ifndef MOTOR_H_ 
#define MOTOR_H_
#include "project.h"
#include "Reflectance.h"
#include <stdbool.h>
void motor_start(); // start motor PWM timers
void motor_stop();  // stop motor PWM timers

/* moving forward */
void motor_forward(int speedl,int speedr,uint32 delay);

/* moving left when l_speed < r_speed or moving right when l_speed > r_speed */
void motor_turn(int l_speed, int r_speed, uint32 delay);

/* moving backward */
void motor_backward(uint8 speedl,uint8 speedr, uint32 delay);
void balance_move_dig(struct sensors_ dig, int value);
void balance_move_ref(struct sensors_ ref, int value, int speed);
void motor_Sturnleft(struct sensors_ dig);
void motor_Sturnright(struct sensors_ dig);
void motor_Cturnleft(struct sensors_ dig);
void motor_Cturnright(struct sensors_ dig);
void balance_move_one(struct sensors_ dig);
#endif