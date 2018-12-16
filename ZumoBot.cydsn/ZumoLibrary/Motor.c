/**
 * @file    Motor.c
 * @brief   Basic methods for operating motor sensor. For more details, please refer to Motor.h file. 
 * @details included in Zumo shield
*/
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Reflectance.h"
/**
* @brief    Starting motor sensors
* @details  
*/
void motor_start()
{
    PWM_Start();
}


/**
* @brief    Stopping motor sensors
* @details
*/
void motor_stop()
{
    PWM_Stop();
}


/**
* @brief    Moving motors forward
* @details  giveing same speed to each side of PWM to make motors go forward
* @param    uint8 speed : speed value
* @param    uint32 delay : delay time
*/
void motor_forward(int speedl,int speedr,uint32 delay)
{
    
    MotorDirLeft_Write(0);      // set LeftMotor forward mode
    MotorDirRight_Write(0);     // set RightMotor forward mode
     if (speedl<0)  
        {
            MotorDirLeft_Write(1);
            speedl=-speedl;
        }
    if (speedr<0)  
        {
            MotorDirRight_Write(1);
            speedr=-speedr;
        }
    PWM_WriteCompare1(speedl); 
    PWM_WriteCompare2(speedr); 
    vTaskDelay(delay);
}


/**
* @brief    Moving motors to make a turn 
* @details  moving left when l_speed < r_speed or moving right when l_speed > r_speed
* @param    uint8 l_speed : left motor speed value
* @param    uint8 r_speed : right motor speed value
* @param    uint32 delay : delay time
*/
void motor_turn(int l_speed, int r_speed, uint32 delay)
{
    if (l_speed<0)  
        {
            MotorDirLeft_Write(1);
            l_speed=-l_speed;
        }
    if (r_speed<0)  
        {
            MotorDirRight_Write(1);
            r_speed=-r_speed;
        }
    PWM_WriteCompare1(l_speed); 
    PWM_WriteCompare2(r_speed); 
    vTaskDelay(delay);
}


/**
* @brief    Moving motors backward
* @details  setting backward mode to each motors and gives same speed to each side of PWM
* @param    uint8 speed : speed value
* @param    uint32 delay : delay time
*/
void motor_backward(uint8 speedl,uint8 speedr, uint32 delay)
{
    MotorDirLeft_Write(1);      // set LeftMotor backward mode
    MotorDirRight_Write(1);     // set RightMotor backward mode
    PWM_WriteCompare1(speedl); 
    PWM_WriteCompare2(speedr); 
    vTaskDelay(delay);
}
void balance_move_dig(struct sensors_ dig, int value)
{
    if (dig.l1 == 1 && dig.r1 == 1)
				motor_forward(value, value, 20);
			else if (dig.l1 == 0 && dig.r1 == 1)
				motor_turn(value, 00, 20);
			else if (dig.l1 == 1 && dig.r1 == 0)
				motor_turn(00, value, 20);
}
void balance_move_ref(struct sensors_ ref, int value, int speed)
{
    if (ref.l1 >= value && ref.r1 >= value)
				motor_forward(speed, speed, 1);
			else if (ref.l1 < value && ref.r1 >= value)
				motor_turn(250, -60, 10);
			else if (ref.l1 >= value && ref.r1 < value)
				motor_turn(-60, 250, 10);
}
void motor_Sturnleft(struct sensors_ dig)
{
     while((dig.l3 == 1) || (dig.l2 == 1) || (dig.l1 == 0) || (dig.r1 == 0) || (dig.r2 == 1) || (dig.r3 == 1))
                {
                    reflectance_digital(&dig);
                    motor_turn(-150, 150, 10);
                }
}
void motor_Sturnright(struct sensors_ dig)
{
    while (dig.r3!=1)
    {
        reflectance_digital(&dig);
        motor_turn(150, -150, 10);
    }
    while((dig.l3 == 1) || (dig.l2 == 1) || (dig.l1 == 0) || (dig.r1 == 0) || (dig.r2 == 1) || (dig.r3 == 1))
    {
        reflectance_digital(&dig);
        motor_turn(150, -150, 10);
    }      
}
void motor_Cturnleft(struct sensors_ dig)
{
     while(((dig.l3 == 1) || (dig.l2 == 1) || (dig.l1 == 0) || (dig.r1 == 0) || (dig.r2 == 1) || (dig.r3 == 1)))
                {
                    reflectance_digital(&dig);
                    motor_turn(-0, 125, 1);
                }
}
void motor_Cturnright(struct sensors_ dig)
{
    while(((dig.l3 == 1) || (dig.l2 == 1) || (dig.l1 == 0) || (dig.r1 == 0) || (dig.r2 == 1) || (dig.r3 == 1)))
                {
                    reflectance_digital(&dig);
                    motor_turn(125, -0, 1);
                }
}
void balance_move_one(struct sensors_ dig)
{
    bool White;
    while(1)
    {
        if( dig.r3==0 && dig.l3==0)
            White= true;
        else White= false;
        while (White)
        {
            balance_move_dig(dig,100);
            reflectance_digital(&dig);
        }
    }
}