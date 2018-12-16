/**
* @mainpage ZumoBot Project
* @brief    You can make your own ZumoBot with various sensors.
* @details  <br><br>
    <p>
    <B>General</B><br>
    You will use Pololu Zumo Shields for your robot project with CY8CKIT-059(PSoC 5LP) from Cypress semiconductor.This 
    library has basic methods of various sensors and communications so that you can make what you want with them. <br> 
    <br><br>
    </p>
    
    <p>
    <B>Sensors</B><br>
    &nbsp;Included: <br>
        &nbsp;&nbsp;&nbsp;&nbsp;LSM303D: Accelerometer & Magnetometer<br>
        &nbsp;&nbsp;&nbsp;&nbsp;L3GD20H: Gyroscope<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Reflectance sensor<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Motors
    &nbsp;Wii nunchuck<br>
    &nbsp;TSOP-2236: IR Receiver<br>
    &nbsp;HC-SR04: Ultrasonic sensor<br>
    &nbsp;APDS-9301: Ambient light zsensor<br>
    &nbsp;IR LED <br><br><br>
    </p>
    
    <p>
    <B>Communication</B><br>
    I2C, UART, Serial<br>
    </p>
*/

#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <stdlib.h>

/**
 * @file    main.c
 * @brief   
 * @details  ** Enable global interrupt since Zumo library uses interrupts. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/

//Line follower
# if 00
void zmain(void)
{
    TickType_t Tready, Tline, Tstart, Tstop;
    int speed=100;
	struct sensors_ ref, dig;
	motor_start();
	motor_forward(0, 0, 0);
	vTaskDelay(300);
	int count = 0;
	bool White = false, Press = false, Miss= false;;
	IR_Start();
	IR_flush();
	reflectance_start();
	reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000);
	while (1)
	{
		if (SW1_Read() == 0)
		{
			Press = !Press;
			vTaskDelay(500);
		}
		if (Press == true)
		{
			reflectance_read(&ref);
			reflectance_digital(&dig);
			balance_move_ref(ref,15000,speed);
			if ((White == true) && (dig.l3 == 1) && (dig.r3 == 1))
			{
				White = false;
				count++;
			}
			else if ((dig.l3 == 0) && (dig.r3 == 0))
				White = true;
            
			if (count == 1)
			{
				Tline = xTaskGetTickCount();
				print_mqtt("Zumo047/ready", "line");
				print_mqtt("Zumo047/start", "%d", Tline - Tready);
				motor_forward(0, 0, 0);
                speed=255; 
				IR_wait();
				Tstart = xTaskGetTickCount();
				count++;
			}
			else if (count == 4)
			{
               Tstop = xTaskGetTickCount();
				print_mqtt("Zumo047/stop", "%d", Tstop - Tready);
				print_mqtt("Zumo047/time", "%d", Tstop - Tline);
				motor_stop();
				break;
			}
            else if (count>1 && count <4)
            {
                if (Miss == false && (dig.l3 == 0) && (dig.r3 == 0))
                {
                    Miss=true;
                    print_mqtt("Zumo047/miss", "%d", xTaskGetTickCount());
                }
                if (Miss == true && ((dig.l3 == 1) || (dig.r3 == 1)))
                {
                    Miss=false;
                    print_mqtt("Zumo047/line", "%d", xTaskGetTickCount());
                }
            }
            
		}
	}
}
#endif
// maze
#if 000 // maze
void movetomiddle(int c);
int maze[13][9] = { {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1, 0, 0, 0, 0, 0, 0, 0,-1},
				 {-1,-1,-1,-1,-1,-1,-1,-1,-1} };
int stack[84][2];
int row[3] = { 1,0,0 };
int col[3] = { 0,-1,1 };
int r = 0, c = 0, count = -1;
bool check = false;
TickType_t timestart = 0, timeend = 0;

void balance_move_dig1(struct sensors_ dig, int c) //Runs robot to the next intersection
{
	if ((c != 1) && (c != 7))
	{
		while (!((dig.l3 == 1) && (dig.r3 == 1))) {
			reflectance_digital(&dig);
			balance_move_dig(dig,100);
		}
		while (!((dig.l3 == 0) && (dig.r3 == 0))) {
			reflectance_digital(&dig);
			balance_move_dig(dig,100);
		}
	}
	else
	{
		if (c == 7)
		{
			while (!(dig.l3 == 1)) {
				reflectance_digital(&dig);
				balance_move_dig(dig,100);
			}
			while (!(dig.l3 == 0)) {
				reflectance_digital(&dig);
				balance_move_dig(dig,100);
			}
		}
		else
		{
			while (!(dig.r3 == 1)) {
				reflectance_digital(&dig);
				balance_move_dig(dig,100);
			}
			while (!(dig.r3 == 0)) {
				reflectance_digital(&dig);
				balance_move_dig(dig,100);
			}
		}
	}
	motor_forward(0, 0, 0);
	//vTaskDelay(100);
}
void balance_turn_dig(struct sensors_ dig, int direction, int left, int right) //undone
{
	if (direction == 2)
	{
		do
		{
			reflectance_digital(&dig);
			motor_turn(left, -right, 5);
		} while ((dig.l1 == 1) || (dig.r1 == 1));
		do
		{
			reflectance_digital(&dig);
			motor_turn(left, -right, 75);
		} while (dig.l1 == 0);
        do
		{
			reflectance_digital(&dig);
			motor_turn(left, -right, 0);
		} while (dig.r1 != 0);
		motor_forward(0, 0, 0);
	}
	else if (direction == 1)
	{
		do
		{
			reflectance_digital(&dig);
			motor_turn(-right, left, 5);
		} while ((dig.l1 == 1) || (dig.r1 == 1));
		do
		{
			reflectance_digital(&dig);
			motor_turn(-right, left, 75);
		} while (dig.r1 == 0);
        do
		{
			reflectance_digital(&dig);
			motor_turn(-right, left, 0);
		} while (dig.l1 != 0);
		motor_forward(0, 0, 0);
	}
	while (!((dig.l3 == 0) && (dig.r3 == 0)))
	{
		reflectance_digital(&dig);
		balance_move_dig(dig,100);
	}
	reflectance_digital(&dig);
	balance_move_dig(dig,100);
	motor_backward(50, 50, 50);
	motor_forward(0, 0, 0);
}


void checkobstacle(int *a) //Check whether there is a obstacle
{
	int p = 0;
	p = Ultra_GetDistance();
	if ((p < 18) && (p > 0)) {
		*a = -1;
	}
}

void pop(int count)
{
	struct sensors_ dig; reflectance_start();
	int y = stack[count][0] - stack[count - 1][0];
	int x = stack[count][1] - stack[count - 1][1];
	if (y == 0) {
		reflectance_digital(&dig);
		balance_turn_dig(dig, 2, 100, 95);
		reflectance_digital(&dig);
		balance_move_dig1(dig, maze[count - 1][1]);
		reflectance_digital(&dig);
		balance_turn_dig(dig, 1, 100, 95);
	}
	else {
		reflectance_digital(&dig);
		balance_turn_dig(dig, 2, 100, 95); balance_turn_dig(dig, 2, 100, 100);
		balance_move_dig1(dig, maze[count - 1][1]);
        motor_forward(100,100,60);
		reflectance_digital(&dig);
		balance_turn_dig(dig, 1, 100, 95); balance_turn_dig(dig, 1, 100, 100);
	}
}

void firststep() //Move robot to (0,0)
{
	uint8 button = 1; struct sensors_ dig; bool condition = false; int count = 0;
	reflectance_start(); motor_start(); IR_Start(); IR_flush();

	motor_forward(0, 0, 0);
	do
	{
		button = SW1_Read();
		if (!button)
		{
			for (;;)
			{
				motor_forward(50, 50, 5);
				reflectance_digital(&dig);
				if ((dig.l3 == 1) && (dig.r3 == 1))
				{
					motor_forward(0, 0, 0);
					condition = true;
					break;
				}
			}
		}
	} while (condition == false);
    print_mqtt("Zumo047/ready", "maze");
	condition = false;
	IR_wait();
    timestart= xTaskGetTickCount();
    print_mqtt("Zumo047/start", "%d", timestart);

	condition = !condition;
	vTaskDelay(500);

	do
	{
		reflectance_digital(&dig);
		balance_move_dig(dig,100);
		if ((dig.l3 == 1) && (dig.r3 == 1) && (condition == false))
		{
			condition = true;
		}
		if ((dig.l3 == 0) && (dig.r3 == 0) && (condition == true))
		{
			condition = false;
			count++;
		}
	} while (count != 2);

	motor_forward(0, 0, 0);
}

void dfs(int r, int c, int direction) //Find the way
{
	struct sensors_ dig; reflectance_start();
	if (check == true) return;

	maze[r][c] = -1;
	count++; stack[count][0] = r; stack[count][1] = c;

	print_mqtt("Zumo047/position", "%d %d", r, c - 4);

	switch (direction)
	{
	case 0:
		checkobstacle(&maze[r + row[direction]][c + col[direction]]);
		break;

	case 1:
		checkobstacle(&maze[r + row[direction]][c + col[direction]]);
		direction = 2; reflectance_digital(&dig); balance_turn_dig(dig, direction, 100, 95); direction = 0;
		checkobstacle(&maze[r + row[direction]][c + col[direction]]);
		break;

	case 2:
		checkobstacle(&maze[r + row[direction]][c + col[direction]]);
		direction = 1; reflectance_digital(&dig); balance_turn_dig(dig, direction, 100, 95); direction = 0;
		checkobstacle(&maze[r + row[direction]][c + col[direction]]);
		break;
	}
	if (r == 11)
	{
		motor_forward(0, 0, 0); check = true; movetomiddle(c);
		return;
	}

	for (int i = 0; i < 3; i++)
	{
		if ((r + row[i] >= 0) && (r + row[i] <= 11) && (c + col[i] >= 0) && (c + col[i] <= 8) && (maze[r + row[i]][c + col[i]] != -1) && (check == false))
		{
			if (i != 0) {
				reflectance_digital(&dig);
				balance_turn_dig(dig, i, 100, 95);
				checkobstacle(&maze[r + row[i]][c + col[i]]);

				if (maze[r + row[i]][c + col[i]] != -1) {
					direction = i;
					reflectance_digital(&dig);
					balance_move_dig1(dig, c);
					dfs(r + row[i], c + col[i], direction);
				}
				else 
                {
                    if (i==1) balance_turn_dig(dig, 2, 100, 95);
                    else  balance_turn_dig(dig, 1, 100, 95);
                }
			}
			else
			{
				direction = i;
				reflectance_digital(&dig);
				balance_move_dig1(dig, c);
				dfs(r + row[i], c + col[i], direction);
			}
		}
	}
	if (check == false) {
		pop(count); count--;
		maze[r][c] = -1; maze[stack[count][0]][stack[count][1]] = -1;
        print_mqtt("Zumo047/position", "%d %d", stack[count][0],stack[count][1]-4);
	}
}

void movetomiddle(int c) //Move robot to (13,0)
{
	struct sensors_ dig; reflectance_start();
    r=11;
	if (c != 4)
	{
		if (c < 4)
		{
			reflectance_digital(&dig);
			balance_turn_dig(dig, 2, 100, 95);
			while (c != 4) {
				reflectance_digital(&dig);
				balance_move_dig1(dig, c);
				c++;
                print_mqtt("Zumo047/position", "%d %d", r, c - 4);
			}
			balance_turn_dig(dig, 1, 100, 95);
		}
		else
		{
			reflectance_digital(&dig);
			balance_turn_dig(dig, 1, 100, 95);
			while (c != 4) {
				reflectance_digital(&dig);
				balance_move_dig1(dig, c);
				c--;
                print_mqtt("Zumo047/position", "%d %d", r, c - 4);
			}
			balance_turn_dig(dig, 2, 100, 95);
		}
	}
}

void laststep() //Move robot from (13,0) to the finish position
{
	motor_forward(0, 0, 0);
	struct sensors_ dig; bool black = false; int count = 0;
	reflectance_start();
    r=11;
	do
	{
		reflectance_digital(&dig);
		balance_move_dig(dig,100);
		if ((dig.l3 == 1) && (dig.r3 == 1) && (black == false))
		{
			black = true;
			count++; r++;
            print_mqtt("Zumo047/position", "%d %d", r, c - 4);
		}
		if ((dig.l3 == 0) && (dig.r3 == 0) && (black == true))
		{
			black = false;
		}
	} while (count != 2);

	motor_forward(50, 50, 3000);
	motor_forward(0, 0, 0);
	motor_stop();
}

void zmain(void)
{
    
	firststep();
	Ultra_Start();

	r = 0, c = 4;
	dfs(r, c, 0);
	laststep();
    timeend= xTaskGetTickCount();
    print_mqtt("Zumo047/time", "%d", timeend-timestart);
}
#endif
// sumo
#if 10 // sumo
struct sensors_ ref;
struct sensors_ dig;
struct accData_ data;
   
int count = 0,timeDiff = 0;
int currentValueX = 0, newValueX = 0, diffX = 0;
int currentValueY = 0, newValueY = 0, diffY = 0;
   
bool isHit = false;
bool isWhite = true;
bool isPressed = true;
    
TickType_t timeIR = 0, timeHit = 0, timeEnd = 0;
    
void checkHit(void) {
    LSM303D_Read_Acc(&data);
    currentValueX = data.accX;
    currentValueY = data.accY;
            
    LSM303D_Read_Acc(&data);
    newValueX = data.accX;
    newValueY = data.accY;
          
    diffX = currentValueX - newValueX;
    diffY = currentValueY - newValueY;
    
    if (isHit == false && diffX > 10000){
        isHit = true;
        timeHit = xTaskGetTickCount();
        print_mqtt("Zumo047/hit", "%d", timeHit);
    }
    else if (isHit == true && diffX <= 10000) {
        isHit = false;
    }
}

void checkBW(void) {
    reflectance_digital(&dig);
    if (isWhite == true && (dig.r3 == 1 || dig.l3 == 1)) {
        isWhite = false;
        count++;
        if (count == 1) {
            motor_forward(0,0,0);
            print_mqtt("Zumo047/ready","zumo");
            IR_wait();
            timeIR = xTaskGetTickCount();
            print_mqtt("Zumo047/start", "%d", timeIR);
        }
        else if (count > 1) {
            motor_forward(0,0,0);
            if (dig.r3 == 1) {
                motor_backward(255,255,50);
                motor_forward(-255,255,220);
            }
            else if (dig.l3 == 1) {
                motor_backward(255,255,50);
                motor_forward(255,-255,220);
            }
        }
    }
    else if (dig.l3 == 0 && dig.r3 == 0) {
        isWhite = true;
        if (count > 1) {
            checkHit();
        }
    }
}
    
void zmain(void) {
    motor_start();
    motor_forward(0,0,0);
    
    IR_Start();
    IR_flush();
    
    reflectance_start();
    reflectance_set_threshold(11000, 11000, 15000, 15000, 11000, 11000);
    
    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else {
        printf("Device Ok...\n");
    }
    
    for(;;){
        if (SW1_Read()== 0) {
            isPressed = !isPressed;
            vTaskDelay(500);
        }
        if(isPressed == false) {
            motor_forward(255,255,0);
              
            //reflectance_read(&ref);
            
            checkBW();
            if (SW1_Read()== 0){ 
                motor_forward(0,0,0);
                isPressed= !isPressed;
                timeEnd = xTaskGetTickCount();
                print_mqtt("Zumo047/stop", "%d", timeEnd);
                timeDiff= timeEnd - timeIR;
                print_mqtt("Zumo047/time", "%d", timeDiff);
                break;
            }
        }
    }    
}
#endif
//test mqtt
#if 0 
    void zmain(void)
    {
        while(1)
        print_mqtt("Zumo047/Greeting", "Hello");
    }
#endif
/* [] END OF FILE */
