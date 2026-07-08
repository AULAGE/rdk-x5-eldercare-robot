#ifndef _control_H
#define _control_H

#include "system.h"
#include "pwm.h"
#include "usart.h"

//#define l				  	500
#define MAX_SPD			   500            // 最大速度
#define DEAD_ZONE   	 35           // 摇杆中位死区
#define SMOOTH				  3					  	//滤波

void Control_bluetooth(void);
void Dance(void);

#endif



