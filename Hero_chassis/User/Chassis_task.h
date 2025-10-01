#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "chassis_fun.h"
#include "main.h"
#include "CMSIS_os.h"

extern DjiMotorInstance_s *wheel_motor[4];

extern DmMotorInstance_s *yaw_motor;

extern DmMotorInstance_s *rammer_motor;

extern ChassisInstance_s *Chassis;

extern CanInstance_s *board_data_exchange;

extern RC_Handle remote_ctrl;

#endif
