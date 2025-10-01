#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__  

#include "Gimbal_Fun.h"
extern RC_instance *rc;
extern DmMotorInstance_s  *gimbal_pitch_motor;
extern DmMotorInitConfig_s  *gimbal_pitch_motor_config;
extern DjiMotorInstance_s *shoot_motor[4];
extern CanInstance_s *board_data_exchange;


#endif 
