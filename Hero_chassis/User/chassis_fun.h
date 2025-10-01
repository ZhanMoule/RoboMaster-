#ifndef CHASSIS_FUN_H
#define CHASSIS_FUN_H

#include <stdbool.h>
#include "dev_motor_dji.h"
#include "dev_remote_control.h"
#include "dev_motor_dm.h"
#include "alg_chassis_calc.h"

typedef struct {
    int16_t rocker_rx;
    int16_t rocker_ry;
    int16_t rocker_lx;
    int16_t rocker_ly;
    uint8_t switch_left;   // ȡֵ1,2,3
    uint8_t switch_right;  // ȡֵ1,2,3
	  float ins_yaw_angle;
} RC_Handle;


void Chassis_Motor_Init();

void data_unpack(CanInstance_s *can_instance);


#endif // CHASSIS_MOTOR_H