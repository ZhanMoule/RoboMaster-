#include "Gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"

float pitch_angle =0.0;

float velocity=0;



void Gimbal_task(){
		
	
	
    Gimbal_Init();
		
    while (1){
				data_pack();
				Can_Transmit(board_data_exchange);	
			
				if(rc->data.RChandle.switch_left==1){
						pitch_angle+=rc->data.RChandle.rocker_ry*0.000001;	
						if(pitch_angle>0.6)
								pitch_angle=0.6;
						if(pitch_angle<-0.2)
								pitch_angle=-0.2;
						
						Motor_Dji_Control(shoot_motor[0], 0);	
						Motor_Dji_Transmit(shoot_motor[0]);
						Motor_Dji_Control(shoot_motor[1], 0);	
						Motor_Dji_Transmit(shoot_motor[1]);
				}
				else if (rc->data.RChandle.switch_left==3){
						Motor_Dji_Control(shoot_motor[0], 100);	
						Motor_Dji_Transmit(shoot_motor[0]);
						Motor_Dji_Control(shoot_motor[1], -100);	
						Motor_Dji_Transmit(shoot_motor[1]);
				}
				
				Motor_Dm_Control(gimbal_pitch_motor,pitch_angle);
        Motor_Dm_Mit_Control(gimbal_pitch_motor, 0.0f, 0.0f, gimbal_pitch_motor->output); // 电机为MIT模式
        Motor_Dm_Transmit(gimbal_pitch_motor); //发送控制报文
			
				osDelay(1); 
    }


}