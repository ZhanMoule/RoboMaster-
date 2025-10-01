#include "Chassis_task.h"

float yaw_angle=0;



void Chassis_task(){
		
		
		Chassis_Motor_Init();
		
		Can_Transmit(board_data_exchange);
		data_unpack(board_data_exchange);	
	
		yaw_angle=remote_ctrl.ins_yaw_angle;
	
	
		Chassis->absolute_chassis_speed.Vx=0.0;
		Chassis->absolute_chassis_speed.Vy=0.0;
		
		Chassis_Mode_Choose(Chassis,CHASSIS_NORMAL);
	
		while(1){						
				Can_Transmit(board_data_exchange);
				data_unpack(board_data_exchange);	

				if(remote_ctrl.switch_right == 1){
					
						Chassis_Mode_Choose(Chassis,CHASSIS_NORMAL);
						
        }
        else if (remote_ctrl.switch_right  == 3){
						Chassis_Mode_Choose(Chassis,CHASSIS_FOLLOW_GIMBAL);

        }
				else if (remote_ctrl.switch_right  == 2){
					  Chassis_Mode_Choose(Chassis,CHASSIS_GYROSCOPE);
        }
				if(remote_ctrl.switch_left == 3){
						Motor_Dm_Control(rammer_motor,remote_ctrl.rocker_ry/132);				
				}
				
				if(remote_ctrl.switch_left != 3){
						if(remote_ctrl.switch_right == 3 || remote_ctrl.switch_right == 1||remote_ctrl.switch_right == 2){
								yaw_angle-=remote_ctrl.rocker_rx*0.000005;	
								
								if(yaw_angle>2*PI)
										yaw_angle-=2*PI;
								if(yaw_angle<-2*PI)
										yaw_angle+=2*PI;	
						}
				}
				
				
				
				Chassis->absolute_chassis_speed.Vx=remote_ctrl.rocker_lx /330.0;
				Chassis->absolute_chassis_speed.Vy=-(remote_ctrl.rocker_ly /330.0);
				
				

        Chassis->gimbal_yaw_angle=yaw_motor->message.position;
        Chassis_Control(Chassis);	


				Motor_Dm_Control(yaw_motor, Pid_Calculate(yaw_motor->angle_pid, yaw_angle, remote_ctrl.ins_yaw_angle)); // 计算力矩
        Motor_Dm_Mit_Control(yaw_motor, 0.0f, 0.0f, yaw_motor->output); // 电机为MIT模式
        Motor_Dm_Transmit(yaw_motor); //发送控制报文

				
				
		    osDelay(1);
			
		}
}
