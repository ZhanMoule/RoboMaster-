#include "Gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"

float pitch_angle =0.0f;
float yaw_angle =0.0f;
float rammer_speed=0.0f;

int enable_flag=0;
int shoot_mode=0;

void Gimbal_task(){
		
    Gimbal_Init();
		enable_flag=0;
	
		yaw_angle =IMU_angle[0];
		pitch_angle =180;
	
		gimbal_data.yaw_position=yaw_motor->message.position;
		gimbal_data.chassis_vx=0.0f;
		gimbal_data.chassis_vx=0.0f;
		gimbal_data.chassis_mode=0;
		
    while (1){
			
				if((rc->dr16_handle.s1!=1&&rc->dr16_handle.s1!=2&&rc->dr16_handle.s1!=3)||rc->dr16_handle.ch1>660||rc->dr16_handle.ch1<-660||rc->dr16_handle.ch0>660||rc->dr16_handle.ch1<-660||rc->dr16_handle.ch3>660||rc->dr16_handle.ch3<-660||rc->dr16_handle.ch2>660||rc->dr16_handle.ch2<-660){
					rc->dr16_handle.s1=1;
					rc->dr16_handle.s2=1;
					rc->dr16_handle.ch1=0;
					rc->dr16_handle.ch1=0;
					rc->dr16_handle.ch1=0;
					rc->dr16_handle.ch1=0;
				}
			
				gimbal_data.yaw_position=yaw_motor->message.position;
						
				data_pack();
				Can_Transmit(board_data_exchange);	
			
			if(rc->dr16_handle.s1==1&&rc->dr16_handle.s2==1){		//双拨杆都在上时失能，其他情况都使能				
						if(enable_flag==1){
								Motor_Dji_Control(shoot_motor1, 0);	
								Motor_Dji_Control(shoot_motor2, 0);
							
								gimbal_data.chassis_mode=0;
							
								Motor_Dm_Cmd(rammer_motor,DM_CMD_MOTOR_DISABLE);
								Motor_Dm_Transmit(rammer_motor);
							
								Motor_Dm_Cmd(gimbal_pitch_motor,DM_CMD_MOTOR_DISABLE);
								Motor_Dm_Transmit(gimbal_pitch_motor);
							
								Motor_Dm_Cmd(yaw_motor,DM_CMD_MOTOR_DISABLE);
								Motor_Dm_Transmit(yaw_motor);
						}
						enable_flag=0;
				}
				else{
						if(enable_flag==0){								
								
								yaw_angle =IMU_angle[0];
								pitch_angle =180;
								
								Motor_Dm_Cmd(rammer_motor,DM_CMD_MOTOR_ENABLE);
								Motor_Dm_Transmit(rammer_motor);
							
								Motor_Dm_Cmd(gimbal_pitch_motor,DM_CMD_MOTOR_ENABLE);
								Motor_Dm_Transmit(gimbal_pitch_motor);
							
								Motor_Dm_Cmd(yaw_motor,DM_CMD_MOTOR_ENABLE);
								Motor_Dm_Transmit(yaw_motor);
						}
						enable_flag=1;
				}
				
				if(enable_flag==1){
						
						pitch_angle+=rc->dr16_handle.ch1*0.0001;		
						if(pitch_angle>200)
						  	pitch_angle=200;
						if(pitch_angle<142)
								pitch_angle=142;
					
						yaw_angle-=rc->dr16_handle.ch2*0.0001;									
						if(yaw_angle>360)
						  	yaw_angle-=360;
						if(yaw_angle<0)
								yaw_angle+=360;
						
				  		gimbal_data.chassis_vx=rc->dr16_handle.ch2/132.0;
						gimbal_data.chassis_vy=rc->dr16_handle.ch3/132.0;
						
						if(rc->dr16_handle.s2==3){
								if(rc->dr16_handle.s1==1){
										gimbal_data.chassis_mode=1;
								}
								if(rc->dr16_handle.s1==3){
										gimbal_data.chassis_mode=2;
								}
								if(rc->dr16_handle.s1==2){
										gimbal_data.chassis_mode=3;
								}															
						}
						if(rc->dr16_handle.s1==2){
								if(rc->dr16_handle.s1==1){
										shoot_mode=0;
								}
								if(rc->dr16_handle.s1==3){
										shoot_mode=1;
								}
								if(rc->dr16_handle.s1==2){
										shoot_mode=0;
								}															
						}
				}
				
				if(shoot_mode==1){					
						Motor_Dji_Control(shoot_motor1, 3000);	
						Motor_Dji_Control(shoot_motor2, -3000);	
						if(rc->dr16_handle.wheel>1300){
								rammer_speed=0.5;
						}
						else if(rc->dr16_handle.wheel<700)
								rammer_speed=-0.5;
						else 
								rammer_speed=0;			
				}
				else{
						rammer_speed=0;	
						Motor_Dji_Control(shoot_motor1, 0);	
						Motor_Dji_Control(shoot_motor2, 0);
				}
				
				
				Motor_Dji_Transmit(shoot_motor1);
				Motor_Dji_Transmit(shoot_motor2);
				
				//Motor_Dm_Control(gimbal_pitch_motor,0);
				Motor_Dm_Control(gimbal_pitch_motor,Pid_Calculate(gimbal_pitch_motor->angle_pid, pitch_angle, IMU_angle[1]));
        Motor_Dm_Mit_Control(gimbal_pitch_motor, 0.0f, 0.0f, gimbal_pitch_motor->output); 
        Motor_Dm_Transmit(gimbal_pitch_motor); //发送控制报文
				
				//Motor_Dm_Control(yaw_motor, Pid_Calculate(yaw_motor->angle_pid, yaw_angle, IMU_angle[0]));
				Motor_Dm_Control(yaw_motor, Pid_Calculate(yaw_motor->angle_pid, yaw_angle, IMU_angle[0]));
        Motor_Dm_Mit_Control(yaw_motor, 0.0f, 0.0f, yaw_motor->output);
        Motor_Dm_Transmit(yaw_motor); 
				
				Motor_Dm_Control(rammer_motor, rammer_speed);
        Motor_Dm_Mit_Control(rammer_motor, 0.0f, 0.0f, rammer_motor->output);
        Motor_Dm_Transmit(rammer_motor); 
		
				osDelay(1); 
    }


}