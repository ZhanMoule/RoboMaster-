#include "chassis_fun.h"
#include "bsp_can.h"
#include "main.h"

DmMotorInstance_s *yaw_motor;

DmMotorInstance_s *rammer_motor;

ChassisInstance_s *Chassis;

CanInstance_s *board_data_exchange;

RC_Handle remote_ctrl;



CanInitConfig_s board_data_exchange_config ={
		.topic_name = "data_exchange",
		.can_number = 2,
    .tx_id = 0x301,
    .rx_id = 0x300,
    .can_module_callback = data_unpack
};

static ChassisInitConfig_s Chassis_config={
    .type = Mecanum_Wheel,
		.gimbal_yaw_zero = -1.9,
		.mecanum_steering_message={
		.wheel_radius= 0.12f,
	  .length_a = 0.230f,
    .length_b = 0.330f,
		}, 
		.gimbal_follow_pid_config ={
				.kp = 5.0f,
				.ki = 0.0f,
				.kd = 0.0f,
				.i_max = 1800.0f,
				.out_max = 8192.0f,
		},
		.motor_config[1] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 19.0f,
    .topic_name = "1",
    .can_config = {
    .can_number=1,
		.rx_id=0x201,	
		.tx_id=0x200,
    },
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  },
    .motor_config[0] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 19.0f,
    .topic_name = "2",
    .can_config = {
    .can_number=1,
		.rx_id=0x202,	
		.tx_id=0x200,
    },
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  },
    .motor_config[3] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 19.0f,
    .topic_name = "3",
    .can_config = {
    .can_number=1,
		.rx_id=0x203,	
		.tx_id=0x200,
		},
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  },
    .motor_config[2] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .topic_name = "4",
   .can_config = {
    .can_number=1,
		.rx_id=0x204, 
		.tx_id=0x200,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  }
};

DmMotorInitConfig_s yaw_motor_config={
		.topic_name = "yaw_motor",
		.control_mode = DM_VELOCITY,
		.type = J4310,
		.can_config ={
					.can_number=1,
					.tx_id = 0x03, 
					.rx_id = 0x14,
		},
		.parameters = {
        .pos_max = 3.141593f,
        .vel_max = 20.0f,   
        .tor_max = 18.0f,
        .kp_max = 0.0f,
        .kd_max = 0.0f,
        .kp_int = 0.0f,
        .kd_int = 0.0f,
    },
		.velocity_pid_config={
				.kp=0.80f,
				.ki=0.00025f,
				.kd=0.0,
				.i_max =4,
				.out_max =8,
		},
		.angle_pid_config={
				.kp=15.0f,
				.ki=0,
				.kd=0,
				.angle_max = 2*PI,
				.i_max=3,
				.out_max=8,
		},
};

DmMotorInitConfig_s rammer_motor_config={
		.control_mode = DM_VELOCITY,
		.topic_name = "rammer_motor",
		.can_config ={
					.can_number=2,
					.tx_id = 0x05, 
					.rx_id = 0x15,
		},
		.parameters = {
        .pos_max = 3.141593f,
        .vel_max = 20.0f,   
        .tor_max = 18.0f,
        .kp_max = 0.0f,
        .kd_max = 0.0f,
        .kp_int = 0.0f,
        .kd_int = 0.0f,
    },
		.velocity_pid_config={
				.kp=0.35f,
				.ki=0.01f,
				.kd=1.5,
				.i_max =10,
				.out_max =3,
		},
		.angle_pid_config={
				.kp=18.0f,
				.ki=0.005,
				.kd=5,
				.angle_max = 2*PI,
				.i_max=10,
				.out_max=5,
		},
};

void Chassis_Motor_Init(){
		board_data_exchange = Can_Register(&board_data_exchange_config);
		
	  yaw_motor = Motor_DM_Register(&yaw_motor_config);
		Motor_Dm_Cmd(yaw_motor,DM_CMD_MOTOR_ENABLE);
	
		rammer_motor = Motor_DM_Register(&rammer_motor_config);
		Motor_Dm_Cmd(rammer_motor,DM_CMD_MOTOR_ENABLE);
	
		Chassis = Chassis_Register(&Chassis_config);
	
}


void data_unpack(CanInstance_s *can_instance)
{
    if(can_instance == NULL){
        remote_ctrl.rocker_rx = 0;
        remote_ctrl.rocker_ry = 0;
        remote_ctrl.rocker_lx = 0;
        remote_ctrl.rocker_ly = 0;
        remote_ctrl.switch_left = 0;
        remote_ctrl.switch_right = 0;
        remote_ctrl.ins_yaw_angle = 0.0f;
        return;
    }
		else{
		
    
				// 摇杆数据处理（保持不变）
				uint16_t rx = ((uint16_t)can_instance->rx_buff[0]) | 
										 (((uint16_t)(can_instance->rx_buff[1] & 0x07)) << 8);
				uint16_t ry = (((uint16_t)(can_instance->rx_buff[1] & 0xF8)) >> 3) | 
										 (((uint16_t)(can_instance->rx_buff[2] & 0x3F)) << 5);
				uint16_t lx = (((uint16_t)(can_instance->rx_buff[2] & 0xC0)) >> 6) | 
										 (((uint16_t)can_instance->rx_buff[3]) << 2) |
										 (((uint16_t)(can_instance->rx_buff[4] & 0x01)) << 10);
				uint16_t ly = (((uint16_t)(can_instance->rx_buff[4] & 0xFE)) >> 1) | 
										 (((uint16_t)(can_instance->rx_buff[5] & 0x0F)) << 7);

				// 拨杆开关处理（保持不变）
				uint8_t sw_left = ((can_instance->rx_buff[5] & 0x30) >> 4) + 1;
				uint8_t sw_right = ((can_instance->rx_buff[5] & 0xC0) >> 6) + 1;

				// 摇杆数据处理（保持不变）
				remote_ctrl.rocker_rx = (int16_t)rx - 660;
				remote_ctrl.rocker_ry = (int16_t)ry - 660;
				remote_ctrl.rocker_lx = (int16_t)lx - 660;
				remote_ctrl.rocker_ly = (int16_t)ly - 660;
				remote_ctrl.switch_left = sw_left;
				remote_ctrl.switch_right = sw_right;
				
				// 新增：INS角度处理（使用字节6）
				uint8_t yaw_byte = can_instance->rx_buff[6];
				remote_ctrl.ins_yaw_angle = (float)yaw_byte * (4.0f * PI / 255.0f) - 2.0f * PI;
				
				if(remote_ctrl.rocker_rx>660||remote_ctrl.rocker_rx<-660)
					remote_ctrl.rocker_rx=0;
				if(remote_ctrl.rocker_ry>660||remote_ctrl.rocker_ry<-660)
					remote_ctrl.rocker_ry=0;
				if(remote_ctrl.rocker_lx>660||remote_ctrl.rocker_lx<-660)
					remote_ctrl.rocker_lx=0;
				if(remote_ctrl.rocker_ly>660||remote_ctrl.rocker_ly<-660)
					remote_ctrl.rocker_ly=0;
		}
}