#include <stdlib.h>
#include <stdio.h>
#include "joint.h"


#define LIMIT_RANGE(Pos, Min, Max) ((Pos) = ((Pos) > (Max) ? (Max) : (Pos) < (Min) ? (Min) : (Pos)))

// 默认电机初始零点
//float zero_left_ID0  = 0.0f;
//float zero_left_ID1  = 0.0f;
//float zero_right_ID0 = 0.0f;
//float zero_right_ID1 = 0.0f;

// 初始化4组电机零点变量
float zero_group1_ID0 = 0.0f;
float zero_group1_ID1 = 0.0f;

float zero_group2_ID0 = 0.0f;
float zero_group2_ID1 = 0.0f;

float zero_group3_ID0 = 0.0f;
float zero_group3_ID1 = 0.0f;

float zero_group4_ID0 = 0.0f;
float zero_group4_ID1 = 0.0f;

float current_pos[CABLE_NUM];
float zeros[CABLE_NUM];

float traj_start_angles[CABLE_NUM] ={0};

uint8_t continuity = 1;

uint8_t STOP = False;

//static float home_speed  = 0.4f;  // 减速后角速度 rad/s
//static float home_torque = 1.0f;  // 减速后力矩 Nm
//static float UP_LIMIT    = 20.0f; // 减速后角度 °
//static float DOWN_LIMIT  = 80.0f; // 减速后角度 °
//static float TOLERANCE   = -5.0f;  // 容差 °

// 创建底盘结构体
//Chassis_ME_t *Chassis_Init()
//{
//    Chassis.zero_l_ID0 = 0.0f;
//    Chassis.zero_l_ID1 = 0.0f;
//    Chassis.zero_r_ID0 = 0.0f;
//    Chassis.zero_r_ID1 = 0.0f;
//    return &Chassis;
//}

// 电机零点自检
int Joint_Zero_OK() {
    
    if (	fabsf(zero_group1_ID0) <= 1e-6f || fabsf(zero_group1_ID1) <= 1e-6f 
			||	fabsf(zero_group2_ID0) <= 1e-6f || fabsf(zero_group2_ID1) <= 1e-6f || 
					fabsf(zero_group3_ID0) <= 1e-6f || fabsf(zero_group3_ID1) <= 1e-6f
			||	fabsf(zero_group4_ID0) <= 1e-6f || fabsf(zero_group4_ID1) <= 1e-6f 
		)
			{
        return 0;  //有零位没有被设置，返回false
			}
    return 1;  // 否则返回 true
}

// 电机零点获取 (初始位置 = 上电位置)
void Joint_Zero_init_Type1()
{
  // 电机零位 默认为1000，为了循环判断所以这么写 
  // 电机零位 定义在最上面
  // 使用while循环确保0位正确

  while (!Joint_Zero_OK()) 
		{
		//group1
			
			// 读取ID0零点
			modify_torque_cmd(&MotorA1_send_group1, 0, 0.0f);
			unitreeA1_rxtx(&huart1, 1);
			zero_group1_ID0 = MotorA1_recv_group1_id0.Pos;
			
			HAL_Delay(5);
			
			// 读取ID1零点
			modify_torque_cmd(&MotorA1_send_group1, 1, 0.0f);
			unitreeA1_rxtx(&huart1, 1);
			zero_group1_ID1 = MotorA1_recv_group1_id1.Pos;
			
			HAL_Delay(5);
			
			
			//group2
			// 读取ID0零点
			modify_torque_cmd(&MotorA1_send_group2, 0, 0.0f);
			unitreeA1_rxtx(&huart2, 2);
			zero_group2_ID0 = MotorA1_recv_group2_id0.Pos;
			
			HAL_Delay(5);
			
			// 读取ID1零点
			modify_torque_cmd(&MotorA1_send_group2, 1, 0.0f);
			unitreeA1_rxtx(&huart2, 2);
			zero_group2_ID1 = MotorA1_recv_group2_id1.Pos;
			
			HAL_Delay(5);
			
			//group3
			// 读取ID0零点
			modify_torque_cmd(&MotorA1_send_group3, 0, 0.0f);
			unitreeA1_rxtx(&huart8, 3);
			zero_group3_ID0 = MotorA1_recv_group3_id0.Pos;
			
			HAL_Delay(5);
		
			// 读取ID1零点
			modify_torque_cmd(&MotorA1_send_group3, 1, 0.0f);
			unitreeA1_rxtx(&huart8, 3);
			zero_group3_ID1 = MotorA1_recv_group3_id1.Pos;
			
			HAL_Delay(5);
			
			//group4
			// 读取ID0零点
			go_torque_cmd(&Motor_go_send_group4,0,0.0f);
			unitreeA1_rxtx(&huart4,4);
			zero_group4_ID0 = Motor_go_recv_group4_id0.Pos;
			
			HAL_Delay(5);

			// 读取ID1零点
			go_torque_cmd(&Motor_go_send_group4,1,0.0f);
			unitreeA1_rxtx(&huart4,4);
			zero_group4_ID1 = Motor_go_recv_group4_id1.Pos;
			
			HAL_Delay(5);
	}

}

void Joint_Zero_init_Type2()
{
  // 电机零位 默认为1000，为了循环判断所以这么写 
  // 电机零位 定义在最上面
  // 使用while循环确保0位正确

  while (!Joint_Zero_OK()) 
		{
		//group1
			
			// 读取ID0零点
			modify_speed_cmd(&MotorA1_send_group1, 0, 0.0f);
			unitreeA1_rxtx(&huart1, 1);
			zero_group1_ID0 = MotorA1_recv_group1_id0.Pos;
			
			HAL_Delay(5);
			
			// 读取ID1零点
			modify_speed_cmd(&MotorA1_send_group1, 1, 0.0f);
			unitreeA1_rxtx(&huart1, 1);
			zero_group1_ID1 = MotorA1_recv_group1_id1.Pos;
			
			HAL_Delay(5);
			
			
			//group2
			// 读取ID0零点
			modify_speed_cmd(&MotorA1_send_group2, 0, 0.0f);
			unitreeA1_rxtx(&huart2, 2);
			zero_group2_ID0 = MotorA1_recv_group2_id0.Pos;
			
			HAL_Delay(5);
			
			// 读取ID1零点
			modify_speed_cmd(&MotorA1_send_group2, 1, 0.0f);
			unitreeA1_rxtx(&huart2, 2);
			zero_group2_ID1 = MotorA1_recv_group2_id1.Pos;
			
			HAL_Delay(5);
			
			//group3
			// 读取ID0零点
			modify_speed_cmd(&MotorA1_send_group3, 0, 0.0f);
			unitreeA1_rxtx(&huart8, 3);
			zero_group3_ID0 = MotorA1_recv_group3_id0.Pos;
			
			HAL_Delay(5);
		
			// 读取ID1零点
			modify_speed_cmd(&MotorA1_send_group3, 1, 0.0f);
			unitreeA1_rxtx(&huart8, 3);
			zero_group3_ID1 = MotorA1_recv_group3_id1.Pos;
			
			HAL_Delay(5);
			
			//group4
			// 读取ID0零点
			go_spd_cmd(&Motor_go_send_group4,0,0.0f);
			unitreeA1_rxtx(&huart4,4);
			zero_group4_ID0 = Motor_go_recv_group4_id0.Pos;
			
			HAL_Delay(5);

			// 读取ID1零点
			go_spd_cmd(&Motor_go_send_group4,1,0.0f);
			unitreeA1_rxtx(&huart4,4);
			zero_group4_ID1 = Motor_go_recv_group4_id1.Pos;
			
			HAL_Delay(5);
	}

}



/**
  * @brief          底盘关节位置控制
  * @param[in]      Pos_Front: 减速后-角度制 正值前腿向上摆动
  * @param[in]      Pos_Back: 减速后-角度制 正值后腿向上摆动
  */
void Joint_Position_Control(uint8_t group, uint8_t id, float Pos[][STEP_NUM], float kp, float kw, uint16_t step)//, float Pos_Back
{   
    // 角度 限幅处理
    //LIMIT_RANGE(Pos_Front, -400, +400);
    //LIMIT_RANGE(Pos_Back,  -79, +19);
	
		float target_pos = 0.0f;
		motor_send_t *send_struct = NULL;
		UART_HandleTypeDef *huart = NULL;
		MotorCmd_t *send_struct_go = NULL;
	
		// 绑定组对应的发送结构体和串口
    switch(group) {
        case 1: send_struct = &MotorA1_send_group1; huart = &huart1; break;
        case 2: send_struct = &MotorA1_send_group2; huart = &huart2; break;
				case 3: send_struct = &MotorA1_send_group3; huart = &huart8; break;
        case 4: send_struct_go = &Motor_go_send_group4; huart = &huart4; break;
        default: return;
    }
		
// 计算目标位置（叠加零点）
    if (id == 0) {
        switch(group) {
            case 1: target_pos = +1.0f * Pos[5][step] + zero_group1_ID0; 
										break;
            case 2: target_pos = +1.0f * Pos[7][step] + zero_group2_ID0;
										break;
            case 3: target_pos = -1.0f * Pos[1][step] + zero_group3_ID0; 
										break;
            case 4: target_pos = -1.0f * Pos[3][step] + zero_group4_ID0; 
										break;
        }
    } else if (id == 1){
        switch(group) {
            case 1: target_pos = -1.0f * Pos[4][step] + zero_group1_ID1; 
										break;
            case 2: target_pos = -1.0f * Pos[6][step] + zero_group2_ID1; 
										break;
            case 3: target_pos = +1.0f * Pos[0][step] + zero_group3_ID1; 
										break;
						case 4: target_pos = Pos[2][step] + zero_group4_ID1; 
										break;
				}

		} 
		
		if(group == 4)
		{
				go_pos_cmd(send_struct_go,id,target_pos,kp,kw);
		}
		else {
				modify_pos_cmd(send_struct, id, target_pos, kp, kw);
    }
			unitreeA1_rxtx(huart, group);

}


void Joint_PW_Control(uint8_t group, uint8_t id,float Pos[][STEP_NUM],float Omega[][STEP_NUM],float kp,float kw,uint16_t step)//, float Pos_Back
{   
		float target_pos = 0.0f;
		float target_spd = 0.0f;
		motor_send_t *send_struct = NULL;
		UART_HandleTypeDef *huart = NULL;
		MotorCmd_t *send_struct_go = NULL;
		
	
		// 绑定组对应的发送结构体和串口
    switch(group) {
        case 1: send_struct = &MotorA1_send_group1; huart = &huart1; break;
        case 2: send_struct = &MotorA1_send_group2; huart = &huart2; break;
				case 3: send_struct = &MotorA1_send_group3; huart = &huart8; break;
        case 4: send_struct_go = &Motor_go_send_group4; huart = &huart4; break;
        //case 4: send_struct = &MotorA1_send_group4; huart = &huart6; break;
        default: return;
    }
		
		//b1 - G3 ID 1
		//b2 - G3 ID 0
		//b3 - G4 ID 1
		//b4 - G4 ID 0
		//b5 - G1 ID 1
		//b6 - G1 ID 0 
		//b7 - G2 ID 1
		//b8 - G2 ID 0 
		// 计算目标位置（叠加零点）
    if (id == 0) {
        switch(group) {
            case 1: target_pos = +1.0f * Pos[5][step] + zero_group1_ID0; 
										target_spd = Omega[5][step];
										break;
            case 2: target_pos = +1.0f * Pos[7][step] + zero_group2_ID0;
										target_spd = Omega[7][step];
										break;
            case 3: target_pos = -1.0f * Pos[1][step] + zero_group3_ID0; 
										target_spd = -1.0f * Omega[1][step];
										break;
            case 4: target_pos = -1.0f * Pos[3][step] + zero_group4_ID0; 
										target_spd = -1.0f * Omega[3][step];
										break;
        }
    } else if (id == 1){
        switch(group) {
            case 1: target_pos = -1.0f * Pos[4][step] + zero_group1_ID1; 
										target_spd = -1.0f * Omega[4][step];
										break;
            case 2: target_pos = -1.0f * Pos[6][step] + zero_group2_ID1; 
										target_spd = -1.0f * Omega[6][step];
										break;
            case 3: target_pos = +1.0f * Pos[0][step] + zero_group3_ID1; 
										target_spd = Omega[0][step];
										break;
						case 4: target_pos = Pos[2][step] + zero_group4_ID1; 
										target_spd = Omega[2][step];
										break;
				}

		} 
		
		if(group == 4)
		{
				go_pw_cmd(send_struct_go,id,target_pos,target_spd,kp,kw);
		}
		else {
				modify_PW_cmd(send_struct, id, target_pos,target_spd, kp, kw);
    }
			unitreeA1_rxtx(huart, group);
}


void Joint_zero_Control(uint8_t group, uint8_t id,float Pos[][STEP_NUM],float Omega[][STEP_NUM],float kp,float kw,uint16_t step)//, float Pos_Back
{   
		float target_pos = 0.0f;
		float target_spd = 0.0f;
		motor_send_t *send_struct = NULL;
		UART_HandleTypeDef *huart = NULL;
		MotorCmd_t *send_struct_go = NULL;
		
	
		// 绑定组对应的发送结构体和串口
    switch(group) {
        case 1: send_struct = &MotorA1_send_group1; huart = &huart1; break;
        case 2: send_struct = &MotorA1_send_group2; huart = &huart2; break;
				case 3: send_struct = &MotorA1_send_group3; huart = &huart8; break;
        case 4: send_struct_go = &Motor_go_send_group4; huart = &huart4; break;
        //case 4: send_struct = &MotorA1_send_group4; huart = &huart6; break;
        default: return;
    }
		// 计算目标位置（叠加零点）
    if (id == 0) {
        switch(group) {
            case 1: target_pos = Pos[0][step] + zeros[0]; 
										target_spd = Omega[0][step];
										break;
            case 2: target_pos = Pos[2][step] + zeros[2];
										target_spd = Omega[2][step];
										break;
            case 3: target_pos = Pos[4][step] + zeros[4]; 
										target_spd = Omega[4][step];
										break;
            case 4: target_pos = Pos[6][step] + zeros[6]; 
										target_spd = Omega[6][step];
										break;
        }
    } else if (id == 1){
        switch(group) {
            case 1: target_pos = Pos[1][step] + zeros[1]; 
										target_spd = Omega[1][step];
										break;
            case 2: target_pos = Pos[3][step] + zeros[3]; 
										target_spd = Omega[3][step];
										break;
            case 3: target_pos = Pos[5][step] + zeros[5]; 
										target_spd = Omega[5][step];
										break;
						case 4: target_pos = Pos[7][step] + zeros[7]; 
										target_spd = Omega[7][step];
										break;
				}

		} 
		
		if(group == 4)
		{
				go_pw_cmd(send_struct_go,id,target_pos,target_spd,kp,kw);
		}
		else {
				modify_PW_cmd(send_struct, id, target_pos,target_spd, kp, kw);
    }
			unitreeA1_rxtx(huart, group);
}

void Joint_Full_Position_Control(uint16_t step)
{
	Joint_Position_Control(1, 0, motor_angle, 0.022f, 0.1f, step);
	Joint_Position_Control(1, 1, motor_angle, 0.022f, 0.1f, step);

	Joint_Position_Control(2, 0, motor_angle, 0.022f, 0.1f, step);
	Joint_Position_Control(2, 1, motor_angle, 0.022f, 0.1f, step);

	Joint_Position_Control(3, 0, motor_angle, 0.022f, 0.1f, step);
	Joint_Position_Control(3, 1, motor_angle, 0.022f, 0.1f, step);
	
	Joint_Position_Control(4, 0, motor_angle, 0.21f, 0.001f, step);
	Joint_Position_Control(4, 1, motor_angle, 0.21f, 0.001f, step);
	
	
}

//b1 - G3 ID 1
//b2 - G3 ID 0
//b3 - G4 ID 1
//b4 - G4 ID 0
//b5 - G1 ID 1
//b6 - G1 ID 0 
//b7 - G2 ID 1
//b8 - G2 ID 0 

void Joint_Full_PW_Control(uint16_t step)
{
	Joint_PW_Control(1, 0, motor_angle, motor_omega, 0.025f, 0.10f, step);//0.025  0.1
	
	Joint_PW_Control(1, 1, motor_angle, motor_omega, 0.025f, 0.10f, step);

	Joint_PW_Control(2, 0, motor_angle, motor_omega, 0.025f, 0.10f, step);
	
	Joint_PW_Control(2, 1, motor_angle, motor_omega, 0.025f, 0.10f, step);

	Joint_PW_Control(3, 0, motor_angle, motor_omega, 0.025f, 0.10f, step);
	
	Joint_PW_Control(3, 1, motor_angle, motor_omega, 0.025f, 0.10f, step);
	
	Joint_PW_Control(4, 0, motor_angle, motor_omega, 0.30f, 0.001f, step);//0.30  0.001
	
	Joint_PW_Control(4, 1, motor_angle, motor_omega, 0.30f, 0.001f, step);
}

//void Joint_Full_T_Control(uint16_t step)
//{
//	Joint_PW_Control(1, 0, motor_angle, motor_omega, 0.020f, 0.10f, step);
//	
//	Joint_PW_Control(1, 1, motor_angle, motor_omega, 0.020f, 0.10f, step);

//	Joint_PW_Control(2, 0, motor_angle, motor_omega, 0.020f, 0.10f, step);
//	
//	Joint_PW_Control(2, 1, motor_angle, motor_omega, 0.020f, 0.10f, step);

//	Joint_PW_Control(3, 0, motor_angle, motor_omega, 0.025f, 0.10f, step);
//	
//	Joint_PW_Control(3, 1, motor_angle, motor_omega, 0.025f, 0.10f, step);
//	
//	Joint_PW_Control(4, 0, motor_angle, motor_omega, 0.21f, 0.001f, step);
//	
//	Joint_PW_Control(4, 1, motor_angle, motor_omega, 0.21f, 0.001f, step);
//}

void Joint_Full_zero_Control(uint16_t step)
{
	Joint_zero_Control(1, 0, zero_return_angle, zero_return_omega, 0.022f, 0.1f, step);
	Joint_zero_Control(1, 1, zero_return_angle, zero_return_omega, 0.022f, 0.1f, step);

	Joint_zero_Control(2, 0, zero_return_angle, zero_return_omega, 0.022f, 0.1f, step);
	Joint_zero_Control(2, 1, zero_return_angle, zero_return_omega, 0.022f, 0.1f, step);

	Joint_zero_Control(3, 0, zero_return_angle, zero_return_omega, 0.022f, 0.1f, step);
	Joint_zero_Control(3, 1, zero_return_angle, zero_return_omega, 0.022f, 0.1f, step);
	
	Joint_zero_Control(4, 0, zero_return_angle, zero_return_omega, 0.21f, 0.001f, step);
	Joint_zero_Control(4, 1, zero_return_angle, zero_return_omega, 0.21f, 0.001f, step);
}

/**
 * @brief 读取指定电机的当前位置
 * @param group 电机组号 (1-4)
 * @param id 电机ID (0-1)
 * @return 当前位置 (度，相对于零位置)
 */
float Joint_ReadCurrentPos(uint8_t group, uint8_t id) {
    // 读取最新电机数据

				motor_recv_t *recv_id0 = NULL;
				motor_recv_t *recv_id1 = NULL;
				MotorData_t *recv_id0_go = NULL;
				MotorData_t *recv_id1_go = NULL;

					// 根据组别绑定缓冲区和结构体
				switch(group) {
						case 1:
								recv_id0 = &MotorA1_recv_group1_id0;
								recv_id1 = &MotorA1_recv_group1_id1;
								break;
						
						case 2:
								recv_id0 = &MotorA1_recv_group2_id0;
								recv_id1 = &MotorA1_recv_group2_id1;
								break;
						
						case 3:
								recv_id0 = &MotorA1_recv_group3_id0;
								recv_id1 = &MotorA1_recv_group3_id1;
								break;
						
						case 4:
								recv_id0_go = &Motor_go_recv_group4_id0;
								recv_id1_go = &Motor_go_recv_group4_id1;
								break;// go协议接收缓冲区
						
						default: 
							break;
				}
						
				if (group == 4)
				{
						if(id == 0)
						{		
								return recv_id0_go->Pos;
						}
						else
						{
								return recv_id1_go->Pos;
								
						}
				}
				else
				{
						if(id == 0)
						{		
								return recv_id0->Pos;
						}
						else
						{
								return recv_id1->Pos;
								
						}
				}

				return 0.0f;

}

void Joint_readall(uint8_t mode)
{
	if(mode == 0)//0 代表分段间隙速度是0的种类
	{
					modify_speed_cmd(&MotorA1_send_group1,0, 0.0f);
					unitreeA1_rxtx(&huart1,1);
					current_pos[0] = MotorA1_recv_group1_id0.Pos;
	
					HAL_Delay(5);
	
					modify_speed_cmd(&MotorA1_send_group1,1, 0.0f);
					unitreeA1_rxtx(&huart1,1);
					current_pos[1] = MotorA1_recv_group1_id1.Pos;
					HAL_Delay(5);
	
					modify_speed_cmd(&MotorA1_send_group2,0, 0.0f);
					unitreeA1_rxtx(&huart2,2);
					current_pos[2] = MotorA1_recv_group2_id0.Pos;
	
					HAL_Delay(5);
	
					modify_speed_cmd(&MotorA1_send_group2,1, 0.0f);
					unitreeA1_rxtx(&huart2,2);
					current_pos[3] = MotorA1_recv_group2_id1.Pos;
	
					HAL_Delay(5);
	
					modify_speed_cmd(&MotorA1_send_group3,0, 0.0f);
					unitreeA1_rxtx(&huart8,3);
					current_pos[4] = MotorA1_recv_group3_id0.Pos;
	
					HAL_Delay(5);
	
					modify_speed_cmd(&MotorA1_send_group3,1, 0.0f);
					unitreeA1_rxtx(&huart8,3);
					current_pos[5] = MotorA1_recv_group3_id1.Pos;
					
					HAL_Delay(5);
					
					go_spd_cmd(&Motor_go_send_group4,0,0.0f);
					unitreeA1_rxtx(&huart4,4);
					current_pos[6] = Motor_go_recv_group4_id0.Pos;
					
					HAL_Delay(5);
					
					go_spd_cmd(&Motor_go_send_group4,1,0.0f);
					unitreeA1_rxtx(&huart4,4);
					current_pos[7] = Motor_go_recv_group4_id1.Pos;
					
					HAL_Delay(5);
				}
	else	
	{
		//Joint_Full_PW_Control(STEP_NUM- 1);
		current_pos[0] = MotorA1_recv_group1_id0.Pos;		
		current_pos[1] = MotorA1_recv_group1_id1.Pos;
		current_pos[2] = MotorA1_recv_group2_id0.Pos;
		current_pos[3] = MotorA1_recv_group2_id1.Pos;
		current_pos[4] = MotorA1_recv_group3_id0.Pos;
		current_pos[5] = MotorA1_recv_group3_id1.Pos;
		current_pos[6] = Motor_go_recv_group4_id0.Pos;
		current_pos[7] = Motor_go_recv_group4_id1.Pos;
	}

}


/**
 * @brief 检查当前角度是否与轨迹起点角度一致（在容忍阈值内）
 * @return 1：角度一致，可执行轨迹；0：角度不一致，需停止任务
 */
uint8_t check_angle_with_start(uint8_t mode) {
    
    // 1. 读取当前所有电机的实际角度

		uint8_t orders[CABLE_NUM] = {5,4,7,6,1,0,3,2};
    
		//b1 - G3 ID 1
		//b2 - G3 ID 0
		//b3 - G4 ID 1
		//b4 - G4 ID 0
		//b5 - G1 ID 1
		//b6 - G1 ID 0 
		//b7 - G2 ID 1
		//b8 - G2 ID 0 
			
    // 2. 逐个电机比较当前角度与轨迹起点角度
	switch(mode)
	{
		case BOUNDRY:
				Joint_readall(0);
				traj_start_angles[0] = motor_angle[orders[0]][0] + zero_group1_ID0;
				traj_start_angles[1] = motor_angle[orders[1]][0] + zero_group1_ID1;
				traj_start_angles[2] = motor_angle[orders[2]][0] + zero_group2_ID0;
				traj_start_angles[3] = motor_angle[orders[3]][0] + zero_group2_ID1;
				traj_start_angles[4] = motor_angle[orders[4]][0] + zero_group3_ID0;
				traj_start_angles[5] = motor_angle[orders[5]][0] + zero_group3_ID1;
				traj_start_angles[6] = motor_angle[orders[6]][0] + zero_group4_ID0;
				traj_start_angles[7] = motor_angle[orders[7]][0] + zero_group4_ID1;
		
				for (uint8_t c = 0; c < CABLE_NUM; c++) {
						float32_t angle_diff = fabsf(current_pos[c] - traj_start_angles[c]);
						if (angle_diff > ANGLE_TOLERANCE) {
								// 单个电机角度偏差超限，打印错误信息;
								continuity = 0;
								return 0;  // 校验失败
						}
				}
				break;
				
		case CIRCLE:
				Joint_readall(1);
				traj_start_angles[0] = motor_angle[orders[0]][0] + zero_group1_ID0;
				traj_start_angles[1] = motor_angle[orders[1]][0] + zero_group1_ID1;
				traj_start_angles[2] = motor_angle[orders[2]][0] + zero_group2_ID0;
				traj_start_angles[3] = motor_angle[orders[3]][0] + zero_group2_ID1;
				traj_start_angles[4] = motor_angle[orders[4]][0] + zero_group3_ID0;
				traj_start_angles[5] = motor_angle[orders[5]][0] + zero_group3_ID1;
				traj_start_angles[6] = motor_angle[orders[6]][0] + zero_group4_ID0;
				traj_start_angles[7] = motor_angle[orders[7]][0] + zero_group4_ID1;
		
				for (uint8_t c = 0; c < CABLE_NUM; c++) {
						float32_t angle_diff = fabsf(current_pos[c] - traj_start_angles[c]);
						if (angle_diff > ANGLE_TOLERANCE) {
								// 单个电机角度偏差超限，打印错误信息;
								continuity = 0;
								return 0;  // 校验失败
						}
				}
				break;
		case ZERO_RETURN:
				Joint_readall(0);
				traj_start_angles[0] = zero_return_angle[0][0] + zeros[0];
				traj_start_angles[1] = zero_return_angle[1][0] + zeros[1];
				traj_start_angles[2] = zero_return_angle[2][0] + zeros[2];
				traj_start_angles[3] = zero_return_angle[3][0] + zeros[3];
				traj_start_angles[4] = zero_return_angle[4][0] + zeros[4];
				traj_start_angles[5] = zero_return_angle[5][0] + zeros[5];
				traj_start_angles[6] = zero_return_angle[6][0] + zeros[6];
				traj_start_angles[7] = zero_return_angle[7][0] + zeros[7];
		
				for (uint8_t c = 0; c < CABLE_NUM; c++) {
						float32_t angle_diff = fabsf(current_pos[c] - traj_start_angles[c]);
						if (angle_diff > ANGLE_TOLERANCE) {
								// 单个电机角度偏差超限，打印错误信息;
								continuity = 0;
								return 0;  // 校验失败
						}
				}
				break;
		default:
					return 0;
	}
    // 3. 所有电机角度均在阈值内，校验通过
		continuity = 1;
    return 1;
}


