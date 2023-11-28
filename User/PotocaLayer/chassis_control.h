#ifndef _CHASSIS_CONTROL_H
#define _CHASSIS_CONTROL_H
#include "stm32f4xx_hal.h"
//#include "control_task.h"
#include <math.h>
//#include "my_ramp.h"
#include "pid.h"
#include "judge.h"
//#include "remote_task.h"
//#include "gimbal_control.h"

typedef struct
{
	float Watch1;
	float Watch2;
	float Watch3;
	float Watch4;
	float Watch5;
}CHASSIS_volt;//new
typedef enum
{
	CHASSIS_FOLLOW_YAW,         //底盘跟随云台模式
	CHASSIS_NO_FOLLOW_YAW,      //底盘不跟随云台模式
	Emergency_mode,//应急模式
}GimbalAndChassis_SportState_e;
typedef enum
{
	SPORT_STATE,         //底盘正常运动状态
	ROTATION_STATE,      //底盘旋转运动（小陀螺模式）
}Chassis_SportState_e;
extern pid_t PID_Chassis_Rotate_Speed;
extern pid_t PID_Chassis_Motor1_Speed;
extern pid_t PID_Chassis_Motor2_Speed;
extern pid_t PID_Chassis_Motor3_Speed;
extern pid_t PID_Chassis_Motor4_Speed;
extern volatile CHASSIS_volt Watch_volt;
extern myrampGen_t Rotate_Ramp;
extern void SetChassisSportState(Chassis_SportState_e state);
extern Chassis_SportState_e GetChassisSportState(void);
extern void SetGimbalSportState(GimbalAndChassis_SportState_e state);
extern GimbalAndChassis_SportState_e GetGimbalAndChassisSportState(void);
extern float chassis_rote_mote;
void Chassis_Init(void);
void Chassis_Control(void);
//void Chassis_Power_Control(void);

 
void Chassis_Power_Limit(void);
void Chassis_VAL_LIMIT(int);
#endif


