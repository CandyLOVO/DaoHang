#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "rc_potocal.h"
#include "main.h"
#include "stm32f4xx_it.h"
//	X是左右方向，Y是前后方向（底盘）
extern DH Tx; //导航传来的数据
pid_struct_t motor_pid_chassis[4];
motor_info_t  motor_info_chassis[8];       //电机信息结构体
 fp32 chassis_motor_pid [3]={30,0.5,10};   //用的原来的pid
float Vx=0,Vy=0,Wz=0;
int16_t Temp_Vx;
int16_t Temp_Vy;
volatile int16_t motor_speed_target[4];
 extern RC_ctrl_t rc_ctrl;
 extern ins_data_t ins_data;
// Save imu data
uint16_t Up_ins_yaw; 
uint16_t Down_ins_yaw = 180;
uint16_t Down_ins_yaw_update = 180;
fp32 Err_yaw;	
fp32 Err_yaw_hudu;
fp32 Err_accident = 0;	//mechanical err	
fp32 Down_ins_pitch;
fp32 Down_ins_row;
fp32 sin_a;		
fp32 cos_a;
int8_t chassis_choice_flag = 0;
int8_t chassis_mode = 1;
 
uint8_t first_x = 0;
uint8_t first_y = 0;
 
extern uint8_t Flag_first;		//比赛开始标志位

//功率限制算法的变量定义
float Watch_Power_Max;
float Watch_Power;
float Watch_Buffer;
double Chassis_pidout;
double Chassis_pidout_target;
static double Scaling1=0,Scaling2=0,Scaling3=0,Scaling4=0;
float Klimit=1;
float Plimit=0;
float Chassis_pidout_max;


//矫正陀螺仪
int16_t Drifting_yaw = 0;

//获取imu——Yaw角度差值参数
static void Get_Err(); 

//参数重置
static void Chassis_loop_Init(); 

//底盘跟随云台
static void Chassis_following();

//mode1
static void Chassis_mode_1();

//mode2
static void Chassis_mode_2();

//模式选择
static void Chassis_choice();

//功率限制算法（2023.5.6）
static void Chassis_Power_Limit(double Chassis_pidout_target_limit);

//PID速度环转电流环
static void Chassis_PID_calc();
	
#define angle_valve 5
#define angle_weight 55
 
   void Chassis_task(void const *pvParameters)
{

        
       for (uint8_t i = 0; i < 4; i++)
	{
        pid_init(&motor_pid_chassis[i], chassis_motor_pid, 15384, 15384); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
	}
  

    for(;;)
    {     
          						
//            RC_to_Vector();                          //遥控器信息转换为底盘速度Vy,Vx,Wz  Remote controller information converted to chassis speed Vy, Vx, Wz
            Vector();
						chassis_motol_speed_calculate();         //电机速度计算，即麦轮运动解算      Calculation of Mecanum Wheel Motion
            Motor_Speed_limiting(motor_speed_target,6000);//限制最大速度，输入参数是限制速度值(同比缩放)
						Chassis_Power_Limit(24000);									//输入参数要是Motor_Speed_limiting（）函数限制速度的4倍
						//Chassis_PID_calc();
            chassis_current_give();                 //发送电流                         send current
            osDelay(1);

    }



}

void Vector()
{
	Vx = Tx.nav_vx;
	Vy = Tx.nav_vy;
	Wz = Tx.nav_yaw;
}

void RC_to_Vector()
{
		Chassis_loop_Init();
		Get_Err();
		// flag for right pattern 
    if(rc_ctrl.rc.s[1]==3 || rc_ctrl.rc.s[1]==1)    
    {
				Chassis_mode_1();
    }
		
		//pattern 2 ( Constantly curl + moving )小陀螺
		else if(rc_ctrl.rc.s[1]==2)			
		{
//			//测试
//			if(Flag_first == 0)
//			{
//				Flag_first = 1;
//			}
//			if(Flag_first == 1)
//			{
//				if(first_x)
//				{
//					Vx = -3000;
//				}
//				if(first_y)
//				{
//					Vy = 3000;
//				}
//			}
//			else if(Flag_first == 2)
//			{
//				Chassis_mode_2();
//			}
			//直接启动小陀螺
			Chassis_mode_2();
		}
		

	
 }

static void Chassis_loop_Init()
{
	Vx = 0;
	Vy = 0;
	Wz = 0;
}

static void Get_Err()
{
			Down_ins_yaw = ins_data.angle[0] + 180;
			Down_ins_pitch = ins_data.angle[1];
			Down_ins_row = ins_data.angle[2];
	
			//校正陀螺仪漂移
			Down_ins_yaw_update = Down_ins_yaw - Drifting_yaw;
			
			//calculate err of yaw
			Err_yaw = Up_ins_yaw - Down_ins_yaw_update;
			//Err_yaw = Err_yaw/57.3f;
		
			//越界处理,保证转动方向不变
			if(Err_yaw < -180)	//	越界时：180 -> -180
			{
				Err_yaw += 360;
			}
					
			else if(Err_yaw > 180)	//	越界时：-180 -> 180
			{
				Err_yaw -= 360;
			}
}

void chassis_motol_speed_calculate()
{
	
	  motor_speed_target[CHAS_LF] =  Vx+Vy+Wz;
    motor_speed_target[CHAS_RF] =  Vx-Vy+Wz;
    motor_speed_target[CHAS_RB] =  -Vy-Vx+Wz; 
    motor_speed_target[CHAS_LB] =  Vy-Vx+Wz;
}

/*这个限制算法同时对4个轮子同时起到了速度限制，不能很好地偏置某个电机*/
  void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }

    }

}

void chassis_current_give() 
{        

    	set_motor_current_can2(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
 

}

static void Chassis_following()
{
	//先检测云台已经停止转动
	if((rc_ctrl.rc.ch[0]<=50)&&(rc_ctrl.rc.ch[0]>=-50)	&& ( !q_flag && !e_flag))
	{

			//阈值判断
			if(Err_yaw > angle_valve || Err_yaw < -angle_valve)
			{
				Wz -= Err_yaw * angle_weight;
			}
	}
}

static void Chassis_mode_1()
{

      
			
			//braking to stop quickly
			if( (rc_ctrl.rc.ch[2]>=-50&&rc_ctrl.rc.ch[2]<=50)&&((rc_ctrl.rc.ch[3]>=-50)&&(rc_ctrl.rc.ch[3]<=50))&&(rc_ctrl.rc.ch[4]<=50)&&(rc_ctrl.rc.ch[4]>=-50)
				&& ( !w_flag && !s_flag && !a_flag && !d_flag) && (Err_yaw <= angle_valve) && (Err_yaw >= -angle_valve))
			{

				for(int i=0;i<4;i++)//减速  slow_down
				{
                 
				if(motor_info_chassis[i].rotor_speed>360||motor_info_chassis[i].rotor_speed<-360)
				{
					motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i],  motor_info_chassis[i].rotor_speed, 0);
				}
				else
				{
					Vx = 0;
					Vy = 0;
					Wz = 0;
					motor_info_chassis[i].set_current=0;
				}
			}
			set_motor_current_can2(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
			}
    
		// moving	control by remote
    else if( !w_flag && !s_flag && !a_flag && !d_flag)
    {
			

        Vy= rc_ctrl.rc.ch[3]/660.0*8000;
        Vx= rc_ctrl.rc.ch[2]/660.0*8000;
        Wz= -rc_ctrl.rc.ch[4]/660.0*8000;

    }
		
		// moving control by keyboard
		else
		{
		
			
				if(w_flag)
				{
					Vy = 9158;
				}
				else if(s_flag)
				{
					Vy = -9158;
				}
				else
				{
					Vy = 0;
				}
				
				if(a_flag)
				{
					Vx = -9158;
				}
				else if(d_flag)
				{
					Vx = 9158;
				}	
				else 
				{
					Vx = 0;
				}
		}

				//Chassis_following();
}	

static void Chassis_mode_2()
{

			//const a number for curling
			Wz = 8000;
			
			
//			// Read imu data
//			Down_ins_yaw = ins_data.angle[0] + 180;
//			Down_ins_pitch = ins_data.angle[1];
//			Down_ins_row = ins_data.angle[2];
//			
//			//校正陀螺仪漂移
//			Down_ins_yaw_update = Down_ins_yaw - Drifting_yaw;
//	
//			//calculate err of yaw
//			Err_yaw = Down_ins_yaw_update - Down_ins_yaw;
//			//Err_yaw = test_up_ins_yaw - Down_ins_yaw;
			Err_yaw_hudu = Err_yaw/57.3f;
			
			
			//calculate sin and cos
			cos_a = cos(Err_yaw_hudu);
			sin_a = sin(Err_yaw_hudu);
			
			
			
		
			

		// moving	control by remote
    if( !w_flag && !s_flag && !a_flag && !d_flag)
    {
    
        Vy= rc_ctrl.rc.ch[3]/660.0*2000;
        Vx= rc_ctrl.rc.ch[2]/660.0*2000;
			
			
				//curl matrix * V
				Temp_Vx = Vx;
				Temp_Vy = Vy;
				Vx = Temp_Vx*cos_a - Temp_Vy*sin_a;
				Vy = Temp_Vx*sin_a + Temp_Vy*cos_a;

    }
				// moving control by keyboard
		else 
		{
	
			
				if(w_flag)
				{
					Vy = 9158;
				}
				else if(s_flag)
				{
					Vy = -9158;
				}
				else
				{
					Vy = 0;
				}
				
				if(a_flag)
				{
					Vx = -9158;
				}
				else if(d_flag)
				{
					Vx = 9158;
				}	
				
				else 
				{
					Vx = 0;
				}
				
				
				//curl matrix * V
				Temp_Vx = Vx;
				Temp_Vy = Vy;
				Vx = Temp_Vx*cos_a - Temp_Vy*sin_a;
				Vy = Temp_Vx*sin_a + Temp_Vy*cos_a;
		}
		
}

static void Chassis_choice()
{
	if(r_flag)
	{
		chassis_choice_flag = 1;
	}
	
	if( (!r_flag) && (chassis_choice_flag == 1) )	
	{
		chassis_choice_flag = 0;
		if(chassis_mode == 1)
		{
			chassis_mode = 2;
		}
		else if(chassis_mode == 2)
		{
			chassis_mode = 1;
		}
	}
}


/*借鉴了防灾科技学院的算法，通过3个环来限制功率，有偏置，平滑曲线，缓冲能量约束的效果*/
//如果不行，试试把61536改成15384试试
static void Chassis_Power_Limit(double Chassis_pidout_target_limit)
{	
	//819.2/A，假设最大功率为120W，那么能够通过的最大电流为5A，取一个保守值：800.0 * 5 = 4000
	Watch_Power_Max=Klimit;	Watch_Power=Hero_chassis_power;	Watch_Buffer=60;//Hero_chassis_power_buffer;//限制值，功率值，缓冲能量值，初始值是1，0，0
	//get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//通过裁判系统和编码器值获取（限制值，实时功率，实时缓冲能量）

		Chassis_pidout_max=61536;//32768，40，960			15384 * 4，取了4个3508电机最大电流的一个保守值

		if(Watch_Power>600)	Motor_Speed_limiting(motor_speed_target,4096);//限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变，不知道为啥一开始用的Power>960,可以观测下这个值，看看能不能压榨缓冲功率
	else{
		Chassis_pidout=(
						fabs(motor_speed_target[0]-motor_info_chassis[0].rotor_speed)+
						fabs(motor_speed_target[1]-motor_info_chassis[1].rotor_speed)+
						fabs(motor_speed_target[2]-motor_info_chassis[2].rotor_speed)+
						fabs(motor_speed_target[3]-motor_info_chassis[3].rotor_speed));//fabs是求绝对值，这里获取了4个轮子的差值求和
		
//	Chassis_pidout_target = fabs(motor_speed_target[0]) + fabs(motor_speed_target[1]) + fabs(motor_speed_target[2]) + fabs(motor_speed_target[3]);

		/*期望滞后占比环，增益个体加速度*/
		if(Chassis_pidout)
		{
		Scaling1=(motor_speed_target[0]-motor_info_chassis[0].rotor_speed)/Chassis_pidout;	
		Scaling2=(motor_speed_target[1]-motor_info_chassis[1].rotor_speed)/Chassis_pidout;
		Scaling3=(motor_speed_target[2]-motor_info_chassis[2].rotor_speed)/Chassis_pidout;	
		Scaling4=(motor_speed_target[3]-motor_info_chassis[3].rotor_speed)/Chassis_pidout;//求比例，4个scaling求和为1
		}
		else{Scaling1=0.25,Scaling2=0.25,Scaling3=0.25,Scaling4=0.25;}
		
		/*功率满输出占比环，车总增益加速度*/
//		if(Chassis_pidout_target) Klimit=Chassis_pidout/Chassis_pidout_target;	//375*4 = 1500
//		else{Klimit = 0;}
		Klimit = Chassis_pidout/Chassis_pidout_target_limit;
		
		if(Klimit > 1) Klimit = 1 ;
		else if(Klimit < -1) Klimit = -1;//限制绝对值不能超过1，也就是Chassis_pidout一定要小于某个速度值，不能超调

		/*缓冲能量占比环，总体约束*/
		if(Watch_Buffer<50&&Watch_Buffer>=40)	Plimit=0.9;		//近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
		else if(Watch_Buffer<40&&Watch_Buffer>=35)	Plimit=0.75;
		else if(Watch_Buffer<35&&Watch_Buffer>=30)	Plimit=0.5;
		else if(Watch_Buffer<30&&Watch_Buffer>=20)	Plimit=0.25;
		else if(Watch_Buffer<20&&Watch_Buffer>=10)	Plimit=0.125;
		else if(Watch_Buffer<10&&Watch_Buffer>=0)	Plimit=0.05;
		else {Plimit=1;}
		
		motor_info_chassis[0].set_current = Scaling1*(Chassis_pidout_max*Klimit)*Plimit;//输出值
		motor_info_chassis[1].set_current = Scaling2*(Chassis_pidout_max*Klimit)*Plimit;
		motor_info_chassis[2].set_current = Scaling3*(Chassis_pidout_max*Klimit)*Plimit;
		motor_info_chassis[3].set_current = Scaling4*(Chassis_pidout_max*Klimit)*Plimit;/*同比缩放电流*/

	}

}

static void Chassis_PID_calc()
{
		for(uint8_t i=0 ; i<4; i++)
		{
			motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], motor_info_chassis[i].rotor_speed,motor_speed_target[i]);
		}
}