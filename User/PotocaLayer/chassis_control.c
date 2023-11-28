#include "chassis_control.h"
pid_t PID_Chassis_Rotate_Speed=PID_PARAM_DEFAULT;
pid_t PID_Chassis_Motor1_Speed=PID_PARAM_DEFAULT;
pid_t PID_Chassis_Motor2_Speed=PID_PARAM_DEFAULT;
pid_t PID_Chassis_Motor3_Speed=PID_PARAM_DEFAULT;
pid_t PID_Chassis_Motor4_Speed=PID_PARAM_DEFAULT;
myrampGen_t Rotate_Ramp = {0};
float Chassis_Rotate_Offset = 0;
//float Gimbal_Yaw_Offset = 0;

//�����˶�ģʽ
Chassis_SportState_e Last_SportState;
Chassis_SportState_e chassisState = SPORT_STATE;

float chassis_rote_mote=120;


/**
 * @Name: SetChassisSportState
 * @Description: ���õ����˶�ģʽ
 * @Param: �����˶�ģʽ,Ϊö������
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void SetChassisSportState(Chassis_SportState_e state)
{
	chassisState = state;
}


/**
 * @Name: GetChassisSportState
 * @Description: ��õ����˶�״̬
 * @Param: void
 * @Return: �����˶�״̬
 * @Author: source
 * @Warning: void
 */
Chassis_SportState_e GetChassisSportState()
{
	return chassisState;
}



/**
 * @Name: Chassis_Power_Limit
 * @Description: �µ��̹�������
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
float CHATS=280;//����ƫת�ٶ�
float CHATY=1;//�����������
float Klimit=1;
float Plimit=0;

float Watch_X=0;
float Watch_Y=0;
float Watch_disage=0;
float Watch_Power_Max;
float Watch_Power;
float Watch_Buffer;
double Chassis_pidout;

double Scaling1=0,Scaling2=0,Scaling3=0,Scaling4=0;
static double Scaling1,Scaling2,Scaling3,Scaling4;
	static uint16_t  Power_Max;
	static float Power,Power_Buffer;
//	static float  Power_Maybe = 0.0f;
//	static float Power_Bufferlimit = 0.0f;
//	static double Chassis_pidout=0.0f;
 float Chassis_pidout_max;
myrampGen_t ROTATION_ramp={0};


void get_chassis_power_and_buffer(float *power, float *buffer,uint16_t *powmax)
{
    *power = referee_receive_data.jundge_power_heat_data.date.chassis_power;
    *buffer = (float)referee_receive_data.jundge_power_heat_data.date.chassis_power_buffer;
	*powmax = referee_receive_data.jundge_robot_state.data.chassis_power_limit;
}

void Chassis_VAL_LIMIT(int X)
{
		VAL_LIMIT(PID_Chassis_Motor1_Speed.out,-X,X);		
		VAL_LIMIT(PID_Chassis_Motor2_Speed.out,-X,X);		
		VAL_LIMIT(PID_Chassis_Motor3_Speed.out,-X,X);
		VAL_LIMIT(PID_Chassis_Motor4_Speed.out,-X,X);	
}

void Chassis_Power_Limit(void)
{	
	//819.2/A�����������Ϊ120W����ô�ܹ�ͨ����������Ϊ5A��ȡһ������ֵ��800.0 * 5 = 4000
	Watch_Power_Max=Klimit;	Watch_Power=Power;	Watch_Buffer=Power_Buffer;//����ֵ������ֵ����������ֵ����ʼֵ��1��0��0
	get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//ͨ������ϵͳ�ͱ�����ֵ��ȡ������ֵ��ʵʱ���ʣ�ʵʱ����������

		Chassis_pidout_max=61536;//32768��40��960			15384 * 4��ȡ��4��3508�����������һ������ֵ
		PID_Chassis_Motor1_Speed.f_calc(&PID_Chassis_Motor1_Speed);		//��ʱ��֪�������˸�ɶ
		PID_Chassis_Motor2_Speed.f_calc(&PID_Chassis_Motor2_Speed);	
		PID_Chassis_Motor3_Speed.f_calc(&PID_Chassis_Motor3_Speed);	
		PID_Chassis_Motor4_Speed.f_calc(&PID_Chassis_Motor4_Speed);	

	if(Power>960)	Chassis_VAL_LIMIT(4096);//5*4*24;������������������ƽ���ı�
	else{
//	else if(Power_Buffer<=55){
		Chassis_pidout=(
						fabs(PID_Chassis_Motor1_Speed.set-PID_Chassis_Motor1_Speed.get)+
						fabs(PID_Chassis_Motor2_Speed.set-PID_Chassis_Motor2_Speed.get)+
						fabs(PID_Chassis_Motor3_Speed.set-PID_Chassis_Motor3_Speed.get)+
						fabs(PID_Chassis_Motor4_Speed.set-PID_Chassis_Motor4_Speed.get));//fabs�������ֵ�������ȡ��4�����ӵĲ�ֵ���

		/*�����ͺ�ռ�Ȼ������������ٶ�*/
		Scaling1=(PID_Chassis_Motor1_Speed.set-PID_Chassis_Motor1_Speed.get)/Chassis_pidout;	
		Scaling2=(PID_Chassis_Motor2_Speed.set-PID_Chassis_Motor2_Speed.get)/Chassis_pidout;
		Scaling3=(PID_Chassis_Motor3_Speed.set-PID_Chassis_Motor3_Speed.get)/Chassis_pidout;	
		Scaling4=(PID_Chassis_Motor4_Speed.set-PID_Chassis_Motor4_Speed.get)/Chassis_pidout;//�������4��scaling���Ϊ1
		
		/*���������ռ�Ȼ�������������ٶ�*/
		Klimit=Chassis_pidout/1500;	//375*4 = 1500��
		VAL_LIMIT(Klimit,-1,1);	//���ƾ���ֵ���ܳ���1��Ҳ����Chassis_pidoutһ��ҪС��ĳ���ٶ�ֵ�����ܳ���

		/*��������ռ�Ȼ�������Լ��*/
		if(Power_Buffer<50&&Power_Buffer>=40)	Plimit=0.9;		//��������һ��������Լ��������Ϊ�˱��ؿ��Ե���Plimit������Ӱ����Ӧ�ٶȣ�
		else if(Power_Buffer<40&&Power_Buffer>=35)	Plimit=0.75;
		else if(Power_Buffer<35&&Power_Buffer>=30)	Plimit=0.5;
		else if(Power_Buffer<30&&Power_Buffer>=20)	Plimit=0.25;
		else if(Power_Buffer<20&&Power_Buffer>=10)	Plimit=0.125;
		else if(Power_Buffer<10&&Power_Buffer>=0)	Plimit=0.05;
		else if(Power_Buffer==60)					Plimit=1;
		PID_Chassis_Motor1_Speed.out=Scaling1*Chassis_pidout_max*Klimit*Plimit;//���ֵ
		PID_Chassis_Motor2_Speed.out=Scaling2*Chassis_pidout_max*Klimit*Plimit;
		PID_Chassis_Motor3_Speed.out=Scaling3*Chassis_pidout_max*Klimit*Plimit;
		PID_Chassis_Motor4_Speed.out=Scaling4*Chassis_pidout_max*Klimit*Plimit;/*ͬ�����ŵ���*/

//		//PID_Chassis_Motor4_Speed.out = PID_Chassis_Motor1_Speed.out*(float)(Power_Buffer+20)/Power_Max*Chassis_pidout_max;//ͬ������������ֵ
		}	
}
void Chassis_Init()
{
	ChassisSpeedRef.forward_back_ref = 0;
	ChassisSpeedRef.left_right_ref = 0;
	ChassisSpeedRef.rotate_ref = 0;
	Pid_Out_Clear(&PID_Chassis_Rotate_Speed);
	Pid_Out_Clear(&PID_Chassis_Motor1_Speed);
	Pid_Out_Clear(&PID_Chassis_Motor2_Speed);
	Pid_Out_Clear(&PID_Chassis_Motor3_Speed);
	Pid_Out_Clear(&PID_Chassis_Motor4_Speed);	
	SetChassisSportState(SPORT_STATE);
}
/**
 * @Name: Chassis_control
 * @Description: ���̿��Ƴ���,�ڲ�ͬģʽ���˶�״̬�µ����Բ�ͬ��ʽ�˶�(δд��,����)
 * 				
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
float rotate_power_scale = 0.003;
float rotate_count;
float dis_angle=0;//static float dis_angle;
float X_MOVE_Ref = 0.0f;
float Y_MOVE_Ref = 0.0f;

/**
 * @Name: Chassis_Init
 * @Description: ����������ݳ�ʼ��		
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Chassis_Control()
{

	if(GetWorkState()==PREPARE_STATE||GetWorkState()==STOP_STATE||GetWorkState()==CALI_STATE)
	{
		ChassisSpeedRef.forward_back_ref = 0;
		ChassisSpeedRef.left_right_ref = 0;
		ChassisSpeedRef.rotate_ref = 0;
		rotate_count = 0;
		
		Pid_Out_Clear(&PID_Chassis_Motor1_Speed);
		Pid_Out_Clear(&PID_Chassis_Motor2_Speed);
		Pid_Out_Clear(&PID_Chassis_Motor3_Speed);
		Pid_Out_Clear(&PID_Chassis_Motor4_Speed);
		SetChassisSportState(SPORT_STATE);
	}
	else
	{
		switch(GetGimbalAndChassisSportState())
		{
			case CHASSIS_FOLLOW_YAW:{//��yaw��			
				switch(GetChassisSportState())
				{
					case SPORT_STATE:{
						PID_Chassis_Rotate_Speed.set = Yaw_Start_Encode_Angle;
						PID_Chassis_Rotate_Speed.get=YawEncoder.ecd_angle;
						ChassisSpeedRef.rotate_ref = PID_Chassis_Rotate_Speed.f_calc(&PID_Chassis_Rotate_Speed);
						ChassisSpeedRef.rotate_ref*=1.25;
						VAL_LIMIT(ChassisSpeedRef.rotate_ref,-CHATS,CHATS);//�������ת�ٶ�����
						if((YawEncoder.ecd_angle > Yaw_Start_Encode_Angle + 360)||(YawEncoder.ecd_angle < Yaw_Start_Encode_Angle - 360))
						{
							YawEncoder.round_cnt=-1;//0
						}
						X_MOVE_Ref = ChassisSpeedRef.forward_back_ref; 
						Y_MOVE_Ref = ChassisSpeedRef.left_right_ref;
					}break;
					case ROTATION_STATE: { //С���ݣ�����!
						ChassisSpeedRef.rotate_ref = chassis_rote_mote;
						/*�����ƶ�����*/
				dis_angle = -(YawEncoder.ecd_angle - Yaw_Start_Encode_Angle);
				/*С��������޷��Ա���̨����ϵ�����ƶ�����disageֵȡ�����������ţ�����*/
				X_MOVE_Ref=(float)((cos(dis_angle/57.3f)*ChassisSpeedRef.forward_back_ref)-(sin(dis_angle/57.3f)*ChassisSpeedRef.left_right_ref));	
				Y_MOVE_Ref=(float)((sin(dis_angle/57.3f)*ChassisSpeedRef.forward_back_ref)+(cos(dis_angle/57.3f)*ChassisSpeedRef.left_right_ref));
					}break;
				}
			}break;
			
			case CHASSIS_NO_FOLLOW_YAW:{//����yaw
				X_MOVE_Ref = ChassisSpeedRef.forward_back_ref; 
				Y_MOVE_Ref = ChassisSpeedRef.left_right_ref;
			};break;
			
			case Emergency_mode:{//Ӧ��ģʽ���̶�yaw�ᣬ�鶯pitch��
//				PID_Chassis_Rotate_Speed.set = Yaw_Start_Encode_Angle;;
//				PID_Chassis_Rotate_Speed.get = YawEncoder.ecd_angle;
//				ChassisSpeedRef.rotate_ref = PID_Chassis_Rotate_Speed.f_calc(&PID_Chassis_Rotate_Speed);
				VAL_LIMIT(ChassisSpeedRef.rotate_ref,-CHATS,CHATS);//�������ת�ٶ�����
				
				X_MOVE_Ref = ChassisSpeedRef.forward_back_ref; 
				Y_MOVE_Ref = ChassisSpeedRef.left_right_ref;
				ChassisSpeedRef.rotate_ref = GimbalRef.yaw_angle_dynamic_ref;	
			}
				
		}
		PID_Chassis_Motor1_Speed.set = -X_MOVE_Ref + Y_MOVE_Ref + ChassisSpeedRef.rotate_ref;//ÿ�������Ŀ��ת��
		PID_Chassis_Motor2_Speed.set =	X_MOVE_Ref + Y_MOVE_Ref + ChassisSpeedRef.rotate_ref;
		PID_Chassis_Motor3_Speed.set =  X_MOVE_Ref - Y_MOVE_Ref + ChassisSpeedRef.rotate_ref;
		PID_Chassis_Motor4_Speed.set = -X_MOVE_Ref - Y_MOVE_Ref + ChassisSpeedRef.rotate_ref;
		
		PID_Chassis_Motor1_Speed.get = CM1Encoder.filter_rate;//��������ֵ
		PID_Chassis_Motor2_Speed.get = CM2Encoder.filter_rate;
		PID_Chassis_Motor3_Speed.get = CM3Encoder.filter_rate;
		PID_Chassis_Motor4_Speed.get = CM4Encoder.filter_rate;
		
		Last_SportState=GetChassisSportState();
		/*�¹�������*/
		#ifdef POWER_LIMIT_OFF	
		PID_Chassis_Motor1_Speed.f_calc(&PID_Chassis_Motor1_Speed);
		PID_Chassis_Motor2_Speed.f_calc(&PID_Chassis_Motor2_Speed);	
		PID_Chassis_Motor3_Speed.f_calc(&PID_Chassis_Motor3_Speed);	
		PID_Chassis_Motor4_Speed.f_calc(&PID_Chassis_Motor4_Speed);		
		#else
		Chassis_Power_Limit();
	}
	#endif
	Last_SportState=GetChassisSportState();
}
  

/**
 * @Name: Chassis_Power_Control
 * @Description: ���̹�������
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
//  float POWER_LIMIT;
//  float WARNING_POWER;
//	float chassis_power=0.0f;
//	float chassis_power_buffer=0.0f;
//  float last_chassis_motor_out[4];
//	float power_scale = 0.0f;
//	float total_cunrrent_limit = 0.0f;
//	float total_cunrrent_sum=0.0f;
//void Chassis_Power_Control()
//{
//	//POWER_LIMIT = Get_Referee_Chassis_Power_Limit();
//	POWER_LIMIT = 50.0f;
//	WARNING_POWER = POWER_LIMIT - 20;
//	
//	Get_Referee_Chassis_Power(&chassis_power,&chassis_power_buffer);
//	
//	if(chassis_power_buffer < WARNING_POWER_BUFF)
//	{
//		if(chassis_power_buffer>6.0f)
//			power_scale  = chassis_power_buffer/WARNING_POWER_BUFF;
//		else
//		{
//			power_scale = 6/WARNING_POWER_BUFF;
//		}
//		total_cunrrent_limit = BUFFER_CURRENT_LIMIT*power_scale;
//	}
//	else
//	{
//		if(chassis_power > WARNING_POWER)
//		{
//			if(chassis_power > POWER_LIMIT)
//				power_scale = (chassis_power-WARNING_POWER)/(POWER_LIMIT-WARNING_POWER);
//			else
//				power_scale = 0;
//			total_cunrrent_limit = BUFFER_CURRENT_LIMIT + POWER_CURRENT_LIMIT*power_scale;
//		}
//		else
//			total_cunrrent_limit = BUFFER_CURRENT_LIMIT + POWER_CURRENT_LIMIT;
//	}
//	
//	PID_Chassis_Motor1_Speed.f_calc(&PID_Chassis_Motor1_Speed);
//	PID_Chassis_Motor2_Speed.f_calc(&PID_Chassis_Motor2_Speed);
//	PID_Chassis_Motor3_Speed.f_calc(&PID_Chassis_Motor3_Speed);
//	PID_Chassis_Motor4_Speed.f_calc(&PID_Chassis_Motor4_Speed);
//	
//	PID_Chassis_Motor1_Speed.out += 0.5f*(last_chassis_motor_out[0] - PID_Chassis_Motor1_Speed.out);
//	PID_Chassis_Motor2_Speed.out += 0.5f*(last_chassis_motor_out[1] - PID_Chassis_Motor2_Speed.out);
//	PID_Chassis_Motor3_Speed.out += 0.5f*(last_chassis_motor_out[2] - PID_Chassis_Motor3_Speed.out);
//	PID_Chassis_Motor4_Speed.out += 0.5f*(last_chassis_motor_out[3] - PID_Chassis_Motor4_Speed.out);
//	
//	last_chassis_motor_out[0] = PID_Chassis_Motor1_Speed.out;
//	last_chassis_motor_out[1] = PID_Chassis_Motor2_Speed.out;
//	last_chassis_motor_out[2] = PID_Chassis_Motor3_Speed.out;
//	last_chassis_motor_out[3] = PID_Chassis_Motor4_Speed.out;
//	
//	
//	
//	total_cunrrent_sum = fabsf(PID_Chassis_Motor1_Speed.out) + fabsf(PID_Chassis_Motor2_Speed.out) + fabsf(PID_Chassis_Motor3_Speed.out) + fabsf(PID_Chassis_Motor4_Speed.out);
//	
//	if(total_cunrrent_sum > total_cunrrent_limit)
//	{
//		float current_scale = total_cunrrent_limit / total_cunrrent_sum;
//		PID_Chassis_Motor1_Speed.out *= current_scale ;
//		PID_Chassis_Motor2_Speed.out *= current_scale;
//		PID_Chassis_Motor3_Speed.out *= current_scale;
//		PID_Chassis_Motor4_Speed.out *= current_scale;
//	}
//}
