#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include "PID.h"
#include "arm_math.h"
#include "StartTask02.h"




void StartTask02(void const * argument)//遥控器连接和初始化
	

{
  for(;;)
  {		
    osDelay(1);
  }

}
