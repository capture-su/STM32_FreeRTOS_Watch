#include "stm32f10x.h"                  // Device header
#include "watch_freertos.h"
#include "Delay.h"
#include "OLED.h"
#include "key.h"
#include "encoder.h"
#include "MyRTC.h"
#include "mpu6050.h"
#include "app_usart.h"

int main(void)
{
	//FreeRTOS条件
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_SetPriority(SysTick_IRQn, 15);
  NVIC_SetPriority(PendSV_IRQn, 15);
	
	MyRTC_Init();
	App_USART_Init();
	//保证初始化的正确
	MPU6050_Init();
	while(MPU6050_GetID() != 0x68)  
	{
		MPU6050_Init();
		Delay_ms(100);
	}
	App_USART_Init();
	Key_Init();
	Encoder_Init();
	//开机动画
	OLED_Init();
	OLED_ShowImage(0,0,128,64,wuxinjinbu);
	OLED_Reverse();
	OLED_Update();
	Delay_ms(1000);
	uint8_t i;
	for(i = 0;i<64;i++)
	{
		OLED_ClearArea(0,i, 128, 1);
		OLED_ClearArea(0, 64 -i, 128, 1);
		OLED_ClearArea(i, 0, 1, 64);
		OLED_ClearArea(i+1, 0, 1, 64);
		OLED_ClearArea(128-i, 0, 1, 64);
		OLED_ClearArea(128-i-1, 0, 1, 64);
		OLED_Update();
		Delay_ms(10);
	}
	

	Watch_FreeRTOS_Init();
	while(1)
	{
		
	}

}
