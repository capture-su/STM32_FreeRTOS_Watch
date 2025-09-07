#include "watch_freertos.h"


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include <stdlib.h>

#include "stepcount.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* 外设 */
#include "key.h"
#include "encoder.h"
#include "MyRTC.h"
#include "mpu6050.h"
#include "Delay.h"
#include "OLED.h"
#include "app_usart.h"



// START_TASK 任务配置
#define START_TASK_PRIO 1                                   /* 任务优先级 */
#define START_STK_SIZE  128                                 /* 任务堆栈大小 */
TaskHandle_t StartTask_Handler;                             /* 任务句柄 */
// RTC_TASK 任务配置
#define RTC_TASK_PRIO 13                                   /* 任务优先级 */
#define RTC_STK_SIZE  128                                 /* 任务堆栈大小 */
TaskHandle_t RTCTask_Handler;                             /* 任务句柄 */
// OLED_TASK 任务配置
#define OLED_TASK_PRIO 11                                   /* 任务优先级 */
#define OLED_STK_SIZE  128*2                                 /* 任务堆栈大小 */
TaskHandle_t OLEDTASK_Handler;                             /* 任务句柄 */
// Keyscan_TASK 任务配置
#define Keyscan_TASK_PRIO 10                                   /* 任务优先级 */
#define Keyscan_STK_SIZE  128                                 /* 任务堆栈大小 */
TaskHandle_t KeyscanTask_Handler;                             /* 任务句柄 */
// Encoder_TASK 任务配置
#define Encoder_TASK_PRIO 9                                   /* 任务优先级 */
#define Encoder_STK_SIZE  128                                 /* 任务堆栈大小 */
TaskHandle_t EncoderTask_Handler;                             /* 任务句柄 */

// MenuContol_TASK 任务配置
#define MenuContol_TASK_PRIO 6                                   /* 任务优先级 */
#define MenuContol_STK_SIZE  128                                 /* 任务堆栈大小 */
TaskHandle_t MenuContolTask_Handler;                             /* 任务句柄 */

// StepCounter_TASK 任务配置
TaskHandle_t StepCounterTask_Handler;
#define StepCounter_TASK_PRIO 8                                 /* 任务优先级 */
#define StepCounter_STK_SIZE  128*2                                /* 任务堆栈大小 */

// USART2_TASK 任务配置
TaskHandle_t USART2Task_Handler;
#define USART2_TASK_PRIO 7                                 /* 任务优先级 */
#define USART2_STK_SIZE  128*2                                /* 任务堆栈大小 */




//队列用于 广播发送接收 用来防止多任务打断
QueueHandle_t           xEncoderQueue;             /* 定义队列 */
#define QUEUE_LENGTH    1                   /* 队列支持的消息个数 */
#define QUEUE_ITEM_SIZE sizeof(int8_t)     /* 队列中每条消息的大小 */

//串口收发的队列
QueueHandle_t xUartRxQueue;
QueueHandle_t xUartTxQueue;
#define UART_QUEUE_LENGTH 128
#define UART_QUEUE_ITEM_SIZE sizeof(uint8_t)




// 任务函数
static void Start_TASK(void *pvParameters);
static void RTC_TASK(void *pvParameters);
static void OLED_TASK(void *pvParameters);
static void Encoder_TASK(void *pvParameters);
static void MenuContol_TASK(void *pvParameters);
static void StepCounter_TASK(void *pvParameters); 
static void USART2_TASK(void *pvParameters); 
//调用函数
static void  key_callback(void);        //按键回调
static void  Main_interface(void);      //显示主界面
static void  Setting_interface(void);   //显示设置界面
static void  Setting_interface1(void);  //设置子界面
static void  Setting_interface2(void);  //设置子界面
static void  Menu_interface(void);      //菜单界面




SemaphoreHandle_t flag_mutex;         //创建信号量 用来保护flag;
/*********封装互斥信号量用来保护全局变量防止多任务打断*********/
uint8_t get_main_flag(void);
void set_main_flag(uint8_t value);
uint8_t get_setting_flag(void);
void set_setting_flag(uint8_t value);
uint8_t get_settingne_flag(void);
void set_settingne_flag(uint8_t value);
uint8_t get_power_flag(void);
void set_power_flag(uint8_t value);
/*******************************************************/


static volatile  uint8_t OLED_main_flag = 0;
static volatile  uint8_t OLED_setting_flag = 0;
static volatile  uint8_t OLED_menu_flag = 0;
static volatile  uint8_t setting_ne  = 0; 
static volatile  uint8_t Menu_flag1 = 0;

//显示与光标移动任务
uint8_t pre_selection;//上次选择的选项
uint8_t target_selection;//目标选项
uint8_t x_pre=48;//上次选项的x坐标
uint8_t Speed=8;//速度
uint8_t move_flag;//开始移动的标志位，1表示开始移动，0表示停止移动
uint8_t menu_flag = 1;
uint8_t DirectFlag = 1;
void Menu_Animation(void);
void Set_Selection(uint8_t move_flag,uint8_t Pre_Selection,uint8_t Target_Selection);






#include "MPU6050_Reg.h"
static  volatile  uint8_t Power_saving_mode = 0;
static  volatile  uint8_t power_flag = 0;
// 进入低功耗
void PRE_SLEEP_PROCESSING(void)
{
	power_flag = get_power_flag();
	//防止省电 与低功耗 冲突
	if(power_flag == 0)
	{
		// 1低功耗模式MPU6050
			// 关闭 STM32 的 I2C2 
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, DISABLE); 
			//进入设备低功耗
			MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x4F); // 进入睡眠模式，并选择最低功耗时钟源
			
		// 2串口低功耗
			// 关闭 USART2 时钟
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE); 
			// 关闭 USART2 的 TX 和 RX 引脚时钟（假设使用 PA2, PA3）
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);	
	}
		
}

// 退出低功耗
void POST_SLEEP_PROCESSING(void)
{
	power_flag = get_power_flag();
	//防止省电 与低功耗 冲突
	if(power_flag == 0)
	{	
	// 重新初始化  
	//恢复MPU6050
		MPU6050_Init();
	//恢复串口
    App_USART_Init();
	}
}

void USART2_IRQHandler(void) 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t ucData;
    
    // 接收中断
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        ucData = USART_ReceiveData(USART2);
        xQueueSendFromISR(xUartRxQueue, &ucData, &xHigherPriorityTaskWoken);
    }
    
    // 发送中断
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        if(xQueueReceiveFromISR(xUartTxQueue, &ucData, &xHigherPriorityTaskWoken) == pdPASS) {
            USART_SendData(USART2, ucData);
        } else {
            // 队列空时关闭发送中断
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void usart_string(const char *str) 
{
    // 参数检查
    if(str == NULL) return;
    
    // 获取字符串长度
		unsigned int len = strlen(str);
    if(len == 0) return;
    
    // 逐个字符放入发送队列
    for(size_t i = 0; i < len; i++) {
        // 带超时发送（防止死锁）
        if(xQueueSend(xUartTxQueue, &str[i], pdMS_TO_TICKS(100)) != pdPASS) {
            // 发送失败处理（可添加错误处理）
            #ifdef DEBUG
            // 调试模式下可触发断点
            __BKPT(0);
            #endif
            break;
        }
    }
    
    // 使能发送中断（触发发送）
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}
void usart_Printf(const char *format, ...)
{
	  char buffer[128];      // 缓冲区，用于存放格式化后的字符串
    va_list args;                         // 用于访问可变参数的变量
    va_start(args, format);               // 初始化可变参数列表
    vsnprintf(buffer, 128, format, args);  // 格式化字符串到缓冲区
    va_end(args);                         // 清理可变参数列表
		usart_string(buffer);
}


static void USART2_TASK(void *pvParameters) 
{
    uint8_t ucRxData;
    
		uint16_t total_steps_old = 0;
		usart_string("suzhun\r\n");
		static uint8_t rx_flag = 0;
	  static uint8_t revdatapacket[30];
		static uint8_t rc_datapacket[30];
		static uint8_t count = 0;
    while(1)
		{
        // 接收处理
        if(xQueueReceive(xUartRxQueue, &ucRxData, 10) == pdPASS) 
				{
            if(ucRxData == '@')
						{
							rx_flag = 1;
							count = 0;
							strcpy((char *)rc_datapacket,(char *)revdatapacket);
							revdatapacket[0] = '\0';
						}
						else 
						{
							revdatapacket[count++] = ucRxData;
							revdatapacket[count] =  '\0';
						}
        }
				if(rx_flag == 1)
				{
					rx_flag = 0;
					if(strncasecmp((const char*)rc_datapacket, "data ", 5) == 0)
					{
						int year,mouth,day;
						if(sscanf((const char*)rc_datapacket, "data %d %d %d",&year,&mouth,&day) == 3)
						{
							MyRTC_Time[0] = year;
							MyRTC_Time[1] = mouth;
							MyRTC_Time[2] = day;
							MyRTC_SetTime();
							usart_Printf("data:%d %d %d\r\n",year,mouth,day);
						}
					}
					else if(strncasecmp((const char*)rc_datapacket, "time ", 5) == 0)
					{
						int hour,minute,second;
						if(sscanf((const char*)rc_datapacket, "time %d %d %d",&hour,&minute,&second) == 3)
						{
							MyRTC_Time[3] = hour;
							MyRTC_Time[4] = minute;
							MyRTC_Time[5] = second;
							MyRTC_SetTime();
							usart_Printf("time:%d %d %d\r\n",hour,minute,second);
						}
					}
					rc_datapacket[0] = '\0';
				}
				
				if(total_steps_old != total_steps)
				{
					usart_Printf("step:%d\r\n",total_steps);
					total_steps_old = total_steps;
				}
				vTaskDelay(10);
    }
}

// 计时变量（可被外部访问）
static volatile  uint8_t ms10_count = 0;
static volatile  uint8_t s_count = 0;
static volatile  uint8_t min_count = 0;
// 定时器句柄
TimerHandle_t xTimer;
void vTimerCallback(TimerHandle_t xTimer)
{
    ms10_count++;

    if (ms10_count > 99)
    {
        ms10_count = 0;
        s_count++;

        if (s_count > 59)
        {
            s_count = 0;
            min_count++;
        }
    }
}
// 初始化定时器（在 main 中调用一次）
void Timer_Init(void)
{
    // 创建周期性定时器，每 10ms 触发一次
    xTimer = xTimerCreate(
        "Timer10ms",
        pdMS_TO_TICKS(10),
        pdTRUE,                 // 周期性
        0,
        vTimerCallback
    );

    if (xTimer == NULL)
    {
        // 创建失败处理（可选）
    }
}

// 启动计时器
void Timer_Start(void)
{
    if (xTimer != NULL)
    {
        // 启动定时器（阻塞时间为 0）
        xTimerStart(xTimer, 0);
    }
}

// 暂停计时器
void Timer_Stop(void)
{
    if (xTimer != NULL)
    {
        // 停止定时器
        xTimerStop(xTimer, 0);
    }
}

// 清零计时器（停止 + 重置变量）
void Timer_Reset(void)
{
    // 先停止，避免在清零时被修改
    Timer_Stop();

    // 重置计时变量
    ms10_count = 0;
    s_count = 0;
    min_count = 0;
}

static void MenuContol_TASK(void *pvParameters)
{
	static int8_t encoder;
	static int8_t encoder_old;
	while(1)
	{
			encoder_old = encoder;
			xQueuePeek(xEncoderQueue, &encoder, portMAX_DELAY);
			int8_t temp = encoder - encoder_old;
			if(temp > 0)
			{
				move_flag = 1;
				DirectFlag = 2;
				menu_flag++;
				if(menu_flag>=7)menu_flag=1;
			}
			else if(temp  < 0)
			{
				move_flag = 1;
				DirectFlag = 1;
				menu_flag--;
				if(menu_flag<=0)menu_flag=6;
			}
	}
}
//读取RTC任务
static void RTC_TASK(void *pvParameters)
{
    while (1)
    {
        MyRTC_ReadTime();  // 更新时间显示
        // 只在设置模式下才允许修改
				setting_ne = get_settingne_flag();
        if (setting_ne >= 2 && setting_ne <= 4)
        {
            int8_t enc;
						xQueuePeek(xEncoderQueue, &enc, portMAX_DELAY);
            if (enc != 0)
            {
                uint8_t index = setting_ne - 2; 
                MyRTC_Time[index] += enc;
								switch(index)
								{
									case 0:
									{
										if (MyRTC_Time[0] < 2000) MyRTC_Time[0] = 2099;
                    if (MyRTC_Time[0] > 2099) MyRTC_Time[0] = 2000;
									}break;
									case 1:
									{
										if (MyRTC_Time[1] < 1) MyRTC_Time[1] = 12;
                    if (MyRTC_Time[1] > 12) MyRTC_Time[1] = 1;
									}break;
									case 2:
									{
										if (MyRTC_Time[2] < 1) MyRTC_Time[2] = 31;
                    if (MyRTC_Time[2] > 31) MyRTC_Time[2] = 1;
									}break;
								}
                MyRTC_SetTime();     // 写入RTC
                Encoder_Clear();     // 清零，防止重复加
                OLED_Update();       // 刷新显示
            }
        }
				else  if (setting_ne >= 6 && setting_ne <= 8)
        {
            int8_t enc;
						xQueuePeek(xEncoderQueue, &enc, portMAX_DELAY);
            if (enc != 0)
            {
                uint8_t index = setting_ne - 3; 
                MyRTC_Time[index] += enc;
								switch(index)
								{
									case 3:
									{
										if (MyRTC_Time[3] > 23) MyRTC_Time[3] = 0;
									}break;
									case 4:
									{
										if (MyRTC_Time[4] > 59) MyRTC_Time[4] = 0;
									}break;
									case 5:
									{
										if (MyRTC_Time[5] > 59) MyRTC_Time[5] = 0;
									}
								}
                MyRTC_SetTime();     // 写入RTC
                Encoder_Clear();     // 清零，防止重复加
                OLED_Update();       // 刷新显示
            }
        }
        vTaskDelay(100);  // 提高响应速度
    }
}

//按键改变状态任务
static void Keyscan_TASK(void *pvParameters)
{
    uint8_t state = 0;
    while (1)
    {
				switch (state)
					{
							case 0: // 等待按键按下
									if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET)
									{
											state = 1; // 进入消抖状态
									}
									break;

							case 1: // 按键消抖（按下状态）
									vTaskDelay(pdMS_TO_TICKS(20)); // 按下消抖
									if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET)
									{
											state = 2; // 确认按下，进入等待松手状态
									}
									else
									{
											state = 0; // 消抖失败，回到初始状态
									}
									break;

							case 2: // 等待按键松手（不加消抖）
									if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_SET)
									{
														
											key_callback();
										
											state = 0; // 立即回到初始状态
									}
									break;
					}

					// 统一轮询延时，不要在状态中使用 vTaskDelay()
					vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//读取编码器任务
static void Encoder_TASK(void *pvParameters)
{
	static  int8_t  encoder; //编码器的值
	while(1)
	{
		encoder = Encoder_GetValue();
		xQueueOverwrite(xEncoderQueue, &encoder);
		vTaskDelay(100);
	}
}
 /* 计步任务 */
uint8_t ID; 
static void StepCounter_TASK(void *pvParameters) 
{  
	
		ID = MPU6050_GetID();
    // 初始化极值
    reset_peaks();
    
    // 初始校准
    vTaskDelay(pdMS_TO_TICKS(2000)); // 2秒校准时间
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms周期
    uint8_t step_time_count = 0;
    
    // 初始采样
    Gyro_sample_update();
    old_ave_GyroValue = ave_GyroValue;
    
    // 记录上一次有效步伐的时间
    TickType_t last_step_time = 0;
    // 计算最小步间间隔的tick数（修复pdTICKS_TO_MS错误）
    const TickType_t min_step_ticks = pdMS_TO_TICKS(MIN_STEP_INTERVAL);
    while(1)
		{
        // 1. 执行采样更新
        Gyro_sample_update();
        
        // 2. 检测活跃轴
        which_is_active();
        
        // 3. 步数检测
        detect_step();
        
        // 4. 步数确认（每300ms）
        step_time_count++;
        if(step_time_count >= 6) { // 6 * 50ms = 300ms
            step_time_count = 0;
            if(step_count > 0) {
                // 获取当前时间
                TickType_t current_time = xTaskGetTickCount();
                
                // 检查时间间隔（使用tick数比较）
                if(last_step_time == 0 || 
                   (current_time - last_step_time) >= min_step_ticks) 
                {
                    total_steps += 1; // 只计1步
                    last_step_time = current_time;
                }
                
                step_count = 0;
            }
        }        
        // 精确延时确保50ms周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
// 
static void OLED_TASK(void *pvParameters)
{
		static uint8_t suspend_state = 0;
    while (1)
    {		
			
			if(get_main_flag() == 1)
			{
				if(suspend_state == 1)
				{
					vTaskResume(MenuContolTask_Handler);
					suspend_state = 0;
				}	
			}
			else 
			{
				if(suspend_state == 0)
				{
					vTaskSuspend(MenuContolTask_Handler);
					suspend_state = 1;
				}
			}				
			switch(get_main_flag())
			{
				case 0:Main_interface();   break;
				case 1:Menu_interface();	 break;
				case 2:Setting_interface();break;
			}
			int8_t  encoder;

			xQueuePeek(xEncoderQueue, &encoder, 100);
			OLED_ShowNum(6*10,0,encoder,3,OLED_6X8);
			
			
			OLED_Update();
			vTaskDelay(100);      // 每隔 100ms
    }
}
//开始任务用完删除
static void Start_TASK(void *pvParameters)
{

		taskENTER_CRITICAL();                                   // 进入临界区
		
		xEncoderQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
		    // 创建接收队列 (中断->任务)
    xUartRxQueue = xQueueCreate(UART_QUEUE_LENGTH, UART_QUEUE_ITEM_SIZE);
    // 创建发送队列 (任务->中断)
    xUartTxQueue = xQueueCreate(UART_QUEUE_LENGTH, UART_QUEUE_ITEM_SIZE);
	
		flag_mutex = xSemaphoreCreateMutex();
    xTaskCreate((TaskFunction_t )RTC_TASK,                /* 任务函数 */
                (const char*    )"RTC_TASK",              /* 任务名称 */
                (uint16_t       )RTC_STK_SIZE,            /* 任务堆栈大小 */
                (void*          )NULL,                      /* 传入给任务函数的参数 */
                (UBaseType_t    )RTC_TASK_PRIO,           /* 任务优先级 */
                (TaskHandle_t*  )&RTCTask_Handler);       /* 任务句柄 */


    xTaskCreate((TaskFunction_t )OLED_TASK,                /* 任务函数 */
                (const char*    )"OLED_TASK",              /* 任务名称 */
                (uint16_t       )OLED_STK_SIZE,            /* 任务堆栈大小 */
                (void*          )NULL,                      /* 传入给任务函数的参数 */
                (UBaseType_t    )OLED_TASK_PRIO,           /* 任务优先级 */
                (TaskHandle_t*  )&OLEDTASK_Handler);       /* 任务句柄 */			
								
    xTaskCreate((TaskFunction_t )Keyscan_TASK,                /* 任务函数 */
                (const char*    )"Keyscan_TASK",              /* 任务名称 */
                (uint16_t       )Keyscan_STK_SIZE,            /* 任务堆栈大小 */
                (void*          )NULL,                      /* 传入给任务函数的参数 */
                (UBaseType_t    )Keyscan_TASK_PRIO,           /* 任务优先级 */
                (TaskHandle_t*  )&KeyscanTask_Handler);       /* 任务句柄 */

    xTaskCreate((TaskFunction_t )Encoder_TASK,                /* 任务函数 */
                (const char*    )"Encoder_TASK",              /* 任务名称 */
                (uint16_t       )Encoder_STK_SIZE,            /* 任务堆栈大小 */
                (void*          )NULL,                      /* 传入给任务函数的参数 */
                (UBaseType_t    )Encoder_TASK_PRIO,           /* 任务优先级 */
                (TaskHandle_t*  )&EncoderTask_Handler);       /* 任务句柄 */		
								
		xTaskCreate((TaskFunction_t )MenuContol_TASK,                /* 任务函数 */
                (const char*    )"MenuContol_TASK",              /* 任务名称 */
                (uint16_t       )MenuContol_STK_SIZE,            /* 任务堆栈大小 */
                (void*          )NULL,                      /* 传入给任务函数的参数 */
                (UBaseType_t    )MenuContol_TASK_PRIO,           /* 任务优先级 */
                (TaskHandle_t*  )&MenuContolTask_Handler);       /* 任务句柄 */		

							
		xTaskCreate((TaskFunction_t )StepCounter_TASK,                /* 任务函数 */
                (const char*    )"StepCounter_TASK",              /* 任务名称 */
                (uint16_t       )StepCounter_STK_SIZE,            /* 任务堆栈大小 */
                (void*          )NULL,                      /* 传入给任务函数的参数 */
                (UBaseType_t    )StepCounter_TASK_PRIO,           /* 任务优先级 */
                (TaskHandle_t*  )&StepCounterTask_Handler);       /* 任务句柄 */	
			
								
		xTaskCreate((TaskFunction_t )USART2_TASK,                /* 任务函数 */
                (const char*    )"USART2_TASK",              /* 任务名称 */
                (uint16_t       )USART2_STK_SIZE,            /* 任务堆栈大小 */
                (void*          )NULL,                      /* 传入给任务函数的参数 */
                (UBaseType_t    )USART2_TASK_PRIO,           /* 任务优先级 */
                (TaskHandle_t*  )&USART2Task_Handler);  
								/* 任务句柄 */										
	xTimer = xTimerCreate("Timer10ms",
												pdMS_TO_TICKS(10),
												pdTRUE,                 // 周期性
												0,
												vTimerCallback );								
								
		vTaskDelete(NULL);                                      //释放			
		taskEXIT_CRITICAL();                                    //退出
}

void Watch_FreeRTOS_Init(void)
{
	
	
	Timer_Init();
	
   xTaskCreate((TaskFunction_t )Start_TASK,                /* 任务函数 */
                (const char*    )"Start_TASK",              /* 任务名称 */
                (uint16_t       )START_STK_SIZE,            /* 任务堆栈大小 */
                (void*          )NULL,                      /* 传入给任务函数的参数 */
                (UBaseType_t    )START_TASK_PRIO,           /* 任务优先级 */
                (TaskHandle_t*  )&StartTask_Handler);       /* 任务句柄 */
   vTaskStartScheduler();

}

// 显示主界面
static void  Main_interface(void)
{
	OLED_Printf(0,0,OLED_6X8,"%d-%d-%d",MyRTC_Time[0],MyRTC_Time[1],MyRTC_Time[2]);
	
	OLED_Printf(6*17,0,OLED_6X8,"%d",total_steps);
	OLED_Printf(16,16,OLED_12X24,"%02d:%02d:%02d",MyRTC_Time[3],MyRTC_Time[4],MyRTC_Time[5]);
	OLED_ShowString(0,3*16,"菜单",OLED_8X16);
	OLED_ShowString(6*16,3*16,"设置",OLED_8X16);
	OLED_ShowImage(6*17-10,0 , 10, 8, step);
	int8_t encoder;

	xQueuePeek(xEncoderQueue, &encoder, 100);
	if(encoder % 2 == 0)
	{
		OLED_ReverseArea(0,48,32,16);
	}
	else
	{
		OLED_ReverseArea(96,48,32,16);
	}
	
}


//滑动菜单动画函数
void Menu_Animation(void)
{
	OLED_Clear();
	OLED_ShowImage(42,10,44,44,Frame);
	
	if(pre_selection<target_selection)
	{
		x_pre-=Speed;
		if(x_pre==0)
		{
			pre_selection++;
			move_flag=0;
			x_pre=48;
		}
	}
	
	if(pre_selection>target_selection)
	{
		x_pre+=Speed;
		if(x_pre==96)
		{
			pre_selection--;
			move_flag=0;
			x_pre=48;
		}
	}
	
	if(pre_selection>=1)
	{
		OLED_ShowImage(x_pre-48,16,32,32,Menu_Graph[pre_selection-1]);
	}
	
	if(pre_selection>=2)
	{
		OLED_ShowImage(x_pre-96,16,32,32,Menu_Graph[pre_selection-2]);
	}
	
	OLED_ShowImage(x_pre,16,32,32,Menu_Graph[pre_selection]);
	OLED_ShowImage(x_pre+48,16,32,32,Menu_Graph[pre_selection+1]);
	OLED_ShowImage(x_pre+96,16,32,32,Menu_Graph[pre_selection+2]);
	
	OLED_Update();
}
//选择移动方向的函数
void Set_Selection(uint8_t move_flag,uint8_t Pre_Selection,uint8_t Target_Selection)
{
	if(move_flag==1)
	{
		pre_selection=Pre_Selection;
		target_selection=Target_Selection;
		
	}
	Menu_Animation();
}
//滑动菜单到具体功能的转场函数（下移转场）
void MenuToFunction(void)
{
	for(uint8_t i=0;i<=6;i++)//每个循环向下移8格，六个循环过后完全移出屏幕
	{
		OLED_Clear();
			if(pre_selection>=1)
		{
			OLED_ShowImage(x_pre-48,16+8*i,32,32,Menu_Graph[pre_selection-1]);
		}
		OLED_ShowImage(x_pre,16+8*i,32,32,Menu_Graph[pre_selection]);
		OLED_ShowImage(x_pre+48,16+8*i,32,32,Menu_Graph[pre_selection+1]);
		OLED_Update();
	}
}
void Menu_interface1(void)
{
		if(menu_flag==1)
		{
			if(DirectFlag==1)Set_Selection(move_flag,1,0);
			else if(DirectFlag==2)Set_Selection(move_flag,0,0);
		}
		
		else
		{
			if(DirectFlag==1)Set_Selection(move_flag,menu_flag,menu_flag-1);
			else if(DirectFlag==2)Set_Selection(move_flag,menu_flag-2,menu_flag-1);
		}
		switch(menu_flag)
		{
			case 1:
			{
				OLED_ShowString(34,0,"< Return >",OLED_6X8);
			}break;
			case 2:
			{
				OLED_ShowString(34,0,"<  Time  >",OLED_6X8);
			}break;
			case 3:
			{
				OLED_ShowString(34,0,"<Lighting>",OLED_6X8);
			}break;
			case 4:
			{
				OLED_ShowString(34,0,"<Stepdata>",OLED_6X8);
			}break;
			case 5:
			{
				OLED_ShowString(34,0,"<RAM_byte>",OLED_6X8);
			}break;
			case 6:
			{
				OLED_ShowString(34,0,"<QRcode>",OLED_6X8);
			}break;
		}
		
}
void Menu_interface2(void)
{
	OLED_ShowString(0,0,"返回",OLED_8X16);
	OLED_Printf(16,16,OLED_12X24,"%02d:%02d:%02d",min_count,s_count,ms10_count);
	OLED_ShowString(8,44,"开始",OLED_8X16);
	OLED_ShowString(48,44,"停止",OLED_8X16);
	OLED_ShowString(88,44,"清除",OLED_8X16);

	int8_t encoder;
	xQueuePeek(xEncoderQueue, &encoder, 100);
	if(encoder >= 0)
	{
		switch(abs(encoder) % 4)
		{
			case 0: OLED_ReverseArea(0,0,32,16);break;
			case 1: OLED_ReverseArea(8,44,32,16);break;
			case 2: OLED_ReverseArea(48,44,32,16);break;
			case 3: OLED_ReverseArea(88,44,32,16);break;
		}		
	}
	else
	{
		switch(abs(encoder) % 4)
		{
			case 0: OLED_ReverseArea(0,0,32,16);break;
			case 3: OLED_ReverseArea(8,44,32,16);break;
			case 2: OLED_ReverseArea(48,44,32,16);break;
			case 1: OLED_ReverseArea(88,44,32,16);break;
		}	
	}
}

void Menu_interface3(void)
{
	OLED_Reverse();
}

void Menu_interface4(void)
{
			// 显示总步数
		OLED_Printf(0, 0, OLED_8X16, "Steps: %-5d", total_steps);
		
		// 显示活跃轴
		const char* axis_name = "None";
		if(most_active_axis == ACTIVE_X) axis_name = "X";
		else if(most_active_axis == ACTIVE_Y) axis_name = "Y";
		else if(most_active_axis == ACTIVE_Z) axis_name = "Z";
		
		OLED_Printf(0, 16, OLED_8X16, "Axis:%s", axis_name);
		OLED_ShowHexNum(8*10,16,ID,2,OLED_8X16);
		
		// 显示当前陀螺仪数据（滤波后）
		OLED_Printf(0, 16*2, OLED_8X16, "X%6d Y%6d", 
							 ave_GyroValue.X, ave_GyroValue.Y);
		OLED_Printf(0, 16*3, OLED_8X16, "Z%6d", ave_GyroValue.Z);    
}
void Menu_interface5(void)
{
	OLED_Printf(0,0,OLED_8X16,"Space");
	OLED_Printf(0,16,OLED_6X8,"RTC :%d",uxTaskGetStackHighWaterMark(RTCTask_Handler));
	OLED_Printf(0,24,OLED_6X8,"OLED:%d",uxTaskGetStackHighWaterMark(OLEDTASK_Handler));
	OLED_Printf(0,32,OLED_6X8,"KEY :%d",uxTaskGetStackHighWaterMark(KeyscanTask_Handler));
	OLED_Printf(0,40,OLED_6X8,"Ecr :%d",uxTaskGetStackHighWaterMark(EncoderTask_Handler));
	OLED_Printf(0,48,OLED_6X8,"Mcl :%d",uxTaskGetStackHighWaterMark(MenuContolTask_Handler));
	OLED_Printf(0,56,OLED_6X8,"Step:%d",uxTaskGetStackHighWaterMark(StepCounterTask_Handler));
}
void Menu_interface6(void)
{
	int8_t encoder;
	xQueuePeek(xEncoderQueue, &encoder, 100);
	if(abs(encoder) % 2 == 0)
	{
		OLED_ShowImage(32,0, 64,64, weixin);
	}
	else
	{
		OLED_ShowImage(32,0, 64,64, zhifubao);
	}
}
void Menu_interface7(void)
{
	
}
void Menu_interface(void)
{
	static uint8_t flag = 1;
	switch(Menu_flag1)
	{
		case 0:
		{
				Menu_interface1();
				flag = 1;
		}break;
		case 1:
		{
				if(flag == 1)
				{
					MenuToFunction();
					flag = 0;
				}
				Menu_interface2();
		}break;
		case 2:
		{
				if(flag == 1)
				{
					MenuToFunction();
					Menu_interface3();
					flag = 0;
				}
		}break;
		case 3:
		{
				if(flag == 1)
				{
					flag = 0;
					MenuToFunction();
				}
				Menu_interface4();
		}break;
		case 4:
		{
				if(flag == 1)
				{
					flag = 0;
					MenuToFunction();
				}
				Menu_interface5();
		}break;
		case 5:
		{
				if(flag == 1)
				{
					flag = 0;
					MenuToFunction();
				}
				Menu_interface6();
		}break;
		case 6:
		{
				if(flag == 1)
				{
					flag = 0;
					MenuToFunction();
				}
				Menu_interface7();
		}break;
	}
}





// 设置界面
static void  Setting_interface1(void)
{
	OLED_ShowString(0,0,"返回",OLED_8X16);
	OLED_ShowString(0,1*16,"日期时间",OLED_8X16);
	OLED_ShowString(0,2*16,"省电模式",OLED_8X16);
	int encoder;

	xQueuePeek(xEncoderQueue, &encoder, 100);
	if(abs(encoder) % 3 == 0)
	{
		OLED_ReverseArea(0,0,32,16);
	}
	else if(abs(encoder) % 3 == 1)
	{
		OLED_ReverseArea(0,16,16*4,16);
	}
	else if(abs(encoder) % 3 == 2)
	{
		OLED_ReverseArea(0,2*16,16*4,16);
	}
}
// 设置界面
static void  Setting_interface3(void)
{
	OLED_ShowString(0,0,"返回",OLED_8X16);
	OLED_ShowString(0,1*16,"YES",OLED_8X16);
	OLED_ShowString(0,2*16,"NO",OLED_8X16);
	int encoder;

	xQueuePeek(xEncoderQueue, &encoder, 100);
	if(abs(encoder) % 3 == 0)
	{
		OLED_ReverseArea(0,0,32,16);
	}
	else if(abs(encoder) % 3 == 1)
	{
		OLED_ReverseArea(0,16,8*3,16);
	}
	else if(abs(encoder) % 3 == 2)
	{
		OLED_ReverseArea(0,2*16,8*2,16);
	}
	if(Power_saving_mode == 1)
	{
		OLED_ShowString(8*10,16,"<---",OLED_8X16);
		
		power_flag = get_power_flag();
		if(power_flag == 0)
		{
				// 1低功耗模式MPU6050
			// 关闭 STM32 的 I2C2 和 GPIOB 时钟 
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, DISABLE); 
			//进入设备低功耗
			MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x4F); // 进入睡眠模式，并选择最低功耗时钟源
			
		// 2串口低功耗
			// 关闭 USART2 时钟
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE); 
			// 关闭 USART2 的 TX 和 RX 引脚时钟（假设使用 PA2, PA3）
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);		
			set_power_flag(1);
		}				
	}
	else if(Power_saving_mode == 0)
	{
		OLED_ShowString(8*10,16*2,"<---",OLED_8X16);
		power_flag = get_power_flag();
		if(power_flag == 1)
		{
		
					// 重新初始化  
			//恢复MPU6050
				MPU6050_Init();
			//恢复串口
				App_USART_Init();
			set_power_flag(0);
		}
	}
	
}
static void  Setting_interface2(void)
{
	OLED_ShowString(0,0,"返回",OLED_8X16);
	OLED_Printf(0,16,OLED_8X16,"DATE:%d-%02d-%02d",MyRTC_Time[0],MyRTC_Time[1],MyRTC_Time[2]);
	OLED_Printf(0,16*2,OLED_8X16,"TIME:%02d-%02d-%02d",MyRTC_Time[3],MyRTC_Time[4],MyRTC_Time[5]);
	
	
	int encoder;
	xQueuePeek(xEncoderQueue, &encoder, 100);
	setting_ne = get_settingne_flag();
	if(setting_ne == 0)
	{
		if(abs(encoder) % 3 == 0)
		{
			OLED_ReverseArea(0,0,32,16);
		}
		else if(abs(encoder) % 3 == 1)
		{
			OLED_ReverseArea(0,16,8*4,16);
		}
		else if(abs(encoder) % 3 == 2)
		{
			OLED_ReverseArea(0,16*2,8*4,16);
		}				
	}
	else if(setting_ne == 1)
	{
		if(abs(encoder) % 4 == 0)
		{
			OLED_ReverseArea(0,16,8*4,16);
		}
		else if(abs(encoder) % 4 == 1)
		{
			OLED_ReverseArea(5*8,16,8*4,16);
		}
		else if(abs(encoder) % 4 == 2)
		{
			OLED_ReverseArea(10*8,16,8*2,16);
		}
		else if(abs(encoder) % 4 == 3)
		{
			OLED_ReverseArea(13*8,16,8*2,16);
		}		
	}
	else if(setting_ne == 5)
	{
		if(abs(encoder) % 4 == 0)
		{
			OLED_ReverseArea(0,16*2,8*4,16);
		}
		else if(abs(encoder) % 4 == 1)
		{
			OLED_ReverseArea(5*8,16*2,8*2,16);
		}
		else if(abs(encoder) % 4 == 2)
		{
			OLED_ReverseArea(8*8,16*2,8*2,16);
		}
		else if(abs(encoder) % 4 == 3)
		{
			OLED_ReverseArea(11*8,16*2,8*2,16);
		}		
	}
}
static void Setting_interface(void)
{
	if(OLED_setting_flag == 0) Setting_interface1();
	else if(OLED_setting_flag == 1) Setting_interface2();
	else if(OLED_setting_flag == 2)  Setting_interface3();
}
static void key_callback(void)
{
	
	int encoder;
	xQueuePeek(xEncoderQueue, &encoder, 100);
	OLED_main_flag  = get_main_flag();
	OLED_setting_flag = get_setting_flag();
	setting_ne = get_settingne_flag();
	// 主界面切换
	
	switch(OLED_main_flag)
	{
		case 0: 
		{
			if(abs(encoder) % 2 == 1)
			{
				set_main_flag(2);
			}	
			if(abs(encoder) % 2 == 0)
			{
				set_main_flag(1);
			}							
		}
		break;
		case 1:
		{
			if(OLED_menu_flag == 0)
			{
				if(Menu_flag1 == 0)
				{
					switch(menu_flag)
					{
						case 1:set_main_flag(0);break;
						default:   Menu_flag1 = menu_flag - 1;break;
					}
				}
				else if(Menu_flag1 == 1)
				{
					switch(abs(encoder) %  4)
					{
						case 0: Menu_flag1 =  0;break;
						case 1: Timer_Start(); break;
						case 2: Timer_Stop();break;
						case 3: Timer_Reset();break;
					}
				}
				else
				{
					Menu_flag1 = 0;
				}
			}
		}break;
		case 2:
		{
				if(OLED_setting_flag == 0)
				{
					if(abs(encoder) % 3 == 0)
					{
						set_main_flag(0);
					}
					else if(abs(encoder) % 3 == 1)
					{
						set_setting_flag(1);			
					}
					else if(abs(encoder) % 3 == 2)
					{
						set_setting_flag(2);
					}
				}
						//设置2级页面
				else if(OLED_setting_flag == 1)
				{
					if(setting_ne == 0)
					{
						switch(abs(encoder) % 3 )
						{
							case 0: set_setting_flag(0);   break;
							case 1: set_settingne_flag(1); break;
							case 2: set_settingne_flag(5); break;
						}
					}
					else if(setting_ne == 1)
					{
						switch(abs(encoder) % 4 )
						{
							case 0: set_settingne_flag(0); break;
							case 1: set_settingne_flag(2); break;
							case 2: set_settingne_flag(3); break;
							case 3: set_settingne_flag(4); break;
						}
					}
					else if(setting_ne >= 2 && setting_ne <= 4)
					{
						set_settingne_flag(1);
					}
					else if(setting_ne == 5)
					{
						switch(abs(encoder) % 4 )
						{
							case 0: set_settingne_flag(0); break;
							case 1: set_settingne_flag(6); break;
							case 2: set_settingne_flag(7); break;
							case 3: set_settingne_flag(8); break;
						}
					}
					else if(setting_ne >= 6 && setting_ne <= 8)
					{
						set_settingne_flag(5);
					}
				}		
			else if(OLED_setting_flag == 2)
			{
				switch(abs(encoder) % 3)
				{
					case 0: set_setting_flag(0); break;
					case 1: Power_saving_mode = 1; break;
					case 2: Power_saving_mode = 0; break;
				}
			}
				
		}
		break;
	}
	
	Encoder_Clear();
	OLED_Clear();
	OLED_Update();
}























//
uint8_t get_main_flag(void) 
{
    uint8_t temp;
    if (xSemaphoreTake(flag_mutex, portMAX_DELAY) == pdTRUE) {
        temp = OLED_main_flag;
        xSemaphoreGive(flag_mutex);
    }
    return temp;
}

void set_main_flag(uint8_t value) 
{
    if (xSemaphoreTake(flag_mutex, portMAX_DELAY) == pdTRUE) {
        OLED_main_flag = value;
        xSemaphoreGive(flag_mutex);
    }
}


uint8_t get_setting_flag(void) 
{
    uint8_t temp;
    if (xSemaphoreTake(flag_mutex, portMAX_DELAY) == pdTRUE) {
        temp = OLED_setting_flag;
        xSemaphoreGive(flag_mutex);
    }
    return temp;
}

void set_setting_flag(uint8_t value) 
{
    if (xSemaphoreTake(flag_mutex, portMAX_DELAY) == pdTRUE) {
        OLED_setting_flag = value;
        xSemaphoreGive(flag_mutex);
    }
}

uint8_t get_settingne_flag(void) 
{
    uint8_t temp;
    if (xSemaphoreTake(flag_mutex, portMAX_DELAY) == pdTRUE) {
        temp = setting_ne;
        xSemaphoreGive(flag_mutex);
    }
    return temp;
}

void set_settingne_flag(uint8_t value) 
{
    if (xSemaphoreTake(flag_mutex, portMAX_DELAY) == pdTRUE) {
        setting_ne = value;
        xSemaphoreGive(flag_mutex);
    }
}


uint8_t get_power_flag(void) 
{
    uint8_t temp;
    if (xSemaphoreTake(flag_mutex, portMAX_DELAY) == pdTRUE) {
        temp = power_flag;
        xSemaphoreGive(flag_mutex);
    }
    return temp;
}

void set_power_flag(uint8_t value) 
{
    if (xSemaphoreTake(flag_mutex, portMAX_DELAY) == pdTRUE) {
        power_flag = value;
        xSemaphoreGive(flag_mutex);
    }
}






