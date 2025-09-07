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

/* ���� */
#include "key.h"
#include "encoder.h"
#include "MyRTC.h"
#include "mpu6050.h"
#include "Delay.h"
#include "OLED.h"
#include "app_usart.h"



// START_TASK ��������
#define START_TASK_PRIO 1                                   /* �������ȼ� */
#define START_STK_SIZE  128                                 /* �����ջ��С */
TaskHandle_t StartTask_Handler;                             /* ������ */
// RTC_TASK ��������
#define RTC_TASK_PRIO 13                                   /* �������ȼ� */
#define RTC_STK_SIZE  128                                 /* �����ջ��С */
TaskHandle_t RTCTask_Handler;                             /* ������ */
// OLED_TASK ��������
#define OLED_TASK_PRIO 11                                   /* �������ȼ� */
#define OLED_STK_SIZE  128*2                                 /* �����ջ��С */
TaskHandle_t OLEDTASK_Handler;                             /* ������ */
// Keyscan_TASK ��������
#define Keyscan_TASK_PRIO 10                                   /* �������ȼ� */
#define Keyscan_STK_SIZE  128                                 /* �����ջ��С */
TaskHandle_t KeyscanTask_Handler;                             /* ������ */
// Encoder_TASK ��������
#define Encoder_TASK_PRIO 9                                   /* �������ȼ� */
#define Encoder_STK_SIZE  128                                 /* �����ջ��С */
TaskHandle_t EncoderTask_Handler;                             /* ������ */

// MenuContol_TASK ��������
#define MenuContol_TASK_PRIO 6                                   /* �������ȼ� */
#define MenuContol_STK_SIZE  128                                 /* �����ջ��С */
TaskHandle_t MenuContolTask_Handler;                             /* ������ */

// StepCounter_TASK ��������
TaskHandle_t StepCounterTask_Handler;
#define StepCounter_TASK_PRIO 8                                 /* �������ȼ� */
#define StepCounter_STK_SIZE  128*2                                /* �����ջ��С */

// USART2_TASK ��������
TaskHandle_t USART2Task_Handler;
#define USART2_TASK_PRIO 7                                 /* �������ȼ� */
#define USART2_STK_SIZE  128*2                                /* �����ջ��С */




//�������� �㲥���ͽ��� ������ֹ��������
QueueHandle_t           xEncoderQueue;             /* ������� */
#define QUEUE_LENGTH    1                   /* ����֧�ֵ���Ϣ���� */
#define QUEUE_ITEM_SIZE sizeof(int8_t)     /* ������ÿ����Ϣ�Ĵ�С */

//�����շ��Ķ���
QueueHandle_t xUartRxQueue;
QueueHandle_t xUartTxQueue;
#define UART_QUEUE_LENGTH 128
#define UART_QUEUE_ITEM_SIZE sizeof(uint8_t)




// ������
static void Start_TASK(void *pvParameters);
static void RTC_TASK(void *pvParameters);
static void OLED_TASK(void *pvParameters);
static void Encoder_TASK(void *pvParameters);
static void MenuContol_TASK(void *pvParameters);
static void StepCounter_TASK(void *pvParameters); 
static void USART2_TASK(void *pvParameters); 
//���ú���
static void  key_callback(void);        //�����ص�
static void  Main_interface(void);      //��ʾ������
static void  Setting_interface(void);   //��ʾ���ý���
static void  Setting_interface1(void);  //�����ӽ���
static void  Setting_interface2(void);  //�����ӽ���
static void  Menu_interface(void);      //�˵�����




SemaphoreHandle_t flag_mutex;         //�����ź��� ��������flag;
/*********��װ�����ź�����������ȫ�ֱ�����ֹ��������*********/
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

//��ʾ�����ƶ�����
uint8_t pre_selection;//�ϴ�ѡ���ѡ��
uint8_t target_selection;//Ŀ��ѡ��
uint8_t x_pre=48;//�ϴ�ѡ���x����
uint8_t Speed=8;//�ٶ�
uint8_t move_flag;//��ʼ�ƶ��ı�־λ��1��ʾ��ʼ�ƶ���0��ʾֹͣ�ƶ�
uint8_t menu_flag = 1;
uint8_t DirectFlag = 1;
void Menu_Animation(void);
void Set_Selection(uint8_t move_flag,uint8_t Pre_Selection,uint8_t Target_Selection);






#include "MPU6050_Reg.h"
static  volatile  uint8_t Power_saving_mode = 0;
static  volatile  uint8_t power_flag = 0;
// ����͹���
void PRE_SLEEP_PROCESSING(void)
{
	power_flag = get_power_flag();
	//��ֹʡ�� ��͹��� ��ͻ
	if(power_flag == 0)
	{
		// 1�͹���ģʽMPU6050
			// �ر� STM32 �� I2C2 
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, DISABLE); 
			//�����豸�͹���
			MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x4F); // ����˯��ģʽ����ѡ����͹���ʱ��Դ
			
		// 2���ڵ͹���
			// �ر� USART2 ʱ��
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE); 
			// �ر� USART2 �� TX �� RX ����ʱ�ӣ�����ʹ�� PA2, PA3��
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);	
	}
		
}

// �˳��͹���
void POST_SLEEP_PROCESSING(void)
{
	power_flag = get_power_flag();
	//��ֹʡ�� ��͹��� ��ͻ
	if(power_flag == 0)
	{	
	// ���³�ʼ��  
	//�ָ�MPU6050
		MPU6050_Init();
	//�ָ�����
    App_USART_Init();
	}
}

void USART2_IRQHandler(void) 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t ucData;
    
    // �����ж�
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        ucData = USART_ReceiveData(USART2);
        xQueueSendFromISR(xUartRxQueue, &ucData, &xHigherPriorityTaskWoken);
    }
    
    // �����ж�
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        if(xQueueReceiveFromISR(xUartTxQueue, &ucData, &xHigherPriorityTaskWoken) == pdPASS) {
            USART_SendData(USART2, ucData);
        } else {
            // ���п�ʱ�رշ����ж�
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void usart_string(const char *str) 
{
    // �������
    if(str == NULL) return;
    
    // ��ȡ�ַ�������
		unsigned int len = strlen(str);
    if(len == 0) return;
    
    // ����ַ����뷢�Ͷ���
    for(size_t i = 0; i < len; i++) {
        // ����ʱ���ͣ���ֹ������
        if(xQueueSend(xUartTxQueue, &str[i], pdMS_TO_TICKS(100)) != pdPASS) {
            // ����ʧ�ܴ�������Ӵ�����
            #ifdef DEBUG
            // ����ģʽ�¿ɴ����ϵ�
            __BKPT(0);
            #endif
            break;
        }
    }
    
    // ʹ�ܷ����жϣ��������ͣ�
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}
void usart_Printf(const char *format, ...)
{
	  char buffer[128];      // �����������ڴ�Ÿ�ʽ������ַ���
    va_list args;                         // ���ڷ��ʿɱ�����ı���
    va_start(args, format);               // ��ʼ���ɱ�����б�
    vsnprintf(buffer, 128, format, args);  // ��ʽ���ַ�����������
    va_end(args);                         // ����ɱ�����б�
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
        // ���մ���
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

// ��ʱ�������ɱ��ⲿ���ʣ�
static volatile  uint8_t ms10_count = 0;
static volatile  uint8_t s_count = 0;
static volatile  uint8_t min_count = 0;
// ��ʱ�����
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
// ��ʼ����ʱ������ main �е���һ�Σ�
void Timer_Init(void)
{
    // ���������Զ�ʱ����ÿ 10ms ����һ��
    xTimer = xTimerCreate(
        "Timer10ms",
        pdMS_TO_TICKS(10),
        pdTRUE,                 // ������
        0,
        vTimerCallback
    );

    if (xTimer == NULL)
    {
        // ����ʧ�ܴ�����ѡ��
    }
}

// ������ʱ��
void Timer_Start(void)
{
    if (xTimer != NULL)
    {
        // ������ʱ��������ʱ��Ϊ 0��
        xTimerStart(xTimer, 0);
    }
}

// ��ͣ��ʱ��
void Timer_Stop(void)
{
    if (xTimer != NULL)
    {
        // ֹͣ��ʱ��
        xTimerStop(xTimer, 0);
    }
}

// �����ʱ����ֹͣ + ���ñ�����
void Timer_Reset(void)
{
    // ��ֹͣ������������ʱ���޸�
    Timer_Stop();

    // ���ü�ʱ����
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
//��ȡRTC����
static void RTC_TASK(void *pvParameters)
{
    while (1)
    {
        MyRTC_ReadTime();  // ����ʱ����ʾ
        // ֻ������ģʽ�²������޸�
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
                MyRTC_SetTime();     // д��RTC
                Encoder_Clear();     // ���㣬��ֹ�ظ���
                OLED_Update();       // ˢ����ʾ
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
                MyRTC_SetTime();     // д��RTC
                Encoder_Clear();     // ���㣬��ֹ�ظ���
                OLED_Update();       // ˢ����ʾ
            }
        }
        vTaskDelay(100);  // �����Ӧ�ٶ�
    }
}

//�����ı�״̬����
static void Keyscan_TASK(void *pvParameters)
{
    uint8_t state = 0;
    while (1)
    {
				switch (state)
					{
							case 0: // �ȴ���������
									if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET)
									{
											state = 1; // ��������״̬
									}
									break;

							case 1: // ��������������״̬��
									vTaskDelay(pdMS_TO_TICKS(20)); // ��������
									if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET)
									{
											state = 2; // ȷ�ϰ��£�����ȴ�����״̬
									}
									else
									{
											state = 0; // ����ʧ�ܣ��ص���ʼ״̬
									}
									break;

							case 2: // �ȴ��������֣�����������
									if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_SET)
									{
														
											key_callback();
										
											state = 0; // �����ص���ʼ״̬
									}
									break;
					}

					// ͳһ��ѯ��ʱ����Ҫ��״̬��ʹ�� vTaskDelay()
					vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//��ȡ����������
static void Encoder_TASK(void *pvParameters)
{
	static  int8_t  encoder; //��������ֵ
	while(1)
	{
		encoder = Encoder_GetValue();
		xQueueOverwrite(xEncoderQueue, &encoder);
		vTaskDelay(100);
	}
}
 /* �Ʋ����� */
uint8_t ID; 
static void StepCounter_TASK(void *pvParameters) 
{  
	
		ID = MPU6050_GetID();
    // ��ʼ����ֵ
    reset_peaks();
    
    // ��ʼУ׼
    vTaskDelay(pdMS_TO_TICKS(2000)); // 2��У׼ʱ��
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms����
    uint8_t step_time_count = 0;
    
    // ��ʼ����
    Gyro_sample_update();
    old_ave_GyroValue = ave_GyroValue;
    
    // ��¼��һ����Ч������ʱ��
    TickType_t last_step_time = 0;
    // ������С��������tick�����޸�pdTICKS_TO_MS����
    const TickType_t min_step_ticks = pdMS_TO_TICKS(MIN_STEP_INTERVAL);
    while(1)
		{
        // 1. ִ�в�������
        Gyro_sample_update();
        
        // 2. ����Ծ��
        which_is_active();
        
        // 3. �������
        detect_step();
        
        // 4. ����ȷ�ϣ�ÿ300ms��
        step_time_count++;
        if(step_time_count >= 6) { // 6 * 50ms = 300ms
            step_time_count = 0;
            if(step_count > 0) {
                // ��ȡ��ǰʱ��
                TickType_t current_time = xTaskGetTickCount();
                
                // ���ʱ������ʹ��tick���Ƚϣ�
                if(last_step_time == 0 || 
                   (current_time - last_step_time) >= min_step_ticks) 
                {
                    total_steps += 1; // ֻ��1��
                    last_step_time = current_time;
                }
                
                step_count = 0;
            }
        }        
        // ��ȷ��ʱȷ��50ms����
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
			vTaskDelay(100);      // ÿ�� 100ms
    }
}
//��ʼ��������ɾ��
static void Start_TASK(void *pvParameters)
{

		taskENTER_CRITICAL();                                   // �����ٽ���
		
		xEncoderQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
		    // �������ն��� (�ж�->����)
    xUartRxQueue = xQueueCreate(UART_QUEUE_LENGTH, UART_QUEUE_ITEM_SIZE);
    // �������Ͷ��� (����->�ж�)
    xUartTxQueue = xQueueCreate(UART_QUEUE_LENGTH, UART_QUEUE_ITEM_SIZE);
	
		flag_mutex = xSemaphoreCreateMutex();
    xTaskCreate((TaskFunction_t )RTC_TASK,                /* ������ */
                (const char*    )"RTC_TASK",              /* �������� */
                (uint16_t       )RTC_STK_SIZE,            /* �����ջ��С */
                (void*          )NULL,                      /* ������������Ĳ��� */
                (UBaseType_t    )RTC_TASK_PRIO,           /* �������ȼ� */
                (TaskHandle_t*  )&RTCTask_Handler);       /* ������ */


    xTaskCreate((TaskFunction_t )OLED_TASK,                /* ������ */
                (const char*    )"OLED_TASK",              /* �������� */
                (uint16_t       )OLED_STK_SIZE,            /* �����ջ��С */
                (void*          )NULL,                      /* ������������Ĳ��� */
                (UBaseType_t    )OLED_TASK_PRIO,           /* �������ȼ� */
                (TaskHandle_t*  )&OLEDTASK_Handler);       /* ������ */			
								
    xTaskCreate((TaskFunction_t )Keyscan_TASK,                /* ������ */
                (const char*    )"Keyscan_TASK",              /* �������� */
                (uint16_t       )Keyscan_STK_SIZE,            /* �����ջ��С */
                (void*          )NULL,                      /* ������������Ĳ��� */
                (UBaseType_t    )Keyscan_TASK_PRIO,           /* �������ȼ� */
                (TaskHandle_t*  )&KeyscanTask_Handler);       /* ������ */

    xTaskCreate((TaskFunction_t )Encoder_TASK,                /* ������ */
                (const char*    )"Encoder_TASK",              /* �������� */
                (uint16_t       )Encoder_STK_SIZE,            /* �����ջ��С */
                (void*          )NULL,                      /* ������������Ĳ��� */
                (UBaseType_t    )Encoder_TASK_PRIO,           /* �������ȼ� */
                (TaskHandle_t*  )&EncoderTask_Handler);       /* ������ */		
								
		xTaskCreate((TaskFunction_t )MenuContol_TASK,                /* ������ */
                (const char*    )"MenuContol_TASK",              /* �������� */
                (uint16_t       )MenuContol_STK_SIZE,            /* �����ջ��С */
                (void*          )NULL,                      /* ������������Ĳ��� */
                (UBaseType_t    )MenuContol_TASK_PRIO,           /* �������ȼ� */
                (TaskHandle_t*  )&MenuContolTask_Handler);       /* ������ */		

							
		xTaskCreate((TaskFunction_t )StepCounter_TASK,                /* ������ */
                (const char*    )"StepCounter_TASK",              /* �������� */
                (uint16_t       )StepCounter_STK_SIZE,            /* �����ջ��С */
                (void*          )NULL,                      /* ������������Ĳ��� */
                (UBaseType_t    )StepCounter_TASK_PRIO,           /* �������ȼ� */
                (TaskHandle_t*  )&StepCounterTask_Handler);       /* ������ */	
			
								
		xTaskCreate((TaskFunction_t )USART2_TASK,                /* ������ */
                (const char*    )"USART2_TASK",              /* �������� */
                (uint16_t       )USART2_STK_SIZE,            /* �����ջ��С */
                (void*          )NULL,                      /* ������������Ĳ��� */
                (UBaseType_t    )USART2_TASK_PRIO,           /* �������ȼ� */
                (TaskHandle_t*  )&USART2Task_Handler);  
								/* ������ */										
	xTimer = xTimerCreate("Timer10ms",
												pdMS_TO_TICKS(10),
												pdTRUE,                 // ������
												0,
												vTimerCallback );								
								
		vTaskDelete(NULL);                                      //�ͷ�			
		taskEXIT_CRITICAL();                                    //�˳�
}

void Watch_FreeRTOS_Init(void)
{
	
	
	Timer_Init();
	
   xTaskCreate((TaskFunction_t )Start_TASK,                /* ������ */
                (const char*    )"Start_TASK",              /* �������� */
                (uint16_t       )START_STK_SIZE,            /* �����ջ��С */
                (void*          )NULL,                      /* ������������Ĳ��� */
                (UBaseType_t    )START_TASK_PRIO,           /* �������ȼ� */
                (TaskHandle_t*  )&StartTask_Handler);       /* ������ */
   vTaskStartScheduler();

}

// ��ʾ������
static void  Main_interface(void)
{
	OLED_Printf(0,0,OLED_6X8,"%d-%d-%d",MyRTC_Time[0],MyRTC_Time[1],MyRTC_Time[2]);
	
	OLED_Printf(6*17,0,OLED_6X8,"%d",total_steps);
	OLED_Printf(16,16,OLED_12X24,"%02d:%02d:%02d",MyRTC_Time[3],MyRTC_Time[4],MyRTC_Time[5]);
	OLED_ShowString(0,3*16,"�˵�",OLED_8X16);
	OLED_ShowString(6*16,3*16,"����",OLED_8X16);
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


//�����˵���������
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
//ѡ���ƶ�����ĺ���
void Set_Selection(uint8_t move_flag,uint8_t Pre_Selection,uint8_t Target_Selection)
{
	if(move_flag==1)
	{
		pre_selection=Pre_Selection;
		target_selection=Target_Selection;
		
	}
	Menu_Animation();
}
//�����˵������幦�ܵ�ת������������ת����
void MenuToFunction(void)
{
	for(uint8_t i=0;i<=6;i++)//ÿ��ѭ��������8������ѭ��������ȫ�Ƴ���Ļ
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
	OLED_ShowString(0,0,"����",OLED_8X16);
	OLED_Printf(16,16,OLED_12X24,"%02d:%02d:%02d",min_count,s_count,ms10_count);
	OLED_ShowString(8,44,"��ʼ",OLED_8X16);
	OLED_ShowString(48,44,"ֹͣ",OLED_8X16);
	OLED_ShowString(88,44,"���",OLED_8X16);

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
			// ��ʾ�ܲ���
		OLED_Printf(0, 0, OLED_8X16, "Steps: %-5d", total_steps);
		
		// ��ʾ��Ծ��
		const char* axis_name = "None";
		if(most_active_axis == ACTIVE_X) axis_name = "X";
		else if(most_active_axis == ACTIVE_Y) axis_name = "Y";
		else if(most_active_axis == ACTIVE_Z) axis_name = "Z";
		
		OLED_Printf(0, 16, OLED_8X16, "Axis:%s", axis_name);
		OLED_ShowHexNum(8*10,16,ID,2,OLED_8X16);
		
		// ��ʾ��ǰ���������ݣ��˲���
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





// ���ý���
static void  Setting_interface1(void)
{
	OLED_ShowString(0,0,"����",OLED_8X16);
	OLED_ShowString(0,1*16,"����ʱ��",OLED_8X16);
	OLED_ShowString(0,2*16,"ʡ��ģʽ",OLED_8X16);
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
// ���ý���
static void  Setting_interface3(void)
{
	OLED_ShowString(0,0,"����",OLED_8X16);
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
				// 1�͹���ģʽMPU6050
			// �ر� STM32 �� I2C2 �� GPIOB ʱ�� 
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, DISABLE); 
			//�����豸�͹���
			MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x4F); // ����˯��ģʽ����ѡ����͹���ʱ��Դ
			
		// 2���ڵ͹���
			// �ر� USART2 ʱ��
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE); 
			// �ر� USART2 �� TX �� RX ����ʱ�ӣ�����ʹ�� PA2, PA3��
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
		
					// ���³�ʼ��  
			//�ָ�MPU6050
				MPU6050_Init();
			//�ָ�����
				App_USART_Init();
			set_power_flag(0);
		}
	}
	
}
static void  Setting_interface2(void)
{
	OLED_ShowString(0,0,"����",OLED_8X16);
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
	// �������л�
	
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
						//����2��ҳ��
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






