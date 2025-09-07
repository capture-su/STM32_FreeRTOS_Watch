#include "encoder.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"

// 编码器状态变量
static volatile int8_t encoder_count = 0;

// 上次 AB 相状态（用于状态机）
static uint8_t last_state = 0; // 存储 (A << 1) | B

// 编码器引脚定义（可配置）
#define ENCODER_A_PORT    GPIOB
#define ENCODER_A_PIN     GPIO_Pin_12
#define ENCODER_B_PORT    GPIOB
#define ENCODER_B_PIN     GPIO_Pin_0

// 编码器方向定义（根据实际接线调整）
#define ENCODER_CW        1   // 顺时针
#define ENCODER_CCW       -1  // 逆时针

// 编码器状态表（格雷码解码）
// prev_state | curr_state | direction
// 使用 2-bit 格雷码判断方向
const int8_t enc_table[16] = {
    0,  -1,  1,  0,
    1,   0,  0, -1,
   -1,   0,  0,  1,
    0,   1, -1,  0
};

void Encoder_Init(void)
{
    // 1. 初始化 IO：A 和 B 相都上拉输入
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
    GPIO_InitStruct.GPIO_Pin = ENCODER_A_PIN | ENCODER_B_PIN;
    GPIO_Init(ENCODER_A_PORT, &GPIO_InitStruct);

    // 2. 外部中断映射：PB12 -> EXTI12, PB0 -> EXTI0
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // 映射 PB12 到 EXTI12
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    // 映射 PB0 到 EXTI0
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

    // 3. 配置 EXTI12（A 相）
    EXTI_InitTypeDef EXTI_InitStruct = {0};
    EXTI_InitStruct.EXTI_Line = EXTI_Line12 | EXTI_Line0;  // 同时监听两个引脚
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // 双边沿
    EXTI_Init(&EXTI_InitStruct);

    // 4. NVIC 配置
    NVIC_InitTypeDef NVIC_InitStruct = {0};
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;  // PB12 -> EXTI12
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStruct);

    // 额外：配置 EXTI0 的 NVIC（PB0）
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    // 读取初始状态
    uint8_t a = GPIO_ReadInputDataBit(ENCODER_A_PORT, ENCODER_A_PIN) == Bit_SET ? 1 : 0;
    uint8_t b = GPIO_ReadInputDataBit(ENCODER_B_PORT, ENCODER_B_PIN) == Bit_SET ? 1 : 0;
    last_state = (a << 1) | b;
}

// 通用处理函数（被两个中断共享）
void encoder_isr_handler(void)
{
    // 读取当前 A、B 状态
    uint8_t a = GPIO_ReadInputDataBit(ENCODER_A_PORT, ENCODER_A_PIN) == Bit_SET ? 1 : 0;
    uint8_t b = GPIO_ReadInputDataBit(ENCODER_B_PORT, ENCODER_B_PIN) == Bit_SET ? 1 : 0;
    uint8_t curr_state = (a << 1) | b;

    // 查表判断方向
    uint8_t index = (last_state << 2) | curr_state;
    int8_t direction = enc_table[index];

    if (direction != 0) {
        taskENTER_CRITICAL();  // 进入临界区
        encoder_count += direction;
        taskEXIT_CRITICAL();   // 退出临界区
    }

    last_state = curr_state;
}

// EXTI12 中断（PB12）
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line12);
        encoder_isr_handler();
    }
}

// EXTI0 中断（PB0）
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line0);
        encoder_isr_handler();
    }
}

// 获取编码器值（线程安全）
int8_t Encoder_GetValue(void)
{
    int8_t value;
    taskENTER_CRITICAL();
    value = encoder_count / 4;
    taskEXIT_CRITICAL();
    return value;
}

// 清零
void Encoder_Clear(void)
{
    taskENTER_CRITICAL();
    encoder_count = 0;
    taskEXIT_CRITICAL();
}
//#include "encoder.h"
//#include "FreeRTOS.h"
//#include "task.h"

//static volatile int8_t encoder = 0;

//void Encoder_Init(void)
//{
//	//1初始化IO
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	GPIO_InitTypeDef GPIO_Init_Struct = {0};
//	GPIO_Init_Struct.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_Init_Struct.GPIO_Pin  = GPIO_Pin_12 | GPIO_Pin_0;
//	GPIO_Init(GPIOB,&GPIO_Init_Struct);
//	//2 中断映射
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);
//	//3 外部中断配置
//	EXTI_InitTypeDef EXTI_Init_Struct = {0};
//	EXTI_Init_Struct.EXTI_Line = EXTI_Line12;
//	EXTI_Init_Struct.EXTI_LineCmd  = ENABLE;
//	EXTI_Init_Struct.EXTI_Mode  = EXTI_Mode_Interrupt;
//	EXTI_Init_Struct.EXTI_Trigger  = EXTI_Trigger_Rising;
//	EXTI_Init(&EXTI_Init_Struct);
//	//4 中断优先级
//	NVIC_InitTypeDef NVIC_Init_Struct = {0};
//	NVIC_Init_Struct.NVIC_IRQChannel =  EXTI15_10_IRQn;
//	NVIC_Init_Struct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init_Struct.NVIC_IRQChannelPreemptionPriority = 5;
//	NVIC_Init_Struct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_Init_Struct);	
//}

//void EXTI15_10_IRQHandler(void)
//{
//	if(EXTI_GetFlagStatus(EXTI_Line12) == SET)
//	{
//		EXTI_ClearFlag(EXTI_Line12);

//		
//		uint8_t a = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12);
//		uint8_t b = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
//		if((a == Bit_SET && b== Bit_RESET))
//		{
//			encoder++;
//		}
//		else if(a == Bit_SET && b== Bit_SET)
//		{
//			encoder--;
//		}
//	}
//}

//int8_t Encoder_GetValue(void)
//{
//		taskENTER_CRITICAL();      // 进入临界区（关闭中断）
//    int8_t value = encoder;   // 原子读取
//    taskEXIT_CRITICAL();       // 退出临界区（开启中断）
//    return value;
//}
//void Encoder_Clear(void)
//{
//	encoder = 0;
//}
