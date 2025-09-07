#include "encoder.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"

// ������״̬����
static volatile int8_t encoder_count = 0;

// �ϴ� AB ��״̬������״̬����
static uint8_t last_state = 0; // �洢 (A << 1) | B

// ���������Ŷ��壨�����ã�
#define ENCODER_A_PORT    GPIOB
#define ENCODER_A_PIN     GPIO_Pin_12
#define ENCODER_B_PORT    GPIOB
#define ENCODER_B_PIN     GPIO_Pin_0

// �����������壨����ʵ�ʽ��ߵ�����
#define ENCODER_CW        1   // ˳ʱ��
#define ENCODER_CCW       -1  // ��ʱ��

// ������״̬����������룩
// prev_state | curr_state | direction
// ʹ�� 2-bit �������жϷ���
const int8_t enc_table[16] = {
    0,  -1,  1,  0,
    1,   0,  0, -1,
   -1,   0,  0,  1,
    0,   1, -1,  0
};

void Encoder_Init(void)
{
    // 1. ��ʼ�� IO��A �� B �඼��������
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;  // ��������
    GPIO_InitStruct.GPIO_Pin = ENCODER_A_PIN | ENCODER_B_PIN;
    GPIO_Init(ENCODER_A_PORT, &GPIO_InitStruct);

    // 2. �ⲿ�ж�ӳ�䣺PB12 -> EXTI12, PB0 -> EXTI0
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // ӳ�� PB12 �� EXTI12
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    // ӳ�� PB0 �� EXTI0
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

    // 3. ���� EXTI12��A �ࣩ
    EXTI_InitTypeDef EXTI_InitStruct = {0};
    EXTI_InitStruct.EXTI_Line = EXTI_Line12 | EXTI_Line0;  // ͬʱ������������
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // ˫����
    EXTI_Init(&EXTI_InitStruct);

    // 4. NVIC ����
    NVIC_InitTypeDef NVIC_InitStruct = {0};
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;  // PB12 -> EXTI12
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStruct);

    // ���⣺���� EXTI0 �� NVIC��PB0��
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    // ��ȡ��ʼ״̬
    uint8_t a = GPIO_ReadInputDataBit(ENCODER_A_PORT, ENCODER_A_PIN) == Bit_SET ? 1 : 0;
    uint8_t b = GPIO_ReadInputDataBit(ENCODER_B_PORT, ENCODER_B_PIN) == Bit_SET ? 1 : 0;
    last_state = (a << 1) | b;
}

// ͨ�ô��������������жϹ���
void encoder_isr_handler(void)
{
    // ��ȡ��ǰ A��B ״̬
    uint8_t a = GPIO_ReadInputDataBit(ENCODER_A_PORT, ENCODER_A_PIN) == Bit_SET ? 1 : 0;
    uint8_t b = GPIO_ReadInputDataBit(ENCODER_B_PORT, ENCODER_B_PIN) == Bit_SET ? 1 : 0;
    uint8_t curr_state = (a << 1) | b;

    // ����жϷ���
    uint8_t index = (last_state << 2) | curr_state;
    int8_t direction = enc_table[index];

    if (direction != 0) {
        taskENTER_CRITICAL();  // �����ٽ���
        encoder_count += direction;
        taskEXIT_CRITICAL();   // �˳��ٽ���
    }

    last_state = curr_state;
}

// EXTI12 �жϣ�PB12��
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line12);
        encoder_isr_handler();
    }
}

// EXTI0 �жϣ�PB0��
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line0);
        encoder_isr_handler();
    }
}

// ��ȡ������ֵ���̰߳�ȫ��
int8_t Encoder_GetValue(void)
{
    int8_t value;
    taskENTER_CRITICAL();
    value = encoder_count / 4;
    taskEXIT_CRITICAL();
    return value;
}

// ����
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
//	//1��ʼ��IO
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	GPIO_InitTypeDef GPIO_Init_Struct = {0};
//	GPIO_Init_Struct.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_Init_Struct.GPIO_Pin  = GPIO_Pin_12 | GPIO_Pin_0;
//	GPIO_Init(GPIOB,&GPIO_Init_Struct);
//	//2 �ж�ӳ��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);
//	//3 �ⲿ�ж�����
//	EXTI_InitTypeDef EXTI_Init_Struct = {0};
//	EXTI_Init_Struct.EXTI_Line = EXTI_Line12;
//	EXTI_Init_Struct.EXTI_LineCmd  = ENABLE;
//	EXTI_Init_Struct.EXTI_Mode  = EXTI_Mode_Interrupt;
//	EXTI_Init_Struct.EXTI_Trigger  = EXTI_Trigger_Rising;
//	EXTI_Init(&EXTI_Init_Struct);
//	//4 �ж����ȼ�
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
//		taskENTER_CRITICAL();      // �����ٽ������ر��жϣ�
//    int8_t value = encoder;   // ԭ�Ӷ�ȡ
//    taskEXIT_CRITICAL();       // �˳��ٽ����������жϣ�
//    return value;
//}
//void Encoder_Clear(void)
//{
//	encoder = 0;
//}
