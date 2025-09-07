#ifndef __STEPCOUNT_H
#define __STEPCOUNT_H
#include "stm32f10x.h"                  // Device header
#include "stm32f10x.h"                  // Device header

/* �Ʋ��㷨��غ궨�� */
#define ABS(a)                  (((a) < 0) ? -(a) : (a))
#define MAX(a, b)               (((a) > (b)) ? (a) : (b))
#define MIN(a, b)               (((a) < (b)) ? (a) : (b))
#define SAMPLE_NUM             5           // ��������
#define MIN_RELIABLE_VARIATION 300         // ��С���ű仯��
#define MAX_RELIABLE_VARIATION 30000       // �����ű仯��
#define ACTIVE_NUM             20          // ��Ծ���������
#define ACTIVE_NULL            0
#define ACTIVE_X               1
#define ACTIVE_Y               2
#define ACTIVE_Z               3
#define RESET_THRESHOLD        15          // ��ֵ������ֵ
#define HYSTERESIS_BAND        100         // �ͺ����С
#define MIN_STEP_INTERVAL      300         // ��С������(ms)

/* ״̬������ */
#define STATE_BELOW_MID 0  // ��һ������ֵ�·�
#define STATE_ABOVE_MID 1  // ��һ������ֵ�Ϸ�

void reset_peaks(void);
void Gyro_sample_update(void);
void which_is_active(void);
void detect_step(void);

/* ���ݽṹ���� */
typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
} axis_value_t;

typedef struct {
    axis_value_t max;
    axis_value_t min;
    uint8_t reset_count; // ��ֵ���ü�����
} peak_value_t;

extern uint8_t axis_state[4];
extern axis_value_t old_ave_GyroValue, ave_GyroValue;
extern peak_value_t peak_value;
extern uint8_t most_active_axis;
extern uint16_t step_count;
extern uint16_t total_steps;

#endif
