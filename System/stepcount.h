#ifndef __STEPCOUNT_H
#define __STEPCOUNT_H
#include "stm32f10x.h"                  // Device header
#include "stm32f10x.h"                  // Device header

/* 计步算法相关宏定义 */
#define ABS(a)                  (((a) < 0) ? -(a) : (a))
#define MAX(a, b)               (((a) > (b)) ? (a) : (b))
#define MIN(a, b)               (((a) < (b)) ? (a) : (b))
#define SAMPLE_NUM             5           // 采样次数
#define MIN_RELIABLE_VARIATION 300         // 最小可信变化量
#define MAX_RELIABLE_VARIATION 30000       // 最大可信变化量
#define ACTIVE_NUM             20          // 活跃轴更新周期
#define ACTIVE_NULL            0
#define ACTIVE_X               1
#define ACTIVE_Y               2
#define ACTIVE_Z               3
#define RESET_THRESHOLD        15          // 极值重置阈值
#define HYSTERESIS_BAND        100         // 滞后带大小
#define MIN_STEP_INTERVAL      300         // 最小步间间隔(ms)

/* 状态机定义 */
#define STATE_BELOW_MID 0  // 上一次在中值下方
#define STATE_ABOVE_MID 1  // 上一次在中值上方

void reset_peaks(void);
void Gyro_sample_update(void);
void which_is_active(void);
void detect_step(void);

/* 数据结构定义 */
typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
} axis_value_t;

typedef struct {
    axis_value_t max;
    axis_value_t min;
    uint8_t reset_count; // 极值重置计数器
} peak_value_t;

extern uint8_t axis_state[4];
extern axis_value_t old_ave_GyroValue, ave_GyroValue;
extern peak_value_t peak_value;
extern uint8_t most_active_axis;
extern uint16_t step_count;
extern uint16_t total_steps;

#endif
