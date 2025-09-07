#include "stepcount.h"
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"

/* 全局变量 */
axis_value_t old_ave_GyroValue, ave_GyroValue;
peak_value_t peak_value;
uint8_t most_active_axis = ACTIVE_NULL;
uint16_t step_count = 0;
uint16_t total_steps = 0;

// 各轴状态记录（全局变量，修复axis_state未定义错误）
uint8_t axis_state[4] = {STATE_BELOW_MID}; // 索引0不使用，1:X, 2:Y, 3:Z


/* 重置极值函数 */
void reset_peaks(void) 
{
    peak_value.max.X = ave_GyroValue.X;
    peak_value.min.X = ave_GyroValue.X;
    peak_value.max.Y = ave_GyroValue.Y;
    peak_value.min.Y = ave_GyroValue.Y;
    peak_value.max.Z = ave_GyroValue.Z;
    peak_value.min.Z = ave_GyroValue.Z;
    peak_value.reset_count = 0;
}

/* 均值采样更新函数 */
void Gyro_sample_update(void) 
{
    axis_value_t GyroValue;
    int sum[3] = {0};
    uint8_t success_num = 0;

    // 保存上一次数据
    old_ave_GyroValue = ave_GyroValue;

    // 多次采样取平均值
    for(uint8_t i = 0; i < SAMPLE_NUM; i++) {
        MPU6050_GetData(&GyroValue.X, &GyroValue.Y, &GyroValue.Z);
        
        sum[0] += GyroValue.X;
        sum[1] += GyroValue.Y;
        sum[2] += GyroValue.Z;
        success_num++;
        
        // 短延时确保采样间隔
        vTaskDelay(1);
    }
    
    if(success_num > 0) {
        ave_GyroValue.X = sum[0] / success_num;
        ave_GyroValue.Y = sum[1] / success_num;
        ave_GyroValue.Z = sum[2] / success_num;
    } else {
        // 采样失败时恢复上次值
        ave_GyroValue = old_ave_GyroValue;
    }

    // 计算原始变化量
    int16_t deltaX = ABS(ave_GyroValue.X - old_ave_GyroValue.X);
    int16_t deltaY = ABS(ave_GyroValue.Y - old_ave_GyroValue.Y);
    int16_t deltaZ = ABS(ave_GyroValue.Z - old_ave_GyroValue.Z);
    
    // 变化量超限处理 - 仅用于数据滤波
    if(deltaX < MIN_RELIABLE_VARIATION || deltaX > MAX_RELIABLE_VARIATION) {
        ave_GyroValue.X = old_ave_GyroValue.X;
    }
    if(deltaY < MIN_RELIABLE_VARIATION || deltaY > MAX_RELIABLE_VARIATION) {
        ave_GyroValue.Y = old_ave_GyroValue.Y;
    }
    if(deltaZ < MIN_RELIABLE_VARIATION || deltaZ > MAX_RELIABLE_VARIATION) {
        ave_GyroValue.Z = old_ave_GyroValue.Z;
    }

    // 更新极值
    peak_value.max.X = MAX(peak_value.max.X, ave_GyroValue.X);
    peak_value.min.X = MIN(peak_value.min.X, ave_GyroValue.X);
    peak_value.max.Y = MAX(peak_value.max.Y, ave_GyroValue.Y);
    peak_value.min.Y = MIN(peak_value.min.Y, ave_GyroValue.Y);
    peak_value.max.Z = MAX(peak_value.max.Z, ave_GyroValue.Z);
    peak_value.min.Z = MIN(peak_value.min.Z, ave_GyroValue.Z);
    
    // 定期重置极值
    peak_value.reset_count++;
    if(peak_value.reset_count > RESET_THRESHOLD) {
        reset_peaks();
    }
}

/* 活跃轴检测函数 */
void which_is_active(void) 
{
    static axis_value_t active = {0};
    static uint8_t active_sample_num = 0;
    
    // 使用原始变化量计算活跃度
    int16_t deltaX = ABS(ave_GyroValue.X - old_ave_GyroValue.X);
    int16_t deltaY = ABS(ave_GyroValue.Y - old_ave_GyroValue.Y);
    int16_t deltaZ = ABS(ave_GyroValue.Z - old_ave_GyroValue.Z);

    // 更新活跃度权重
    if(deltaX > deltaY && deltaX > deltaZ) {
        active.X++;
    } else if(deltaY > deltaX && deltaY > deltaZ) {
        active.Y++;
    } else if(deltaZ > deltaX && deltaZ > deltaY) {
        active.Z++;
    }

    active_sample_num++;

    // 周期更新最活跃轴
    if(active_sample_num >= ACTIVE_NUM) {
        // 权重竞争机制
        if(active.X > active.Y && active.X > active.Z && active.X > 5) {
            most_active_axis = ACTIVE_X;
        } else if(active.Y > active.X && active.Y > active.Z && active.Y > 5) {
            most_active_axis = ACTIVE_Y;
        } else if(active.Z > active.X && active.Z > active.Y && active.Z > 5) {
            most_active_axis = ACTIVE_Z;
        } else {
            most_active_axis = ACTIVE_NULL;
        }
        
        // 重置计数
        active_sample_num = 0;
        active.X = 0;
        active.Y = 0;
        active.Z = 0;
    }
}

/* 步数检测函数 */
void detect_step(void) 
{
    if(most_active_axis == ACTIVE_NULL) return;
    
    int16_t mid;
    int16_t current;

    // 根据活跃轴选择数据
    switch(most_active_axis) {
        case ACTIVE_X:
            mid = (peak_value.max.X + peak_value.min.X) / 2;
            current = ave_GyroValue.X;
            break;
        case ACTIVE_Y:
            mid = (peak_value.max.Y + peak_value.min.Y) / 2;
            current = ave_GyroValue.Y;
            break;
        case ACTIVE_Z:
            mid = (peak_value.max.Z + peak_value.min.Z) / 2;
            current = ave_GyroValue.Z;
            break;
        default:
            return;
    }
    
    // 低通滤波
    static int16_t filtered_current = 0;
    filtered_current = (filtered_current * 3 + current) / 4;
    
    // 检测跨越中值事件（带状态机和滞后带）
    uint8_t current_state = axis_state[most_active_axis];
    uint8_t new_state = current_state;
    
    if(filtered_current > mid + HYSTERESIS_BAND) {
        new_state = STATE_ABOVE_MID;
    } else if(filtered_current < mid - HYSTERESIS_BAND) {
        new_state = STATE_BELOW_MID;
    }
    
    // 状态变化检测
    if(current_state != new_state) {
        // 检测跨越
        if(current_state == STATE_BELOW_MID && new_state == STATE_ABOVE_MID) {
            step_count++;
            reset_peaks();
        }
        
        axis_state[most_active_axis] = new_state;
    }
}



