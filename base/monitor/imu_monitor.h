//
// Created by 98383 on 24-10-30.
//

#ifndef RM_FRAME_IMU_MONITOR_H
#define RM_FRAME_IMU_MONITOR_H

#include <stdint.h>

#include "algorithm/pid/pid.h"
#include "base/imu/imu.h"
#include "hardware_config.h"
#include "tim.h"

namespace imu {

// IMU temperature control IMU温度控制
class TempControl {
 public:
  TempControl(TIM_HandleTypeDef* htim, uint32_t tim_channel, PID pid)
      : pid_(pid), pwm_(0), htim_(htim), tim_channel_(tim_channel) {}

  void init(void);
  void handle(float temp_ref, float temp_fdb);

 private:
  PID pid_;
  uint16_t pwm_;
  // 加热电压输出端口
  TIM_HandleTypeDef* htim_;
  uint32_t tim_channel_;
};

// 初始化所有IMU
void initAll(void);

// 处理所有IMU
void handleAll(void);

}  // namespace imu

extern IMU board_imu;

#endif  // RM_FRAME_IMU_MONITOR_H
