//
// Created by 98383 on 24-10-25.
//

#ifndef RM_FRAME_GIMBAL_H
#define RM_FRAME_GIMBAL_H

#include "algorithm/math/math.h"
#include "base/imu/imu.h"
#include "base/motor/motor.h"
#include "common/message_center/message_center.h"
#include "common/message_center/msg_def.h"
#include "stm32f4xx_hal.h"

// 云台反馈模式(imu/编码器)
typedef enum GimbalFdbMode {
  IMU_MODE,
  ENCODER_MODE,
} GimbalFdbMode_e;

// 云台运动状态
typedef struct GimbalStatus {
  float yaw;
  float pitch;
  float yaw_speed;
  float pitch_speed;
} GimbalStatus_t;

class Gimbal {
 public:
  Gimbal(Motor* gm_yaw, Motor* gm_pitch, IMU* imu);

  // Initialize gimbal, reset init flag
  // 云台初始化，重置回正标记
  void init(void);

  // 云台初始化判断
  bool initFinish(void) {
    return init_status_.yaw_finish && init_status_.pitch_finish;
  }

  // Set gimbal angle and speed
  // 设置云台角度&速度(deg, dps)
  void setAngleSpeed(const float& yaw, const float& pitch,
                     const float& yaw_speed = 0, const float& pitch_speed = 0);

  // Set gimbal angle
  // 设置云台角度(deg)
  void setAngle(const float& yaw, const float& pitch);

  // Add gimbal angle
  // 设置云台角度增量(deg)
  void addAngle(const float& d_yaw, const float& d_pitch);

  // Set gimbal mode(imu)
  // 设置云台模式(imu反馈或编码器反馈)
  void setMode(GimbalFdbMode_e mode);

  // Send target status to motor and update feedback
  // 设置电机目标状态，更新反馈数据
  void handle(void);

 private:
  // pitch轴力矩补偿(作为力矩前馈输入)
  float pitchCompensate(const float& pitch);

 public:
  Motor *gm_yaw_, *gm_pitch_;  // 电机指针
  IMU* imu_;                   // imu指针

  uint32_t now_tick;

  // 反馈发布者
  Publisher_t* gimbal_pub_;
  // 命令接收者
  Subscriber_t* gimbal_sub_;
  Subscriber_t* chassis_sub_;
  // 指令存放位置
  GimbalCtrlCmd gimbal_cmd_rcv_;
  GimbalFdbData gimbal_fdb_send;
  ChassisCtrlCmd chassis_cmd_rcv_;

  // 目标状态数据
  GimbalStatus_t ref_;
  // 反馈状态数据
  GimbalStatus_t fdb_;
  // 控制模式
  GimbalFdbMode_e mode_;
  // pitch轴补偿
  float pitch_compensate_;
  // 角度限位(deg)
  struct GimbalAngleLimit_t {
    float yaw_min;
    float yaw_max;
    float pitch_min;
    float pitch_max;
  } limit_;

  // 上电按编码器初始化相关参数
  struct InitStatus_t {
    bool yaw_finish;
    bool pitch_finish;
  } init_status_;

  uint32_t first_init_tick;
  bool if_first_init = false;
  // 陀螺前馈系数
  float yaw_chassis_feedforward = 0;
};

#endif  // RM_FRAME_GIMBAL_H
