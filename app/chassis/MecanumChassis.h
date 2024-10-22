//
// Created by 98383 on 24-10-10.
//

#ifndef RM_FRAME_MECANUMCHASSIS_H
#define RM_FRAME_MECANUMCHASSIS_H

#include "algorithm/math/math.h"
#include "algorithm/pid/pid.h"
#include "base/motor/motor.h"
#include "chassis.h"
#include "cmath"
#include "common/log/bsp_log.h"
#include "common/message_center/message_center.h"
#include "common/message_center/msg_def.h"
#include "stm32f4xx_hal.h"

struct WheelSpeed {
  float fl;
  float fr;
  float bl;
  float br;
};

class MecanumChassis : public Chassis {
 private:
  // 轮速返回值
  WheelSpeed wheel_fdb_;
  // 轮速目标值
  WheelSpeed wheel_ref_;
  // 底盘跟随PID
  PID angle_pid_;
  // 底盘反馈发布者
  Publisher_t* chassis_pub_;
  // 底盘命令接收者
  Subscriber_t* chassis_sub_;
  // 底盘接收数据指针
  ChassisCtrlCmd chassis_cmd_rcv_;
  // 底盘反馈数据指针
  ChassisCtrlCmd chassis_cmd_tsm_;

  float x_bias;            // x方向偏移 (底盘前方为正方向) (m)
  float y_bias;            // y方向偏移 (底盘左方为正方向) (m)
  float wheel_radius;      // 轮半径 (m)
  float half_track_width;  // 1/2轴距 (m)
  float half_wheel_base;   // 1/2轮距 (m)
 public:
  // 构造函数
  MecanumChassis(Motor* cmfl, Motor* cmfr, Motor* cmbl, Motor* cmbr,
                 PID angle_pid, LowPassFilter speed_filter);
  // 底盘角速度前馈
  float feedforward_wz;
  // 电机指针
  Motor *cmfl_, *cmfr_, *cmbl_, *cmbr_;

  // 目标速度滤波(逐渐加速)
  LowPassFilter vx_filter_;
  LowPassFilter vy_filter_;

  // 逆运动学，底盘状态->轮速
  void ikine(void) override;

  // 正运动学，轮速->底盘状态
  void fkine(void) override;

  // 底盘相关处理
  void handle(void) override;
};

#endif  // RM_FRAME_MECANUMCHASSIS_H
