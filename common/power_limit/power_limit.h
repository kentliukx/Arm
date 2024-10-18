/**
******************************************************************************
* @file    power_limit.cpp/h
* @brief   Power limitation. 功率限制
* @author  Likai Fu
******************************************************************************
* Copyright (c) 2025 Team JiaoLong-SJTU
* All rights reserved.
******************************************************************************
*/

#ifndef POWER_LIMIT_H
#define POWER_LIMIT_H

#include "algorithm/filter/filter.h"
#include "app/chassis/MecanumChassis.h"
#include "app/chassis/SwerveChassis.h"
#include "app/chassis/chassis.h"
#include "base/motor/motor.h"
#include "common/cap_comm/cap_comm.h"
#include "common/referee_comm/referee_comm.h"

namespace powerlimit {
// 电机功率计算
float motorPower(const Motor::Type_e& type, const float& i, const float& w);
}  // namespace powerlimit

class Power {
 public:
  // 电机功率
  typedef struct MotorPower {
    float fl, fr, bl, br;
  } MotorPower_t;

  // 功率限制初始化
  virtual void init(void);
  // 功率限制处理
  virtual void handle(float extra_power_max = 0);

 public:
  // 功率限制方程求解，返回求解功率限制方程得到的实际功率限制
  virtual void solve(const float& p_ref);

 public:
  // 程序设置功率上限
  float ref_power_limit_;

  // 电机功率反馈（估计值）
  MotorPower_t motor_power_fdb_;
  // 底盘功率反馈（估计值）
  float chassis_power_fdb_;

  // 电机预估功率
  MotorPower_t motor_power_est_;
  // 底盘预估功率
  float chassis_power_est_;

  struct PowerLimitEquation {
    float kp, rm;   // kp-速度环，rm-减速比
    float k[4];     // 电机功率相关常数
    float a, b, c;  // 方程系数
    float delta;    // Δ
    float r;        // 速度比例
  } eq_;
};

class SwerveChassisPower : public Power {
 public:
  SwerveChassisPower(SwerveChassis* chassis, RefereeComm* referee,
                     CapComm* cap);

  // 功率限制初始化
  void init(void) override;
  // 功率限制处理
  void handle(float extra_power_max = 0) override;

 private:
  SwerveChassis* chassis_;
  RefereeComm* referee_;
  CapComm* ultra_cap_;

  LowPassFilter chassis_power_fdb_filter_;

  // 减速比例
  float speed_rate_;
  LowPassFilter speed_rate_filter_;

 protected:
  // 功率限制方程求解，返回求解功率限制方程得到的实际功率限制
  void solve(const float& p_ref) override;
};

class WheelLeggedChassisPower : Power {
 public:
  WheelLeggedChassis* chassis_;

  // reference_power_limit = referee_limit + cap + buffer
  float cap_volt_;             // capacity voltage
  float referee_power_limit_;  // power limit read from referee
  float referee_buffer_;       // buffer read from referee
  float extra_power_limit_;    // capacity power
  float ref_power_limit_;      // total reference power

  // power feedback (estimated)
  float motor_power_fdb_l_;  // left wheel motor feedback
  float motor_power_fdb_r_;  // right wheel motor feedback
  float chassis_power_fdb_;  // total feedback

  const float static_power_ = 13;

  float velocity_max_;
  float yaw_velocity_max_;

  LowPassFilter chassis_power_fdb_filter_;  // filter of feedback power

 public:
  float ref_power_limit() const { return ref_power_limit_; }
  float power_fdb() const { return chassis_power_fdb_; }
  float velocity_max() const { return velocity_max_; }
  float yaw_velocity_max() const { return yaw_velocity_max_; }
};

#endif  // POWER_LIMIT_H