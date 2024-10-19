/**
 ******************************************************************************
 * @file    swerve_steering_motor.cpp/h
 * @brief   舵轮航向电机与光电门控制
 * @author  Tianzong Cheng
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef RM_SWERVE_INFANTRY_2023_APP_SWERVE_STEERING_MOTOR_H_
#define RM_SWERVE_INFANTRY_2023_APP_SWERVE_STEERING_MOTOR_H_

#include "main.h"
#include "motor.h"

class SwerveSteering {
 public:
  /**
   * @brief M3508 航向电机构造函数
   * @param steering_motor 电机指针
   * @param port 光电门 GPIO 端口
   * @param pin  光电门 GPIO 针脚
   * @param reset_offset 航向电机零点偏移量
   */
  SwerveSteering(Motor* steering_motor, GPIO_TypeDef* port, uint16_t pin,
                 float reset_offset);

  /**
   * @brief GM6020 航向电机构造函数
   * @param steering_motor 电机指针
   * @param reset_offset 航向电机零点偏移量
   */
  SwerveSteering(Motor* steering_motor, float reset_offset);

  /**
   * @brief 读取光电门状态
   */
  void read_photogate_state();

  /**
   * @return M3508 航向电机是否复位完成
   */
  inline bool if_reset_done() { return !(!reset_stage_1_ || !reset_stage_2_); }

  /**
   * @brief 复位航向电机
   */
  void reset_steering_motor();

  /**
   * @brief 如果航向电机还未复位，进行光电门复位
   */
  void reset_steering_motor_handle();

  /**
   * @brief 计算航向电机最短路径
   * @return 最近目标角度
   */
  float shortest_steering_path(float);

  /**
   * @return 航向电机是否反向
   */
  inline float get_reverse_speed() { return reverse_speed_; }

  Motor* motor_;

 private:
  /**
   * @brief 获取光电门状态
   * @return 光电门状态，1 表示通，0 表示断
   */
  inline bool get_photogate_state() { return photogate_state != GPIO_PIN_SET; }

  float reverse_speed_ = 1.0f;  // 航向电机是否反向，1 表示正，-1 表示反

  float reset_offset_;  // 航向电机零点偏移量

  GPIO_PinState photogate_state;  // 光电门状态，1 表示通，0 表示断
  GPIO_TypeDef* port_;
  uint16_t pin_;

  // M3508 航向电机复位状态
  bool reset_stage_1_ = false;
  bool reset_stage_2_ = false;
};

#endif  // RM_SWERVE_INFANTRY_2023_APP_SWERVE_STEERING_MOTOR_H_
