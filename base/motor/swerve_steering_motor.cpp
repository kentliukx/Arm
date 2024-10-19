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

#include "swerve_steering_motor.h"

#include "algorithm/math/math.h"
#include "main.h"

void SwerveSteering::read_photogate_state() {
  photogate_state = HAL_GPIO_ReadPin(port_, pin_);
}

SwerveSteering::SwerveSteering(Motor* steering_motor, GPIO_TypeDef* port,
                               uint16_t pin, float reset_offset)
    : motor_(steering_motor),
      port_(port),
      pin_(pin),
      reset_offset_(reset_offset) {}

SwerveSteering::SwerveSteering(Motor* steering_motor, float reset_offset)
    : motor_(steering_motor), reset_offset_(reset_offset) {}

void SwerveSteering::reset_steering_motor_handle() {
  if (if_reset_done()) {
    motor_->method_ = Motor::POSITION_SPEED;
    return;
  }
  motor_->method_ = Motor::SPEED;
  motor_->setSpeed(500);
  if (!reset_stage_1_ && !get_photogate_state()) {
    reset_stage_1_ = true;
  }
  if (reset_stage_1_ && get_photogate_state()) {
    reset_stage_2_ = true;
    motor_->resetFeedbackAngle(reset_offset_);
  }
}

float SwerveSteering::shortest_steering_path(float target_angle) {
  target_angle = math::degNormalize180(target_angle);
  float& current_angle = motor_->control_data_.fdb_angle;
  float target_angle_use = target_angle;
  if (target_angle > current_angle) {
    if (target_angle - current_angle < 90.0f) {
      target_angle_use = target_angle;
      reverse_speed_ = 1.0f;
    } else if (target_angle - current_angle >= 90.0f &&
               target_angle - current_angle < 270.0f) {
      target_angle_use = target_angle - 180.0f;
      reverse_speed_ = -1.0f;  // 因为舵反向，所以轮电机速度需要反向
    } else if (target_angle - current_angle >= 270.0f &&
               target_angle - current_angle <= 360.0f) {
      target_angle_use = target_angle - 360.0f;
      reverse_speed_ = 1.0f;
    }
  } else {
    if (current_angle - target_angle < 90.0f) {
      target_angle_use = target_angle;
      reverse_speed_ = 1.0f;
    } else if (current_angle - target_angle >= 90.0f &&
               current_angle - target_angle < 270.0f) {
      target_angle_use = target_angle + 180.0f;
      reverse_speed_ = -1.0f;
    } else if (current_angle - target_angle >= 270.0f &&
               current_angle - target_angle <= 360.0f) {
      target_angle_use = target_angle + 360.0f;
      reverse_speed_ = 1.0f;
    }
  }
  return target_angle_use;
}

void SwerveSteering::reset_steering_motor() {
  if (motor_->info_.type == Motor::M3508) {
    reset_stage_1_ = false;
    reset_stage_2_ = false;
  } else if (motor_->info_.type == Motor::GM6020) {
    motor_->resetFeedbackAngle((motor_->motor_data_.ecd_angle - reset_offset_) /
                               motor_->ratio_);
  }
}