//
// Created by 98383 on 24-10-25.
//

#include "motorinit.h"

void MotorInit(Motor* motor_) {
  if (motor_->init_mode_ == Motor::None) {
    return;
  }

  // 校零，开始初始化
  if (motor_->init_state_ == Motor::Prepare) {
    motor_->setAngle(motor_->realAngle());
    motor_->init_state_ = Motor::ReverseRotation;
  }
  // 电机反转撞限位
  else if (motor_->init_state_ == Motor::ReverseRotation) {
    if ((fabs(motor_->motor_data_.torque) >= motor_->info_.max_torque * 0.8f)) {
      motor_->init_state_ = Motor::Block;
      motor_->init_tick_ = HAL_GetTick();
    } else {
      motor_->setAngle(motor_->targetAngle() -
                       0.1f * motor_->ratio_ / fabs(motor_->ratio_));
    }
  }
  // 堵转计时
  else if (motor_->init_state_ == Motor::Block) {
    if (fabs(motor_->motor_data_.torque) < motor_->info_.max_torque * 0.6f) {
      motor_->init_state_ = Motor::ReverseRotation;
      motor_->setAngle(motor_->realAngle());
    }
    if (HAL_GetTick() - motor_->init_tick_ >= 300) {
      motor_->resetFeedbackAngle(0);
      motor_->init_state_ = Motor::Init;
      motor_->setAngle(motor_->offset_);
    }
  } else if (motor_->init_state_ == Motor::Init) {
    if (fabs(motor_->targetAngle() - motor_->realAngle()) < 0.2f) {
      motor_->init_state_ = Motor::Ready;
    }
  }
}