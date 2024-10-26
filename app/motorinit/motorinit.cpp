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
    if ((motor_->motor_data_.torque >= motor_->info_.max_torque * 0.8f)) {
      motor_->init_state_ = Motor::Block;
      motor_->init_tick_ = HAL_GetTick();
    } else {
      motor_->setAngle(motor_->targetAngle() - 0.1f);
    }
  }
  // 堵转计时
  else if (motor_->init_state_ == Motor::Block) {
    if (motor_->motor_data_.torque < motor_->info_.max_torque * 0.8f) {
      motor_->init_state_ = Motor::ReverseRotation;
      motor_->setAngle(motor_->realAngle());
    }
    if (HAL_GetTick() - motor_->init_tick_ >= 300) {
      motor_->init_state_ = Motor::Ready;
    }
  }
  // 结束,这里为了不引入新的状态量更希望在调用这个函数的地方重置init_state_为Prepare
  else if (motor_->init_state_ == Motor::Ready) {
    motor_->init_state_ = Motor::Prepare;
  }
}