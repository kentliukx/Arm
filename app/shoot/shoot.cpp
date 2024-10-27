//
// Created by 98383 on 24-10-26.
//

#include "shoot.h"

const float stir_step_angle = 45;
const float heat_17mm_bullet = 10;

const float stir_block_current = 8000;
const float stir_block_angle = 8 * stir_step_angle;

// 构造函数 参数在使用时务必调整
Shoot::Shoot(Motor* fric_l, Motor* fric_r, Motor* stir)
    : fric_l_(fric_l),
      fric_r_(fric_r),
      stir_(stir),
      fric_state_(false),
      block_state_(false),
      heat_state_(true),
      speed_limit_(30),
      heat_limit_(200),
      cooling_rate_(10),
      cd_(40) {
  shoot_pub_ = PubRegister("shoot_fdb", sizeof(ShootFdbData));
  shoot_sub_ = SubRegister("shoot_cmd", sizeof(ShootCtrlCmd));
  referee_sub_ = SubRegister("referee_shoot", sizeof(RefereeShootFdb));
}

// 发射一发弹丸(发射-true，未发射-false)
bool Shoot::shootOneBullet(void) {
  if (shoot_state_) {
    stir_->targetAngle() -= stir_step_angle;
    last_tick_ = HAL_GetTick();
    calc_heat_ += heat_17mm_bullet;
  }
  return shoot_state_;
}

// 设置射击参数
void Shoot::setShootParam(const float& speed_limit, const float& heat_limit,
                          const float& cooling_rate) {
  speed_limit_ = speed_limit;
  heat_limit_ = heat_limit;
  cooling_rate_ = cooling_rate;
}

// 射击速度
float Shoot::getBulletSpeed(void) {
  if (referee_data_.if_connect && referee_data_.bullet_speed != 0) {
    bullet_speed_ = referee_data_.bullet_speed;
  } else {
    bullet_speed_ = speed_limit_ - 1.0f;  // todo弹速估计
  }
  return bullet_speed_;
}

// 射速处理
void Shoot::speedHandle(void) {
  if (!fric_state_) {
    // 摩擦轮关闭
    fric_l_->setSpeed(0);
    fric_r_->setSpeed(0);
  } else {
    // 摩擦轮开启
    // 根据射速上限计算摩擦轮转速(dps) todo: 自适应射速调整
    fric_speed_ = 1e3f * speed_limit_ + 1e4f - 1452;
    fric_l_->setSpeed(-(fric_speed_ + fric_speed_offset));
    fric_r_->setSpeed((fric_speed_ + fric_speed_offset) - 407);
  }
}