//
// Created by 98383 on 24-10-26.
//

#include "shoot.h"

const float stir_step_angle = 45;
const float heat_17mm_bullet = 10;

const float stir_block_torque = 8000;
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
bool Shoot::ShootOneBullet(void) {
  if (shoot_state_) {
    stir_->targetAngle() -= stir_step_angle;
    last_tick_ = HAL_GetTick();
    calc_heat_ += heat_17mm_bullet;
  }
  return shoot_state_;
}

// 设置射击参数
void Shoot::SetShootParam(const float& speed_limit, const float& heat_limit,
                          const float& cooling_rate) {
  speed_limit_ = speed_limit;
  heat_limit_ = heat_limit;
  cooling_rate_ = cooling_rate;
}

// 射击速度
float Shoot::GetBulletSpeed(void) {
  if (referee_data_.if_connect && referee_data_.bullet_speed != 0) {
    bullet_speed_ = referee_data_.bullet_speed;
  } else {
    bullet_speed_ = speed_limit_ - 1.0f;  // todo弹速估计
  }
  return bullet_speed_;
}

// 射速处理
void Shoot::SpeedHandle(void) {
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

// 卡弹处理
void Shoot::BlockHandle(void) {
  if (!block_state_) {
    if (stir_->motor_data_.torque * (stir_->ratio_ > 0 ? 1.0f : -1.0f) <
            -stir_block_torque &&
        stir_->targetAngle() - stir_->realAngle() < -stir_block_angle) {
      // 检测到卡弹
      block_state_ = true;
      block_tick_ = HAL_GetTick();
      stir_->targetAngle() =
          math::loopLimit(stir_->targetAngle(), stir_->realAngle(),
                          stir_->realAngle() + stir_step_angle);
    }
  } else if (HAL_GetTick() - block_tick_ > 100) {
    // 从卡弹状态下恢复
    block_state_ = false;
  }
}

// 热量处理(裁判系统通信热量实时性不够，需自行计算保证不超热量)
void Shoot::HeatHandle(void) {
#ifndef REFEREE_DELAY
  if (referee_data_.if_connect && HAL_GetTick() - last_tick_ > 500) {
    // 距离上次发射大于500ms，用裁判系统数据更新计算热量
    // 当裁判系统有延迟时读数可能偏高
    calc_heat_ = referee_data_.cooling_heat;
  }
#endif
  calc_heat_ = math::limitMin(calc_heat_ - cooling_rate_ * 1e-3f, 0);
  // 判断下一发是否会超热量
  if (heat_limit_ - calc_heat_ > heat_17mm_bullet + 10.0f) {
    heat_state_ = true;
  } else {
    heat_state_ = false;
  }
}

void Shoot::BulletSpeedControl() {
  if (GetBulletSpeed() > speed_limit_ - 8.0f &&
      GetBulletSpeed() < speed_limit_ + 8.0f) {
    speed_record[record_count++] = GetBulletSpeed();
  }
  if (record_count == 5) {
    float average_speed = (speed_record[0] + speed_record[1] + speed_record[2] +
                           speed_record[3] + speed_record[4]) /
                          5.0f;
    if (average_speed > adaptation_target) {
      fric_speed_offset += math::limit(
          decelerate_kp * (adaptation_target - average_speed), -800, 0);
    } else {
      fric_speed_offset += math::limit(
          accelerate_kp * (adaptation_target - average_speed), 0, 800);
    }
    record_count = 0;
  }
  fric_speed_offset = math::limit(fric_speed_offset, -3000.0f, 3000.0f);
}

// 发射数据处理
void Shoot::handle(void) {
  // 接收数据
  SubGetMessage(shoot_sub_, &shoot_cmd_rcv_);
  SubGetMessage(referee_sub_, &referee_data_);

  // 速度上限变化重置自适应弹速
  static float last_limit = speed_limit_;
  if (fabs(speed_limit_ - last_limit) > 0.1f) {
    ResetSpeedControl();
  }
  last_limit = speed_limit_;

  // 检测到发弹，进行弹速控制
  if (new_bullet_) {
    new_bullet_ = false;
    BulletSpeedControl();
  }

  if (referee_data_.if_connect) {
    // 连接裁判系统(未连接裁判系统可在control中调用setShootParam设置不同模式的射击参数)
    SetShootParam(30, referee_data_.cooling_limit, referee_data_.cooling_rate);
  }
  SpeedHandle();
  BlockHandle();
  HeatHandle();
  // 判断是否可发射(摩擦轮开 & 未卡弹 & 下一发不会超热量 & 发射未cd)
  shoot_state_ = fric_state_ && !block_state_ && heat_state_ &&
                 HAL_GetTick() - last_tick_ > cd_;

  // 发弹
  if (shoot_cmd_rcv_.shoot_one_bullet) {
    ShootOneBullet();
  }
}
