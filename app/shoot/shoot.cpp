//
// Created by 98383 on 24-10-26.
//

#include "shoot.h"

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
}
