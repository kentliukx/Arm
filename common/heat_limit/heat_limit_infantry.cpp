//
// Created by 98383 on 24-10-27.
//

#include "heat_limit_infantry.h"

// 热量处理(裁判系统通信热量实时性不够，需自行计算保证不超热量)
void HeatLimitInfantry::HeatHandle(void) {
  calc_heat_ = math::limitMin(calc_heat_ - cooling_rate_ * 1e-3f, 0);
  // 判断下一发是否会超热量
  if (heat_limit_ - calc_heat_ > heat_17mm_bullet + 10.0f) {
    heat_state_ = true;
  } else {
    heat_state_ = false;
  }
}

HeatLimitInfantry::HeatLimitInfantry(float heat_limit, float cooling_rate)
    : heat_limit_(heat_limit),
      cooling_rate_(cooling_rate),
      heat_state_(true),
      calc_heat_(0) {}
