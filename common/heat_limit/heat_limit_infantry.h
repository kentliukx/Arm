//
// Created by 98383 on 24-10-27.
//

#ifndef RM_FRAME_HEAT_LIMIT_INFANTRY_H
#define RM_FRAME_HEAT_LIMIT_INFANTRY_H

#include "algorithm/math/math.h"
#include "common/communication/referee_comm/referee_comm.h"

class HeatLimitInfantry {
 public:
  HeatLimitInfantry(float heat_limit, float cooling_rate);
  float heat_limit_;
  float calc_heat_;
  bool heat_state_;
  float cooling_rate_;
  const float heat_17mm_bullet = 10;
  void HeatHandle(void);
};

#endif  // RM_FRAME_HEAT_LIMIT_INFANTRY_H
