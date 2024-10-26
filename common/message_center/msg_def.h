//
// Created by 98383 on 24-10-15.
//

#ifndef RM_FRAME_MSG_DEF_H
#define RM_FRAME_MSG_DEF_H

#include "app/chassis/chassis.h"

typedef struct {
  float vx;         // x向速度
  float vy;         // y向速度
  float wz;         // 旋转角
  float ref_angle;  // 底盘角度

  float fdb_angle, follow_fdb_angle;           // 底盘反馈角度和跟随角度
  float chassis_power_limit, extra_power_max;  // 功率限制参数
  ChassisMode_e mode_;                         // 底盘模式
} ChassisCtrlCmd;

typedef struct {
  uint8_t shoot_one_bullet;  // 0表示不发射，1表示发射
  uint32_t cmd_tick;         // 命令时间戳
} ShootCtrlCmd;

typedef struct {
  uint8_t shoot_state;  // 0表示未发射，1表示已发射
  float bullet_speed;   // 弹速
} ShootFdbData;

#endif  // RM_FRAME_MSG_DEF_H
