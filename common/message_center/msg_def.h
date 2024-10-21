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

  float fdb_angle, follow_fdb_angle;  // 底盘反馈角度和跟随角度
  float chassis_power_limit, extra_power_max;  // 功率限制参数
  ChassisMode_e mode_;                         // 底盘模式
} ChassisCtrlCmd;

typedef struct {
  float yaw_offset;    // 电控yaw补偿
  float pitch_offset;  // 电控pitch补偿
} GimbalUploadData;

typedef struct {
  float yaw_angle;    // deg
  float pitch_angle;  // deg
  float yaw_speed;    // dps
  float pitch_speed;  // dps
  float dist;         // m，视觉测量距离
} GimbalCtrlCmd;

typedef struct {
  uint8_t shoot_flag;  // 发射标志
  uint32_t shoot_id;   // 发射id
  uint8_t enemy_id;    // 敌方id
} ShootCtrlCmd;

typedef struct {
  float shoot_speed;      // m/s，射速
  uint8_t shoot_delay;    // ms 发射延迟
  uint32_t shoot_id_fdb;  // 发射id反馈(=对应的shoot_id)
  uint8_t is_big_energy;  // 0-小符，1-大符
} ShootUploadData;

typedef struct RxMsg {
  uint8_t cap_state;
  uint16_t cap_voltage;
  float chassis_power;
} CapUploadData;

#endif  // RM_FRAME_MSG_DEF_H
