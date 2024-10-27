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

// 裁判系统->视觉通信数据
typedef struct {
  uint8_t robot_id;
  float game_robot_pos_x;
  float game_robot_pos_y;
} RefereeCVData;

// 云台->视觉通信数据
typedef struct {
  float yaw_offset;    // 电控yaw补偿
  float pitch_offset;  // 电控pitch补偿
} GimbalUploadData;

// 视觉->云台通信数据
typedef struct {
  float yaw_angle;    // deg
  float pitch_angle;  // deg
  float yaw_speed;    // dps
  float pitch_speed;  // dps
  float dist;         // m，视觉测量距离
} GimbalCtrlCmd;

// 视觉->发射通信数据
typedef struct {
  uint8_t shoot_flag;  // 发射标志
  uint32_t shoot_id;   // 发射id
  uint8_t enemy_id;    // 敌方id
} ShootCtrlCmdCV;

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

typedef struct {
  float bullet_speed;
  float cooling_heat;
  uint8_t if_connect;
  float cooling_limit;
  float cooling_rate;
  uint8_t new_bullet;
} RefereeShootFdb;

#endif  // RM_FRAME_MSG_DEF_H
