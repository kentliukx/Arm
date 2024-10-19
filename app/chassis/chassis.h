//
// Created by 98383 on 24-10-8.
//

#ifndef CHASSIS25_CHASSIS_H
#define CHASSIS25_CHASSIS_H

#include <cstdint>

typedef struct ChassisStatus {
  float vx = 0;     // x方向速度(m/s)
  float vy = 0;     // y方向速度(m/s)
  float wz = 0;     // 旋转角速度(rad/s)
  float angle = 0;  // 底盘角度(rad)(yaw电机角度-相对，陀螺仪角度-绝对)
} ChassisStatus_t;

enum ChassisMode_e {
  Separate,  // 云台底盘分离
  Follow,    // 底盘跟随云台
  Lock,      // 锁定底盘
};

class Chassis {
 protected:
  // 目标状态(机器人坐标系)
  ChassisStatus_t ref_spd;
  // 反馈状态(机器人坐标系)
  ChassisStatus_t fdb_spd;
  // 目标状态(底盘坐标系)
  ChassisStatus_t chassis_ref_spd_;
  // 反馈状态(底盘坐标系)
  ChassisStatus_t chassis_fdb_spd_;

  uint8_t chassis_lock = 0;  // 0锁定底盘, 1解锁
 public:
  ChassisMode_e mode_;

  // 逆运动学，底盘状态->轮速
  virtual void ikine(void);

  // 正运动学，轮速->底盘状态
  virtual void fkine(void);

  // 底盘相关处理
  virtual void handle(void);

 protected:
  // 设置速度
  void SetVx(float vx_);
  void SetVy(float vy_);
  void SetWz(float wz_);
  void SetAngle(float angle_);

  void SetSpeed(float vx_, float vy_, float wz_);
};

#endif  // CHASSIS25_CHASSIS_H
