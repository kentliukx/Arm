//
// Created by 98383 on 24-10-15.
//

#ifndef RM_FRAME_SWERVECHASSIS_H
#define RM_FRAME_SWERVECHASSIS_H

#include "algorithm/math/math.h"
#include "algorithm/pid/pid.h"
#include "chassis.h"
#include "cmath"
#include "common/message_center/message_center.h"
#include "common/message_center/msg_def.h"

#define PI 3.14159265358979f

typedef struct SwerveStatus {
  struct WheelSpeed {
    float fl;
    float fr;
    float bl;
    float br;
  } wheel_speed;
  struct SteeringAngle {
    float fl = 0;
    float fr = 0;
    float bl = 0;
    float br = 0;
  } steering_angle;
} SwerveStatus_t;

class SwerveChassis : public Chassis {
 private:
  float wheel_radius;  // 轮半径
  float wheel_base;    // 轴距
  SwerveStatus_t fdb_chassis_;
  SwerveStatus_t fdb_robot_;
  SwerveStatus_t ref_chassis_;
  SwerveStatus_t ref_robot_;

  PID angle_pid_;

  // 控制 x 轴加速度滤波器
  LowPassFilter vx_filter_;
  // 控制 y 轴加速度滤波器
  LowPassFilter vy_filter_;

  // 底盘跟随低通滤波器
  LowPassFilter follow_filter_;

  //    inline bool motor_connection_check() {
  //        return CMFL_->connect_.check() && CMFR_->connect_.check() &&
  //               CMBL_->connect_.check() && CMBR_->connect_.check() &&
  //               STFL_.motor_->connect_.check() &&
  //               STFR_.motor_->connect_.check() &&
  //               STBL_.motor_->connect_.check() &&
  //               STBR_.motor_->connect_.check();
  //    }

  // 底盘反馈发布者
  Publisher_t* chassis_pub_;
  // 底盘命令接收者
  Subscriber_t* chassis_sub_;

  // 逆运动学，底盘状态->轮速
  void ikine(void) override;

  // 正运动学，轮速->底盘状态
  void fkine(void) override;

  // 底盘相关处理
  void handle(void) override;

  // 底盘旋转控制
  void RotateControl(float fdb_angle, float follow_fdb_angle) override;

  // 坐标系转换
  void CoordinateTransformation(void);
};

#endif  // RM_FRAME_SWERVECHASSIS_H
