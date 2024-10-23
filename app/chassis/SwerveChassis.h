//
// Created by 98383 on 24-10-15.
//

#ifndef RM_FRAME_SWERVECHASSIS_H
#define RM_FRAME_SWERVECHASSIS_H

#include "algorithm/math/math.h"
#include "algorithm/pid/pid.h"
#include "base/motor/motor.h"
#include "base/motor/swerve_steering_motor.h"
#include "chassis.h"
#include "cmath"
#include "common/log/bsp_log.h"
#include "common/message_center/message_center.h"
#include "common/message_center/msg_def.h"
#include "common/power_limit/power_limit.h"

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

const struct {
  float FL = -120.0f;  // 239  284
  float FR = 119.0f;   // 119.48  75.98
  float BL = -136.5f;  // 224  180 223.6 179.2
  float BR = 0.0f;     // 359.6 43.9
} STEERING_OFFSET;

class SwerveChassis : public Chassis {
 public:
  Motor *CMFL_, *CMFR_, *CMBL_, *CMBR_;
  SwerveSteering STFL_, STFR_, STBL_, STBR_;

  Motor::Type_e type_;

  SwerveChassis(Motor* CMFL, Motor* CMFR, Motor* CMBL, Motor* CMBR, Motor* STFL,
                Motor* STFR, Motor* STBL, Motor* STBR,
                LowPassFilter speed_filter, PID angle_pid);

 private:
  const float wheel_radius = 0.06f;  // 轮半径
  const float wheel_base = 0.36f;    // 轴距
  bool chassis_lock_;
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

  inline bool motor_connection_check() {
    return CMFL_->connect_.check() && CMFR_->connect_.check() &&
           CMBL_->connect_.check() && CMBR_->connect_.check() &&
           STFL_.motor_->connect_.check() && STFR_.motor_->connect_.check() &&
           STBL_.motor_->connect_.check() && STBR_.motor_->connect_.check();
  }

  // 底盘反馈发布者
  Publisher_t* chassis_pub_;
  // 底盘命令接收者
  Subscriber_t* chassis_sub_;
  // 指令存放位置
  ChassisCtrlCmd chassis_cmd_rcv_;

 public:
  // 构造函数
  SwerveChassis();

  // 逆运动学，底盘状态->轮速
  void ikine(void) override;

  // 正运动学，轮速->底盘状态
  void fkine(void) override;

  // 底盘相关处理
  void handle(void) override;

  // 坐标系转换
  void CoordinateTransformation(void);

  // 电机反馈处理
  void MotorFeedbackUpdate(void);

  // 电机控制
  void MotorControl(void);

  // 舵控制
  void SteeringHandle(void);

  // 航向电机断联处理
  void DisconnectHandle(void);

  friend class SwerveChassisPower;
};

#endif  // RM_FRAME_SWERVECHASSIS_H
