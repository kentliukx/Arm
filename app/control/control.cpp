/**
******************************************************************************
* @file    control.cpp/h
* @brief   全车主要控制逻辑，以及主控数据收发点
* @author  Likai Fu
******************************************************************************
* Copyright (c) 2025 Team JiaoLong-SJTU
* All rights reserved.
******************************************************************************
*/

#include "control.h"

#include "algorithm/filter/filter.h"
#include "app/chassis/MecanumChassis.h"
#include "app/chassis/SwerveChassis.h"
#include "app/gimbal/gimbal.h"
#include "app/motorinit/motorinit.h"
#include "app/shoot/shoot.h"
#include "base/monitor/imu_monitor.h"
#include "base/monitor/motor_monitor.h"
#include "base/motor/motor.h"
#include "hardware_config.h"

// 机器人组件定义
// 底盘
#ifdef mecanum_chassis
MecanumChassis chassis(&CMFL, &CMFR, &CMBL, &CMBR,
                       PID(7 / 58.f, 0, 15 / 58.f, 100 / 58.f, 270 / 58.f),
                       LowPassFilter(5e-3f));
#elif defined swerve_chassis
SwerveChassis chassis(&CMFL, &CMFR, &CMBL, &CMBR, &STFL, &STFR, &STBL, &STBR,
                      LowPassFilter(2e-2f), PID(-1.2, 0, -1, 0, 100));
#endif

#ifdef infantry_shoot
Shoot shoot(&FRICL, &FRICR, &STIR);
#endif

Gimbal gimbal(&GMY, &GMP, &board_imu);

// 全局变量声明
extern RC rc;

// msg接收、发送mailbox
ChassisCtrlCmd chassis_ctrl_ref_, chassis_ctrl_fdb_;
ShootCtrlCmd shoot_ctrl_ref_;
ShootFdbData shoot_ctrl_fdb_;
GimbalCtrlCmd gimbal_ctrl_ref_;
GimbalFdbData gimbal_ctrl_fdb_;

// msg的订阅者与发布者
Subscriber_t *chassis_fdb_sub_, *shoot_fdb_sub_, *gimbal_fdb_sub_;
Publisher_t *chassis_cmd_pub_, *shoot_cmd_pub_, *gimbal_cmd_pub_;

// 函数声明
void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void robotCmdSend(void);
void robotCmdInit(void);
void ModuleControl(void);

// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;

// 更多的底盘状态(用于UI等)
enum ChassisStateExt_e {
  LOCK,
  FOLLOW,
  SEPARATE,
  TWIST,
  GYRO,
  GYROCHANGE,
  RAW,
} chassis_state;

// 底盘旋转方向
float chassis_gyro_dir = 1;

// 遥控器相关参数
namespace rcctrl {
float arm_position_rate = 2e-4f;
float arm_direction_rate = 3e-6f;
float arm_joint_rate = 1.5e-6f;

float chassis_speed_rate = 2.376f;
float chassis_rotate_rate = 0.72f;
float chassis_follow_ff_rate = 0.3f;
float chassis_rand_gyro_rate = 20.f;

float gimbal_rate = 0.165f;
}  // namespace rcctrl

// 初始化标志
bool startup_flag = false;
bool last_gimbal_connect;
bool gimbal_first_enter_working = true;
// 遥控器挡位记录
RC::RCSwitch last_rc_switch;
RC::RCChannel last_rc_channel;

void controlInit(void) {
  robot_state = STOP;
  chassis_ctrl_ref_.mode_ = ChassisMode_e::Lock;
  chassis_state = ChassisStateExt_e::LOCK;
}

// 控制主循环
void controlLoop(void) {
  iwdgHandler(rc.connect_.check());
  robotPowerStateFSM(!rc.connect_.check() || rc.switch_.r == RC::DOWN);

  if (!last_gimbal_connect && GMY.connect_.check() ||
      last_rc_switch.r == RC::DOWN && rc.switch_.r != RC::DOWN) {
    gimbal_ctrl_ref_.init_flag = 1;
  } else {
    gimbal_ctrl_ref_.init_flag = 0;
  }
  last_gimbal_connect = GMY.connect_.check();

  if (robot_state == STOP) {
    gimbal_ctrl_ref_.if_robot_power_on = 0;
    allMotorsStopShutoff();
    robotReset();
  } else if (robot_state == STARTUP) {
    allMotorsOn();                  // 电机上电
    startup_flag = robotStartup();  // 开机状态判断
  } else if (robot_state == WORKING) {
    gimbal_ctrl_ref_.if_robot_power_on = 1;
    allMotorsOn();   // 电机上电
    robotControl();  // 机器人控制
  }

  // 记录遥控器挡位状态
  last_rc_switch = rc.switch_;
  last_rc_channel = rc.channel_;
}

// IWDG处理，true持续刷新，false进入STOP状态并停止刷新
void iwdgHandler(bool iwdg_refresh_flag) {
  if (!iwdg_refresh_flag) {
    robot_state = STOP;
  } else {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

// STOP(断电，安全模式)/开机/正常运行 状态机
void robotPowerStateFSM(bool stop_flag) {
  if (robot_state == STOP) {
    if (!stop_flag) {
      robot_state = STARTUP;
    }
  } else if (robot_state == STARTUP) {
    if (stop_flag) {
      robot_state = STOP;
    } else if (startup_flag) {
      // 初始化/复位完成
      robot_state = WORKING;
    }
  } else if (robot_state == WORKING) {
    if (stop_flag) {
      robot_state = STOP;
    }
  }

  if (gimbal_first_enter_working) {
    gimbal_first_enter_working = false;
    gimbal_ctrl_ref_.first_enter_flag = 1;
  } else {
    gimbal_ctrl_ref_.first_enter_flag = 0;
  }
}

// 重置各功能状态
void robotReset(void) {
  startup_flag = false;
  chassis_state = ChassisStateExt_e::LOCK;
  chassis_ctrl_ref_.mode_ = ChassisMode_e::Lock;
  shoot_ctrl_ref_.fric_state = 1;
  shoot_ctrl_ref_.stir_reset = 1;
  gimbal_ctrl_ref_.mode = 0;
  gimbal_ctrl_ref_.set_init_false = 0;
}

// 开机上电启动处理
bool robotStartup(void) {
  bool flag = true;
  chassis_ctrl_ref_.mode_ = ChassisMode_e::Separate;
  chassis_state = ChassisStateExt_e::RAW;
  shoot_ctrl_ref_.fric_state = 0;
  shoot_ctrl_ref_.stir_reset = 0;
  if (STIR.init_state_ != Motor::Ready) {
    MotorInit(&STIR);
    //    flag = false;
  }
  if (!gimbal_ctrl_fdb_.yaw_init_finish) {
    chassis_state = LOCK;
    chassis_ctrl_ref_.mode_ = Lock;
    flag = false;
  } else {
    chassis_ctrl_ref_.mode_ = Follow;
    chassis_state = FOLLOW;
  }
  if (!gimbal_ctrl_fdb_.pitch_init_finish) {
    flag = false;
  }
  if (HAL_GetTick() < 1000) {
    flag = false;
  }
  return flag;
}

void robotCmdInit(void) {
  chassis_cmd_pub_ = PubRegister("chassis_cmd", sizeof(ChassisCtrlCmd));
  chassis_fdb_sub_ = SubRegister("chassis_fdb", sizeof(ChassisCtrlCmd));
  shoot_cmd_pub_ = PubRegister("shoot_cmd", sizeof(ShootCtrlCmd));
  shoot_fdb_sub_ = SubRegister("shoot_fdb", sizeof(ShootFdbData));
  gimbal_cmd_pub_ = PubRegister("gimbal_cmd", sizeof(GimbalCtrlCmd));
  gimbal_fdb_sub_ = SubRegister("gimbal_fdb", sizeof(GimbalFdbData));
}

void robotCmdSend(void) {
#if defined(swerve_chassis) || defined(mecanum_chassis)
  SubGetMessage(chassis_fdb_sub_, &chassis_ctrl_fdb_);
  PubPushMessage(chassis_cmd_pub_, &chassis_ctrl_ref_);
#endif
#if defined(infantry_shoot)
  SubGetMessage(shoot_fdb_sub_, &shoot_ctrl_fdb_);
  PubPushMessage(shoot_cmd_pub_, &shoot_ctrl_ref_);
#endif
  SubGetMessage(gimbal_fdb_sub_, &gimbal_ctrl_fdb_);
  PubPushMessage(gimbal_cmd_pub_, &gimbal_ctrl_ref_);
}

void robotControl(void) {
  // 重置一些冲激信号
  shoot_ctrl_ref_.shoot_CD = 0;
  shoot_ctrl_ref_.fric_state = 0;
  gimbal_ctrl_ref_.set_init_false = 1;

  // 遥控器左上右上
  if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
    shoot_ctrl_ref_.fric_state = 2;
    shoot_ctrl_ref_.shoot_CD = 50;
    if (rc.channel_.l_col > 0.8 && !shoot_ctrl_ref_.shoot_one_bullet) {
      shoot_ctrl_ref_.shoot_one_bullet = true;
      shoot_ctrl_ref_.cmd_tick = HAL_GetTick();
    } else if (rc.channel_.l_col < 0.8) {
      shoot_ctrl_ref_.shoot_one_bullet = false;
    }
  }
  // 遥控器左上右中
  else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
    // 云台底盘测试
    shoot_ctrl_ref_.fric_state = 1;
    if (rc.channel_.dial_wheel < -0.3) {
      // 开陀螺
      if (rc.channel_.r_col > 0.5) {
        chassis_state = ChassisStateExt_e::TWIST;
        chassis_ctrl_ref_.mode_ = ChassisMode_e::Separate;
      } else {
        chassis_state = ChassisStateExt_e::GYRO;
        chassis_ctrl_ref_.mode_ = ChassisMode_e::Separate;
      }
    } else if (rc.channel_.dial_wheel > 0.3) {
      // 关陀螺
      chassis_state = ChassisStateExt_e::FOLLOW;
      chassis_ctrl_ref_.mode_ = ChassisMode_e::Follow;
    } else {
    }

    float vel_norm = sqrtf(rc.channel_.r_col * rc.channel_.r_col +
                           rc.channel_.r_row * rc.channel_.r_row);
    if (chassis_state == ChassisStateExt_e::FOLLOW) {
      chassis_ctrl_ref_.vx =
          -rc.channel_.r_col * rcctrl::chassis_speed_rate / vel_norm;
      chassis_ctrl_ref_.vy =
          rc.channel_.r_row * rcctrl::chassis_speed_rate / vel_norm;
      chassis_ctrl_ref_.wz = 0;
    } else if (chassis_state == ChassisStateExt_e::GYRO) {
      float dice = sinf(HAL_GetTick() / 100.f);
      float wz = 1.5;
      if (dice >= 0) wz = 3;
      chassis_ctrl_ref_.vx =
          -rc.channel_.r_col * rcctrl::chassis_speed_rate / vel_norm;
      chassis_ctrl_ref_.vy =
          rc.channel_.r_row * rcctrl::chassis_speed_rate / vel_norm;
      chassis_ctrl_ref_.wz = wz * chassis_gyro_dir;
    } else if (chassis_state == ChassisStateExt_e::TWIST) {
      float wz = 4.8 * sin(HAL_GetTick() * 6e-3f);
      chassis_ctrl_ref_.vx =
          -rc.channel_.r_col * rcctrl::chassis_speed_rate / vel_norm;
      chassis_ctrl_ref_.vy =
          rc.channel_.r_row * rcctrl::chassis_speed_rate / vel_norm;
      chassis_ctrl_ref_.wz = wz;
    } else if (chassis_state == ChassisStateExt_e::RAW) {
      chassis_ctrl_ref_.vx =
          -rc.channel_.r_col * rcctrl::chassis_speed_rate / vel_norm;
      chassis_ctrl_ref_.vy =
          rc.channel_.r_row * rcctrl::chassis_speed_rate / vel_norm;
      chassis_ctrl_ref_.wz = -rc.channel_.l_row * rcctrl::gimbal_rate;
    }

    gimbal_ctrl_ref_.add_yaw = -rc.channel_.l_row * rcctrl::gimbal_rate;
    gimbal_ctrl_ref_.add_pitch = -rc.channel_.l_col * rcctrl::gimbal_rate;
  }
}

void ChassisFollow(void) {
  chassis_ctrl_ref_.fdb_angle =
      gimbal_ctrl_fdb_.gimbal_yaw_encoder - gimbal_ctrl_fdb_.gimbal_yaw_zero;
  if (gimbal_ctrl_fdb_.yaw_init_finish) {
    chassis_ctrl_ref_.follow_fdb_angle = math::loopLimit(
        gimbal_ctrl_fdb_.gimbal_yaw_encoder - gimbal_ctrl_fdb_.gimbal_yaw_zero,
        -180, 180);
  } else {
    chassis_ctrl_ref_.follow_fdb_angle = chassis_ctrl_ref_.ref_angle;
  }
}

void ModuleControl(void) {
  ChassisFollow();
#if defined swerve_chassis || defined mecanum_chassis
  chassis.handle();
#endif
  gimbal.handle();
#if defined infantry_shoot
  shoot.handle();
#endif
}
