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

// 全局变量声明
extern RC rc;

// msg接收、发送mailbox
ChassisCtrlCmd chassis_ctrl_ref_, chassis_ctrl_fdb_;

// msg的订阅者与发布者
Subscriber_t* chassis_fdb_sub_;
Publisher_t* chassis_cmd_pub_;

// 函数声明
void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void robotCmdSend(void);
void robotCmdInit(void);

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
} chassis_state;

// 底盘旋转方向
float chassis_gyro_dir = 1;

// 遥控器相关参数
namespace rcctrl {
float arm_position_rate = 2e-4f;
float arm_direction_rate = 3e-6f;
float arm_joint_rate = 1.5e-6f;

float chassis_speed_rate = 3.6e-3f;
float chassis_rotate_rate = 0.72f;
float chassis_follow_ff_rate = 0.3f;
float chassis_rand_gyro_rate = 20.f;

float gimbal_rate = 2e-4f;
}  // namespace rcctrl

// 初始化标志
bool startup_flag = false;
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

  if (robot_state == STOP) {
    allMotorsStopShutoff();
    robotReset();
  } else if (robot_state == STARTUP) {
    allMotorsOn();                  // 电机上电
    startup_flag = robotStartup();  // 开机状态判断
    robot_state = WORKING;
  } else if (robot_state == WORKING) {
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
}

// 重置各功能状态
void robotReset(void) {
  startup_flag = false;
  chassis_state = ChassisStateExt_e::FOLLOW;
  chassis_ctrl_ref_.mode_ = ChassisMode_e::Follow;
}

// 开机上电启动处理
bool robotStartup(void) {
  bool flag = true;
  chassis_ctrl_ref_.mode_ = ChassisMode_e::Follow;
  //  if (!gimbal.init_.j0_finish) {
  //    chassis.lock_ = true;
  //    flag = false;
  //  } else {
  //    chassis.lock_ = false;
  //  }
  //  if (!gimbal.init_.pitch_finish) {
  //    flag = false;
  //  }
  if (HAL_GetTick() < 1000) {
    flag = false;
  }
  return flag;
}

void robotCmdInit(void) {
  chassis_cmd_pub_ = PubRegister("chassis_cmd", sizeof(ChassisCtrlCmd));
  chassis_fdb_sub_ = SubRegister("chassis_fdb", sizeof(ChassisCtrlCmd));
}

void robotCmdSend(void) {
  SubGetMessage(chassis_fdb_sub_, &chassis_ctrl_fdb_);
  PubPushMessage(chassis_cmd_pub_, &chassis_ctrl_ref_);
}

void robotControl(void) {
  if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
    // 云台底盘测试
    if (rc.channel_.dial_wheel < -100) {
      // 开陀螺
      if (rc.channel_.r_col > 300) {
        chassis_state = ChassisStateExt_e::TWIST;
        chassis_ctrl_ref_.mode_ = ChassisMode_e::Separate;
      } else {
        chassis_state = ChassisStateExt_e::GYRO;
        chassis_ctrl_ref_.mode_ = ChassisMode_e::Separate;
      }
    } else if (rc.channel_.dial_wheel > 100) {
      // 关陀螺
      chassis_state = ChassisStateExt_e::FOLLOW;
      chassis_ctrl_ref_.mode_ = ChassisMode_e::Follow;
    } else {
    }

    if (chassis_state == ChassisStateExt_e::FOLLOW) {
      chassis_ctrl_ref_.vx = rc.channel_.r_col * rcctrl::chassis_speed_rate;
      chassis_ctrl_ref_.vy = -rc.channel_.r_row * rcctrl::chassis_speed_rate;
      chassis_ctrl_ref_.wz = 0;
    } else if (chassis_state == ChassisStateExt_e::GYRO) {
      float dice = sinf(HAL_GetTick() / 100.f);
      float wz = 450;
      if (dice >= 0) wz = 250;
      chassis_ctrl_ref_.vx = rc.channel_.r_col * rcctrl::chassis_speed_rate;
      chassis_ctrl_ref_.vy = -rc.channel_.r_row * rcctrl::chassis_speed_rate;
      chassis_ctrl_ref_.wz = wz * chassis_gyro_dir;
    } else if (chassis_state == ChassisStateExt_e::TWIST) {
      float wz = 480 * sin(HAL_GetTick() * 6e-3f);
      chassis_ctrl_ref_.vx = rc.channel_.r_col * rcctrl::chassis_speed_rate;
      chassis_ctrl_ref_.vy = -rc.channel_.r_row * rcctrl::chassis_speed_rate;
      chassis_ctrl_ref_.wz = wz;
    }
  }
}