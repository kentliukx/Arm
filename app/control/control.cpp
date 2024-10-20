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

// 初始化标志
bool startup_flag = false;
// 遥控器挡位记录
RC::RCSwitch last_rc_switch;
RC::RCChannel last_rc_channel;

void controlInit(void) { robot_state = STOP; }

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
void robotReset(void) { startup_flag = false; }

// 开机上电启动处理
bool robotStartup(void) {
  bool flag = true;
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
