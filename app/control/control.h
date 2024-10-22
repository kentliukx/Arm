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

#ifndef RM_FRAME_CONTROL_H
#define RM_FRAME_CONTROL_H

#include "base/monitor/motor_monitor.h"
#include "base/remote/remote.h"
#include "common/message_center/message_center.h"
#include "common/message_center/msg_def.h"
#include "iwdg.h"

// 控制初始化
void controlInit(void);
// 控制主循环
void controlLoop(void);
// 初始化主控cmd模块
void robotCmdInit(void);
// 主控发送指令
void robotCmdSend(void);
// 机器人模组控制
void ModuleControl(void);

#endif  // RM_FRAME_CONTROL_H
