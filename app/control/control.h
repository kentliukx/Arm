/**
******************************************************************************
* @file    motor_monitor.cpp/h
* @brief   Motor parameter, id config and management. 电机参数id配置和统一管理
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
#include "common/message_center/msg_def.h"
#include "iwdg.h"

// 控制初始化
void controlInit(void);
// 控制主循环
void controlLoop(void);

#endif  // RM_FRAME_CONTROL_H
