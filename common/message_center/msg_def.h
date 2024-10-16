//
// Created by 98383 on 24-10-15.
//

#ifndef RM_FRAME_MSG_DEF_H
#define RM_FRAME_MSG_DEF_H

#include "app/chassis/chassis.h"

typedef struct {
    float vx; // x向速度
    float vy; // y向速度
    float wz; // 旋转角
    float ref_angle; // 底盘角度

    float fdb_angle, follow_fdb_angle; // 底盘反馈角度和跟随角度
    float chassis_power_limit, extra_power_max; // 功率限制参数
    ChassisMode_e mode_; // 底盘模式
} ChassisCtrlCmd;

#endif//RM_FRAME_MSG_DEF_H
