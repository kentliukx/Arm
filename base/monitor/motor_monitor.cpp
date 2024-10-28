/**
******************************************************************************
* @file    motor_monitor.cpp/h
* @brief   Motor parameter, id config and management. 电机参数id配置和统一管理
* @author  Spoon Guan
******************************************************************************
* Copyright (c) 2023 Team JiaoLong-SJTU
* All rights reserved.
******************************************************************************
*/

#include "motor_monitor.h"

#include "lib/arm_math/arm_math.h"

// Motor parameter config
// 电机参数配置

// Chassis motor 底盘电机
const PID chassis_wheel_spid(10, 0, 0, 0, 16384);
const float CHASSIS_MOTOR_RATIO = -13.9f;
Motor CMFL(Motor::M3508, CHASSIS_MOTOR_RATIO,
           Motor::SPEED,                     // type, ratio, method
           PID(), PID(chassis_wheel_spid));  // ppid, spid
Motor CMFR(Motor::M3508, CHASSIS_MOTOR_RATIO,
           Motor::SPEED,                     // type, ratio, method
           PID(), PID(chassis_wheel_spid));  // ppid, spid
Motor CMBL(Motor::M3508, CHASSIS_MOTOR_RATIO,
           Motor::SPEED,                     // type, ratio, method
           PID(), PID(chassis_wheel_spid));  // ppid, spid
Motor CMBR(Motor::M3508, CHASSIS_MOTOR_RATIO,
           Motor::SPEED,                     // type, ratio, method
           PID(), PID(chassis_wheel_spid));  // ppid, spid

// 航向电机
const float steering_motor_ratio = -1.0f;
const PID STEERING_POSITION_PID(30, 6, 150, 22, 1000);
const PID STEERING_SPEED_PID(30, 5, 0, 10, 30000);
Motor STFL(Motor::GM6020, steering_motor_ratio, Motor::POSITION_SPEED,
           PID(STEERING_POSITION_PID), PID(STEERING_SPEED_PID));
Motor STFR(Motor::GM6020, steering_motor_ratio, Motor::POSITION_SPEED,
           PID(STEERING_POSITION_PID), PID(STEERING_SPEED_PID));
Motor STBL(Motor::GM6020, steering_motor_ratio, Motor::POSITION_SPEED,
           PID(STEERING_POSITION_PID), PID(STEERING_SPEED_PID));
Motor STBR(Motor::GM6020, steering_motor_ratio, Motor::POSITION_SPEED,
           PID(STEERING_POSITION_PID), PID(STEERING_SPEED_PID));

// 摩擦轮电机
const PID fric_spid(4, 0, 15, 0, 16384);

Motor FRICL(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
            PID(), PID(fric_spid));         // ppid, spid
Motor FRICR(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
            PID(), PID(fric_spid));         // ppid, spid

// 拨盘电机
Motor STIR(Motor::M2006, -36, Motor::POSITION_SPEED,  // type, ratio, method
           PID(20, 0.1, 10, 10, 2500),                // ppid
           PID(60, 0.1, 0, 1000, 10000), Motor::Hit, -2);  // spid

// 上板
// Motor* can1_dji_motor[11] = {
//    &CMFL,    // id:1
//    &CMBL,    // id:2
//    nullptr,  // id:3
//    nullptr,  // id:4
//    &STFL,    // id:5
//    &STBL,    // id:6
//    nullptr,  // id:7
//    nullptr,  // id:8
//    nullptr,  // id:9
//    nullptr,  // id:10
//    nullptr   // id:11
//};
// Motor* can2_dji_motor[11] = {
//    &CMFR,    // id:1
//    &CMBR,    // id:2
//    nullptr,  // id:3
//    nullptr,  // id:4
//    &STFR,    // id:5
//    &STBR,    // id:6
//    nullptr,  // id:7
//    nullptr,  // id:8
//    nullptr,  // id:9
//    nullptr,  // id:10
//    nullptr   // id:11
//};

// 下板
Motor* can1_dji_motor[11] = {
    nullptr,  // id:1
    nullptr,  // id:2
    &FRICL,   // id:3
    &FRICR,   // id:4
    nullptr,  // id:5
    nullptr,  // id:6
    &STIR,    // id:7
    nullptr,  // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};
Motor* can2_dji_motor[11] = {
    nullptr,  // id:1
    nullptr,  // id:2
    nullptr,  // id:3
    nullptr,  // id:4
    nullptr,  // id:5
    nullptr,  // id:6
    nullptr,  // id:7
    nullptr,  // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};

DJIMotorDriver dji_motor_driver(can1_dji_motor, can2_dji_motor);

// 电机停转速度上限(dps)
const float motor_stop_rotate_speed_thres = 1200;

// Stop and shut off all motors
// 所有电机先停转再断电
void allMotorsStopShutoff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can1_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can1_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can1_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can2_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can2_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can2_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
  }
}

// Shut off all motors
// 所有电机断电
void allMotorsShutOff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
  }
}

// Stop all motors
// 所有电机停转
void allMotorsStop(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::STOP;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::STOP;
    }
  }
}

// Startup all motors
// 所有电机启动
void allMotorsOn(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::WORKING;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::WORKING;
    }
  }
}

// Reset all motors.
// 电机统一初始化
void allMotorsInit(void) {
  dji_motor_driver.idConfig();
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->reset();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->reset();
    }
  }
}

// Handle all motors. Called in motorTask
// 电机统一处理，在motorTask中调用
void allMotorsHandle(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->handle();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->handle();
    }
  }
}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID，调用对应回调函数
void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data) {
  if (dji_motor_driver.canRxMsgCheck(hcan, rx_header)) {
    dji_motor_driver.canRxMsgCallback(hcan, rx_header, rx_data);
  }
}
