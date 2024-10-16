/**
******************************************************************************
* @file    lk_motor_driver.cpp/h
* @brief   Driver program for motor use LK_tech protocol. 瓴控电机驱动
* @author  Tianzhe Yang
******************************************************************************
* Copyright (c) 2025 Team JiaoLong-SJTU
* All rights reserved.
******************************************************************************
*/

#ifndef WHEEL_LEGGED_CHASSIS_CPP_LK_MOTOR_DRIVER_H
#define WHEEL_LEGGED_CHASSIS_CPP_LK_MOTOR_DRIVER_H

#include <queue>
#include <vector>

#include "can.h"

#include "base/motor/motor.h"

namespace lk_motor {
// read by READ_STATE command
typedef enum { NORMAL_VOL, LOW_VOLTAGE } VoltageState_e;
typedef enum { NORMAL_TEMP, OVER_HEAT } TemperatureState_e;

// motor command that used in data[0]
typedef enum {
  TORQ_CLOSED_LOOP_CONTROL = 0xA1,  // 转矩闭环控制命令
  READ_STATE = 0X9A,   // 读取电机状态（包括错误状态）
  CLEAR_ERROR = 0X9B,  // 清除错误状态
                       // “电机状态没有恢复正常时，错误状态无法消除”
  SHUT_UP = 0x80,  // shut up motor, and clear previous control commands
  STOP = 0x81,     // stop motor, but not clear previous control commands
  OPERATE = 0x88   // recover from stop state
} CmdType_e;

// feedback raw data from TORQ_CLOSED_LOOP_CONTROL
typedef struct TorqCLControlRawData {
  int8_t temp;           // temperature; unit: Celsius degree
  int16_t current;       // torque current
                         // range:[-2000,2000] ,corresponding to [-33A, +33A]
  int16_t rotate_speed;  // rotate speed; unit: dps
  uint16_t ecd;          // encoder value;
                         // range: [0~65535]
} RawData_t;

#define LK_FRAME_IDENTIFIER 0x140

};  // namespace lk_motor

class LKMotorDriver {
 private:
  // can transmit
  CAN_HandleTypeDef* hcan_;
  uint32_t frame_id_;
  CAN_TxHeaderTypeDef can_tx_header_;
  uint8_t can_tx_data_[8];
  uint32_t can_tx_mail_box_;

  std::queue<std::vector<uint8_t>> cmd_queue_;

  uint32_t tx_cnt_ = 0;

  // can receive
  uint8_t cmd_type_;
  lk_motor::RawData_t raw_data_;

  lk_motor::VoltageState_e voltage_state_;
  lk_motor::TemperatureState_e temp_state_;

 public:
  Motor* motor_;

 public:
  LKMotorDriver(Motor* motor, CAN_HandleTypeDef* hcan, uint32_t frame_id);

  void handle();

  bool add_cmd(const lk_motor::CmdType_e& cmd, const uint8_t& byte1 = 0,
               const uint8_t& byte2 = 0, const uint8_t& byte3 = 0,
               const uint8_t& byte4 = 0, const uint8_t& byte5 = 0,
               const uint8_t& byte6 = 0, const uint8_t& byte7 = 0);

  void canTxMsg();

  // Check whether CAN channel and message header id is this motor
  bool canRxMsgCheck(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);

  // ReceiveData message callback. Called in HAL_CAN_RxFifo0MsgPendingCallback()
  void canRxMsgCallback(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                        uint8_t rx_data[8]);
};

#endif  // WHEEL_LEGGED_CHASSIS_CPP_LK_MOTOR_DRIVER_H
