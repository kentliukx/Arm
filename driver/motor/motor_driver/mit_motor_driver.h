/**
 ******************************************************************************
 * @file    mit_motor_driver.cpp/h
 * @brief   Driver program for motor use MIT protocol. MIT协议电机驱动
 * @author  Tianzhe Yang
 ******************************************************************************
 * Copyright (c) 2025 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MIT_MOTOR_DRIVER_H
#define MIT_MOTOR_DRIVER_H

#include "can.h"

#include "driver/motor/motor.h"

namespace mitmotor {

// motor mode command
typedef enum CmdType {
  NO_CMD = 0,
  MOTOR_MODE = 0xFC,
  RESET_MODE = 0xFD,
  ZERO_POSITION = 0xFE,
  CLEAR_ERROR = 0xFB,
} CmdType_e;

// state type from RX message

#define MIT_MOTOR_STATE_DISABLE 0
#define MIT_MOTOR_STATE_ENABLE 1
#define MIT_MOTOR_STATE_OVER_VOLTAGE 0x8
#define MIT_MOTOR_STATE_UNDER_VOLTAGE 0x9
#define MIT_MOTOR_STATE_OVER_CURRENT 0xA
#define MIT_MOTOR_STATE_OVER_TEMPERATURE_MOS 0xB
#define MIT_MOTOR_STATE_OVER_TEMPERATURE_COIL 0xC
#define MIT_MOTOR_STATE_LOST_COMMUNICATION 0xD
#define MIT_MOTOR_STATE_OVERLOAD 0xE

#define MIT_MOTOR_STATE_ISERROR(X) ((X) & 0x8)

// feedback data
typedef struct Feedback {
  uint8_t id;
  float position;  // rad
  float velocity;  // rad/s
  float torq;      // N·m
} Feedback_t;

};  // namespace mitmotor

//
class MITMotorDriver {
 public:
  MITMotorDriver(Motor* motor, CAN_HandleTypeDef* hcan, uint32_t master_id,
                 uint32_t slave_id, float p_min, float p_max, float v_min,
                 float v_max, float kp_min, float kp_max, float kv_min,
                 float kv_max, float t_ff_min, float t_ff_max, float t_min,
                 float t_max);

  // deal with error situations and call motor::handle
  void handle(void);

  // set motor mode command
  bool setCmd(mitmotor::CmdType_e cmd);

  // set motor control parameter
  // p: target position(rad)
  // v: target velocity(rad/s)
  // kp/kv: control gain
  // t: feedforward torque
  // T = kp*(p-p_fdb)+kv*(v-v_fdb)+t
  void setControlParam(float p, float v, float kp, float kv, float t_ff);

  // Transmit CAN message
  void canTxMsg(void);

  // Check CAN channel and CAN message header id
  bool canRxMsgCheck(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);

  // Receive feedback data message callback. Called in
  // HAL_CAN_RxFifo0MsgPendingCallback()
  void canRxMsgCallback(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                        uint8_t rx_data[8]);

 public:
  // motor pointer
  Motor* motor_;

  // transmit data pack
  struct TxData {
    uint16_t p;
    uint16_t v;
    uint16_t kp;
    uint16_t kv;
    uint16_t t_ff;
  } tx_data_;

  // receive data pack
  struct RxData {
    uint8_t id;
    uint8_t state;
    float position;  // deg
    float velocity;  // dps
    float torq;      // N·m
  } rx_data_;

  // state code
  uint8_t state_code_;

 private:
  CAN_HandleTypeDef* hcan_;
  uint32_t master_id_;  // motor to board
  uint32_t slave_id_;   // board to motor

  // can transmit
  CAN_TxHeaderTypeDef can_tx_header_;
  uint8_t can_tx_data_[8];
  uint32_t can_tx_mail_box_;

  // motor command
  struct Command_t {
    uint32_t tick;
    const uint32_t interval = 100;
    // motor command queue
    mitmotor::CmdType_e list[5] = {mitmotor::NO_CMD};
    uint8_t cnt = 0;
  } cmd_;

  // motor parameters
  struct Param_t {
    // position limit (deg)
    float p_min;
    float p_max;
    // velocity limit (dps)
    float v_min;
    float v_max;
    // PD param limit
    float kp_min;
    float kp_max;
    float kv_min;
    float kv_max;
    // feedforward torque limit(N·m)
    float t_ff_min;
    float t_ff_max;
    // feedback torque limit(N·m)
    float t_min;
    float t_max;
  } param_;
};

#endif  // MIT_MOTOR_DRIVER_H
