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

#include "base/motor/motor_driver/lk_motor_driver.h"
#include "algorithm/math/math.h"

LKMotorDriver::LKMotorDriver(Motor* motor, CAN_HandleTypeDef* hcan,
                             uint32_t frame_id)
    : motor_(motor), hcan_(hcan), frame_id_(frame_id) {
  if (hcan_ == &hcan1) {
    motor_->CANIdConfig(1, frame_id_);
  } else if (hcan_ == &hcan2) {
    motor_->CANIdConfig(2, frame_id_);
  }
}

bool LKMotorDriver::add_cmd(const lk_motor::CmdType_e& cmd,
                            const uint8_t& byte1, const uint8_t& byte2,
                            const uint8_t& byte3, const uint8_t& byte4,
                            const uint8_t& byte5, const uint8_t& byte6,
                            const uint8_t& byte7) {
  if (cmd_queue_.size() > 10) {
    return false;
  }
  std::vector<uint8_t> cmd_msg(8);
  cmd_msg[0] = cmd;
  cmd_msg[1] = byte1;
  cmd_msg[2] = byte2;
  cmd_msg[3] = byte3;
  cmd_msg[4] = byte4;
  cmd_msg[5] = byte5;
  cmd_msg[6] = byte6;
  cmd_msg[7] = byte7;
  cmd_queue_.push(cmd_msg);
  return true;
}

void LKMotorDriver::canTxMsg() {
  // can message header
  can_tx_header_.IDE = CAN_ID_STD;
  can_tx_header_.RTR = CAN_RTR_DATA;
  can_tx_header_.DLC = 8;
  can_tx_header_.StdId = frame_id_;
  if (!cmd_queue_.empty()) {
    std::vector<uint8_t> cmd_data;
    cmd_data = cmd_queue_.front();
    can_tx_data_[0] = cmd_data[0];
    can_tx_data_[1] = cmd_data[1];
    can_tx_data_[2] = cmd_data[2];
    can_tx_data_[3] = cmd_data[3];
    can_tx_data_[4] = cmd_data[4];
    can_tx_data_[5] = cmd_data[5];
    can_tx_data_[6] = cmd_data[6];
    can_tx_data_[7] = cmd_data[7];
    cmd_queue_.pop();
  } else {  // TORQ_CLOSED_LOOP_CONTROL
    can_tx_data_[0] = lk_motor::CmdType_e::TORQ_CLOSED_LOOP_CONTROL;
    can_tx_data_[1] = 0;
    can_tx_data_[2] = 0;
    can_tx_data_[3] = 0;
    can_tx_data_[4] = (motor_->intensity_ & 0xff);  // 低字节 lower byte
    can_tx_data_[5] = motor_->intensity_ >> 8;      // 高字节 higher byte
    can_tx_data_[6] = 0;
    can_tx_data_[7] = 0;
  }

  ++tx_cnt_;

  HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_, &can_tx_mail_box_);
}

bool LKMotorDriver::canRxMsgCheck(CAN_HandleTypeDef* hcan,
                                  CAN_RxHeaderTypeDef rx_header) {
  return hcan == hcan_ && rx_header.StdId == frame_id_;
}

void LKMotorDriver::canRxMsgCallback(CAN_HandleTypeDef* hcan,
                                     CAN_RxHeaderTypeDef rx_header,
                                     uint8_t rx_data[8]) {
  // Select feedback command type
  cmd_type_ = rx_data[0];
  if (cmd_type_ == lk_motor::CmdType_e::TORQ_CLOSED_LOOP_CONTROL) {
    raw_data_.temp = (int8_t)rx_data[1];
    raw_data_.current = (int16_t)(rx_data[3] << 8 | rx_data[2]);
    raw_data_.rotate_speed = (int16_t)(rx_data[5] << 8 | rx_data[4]);
    raw_data_.ecd = (uint16_t)(rx_data[7] << 8 | rx_data[6]);
    // Update motor feedback data (raw_data->motor_data)
    // Encoder angle(deg)
    // 编码器角度
    motor_->motor_data_.ecd_angle = math::ecd2deg(raw_data_.ecd, 65536);
    // Use incremental calculation to deals with encoder overflow and underflow
    // 增量计算处理编码器上下溢问题
    float delta =
        motor_->motor_data_.ecd_angle - motor_->motor_data_.last_ecd_angle;
    delta = math::degNormalize180(delta) / motor_->ratio_;
    motor_->motor_data_.angle += delta;  // deg
    // feedback rotational speed
    // 反馈转速
    motor_->motor_data_.rotate_speed =
        (float)raw_data_.rotate_speed / motor_->ratio_;  // dps
    // Update current and temperature
    // 更新转矩电流和温度
    motor_->motor_data_.torque = (float)raw_data_.current * 0.0693333f;
    motor_->motor_data_.temp = raw_data_.temp;
    // update encoder angle record
    // 更新编码器角度记录
    motor_->motor_data_.last_ecd_angle = motor_->motor_data_.ecd_angle;

  } else if (cmd_type_ == lk_motor::CmdType_e::READ_STATE ||
             cmd_type_ == lk_motor::CmdType_e::CLEAR_ERROR) {
    voltage_state_ = (rx_data[7] & 0x01) == 0
                         ? lk_motor::VoltageState_e::NORMAL_VOL
                         : lk_motor::VoltageState_e::LOW_VOLTAGE;
    temp_state_ = ((rx_data[7] >> 3) & 0x01) == 0
                      ? lk_motor::TemperatureState_e::NORMAL_TEMP
                      : lk_motor::TemperatureState_e::OVER_HEAT;
  }
  motor_->rxCallback();
}

void LKMotorDriver::handle() {
  if (tx_cnt_ == 5) {
    // if temperature is high, tx state request after 5 torque msg
    if (motor_->motor_data_.temp > 75) {
      add_cmd(lk_motor::CmdType_e::READ_STATE);
    }
    if (temp_state_ == lk_motor::TemperatureState_e::OVER_HEAT ||
        voltage_state_ == lk_motor::VoltageState_e::LOW_VOLTAGE) {
      add_cmd(lk_motor::CmdType_e::CLEAR_ERROR);
    }
    tx_cnt_ = 0;
  }
  if (!motor_->connect_.check()) {
    add_cmd(lk_motor::CmdType_e::OPERATE);
  }
  motor_->handle();
}
