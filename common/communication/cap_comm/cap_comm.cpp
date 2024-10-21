/**
******************************************************************************
* @file    cap_comm.cpp/h
* @brief   Capacity communication(CAN). 超级电容通信(CAN)
* @author Guan Huai
******************************************************************************
* Copyright (c) 2025 Team JiaoLong-SJTU
* All rights reserved.
******************************************************************************
*/

#include "cap_comm.h"

#include <cstring>

const uint32_t capacity_timeout = 1000;

CapComm::CapComm(CAN_HandleTypeDef* hcan)
    : connect_(capacity_timeout), hcan_(hcan) {}

void CapComm::init(void) {
  cap_upload_pub_ = PubRegister("cap_upload", sizeof(CapUploadData));
}

void CapComm::handle(void) { connect_.check(); }

// Transmit data to capacity
// 向电容控制板发送数据(未使用)
void CapComm::canTxMsg(void) {
  can_tx_header_.IDE = CAN_ID_STD;
  can_tx_header_.RTR = CAN_RTR_DATA;
  can_tx_header_.DLC = 8;
  can_tx_header_.StdId = board_id_;
  memcpy(can_tx_data_, &tx_msg_, 8);
  // transmit
  HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_, &can_tx_mail_box_);
}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID
bool CapComm::canRxMsgCheck(CAN_HandleTypeDef* hcan,
                            CAN_RxHeaderTypeDef rx_header) {
  return hcan == hcan_ && rx_header.StdId == capacity_id_;
}

// Receive feedback data message callback. Called in
// HAL_CAN_RxFifo0MsgPendingCallback()
// 电容信息接收回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
void CapComm::canRxMsgCallback(CAN_HandleTypeDef* hcan,
                               CAN_RxHeaderTypeDef rx_header,
                               uint8_t rx_data[8]) {
  memcpy(&rx_msg_, rx_data, 8);

  // publish data
  static CapUploadData cap_upload_data;
  cap_upload_data.cap_state = rx_msg_.cap_state;
  cap_upload_data.cap_voltage = rx_msg_.cap_voltage;
  cap_upload_data.chassis_power = rx_msg_.chassis_power;
  PubPushMessage(cap_upload_pub_, &cap_upload_data);

  connect_.refresh();
}
