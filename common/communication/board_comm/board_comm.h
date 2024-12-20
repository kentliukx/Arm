/**
 ******************************************************************************
 * @file    board_comm.cpp/h
 * @brief   Board communication(CAN). 板间通信(CAN)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef BOARD_COMM_H
#define BOARD_COMM_H

#include "common/connect/connect.h"
#include "can.h"

class BoardComm {
 public:
  BoardComm(CAN_HandleTypeDef* hcan);

  void handle(void);

  // Transmit data
  // 发送数据
  void canTxMsg(void);

  // Check CAN channel and id of received CAN message
  // 校验接收信息的CAN通道和ID
  bool canRxMsgCheck(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);

  // Receive feedback data message callback. Called in
  // HAL_CAN_RxFifo0MsgPendingCallback()
  // 信息接收回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
  void canRxMsgCallback(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                        uint8_t rx_data[8]);

 public:
  Connect connect_;

  // board to capacity
  struct TxMsg {
    uint8_t reserve[8];
  } tx_msg_;

  // capacity to board
  struct RxMsg {
    uint8_t reserve[8];
  } rx_msg_;

 private:
  CAN_HandleTypeDef* hcan_;
  CAN_TxHeaderTypeDef can_tx_header_;
  uint8_t can_tx_data_[8];
  uint32_t can_tx_mail_box_;

  const uint16_t master_id_ = 0x101;
  const uint16_t slave_id_ = 0x102;
};

#endif  // BOARD_COMM_H
