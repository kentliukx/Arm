//
// Created by guanhuai on 24-10-21.
//

#include "common/communication/cap_comm/cap_comm.h"
#include "common/communication/cv_comm/cv_comm.h"
#include "common/communication/referee_comm/referee_comm.h"
#include "common/message_center/message_center.h"
#include "common/message_center/msg_def.h"

#ifndef RM_FRAME_COMM_MONITOR_H
#define RM_FRAME_COMM_MONITOR_H

class CommMonitor {
 private:
  CVComm* cv_comm_;
  RefereeComm* referee_comm_;
  CapComm* cap_comm_;

  Subscriber_t* gimbal_upload_sub_;
  Subscriber_t* shoot_upload_sub_;
  Publisher_t* gimbal_cmd_pub_;
  Publisher_t* shoot_cmd_pub_;
  Publisher_t* cap_upload_pub_;

  GimbalUploadData gimbal_upload_data_;
  ShootUploadData shoot_upload_data_;
  CapUploadData cap_upload_data_;
  GimbalCtrlCmd gimbal_ctrl_cmd_;
  ShootCtrlCmd shoot_ctrl_cmd_;

  uint8_t has_init = false;

 public:
  void init(void);
  void handle(void);
  void txHandle(uint32_t* tick);                 // 发送数据
  void rxHandle(void);                           // 处理接收到的数据
  void uartRxHandle(UART_HandleTypeDef* huart);  // UART接收回调
  void canRxHandle(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                   uint8_t* rx_data);  // CAN接收回调

  CVMode mode_;
};

#endif  // RM_FRAME_COMM_MONITOR_H
