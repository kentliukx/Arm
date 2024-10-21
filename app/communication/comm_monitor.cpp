//
// Created by guanhuai on 24-10-21.
//

#include "comm_monitor.h"

#include "cmsis_os.h"
#include "hardware_config.h"

void CommMonitor::init(void) {
#ifdef CV_UART
  cv_comm_ = new CVComm(CV_UART);
#else
  cv_comm_ = new CVComm();
#endif  // CV_UART
#ifdef REFEREE_UART
  referee_comm_ = new RefereeComm(REFEREE_UART);
#else
  RefereeComm referee_comm_;
#endif  // REFEREE_UART
  cap_comm_ = new CapComm(&hcan2);

  cv_comm_->init();
  referee_comm_->init();

  gimbal_upload_sub_ = SubRegister("gimbal_upload", sizeof(GimbalUploadData));
  shoot_upload_sub_ = SubRegister("shoot_upload", sizeof(ShootUploadData));
  gimbal_cmd_pub_ = PubRegister("gimbal_cmd", sizeof(GimbalCtrlCmd));
  shoot_cmd_pub_ = PubRegister("shoot_cmd", sizeof(ShootCtrlCmd));
  cap_upload_pub_ = PubRegister("cap_upload", sizeof(CapUploadData));

  has_init = true;
}

void CommMonitor::handle() {
  cv_comm_->handle();
  referee_comm_->handle();
  cap_comm_->handle();
}

void CommMonitor::txHandle(uint32_t* tick) {
  if (!has_init) {
    return;
  }
  // CV
  cvcomm::MsgStream_e stream = cvcomm::MsgStream::BOARD2PC;
  cv_comm_->general_board2pc_msg_.mode = (uint8_t)mode_;
  cv_comm_->txMsg(stream, cvcomm::MsgType::GENERAL,
                  cv_comm_->general_pc2board_msg_);
  osDelayUntil(tick, 5);  // 自瞄/打符/反符

  static const auto update_aimshoot_msg = [&]() {
    SubGetMessage(gimbal_upload_sub_, &gimbal_upload_data_);
    SubGetMessage(shoot_upload_sub_, &shoot_upload_data_);

    cv_comm_->aim_shoot_board2pc_msg_.self_id =
        referee_comm_->game_robot_status_.robot_id;
    cv_comm_->aim_shoot_board2pc_msg_.yaw_offset =
        gimbal_upload_data_.yaw_offset;
    cv_comm_->aim_shoot_board2pc_msg_.pitch_offset =
        gimbal_upload_data_.pitch_offset;
    cv_comm_->aim_shoot_board2pc_msg_.shoot_speed =
        shoot_upload_data_.shoot_speed;
    cv_comm_->aim_shoot_board2pc_msg_.shoot_delay =
        shoot_upload_data_.shoot_delay;
    cv_comm_->aim_shoot_board2pc_msg_.shoot_id_fdb =
        shoot_upload_data_.shoot_id_fdb;
    cv_comm_->aim_shoot_board2pc_msg_.is_big_energy =
        shoot_upload_data_.is_big_energy;
    cv_comm_->aim_shoot_board2pc_msg_.pos_x = referee_comm_->game_robot_pos_.x;
    cv_comm_->aim_shoot_board2pc_msg_.pos_y = referee_comm_->game_robot_pos_.y;
  };

  static const auto update_navigation_msg = [&]() {
    // TODO: 更新导航数据
  };

  static const auto update_game_status_msg = [&]() {
    // TODO: 更新比赛状态数据
  };

  if (mode_ == CVMode::AUTOAIM || mode_ == CVMode::ENERGY ||
      mode_ == CVMode::ENERGY_DISTURB) {
    for (int i = 0; i < 20; i++) {
      update_aimshoot_msg();
      cv_comm_->txMsg(stream, cvcomm::MsgType::AIM_SHOOT,
                      cv_comm_->aim_shoot_board2pc_msg_);
      osDelayUntil(tick, 5);
    }
  }
  // 导航
  else if (mode_ == CVMode::NAVIGATION) {
    for (int i = 0; i < 20; i++) {
      update_navigation_msg();
      cv_comm_->txMsg(stream, cvcomm::MsgType::NAVIGATION,
                      cv_comm_->navigation_board2pc_msg_);
      osDelayUntil(tick, 5);
    }
  }
  // 决策（导航+自瞄）
  else if (mode_ == CVMode::DECISION) {
    for (int i = 0; i < 10; i++) {
      update_aimshoot_msg();
      cv_comm_->txMsg(stream, cvcomm::MsgType::AIM_SHOOT,
                      cv_comm_->aim_shoot_board2pc_msg_);
      osDelayUntil(tick, 5);
      update_navigation_msg();
      cv_comm_->txMsg(stream, cvcomm::MsgType::NAVIGATION,
                      cv_comm_->navigation_board2pc_msg_);
      osDelayUntil(tick, 5);
    }
    update_game_status_msg();
    cv_comm_->txMsg(stream, cvcomm::MsgType::GAME_STATUS,
                    cv_comm_->game_status_board2pc_msg_);
    osDelayUntil(tick, 10);
  } else {
    osDelayUntil(tick, 95);
  }
}

void CommMonitor::rxHandle() {
  if (!has_init) {
    return;
  }
  // CV
  gimbal_ctrl_cmd_.yaw_angle = cv_comm_->aim_shoot_pc2board_msg_.yaw_angle;
  gimbal_ctrl_cmd_.pitch_angle = cv_comm_->aim_shoot_pc2board_msg_.pitch_angle;
  gimbal_ctrl_cmd_.yaw_speed = cv_comm_->aim_shoot_pc2board_msg_.yaw_speed;
  gimbal_ctrl_cmd_.pitch_speed = cv_comm_->aim_shoot_pc2board_msg_.pitch_speed;
  gimbal_ctrl_cmd_.dist = cv_comm_->aim_shoot_pc2board_msg_.dist;
  PubPushMessage(gimbal_cmd_pub_, &gimbal_ctrl_cmd_);

  shoot_ctrl_cmd_.shoot_flag = cv_comm_->aim_shoot_pc2board_msg_.shoot_flag;
  shoot_ctrl_cmd_.shoot_id = cv_comm_->aim_shoot_pc2board_msg_.shoot_id;
  shoot_ctrl_cmd_.enemy_id = cv_comm_->aim_shoot_pc2board_msg_.enemy_id;
  PubPushMessage(shoot_cmd_pub_, &shoot_ctrl_cmd_);

  // Referee
  // TODO: publish裁判系统数据

  // Cap
  cap_upload_data_.cap_state = cap_comm_->rx_msg_.cap_state;
  cap_upload_data_.cap_voltage = cap_comm_->rx_msg_.cap_voltage;
  cap_upload_data_.chassis_power = cap_comm_->rx_msg_.chassis_power;
  PubPushMessage(cap_upload_pub_, &cap_upload_data_);
}

void CommMonitor::uartRxHandle(UART_HandleTypeDef* huart) {
  if (cv_comm_->uartCheck(huart)) {
    cv_comm_->rxCallback();
  }
  if (referee_comm_->uartCheck(huart)) {
    referee_comm_->rxCallback();
  }
}

void CommMonitor::canRxHandle(CAN_HandleTypeDef* hcan,
                              CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data) {
  if (cap_comm_->canRxMsgCheck(hcan, rx_header)) {
    cap_comm_->canRxMsgCallback(hcan, rx_header, rx_data);
  }
}
