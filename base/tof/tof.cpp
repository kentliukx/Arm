//
// Created by 98383 on 2024/7/6.
//

#include "tof.h"

#include <cstring>

// CAN模式单点式代码
// TOF_Sense::TOF_Sense(CAN_HandleTypeDef *hcan, uint8_t id) : hcan_(hcan),
// connect_(50) {
//    connect_.check();
//    id_config(id);
//}
//
// void TOF_Sense::handle(void) {}
//
// bool TOF_Sense::canRxMsgCheck(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef
// rx_header) {
//    return hcan == hcan_ && (rx_header.StdId == id_);
//}
//
// void TOF_Sense::canRxMsgCallback(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef
// rx_header, uint8_t *rx_data) {
//    memcpy(&raw_data, rx_data, sizeof(raw_data));
//    rx_data_.dis_temp =  (int32_t)(raw_data[0] << 8 | raw_data[1] << 16 |
//    raw_data[2] << 24) / 256; rx_data_.dis = (float)rx_data_.dis_temp /
//    1000.f; rx_data_.dis_status = raw_data[3]; rx_data_.signal_strength =
//    (uint16_t)(raw_data[4] << 8) + (uint16_t)raw_data[5]; connect_.refresh();
//}

// UART模式代码
TOFSenseDriver::TOFSenseDriver(UART_HandleTypeDef* huart, TOF_Sense* sensor1,
                               TOF_Sense* sensor2)
    : huart_(huart), step_(WAIT) {
  sensor_[0] = sensor1;
  sensor_[1] = sensor2;
}

void TOFSenseDriver::rxCallback() {
  fifo1 = data_;
  for (int i = 0; i < 6; i++) fifo6[i] = raw_data[i];
  if (step_ == WAIT) {
    step_ = HEAD;
    if (fifo1 != 0x57) {
      step_ = WAIT;
    }
    sum = 0x57;
    index_ = 0;
    HAL_UART_Receive_IT(huart_, &data_, 1);
  } else if (step_ == HEAD) {
    step_ = MARK;
    if (fifo1 != 0x01) {
      step_ = WAIT;
    }
    sum += fifo1;
    HAL_UART_Receive_IT(huart_, raw_data, sizeof(raw_data));
  } else if (step_ == MARK) {
    step_ = WASTE1;
    id = fifo6[1];
    add();
    HAL_UART_Receive_IT(huart_, &data_, 1);
  } else if (step_ == WASTE1) {
    step_ = LEN;
    length = fifo1;
    sum += fifo1;
    HAL_UART_Receive_IT(huart_, raw_data, sizeof(raw_data));
  } else if (step_ == LEN) {
    HAL_UART_Receive_IT(huart_, raw_data, sizeof(raw_data));
    int32_t temp =
        (int32_t)(fifo6[0] << 8 | fifo6[1] << 16 | fifo6[2] << 24) / 256;
    sensor_[id]->dis[index_] = (float)temp / 1e6f;  // 单位化成m
    sensor_[id]->dis_state[index_] = fifo6[3];
    sensor_[id]->signal_strength[index_++] =
        (uint16_t)(fifo6[4] << 8) | fifo6[5];
    if (index_ == length) {
      step_ = DATA;
    }
    add();
  } else if (step_ == DATA) {
    step_ = WASTE2;
    add();
    HAL_UART_Receive_IT(huart_, &data_, 1);
  } else if (step_ == WASTE2) {
    if (sum == fifo1) {
      sensor_[id]->state = 1;
    } else {
      sensor_[id]->state = 0;
    }
    step_ = WAIT;
    HAL_UART_Receive_IT(huart_, &data_, 1);
    sensor_[id]->connect_.refresh();
  }
  sensor_[id]->tick = HAL_GetTick();
}

void TOFSenseDriver::init() {
  if (huart_ != nullptr) {
    HAL_UART_Receive_IT(huart_, &data_, 1);
  }
}

TOF_Sense::TOF_Sense(uint32_t tick) : connect_(tick) {}

void TOFSenseDriver::query(uint8_t id_) {
  uint8_t data[8] = {0x57, 0x10, 0xff, 0xff, 0x00, 0xff, 0xff, 0x63};
  data[4] = id_;
  data[7] += data[4];
  HAL_UART_Transmit(huart_, data, sizeof(data), HAL_MAX_DELAY);
}
