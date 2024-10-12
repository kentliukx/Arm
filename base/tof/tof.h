//
// Created by 98383 on 24-10-12.
//

#ifndef RM_FRAME_TOF_H
#define RM_FRAME_TOF_H

#include "common/connect/connect.h"
#include "can.h"
#include "hardware_config.h"

// CAN通信模式单点式代码
//class TOF_Sense {
//public:
//    TOF_Sense(CAN_HandleTypeDef* hcan, uint8_t id);
//
//    void handle(void);
//
//    // Check CAN channel and id of received CAN message
//    // 校验接收信息的CAN通道和ID
//    bool canRxMsgCheck(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);
//
//    // Receive feedback data message callback. Called in
//    // HAL_CAN_RxFifo0MsgPendingCallback()
//    // 信息接收回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
//    void canRxMsgCallback(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
//                          uint8_t* rx_data);
//
//    //确认can包ID
//    void id_config(uint8_t id) {id_ += id;}
//
//public:
//    Connect connect_;
//    uint8_t raw_data[8];
//    struct Rxdata_t {
//        int32_t dis_temp;
//        uint8_t dis_status;
//        uint16_t signal_strength;
//        float dis;
//    } rx_data_;
//private:
//    CAN_HandleTypeDef* hcan_;
//    uint16_t id_ = 0x210;
//};

// UART模式代码

class TOF_Sense {
public:
    float dis[64];
    uint8_t state;
    uint8_t dis_state[64];
    uint16_t signal_strength[64];
    uint64_t tick;
    float avg_dis;
    float dif;
    uint8_t id;
    Connect connect_;
    explicit TOF_Sense(uint32_t tick);
};

class TOFSenseDriver{
public:
    explicit TOFSenseDriver(UART_HandleTypeDef* huart = nullptr, TOF_Sense* sensor1 = nullptr, TOF_Sense* sensor2 = nullptr);
    void rxCallback();
    void init();
    bool uartCheck(UART_HandleTypeDef* huart) { return huart == huart_; }

    enum Step{
        WAIT,
        HEAD,
        MARK,
        WASTE1,
        LEN,
        DATA,
        WASTE2,
        CHECK,
    } step_;

    TOF_Sense* sensor_[2];

    void query(uint8_t id_);
private:
    inline void add(){
        for (unsigned char i : fifo6) sum += i;
    }

    UART_HandleTypeDef* huart_;
    uint8_t data_;
    uint8_t raw_data[6];
    uint8_t fifo1, fifo6[6];
    uint8_t sum, index_, length, id;
};

#endif//RM_FRAME_TOF_H
