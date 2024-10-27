//
// Created by 98383 on 24-10-26.
//

#ifndef RM_FRAME_SHOOT_H
#define RM_FRAME_SHOOT_H

#include "algorithm/math/math.h"
#include "base/motor/motor.h"
#include "base/servo/servo.h"
#include "common/message_center/message_center.h"
#include "common/message_center/msg_def.h"

class Shoot {
 public:
  Shoot(Motor* fric_l, Motor* fric_r, Motor* stir);

  // 发射一发弹丸(发射-true，未发射-false)
  bool ShootOneBullet(void);
  // 发射数据处理
  void handle(void);
  // 设置射击参数
  void setShootParam(const float& speed_limit, const float& heat_limit,
                     const float& cooling_rate);
  // 设置最小射击间隔(<20ms)
  void setCD(const uint32_t& cd) { cd_ = cd; }

  // 开关摩擦轮
  void fricOn(void) { fric_state_ = true; }
  void fricOff(void) { fric_state_ = false; }

  // 射击速度
  float getBulletSpeed(void);
  // 发射延迟
  uint8_t getShootDelay(void) { return delay_; }

  void bullet_speed_control();

  void new_bullet() { new_bullet_ = true; }

  inline float get_speed_target() { return adaptation_target; }

  inline void stir_reset() {
    stir_->control_data_.target_angle = stir_->control_data_.fdb_angle;
  }

 protected:
  // 射速处理
  void speedHandle(void);
  // 卡弹处理
  void blockHandle(void);
  // 热量处理
  void heatHandle(void);
  // 重置自适应弹速
  inline void reset_speed_control() {
    record_count = 0;
    adaptation_target = 0.9f * speed_limit_;
    fric_speed_offset = 0.0f;
  };

 protected:
  // 发射机构反馈发布者
  Publisher_t* shoot_pub_;
  // 发射机构命令接收者
  Subscriber_t* shoot_sub_;
  // 指令存放位置
  ShootCtrlCmd shoot_cmd_rcv_;
  ShootFdbData shoot_fdb_send;
  // 裁判系统相关
  Subscriber_t* referee_sub_;
  RefereeShootFdb referee_data_;

  Motor *fric_l_, *fric_r_, *stir_;  // 电机指针

  bool shoot_state_;  // true-可以发射，false-不能发射
  bool fric_state_;   // 摩擦轮状态，true-开启，false-停止
  bool block_state_;  // 卡弹状态，true-卡弹，false-未卡弹
  bool heat_state_;   // 热量状态，true-剩余热量大于1发弹丸热量

  float speed_limit_;   // 射速上限
  float heat_limit_;    // 热量上限
  float cooling_rate_;  // 冷却

  float fric_speed_;          // 摩擦轮转速
  float bullet_speed_;        // 弹速
  uint32_t cd_;               // 发射间隔(ms)
  uint32_t last_tick_;        // 发射时间记录(ms)
  uint32_t block_tick_;       // 卡弹时间记录(ms)
  uint32_t last_cmd_tick_;    // 发射命令时间戳记录(ms)
  float calc_heat_;           // 计算热量
  const uint8_t delay_ = 20;  // 发弹延迟(控制+机械延迟，ms)

  // 自适应弹速相关
  bool new_bullet_ = false;
  uint8_t record_count = 0;
  float speed_record[5];
  float adaptation_target = 27.0f;
  const float accelerate_kp = 500.0f;
  const float decelerate_kp = 600.0f;
  float fric_speed_offset = 0.0f;
};

#endif  // RM_FRAME_SHOOT_H
