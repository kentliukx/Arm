//
// Created by 98383 on 24-10-15.
//

#include "SwerveChassis.h"

float vector_sum(float x_component, float y_component) {
  return sqrtf(powf(x_component, 2) + powf(y_component, 2));
}

SwerveChassis::SwerveChassis() {
  chassis_pub_ = PubRegister("chassis_cmd", sizeof(ChassisCtrlCmd));
  chassis_sub_ = SubRegister("chassis_fdb", sizeof(ChassisCtrlCmd));
}

void SwerveChassis::ikine(void) {
  float& vx = chassis_ref_spd_.vx;
  float& vy = chassis_ref_spd_.vy;
  float& wz = chassis_ref_spd_.wz;

  // 将转速（rpm）换算为轮子线速度（m/s）
  float rotate_component =
      wz * (2.0f * PI * (wheel_base / sqrtf(2.0f))) / 60.0f;

  // 解算轮速，将单位由 m/s 换算为 dps
  ref_chassis_.wheel_speed.fl =
      vector_sum(vx - rotate_component * sinf(math::deg2rad(45.0f)),
                 vy + rotate_component * cosf(math::deg2rad(45.0f))) /
      (2 * PI * wheel_radius) * 360.0f;
  ref_chassis_.wheel_speed.fr =
      vector_sum(vx + rotate_component * sinf(math::deg2rad(45.0f)),
                 vy + rotate_component * cosf(math::deg2rad(45.0f))) /
      (2 * PI * wheel_radius) * 360.0f;
  ref_chassis_.wheel_speed.bl =
      vector_sum(vx - rotate_component * sinf(math::deg2rad(45.0f)),
                 vy - rotate_component * cosf(math::deg2rad(45.0f))) /
      (2 * PI * wheel_radius) * 360.0f;
  ref_chassis_.wheel_speed.br =
      vector_sum(vx + rotate_component * sinf(math::deg2rad(45.0f)),
                 vy - rotate_component * cosf(math::deg2rad(45.0f))) /
      (2 * PI * wheel_radius) * 360.0f;

  if (!(math::float_equal(vx, 0) && math::float_equal(vy, 0) &&
        math::float_equal(rotate_component, 0))) {
    ref_chassis_.steering_angle.fl = math::rad2deg(
        atan2f(vy + rotate_component * cosf(math::deg2rad(45.0f)),
               vx - rotate_component * sinf(math::deg2rad(45.0f))));
    ref_chassis_.steering_angle.fr = math::rad2deg(
        atan2f(vy + rotate_component * cosf(math::deg2rad(45.0f)),
               vx + rotate_component * sinf(math::deg2rad(45.0f))));
    ref_chassis_.steering_angle.bl = math::rad2deg(
        atan2f(vy - rotate_component * cosf(math::deg2rad(45.0f)),
               vx - rotate_component * sinf(math::deg2rad(45.0f))));
    ref_chassis_.steering_angle.br = math::rad2deg(
        atan2f(vy - rotate_component * cosf(math::deg2rad(45.0f)),
               vx + rotate_component * sinf(math::deg2rad(45.0f))));
  }

  ref_chassis_.steering_angle.fl =
      STFL_.shortest_steering_path(ref_chassis_.steering_angle.fl);
  ref_chassis_.steering_angle.fr =
      STFR_.shortest_steering_path(ref_chassis_.steering_angle.fr);
  ref_chassis_.steering_angle.bl =
      STBL_.shortest_steering_path(ref_chassis_.steering_angle.bl);
  ref_chassis_.steering_angle.br =
      STBR_.shortest_steering_path(ref_chassis_.steering_angle.br);

  ref_chassis_.wheel_speed.fl *= STFL_.get_reverse_speed();
  ref_chassis_.wheel_speed.fr *= STFR_.get_reverse_speed();
  ref_chassis_.wheel_speed.bl *= STBL_.get_reverse_speed();
  ref_chassis_.wheel_speed.br *= STBR_.get_reverse_speed();
}

void SwerveChassis::fkine(void) {
  float vx = (fdb_robot_.wheel_speed.fl *
                  cosf(math::deg2rad(fdb_robot_.steering_angle.fl)) +
              fdb_robot_.wheel_speed.fr *
                  cosf(math::deg2rad(fdb_robot_.steering_angle.fr)) +
              fdb_robot_.wheel_speed.bl *
                  cosf(math::deg2rad(fdb_robot_.steering_angle.bl)) +
              fdb_robot_.wheel_speed.br *
                  cosf(math::deg2rad(fdb_robot_.steering_angle.br))) /
             4.0f / 360.0f * (PI * 2 * wheel_radius);
  float vy = (fdb_robot_.wheel_speed.fl *
                  sinf(math::deg2rad(fdb_robot_.steering_angle.fl)) +
              fdb_robot_.wheel_speed.fr *
                  sinf(math::deg2rad(fdb_robot_.steering_angle.fr)) +
              fdb_robot_.wheel_speed.bl *
                  sinf(math::deg2rad(fdb_robot_.steering_angle.bl)) +
              fdb_robot_.wheel_speed.br *
                  sinf(math::deg2rad(fdb_robot_.steering_angle.br))) /
             4.0f / 360.0f * (PI * 2 * wheel_radius);

  chassis_fdb_spd_.vx = vx;
  chassis_fdb_spd_.vy = vy;

  // 轮速 dps 转换为底盘角速度 rpm
  float wz = (fdb_robot_.wheel_speed.fl *
                  sinf(math::deg2rad(fdb_robot_.steering_angle.fl - 45.0f)) +
              fdb_robot_.wheel_speed.fr *
                  sinf(math::deg2rad(fdb_robot_.steering_angle.fr + 45.0f)) +
              fdb_robot_.wheel_speed.bl *
                  sinf(math::deg2rad(fdb_robot_.steering_angle.bl - 135.0f)) +
              fdb_robot_.wheel_speed.br *
                  sinf(math::deg2rad(fdb_robot_.steering_angle.br + 135.0f))) /
             4.0f / 360.0f * wheel_radius / (wheel_base / sqrtf(2.0f)) * 60.0f;

  fdb_spd.wz = wz;
  chassis_fdb_spd_.wz = wz;
}

void SwerveChassis::handle(void) {
  // 获取控制命令
  SubGetMessage(chassis_sub_, &chassis_cmd_rcv_);

  // 电机断连处理
  disconnect_handle();

  // 读取光电门状态，航向电机复位控制
  if (type_ == Motor::M3508) {
    steering_handle();
  }

  // 电机反馈值更新
  motor_feedback_update();

  // 正运动学解算，通过反馈轮速
  fkine();

  // 云台坐标系到底盘坐标系转换
  CoordinateTransformation();

  // 逆运动学解算
  ikine();

  // 逆运动学解算，通过底盘坐标系目标速度解算出每个轮子的线速度和航向电机角度
  power_limit.handle(extra_power_max);

  // 设置电机控制量
  motor_control();
}

void SwerveChassis::CoordinateTransformation() {
  chassis_ref_spd_.vx = ref_spd.vx * cosf(math::deg2rad(fdb_spd.angle)) -
                        ref_spd.vy * sinf(math::deg2rad(fdb_spd.angle));
  chassis_ref_spd_.vy = ref_spd.vx * sinf(math::deg2rad(fdb_spd.angle)) +
                        ref_spd.vy * cosf(math::deg2rad(fdb_spd.angle));

  fdb_spd.vx = chassis_fdb_spd_.vx * cosf(math::deg2rad(fdb_spd.angle)) +
               chassis_fdb_spd_.vy * sinf(math::deg2rad(fdb_spd.angle));
  fdb_spd.vy = chassis_fdb_spd_.vy * cosf(math::deg2rad(fdb_spd.angle)) -
               chassis_fdb_spd_.vx * sinf(math::deg2rad(fdb_spd.angle));
}