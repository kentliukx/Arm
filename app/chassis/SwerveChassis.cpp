//
// Created by 98383 on 24-10-15.
//

#include "SwerveChassis.h"

float vector_sum(float x_component, float y_component) {
  return sqrtf(powf(x_component, 2) + powf(y_component, 2));
}

void SwerveChassis::ikine(void) {
  float& vx = chassis_ref_.vx;
  float& vy = chassis_ref_.vy;
  float& wz = chassis_ref_.wz;

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

  chassis_fdb_.vx = vx;
  chassis_fdb_.vy = vy;

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

  fdb_.wz = wz;
  chassis_fdb_.wz = wz;
}

void SwerveChassis::handle(void) {}

void SwerveChassis::RotateControl(float fdb_angle, float follow_fdb_angle) {}

void SwerveChassis::CoordinateTransformation() {
  chassis_ref_.vx = ref_.vx * cosf(math::deg2rad(fdb_.angle)) -
                    ref_.vy * sinf(math::deg2rad(fdb_.angle));
  chassis_ref_.vy = ref_.vx * sinf(math::deg2rad(fdb_.angle)) +
                    ref_.vy * cosf(math::deg2rad(fdb_.angle));

  fdb_.vx = chassis_fdb_.vx * cosf(math::deg2rad(fdb_.angle)) +
            chassis_fdb_.vy * sinf(math::deg2rad(fdb_.angle));
  fdb_.vy = chassis_fdb_.vy * cosf(math::deg2rad(fdb_.angle)) -
            chassis_fdb_.vx * sinf(math::deg2rad(fdb_.angle));
}