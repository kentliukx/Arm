//
// Created by 98383 on 24-10-10.
//

#include "MecanumChassis.h"

void MecanumChassis::ikine(void) {
    // 机器人坐标系目标状态->底盘坐标系目标状态
    chassis_ref_.vx = ref_.vx * cosf(math::deg2rad(fdb_.angle)) +
                      ref_.vy * sinf(math::deg2rad(fdb_.angle)) +
                      math::dps2radps(ref_.wz) * y_bias;
    chassis_ref_.vy = -ref_.vx * sinf(math::deg2rad(fdb_.angle)) +
                      ref_.vy * cosf(math::deg2rad(fdb_.angle)) -
                      math::dps2radps(ref_.wz) * x_bias;
    chassis_ref_.wz = ref_.wz;
    // 底盘坐标系目标状态->电机转速
    float vx = chassis_ref_.vx;
    float vy = chassis_ref_.vy;
    float wv =
            math::dps2radps(chassis_ref_.wz) * (half_track_width + half_wheel_base);
    wheel_ref_.fl = math::radps2dps((-wv + vx - vy) / wheel_radius);
    wheel_ref_.fr = math::radps2dps((-wv - vx - vy) / wheel_radius);
    wheel_ref_.bl = math::radps2dps((-wv + vx + vy) / wheel_radius);
    wheel_ref_.br = math::radps2dps((-wv - vx + vy) / wheel_radius);
}

void MecanumChassis::fkine(void) {
    // 电机转速->底盘坐标系反馈状态
    float rv_fl = wheel_fdb_.fl;
    float rv_fr = wheel_fdb_.fr;
    float rv_bl = wheel_fdb_.bl;
    float rv_br = wheel_fdb_.br;
    chassis_fdb_.vx =
            0.25f * math::dps2radps(rv_fl - rv_fr + rv_bl - rv_br) * wheel_radius;
    chassis_fdb_.vy =
            0.25f * math::dps2radps(-rv_fl - rv_fr + rv_bl + rv_br) * wheel_radius;
    chassis_fdb_.wz = 0.25f * (-rv_fl - rv_fr - rv_bl - rv_br) * wheel_radius /
                      (half_track_width + half_wheel_base);
    chassis_fdb_.angle = chassis_fdb_.angle + chassis_fdb_.wz * 1e-3f;
    // 底盘坐标系反馈状态->机器人坐标系反馈状态
    fdb_.vx = (chassis_fdb_.vx - math::dps2radps(chassis_fdb_.wz) * y_bias) *
                      cosf(math::deg2rad(fdb_.angle)) -
              (chassis_fdb_.vy - math::dps2radps(chassis_fdb_.wz) * x_bias) *
                      sinf(math::deg2rad(fdb_.angle));
    fdb_.vy = (chassis_fdb_.vx - math::dps2radps(chassis_fdb_.wz) * y_bias) *
                      sinf(math::deg2rad(fdb_.angle)) +
              (chassis_fdb_.vy - math::dps2radps(chassis_fdb_.wz) * x_bias) *
                      cosf(math::deg2rad(fdb_.angle));
    fdb_.wz = chassis_fdb_.wz;
}

void MecanumChassis::handle(void) {
    // 获取电机反馈数据
    // SubGetMessage();
}

void MecanumChassis::RotateControl(void) {

}