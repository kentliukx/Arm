//
// Created by 98383 on 24-10-10.
//

#include "MecanumChassis.h"

MecanumChassis::MecanumChassis() {
  chassis_pub_ = PubRegister("chassis_cmd", sizeof(ChassisCtrlCmd));
  chassis_sub_ = SubRegister("chassis_fdb", sizeof(ChassisCtrlCmd));
}

void MecanumChassis::ikine(void) {
  // 机器人坐标系目标状态->底盘坐标系目标状态
  chassis_ref_spd_.vx = ref_spd.vx * cosf(math::deg2rad(fdb_spd.angle)) +
                        ref_spd.vy * sinf(math::deg2rad(fdb_spd.angle)) +
                        math::dps2radps(ref_spd.wz) * y_bias;
  chassis_ref_spd_.vy = -ref_spd.vx * sinf(math::deg2rad(fdb_spd.angle)) +
                        ref_spd.vy * cosf(math::deg2rad(fdb_spd.angle)) -
                        math::dps2radps(ref_spd.wz) * x_bias;
  chassis_ref_spd_.wz = ref_spd.wz;
  // 底盘坐标系目标状态->电机转速
  float vx = chassis_ref_spd_.vx;
  float vy = chassis_ref_spd_.vy;
  float wv = math::dps2radps(chassis_ref_spd_.wz) *
             (half_track_width + half_wheel_base);
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
  chassis_fdb_spd_.vx =
      0.25f * math::dps2radps(rv_fl - rv_fr + rv_bl - rv_br) * wheel_radius;
  chassis_fdb_spd_.vy =
      0.25f * math::dps2radps(-rv_fl - rv_fr + rv_bl + rv_br) * wheel_radius;
  chassis_fdb_spd_.wz = 0.25f * (-rv_fl - rv_fr - rv_bl - rv_br) *
                        wheel_radius / (half_track_width + half_wheel_base);
  chassis_fdb_spd_.angle = chassis_fdb_spd_.angle + chassis_fdb_spd_.wz * 1e-3f;
  // 底盘坐标系反馈状态->机器人坐标系反馈状态
  fdb_spd.vx =
      (chassis_fdb_spd_.vx - math::dps2radps(chassis_fdb_spd_.wz) * y_bias) *
          cosf(math::deg2rad(fdb_spd.angle)) -
      (chassis_fdb_spd_.vy - math::dps2radps(chassis_fdb_spd_.wz) * x_bias) *
          sinf(math::deg2rad(fdb_spd.angle));
  fdb_spd.vy =
      (chassis_fdb_spd_.vx - math::dps2radps(chassis_fdb_spd_.wz) * y_bias) *
          sinf(math::deg2rad(fdb_spd.angle)) +
      (chassis_fdb_spd_.vy - math::dps2radps(chassis_fdb_spd_.wz) * x_bias) *
          cosf(math::deg2rad(fdb_spd.angle));
  fdb_spd.wz = chassis_fdb_spd_.wz;
}

void MecanumChassis::handle(void) {
  // 获取控制数据
  SubGetMessage(chassis_sub_, &chassis_cmd_rcv_);
  SetSpeed(chassis_cmd_rcv_.vx, chassis_cmd_rcv_.vy, chassis_cmd_rcv_.wz);
  SetAngle(chassis_cmd_rcv_.ref_angle);
  if (mode_ != chassis_cmd_rcv_.mode_) {
    LOGINFO("Set chassis mode from %d to %d", mode_, chassis_cmd_rcv_.mode_);
    mode_ = chassis_cmd_rcv_.mode_;
  }

  wheel_fdb_.fl = cmfl_->realSpeed();
  wheel_fdb_.fr = cmfr_->realSpeed();
  wheel_fdb_.bl = cmbl_->realSpeed();
  wheel_fdb_.br = cmbr_->realSpeed();

  fkine();

  ikine();

  if (mode_ != Lock) {
    cmfl_->setSpeed(wheel_ref_.fl);
    cmfr_->setSpeed(wheel_ref_.fr);
    cmbl_->setSpeed(wheel_ref_.bl);
    cmbr_->setSpeed(wheel_ref_.br);
  } else {
    cmfl_->setSpeed(0);
    cmfr_->setSpeed(0);
    cmbl_->setSpeed(0);
    cmbr_->setSpeed(0);
  }
}
