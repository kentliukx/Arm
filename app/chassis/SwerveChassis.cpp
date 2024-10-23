//
// Created by 98383 on 24-10-15.
//

#include "SwerveChassis.h"

float vector_sum(float x_component, float y_component) {
  return sqrtf(powf(x_component, 2) + powf(y_component, 2));
}

SwerveChassis::SwerveChassis(Motor* CMFL, Motor* CMFR, Motor* CMBL, Motor* CMBR,
                             Motor* STFL, Motor* STFR, Motor* STBL, Motor* STBR,
                             LowPassFilter speed_filter, PID angle_pid)
    : CMFL_(CMFL),
      CMFR_(CMFR),
      CMBL_(CMBL),
      CMBR_(CMBR),
      STFL_(STFL, STEERING_OFFSET.FL),
      STFR_(STFR, STEERING_OFFSET.FR),
      STBL_(STBL, STEERING_OFFSET.BL),
      STBR_(STBR, STEERING_OFFSET.BR),
      type_(Motor::GM6020),
      // #endif
      vx_filter_(speed_filter),
      vy_filter_(speed_filter),
      angle_pid_(angle_pid),
      follow_filter_(LowPassFilter(0.1f)) {
  mode_ = Lock;
  chassis_pub_ = PubRegister("chassis_fdb", sizeof(ChassisCtrlCmd));
  chassis_sub_ = SubRegister("chassis_cmd", sizeof(ChassisCtrlCmd));
}

void SwerveChassis::ikine(void) {
  float& vx = ref_spd.vx;
  float& vy = ref_spd.vy;
  float& wz = ref_spd.wz;
  wz = wz * 60.f / 2.f / PI;

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

void SwerveChassis::SteeringHandle() {
  // 读取光电门当前状态
  STFL_.read_photogate_state();
  STFR_.read_photogate_state();
  STBL_.read_photogate_state();
  STBR_.read_photogate_state();

  // 如果航向电机还未复位，进行光电门复位
  STFL_.reset_steering_motor_handle();
  STFR_.reset_steering_motor_handle();
  STBL_.reset_steering_motor_handle();
  STBR_.reset_steering_motor_handle();
}

void SwerveChassis::DisconnectHandle() {
  // 航向电机断连处理
  if (type_ == Motor::M3508) {
    if (!STFL_.motor_->connect_.check()) {
      STFL_.reset_steering_motor();
    }
    if (!STFR_.motor_->connect_.check()) {
      STFR_.reset_steering_motor();
    }
    if (!STBL_.motor_->connect_.check()) {
      STBL_.reset_steering_motor();
    }
    if (!STBR_.motor_->connect_.check()) {
      STBR_.reset_steering_motor();
    }
  } else if (type_ == Motor::GM6020) {
    STFL_.reset_steering_motor();
    STFR_.reset_steering_motor();
    STBL_.reset_steering_motor();
    STBR_.reset_steering_motor();
  }

  // 轮电机断连处理
  if (!CMFL_->connect_.check() || !CMFR_->connect_.check() ||
      !CMBL_->connect_.check() || !CMBR_->connect_.check()) {
    chassis_lock_ = true;
  }
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

void SwerveChassis::MotorFeedbackUpdate() {
  // 更新轮速反馈
  SwerveStatus_t feedback_temp;

  feedback_temp.wheel_speed.fl = CMFL_->realSpeed();
  feedback_temp.wheel_speed.fr = CMFR_->realSpeed();
  feedback_temp.wheel_speed.bl = CMBL_->realSpeed();
  feedback_temp.wheel_speed.br = CMBR_->realSpeed();

  fdb_chassis_.wheel_speed = feedback_temp.wheel_speed;
  fdb_robot_.wheel_speed = feedback_temp.wheel_speed;

  // 规范化航向电机反馈角度
  STFL_.motor_->resetFeedbackAngle(
      math::degNormalize180(STFL_.motor_->control_data_.fdb_angle));
  STFR_.motor_->resetFeedbackAngle(
      math::degNormalize180(STFR_.motor_->control_data_.fdb_angle));
  STBL_.motor_->resetFeedbackAngle(
      math::degNormalize180(STBL_.motor_->control_data_.fdb_angle));
  STBR_.motor_->resetFeedbackAngle(
      math::degNormalize180(STBR_.motor_->control_data_.fdb_angle));

  // 更新航向电机角度反馈
  feedback_temp.steering_angle.fl = STFL_.motor_->control_data_.fdb_angle;
  feedback_temp.steering_angle.fr = STFR_.motor_->control_data_.fdb_angle;
  feedback_temp.steering_angle.bl = STBL_.motor_->control_data_.fdb_angle;
  feedback_temp.steering_angle.br = STBR_.motor_->control_data_.fdb_angle;

  fdb_chassis_.steering_angle = feedback_temp.steering_angle;
  fdb_robot_.steering_angle = feedback_temp.steering_angle;

  // 更新云台-底盘夹角反馈、夹角做预测（解决陀螺平移走偏问题）
  //  float chassis_rotate_speed = math::rpm2dps(fdb_chassis_.wz);
  //  fdb_robot_.gimbal_chassis_angle =
  //      math::degNormalize180((GMY.motor_data_.ecd_angle - yaw_zero_ecd) /
  //                            GMY.ratio_) +
  //      chassis_rotate_speed * -0.03f;
  //  fdb_chassis_.gimbal_chassis_angle =
  //      math::degNormalize180((GMY.motor_data_.ecd_angle - yaw_zero_ecd) /
  //                            GMY.ratio_) +
  //      chassis_rotate_speed * -0.03f;
}

void SwerveChassis::MotorControl() {
  if (type_ == Motor::M3508) {
    if (!(STFL_.if_reset_done() && STFR_.if_reset_done() &&
          STBL_.if_reset_done() && STBR_.if_reset_done())) {
      mode_ = ChassisMode_e::Lock;
      LOGINFO("Steer motor disconnected. Chassis locked.");
    }
  }

  STFL_.motor_->setAngle(ref_chassis_.steering_angle.fl);
  STFR_.motor_->setAngle(ref_chassis_.steering_angle.fr);
  STBL_.motor_->setAngle(ref_chassis_.steering_angle.bl);
  STBR_.motor_->setAngle(ref_chassis_.steering_angle.br);

  if (mode_ == Lock) {
    CMFL_->setSpeed(0);
    CMFR_->setSpeed(0);
    CMBL_->setSpeed(0);
    CMBR_->setSpeed(0);
  } else {
    CMFL_->setSpeed(ref_chassis_.wheel_speed.fl);
    CMFR_->setSpeed(ref_chassis_.wheel_speed.fr);
    CMBL_->setSpeed(ref_chassis_.wheel_speed.bl);
    CMBR_->setSpeed(ref_chassis_.wheel_speed.br);
  }
}

void SwerveChassis::handle(void) {
  // 获取控制命令
  SubGetMessage(chassis_sub_, &chassis_cmd_rcv_);
  SetSpeed(chassis_cmd_rcv_.vx, chassis_cmd_rcv_.vy, chassis_cmd_rcv_.wz);
  SetAngle(chassis_cmd_rcv_.ref_angle);
  if (mode_ != chassis_cmd_rcv_.mode_) {
    LOGINFO("Set chassis mode from %d to %d", mode_, chassis_cmd_rcv_.mode_);
    mode_ = chassis_cmd_rcv_.mode_;
  }

  // 电机断连处理
  DisconnectHandle();

  // 读取光电门状态，航向电机复位控制
  if (type_ == Motor::M3508) {
    SteeringHandle();
  }

  // 电机反馈值更新
  MotorFeedbackUpdate();

  // 正运动学解算，通过反馈轮速
  fkine();

  // 云台坐标系到底盘坐标系转换
  CoordinateTransformation();

  // 逆运动学解算
  ikine();

  // 底盘功率限制
  //  power_limit.handle(extra_power_max);

  // 设置电机控制量
  MotorControl();
}
