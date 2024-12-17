//
// Created by h on 2024/12/16.
//

#include "arm.h"

#include <stm32f4xx_hal.h>
#include <tgmath.h>
#include <base/remote/remote.h>

#include "rc_to_theta.h"
#include "base/monitor/motor_monitor.h"


//pid 待调
Motor m1(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(200, 0, 0, 1000, 16384), Motor::None, 0);
Motor m2(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(200, 0, 0, 1000, 16384), Motor::None, 0);
Motor m3(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(200, 0, 0, 1000, 16384), Motor::None, 0);
Motor m4(Motor::M3508, 19.2, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(20, 0, 0, 1000, 16384), Motor::None, 0);
Motor m5(Motor::M3508, 1, Motor::POSITION_SPEED,
        PID(10, 0, 0, 1000, 5000),
        PID(10, 0, 0, 1000, 5000), Motor::None, 0);
Motor m6(Motor::M3508, 1, Motor::POSITION_SPEED,
        PID(10, 0, 0, 1000, 5000),
        PID(10, 0, 0, 1000, 5000), Motor::None, 0);




Arm arm(m1, m2, m3, m4, m5, m6);



void Trajectory::handle() {
    start_ticks = HAL_GetTick();
    sigma = (float)(HAL_GetTick() - start_ticks) / (float)ticks;
    for (int i = 0; i < 6; ++i) {
        interpolate_joint.q[i] = sigma * end_joint.q[i] + (1 - sigma) * start_joint.q[i];
    }
}

void Arm::ikine(Pose &ref_pose, Joint &ref_joint) {
  float dist=ref_pose.x*ref_pose.x+ref_pose.y*ref_pose.y+ref_pose.z*ref_pose.z;//距离过大时，指向要达到的那个点
  float max_dist=0.99*(l1+l2)*(l1+l2);
  float control_x=ref_pose.x,control_y=ref_pose.y,control_z=ref_pose.z;
  if(dist>max_dist) {
    control_x=ref_pose.x*max_dist/dist;
    control_y=ref_pose.y*max_dist/dist;
    control_z=ref_pose.z*max_dist/dist;
  }

  if(ref_pose.x<0.01&&ref_pose.x>-0.01) control_x=0.01;
  if(ref_pose.y<0.01&&ref_pose.y>-0.01) control_y=0.01;
  if(ref_pose.z<0.01&&ref_pose.z>-0.01) control_z=0.01;

  ref_joint.q[0] = atan2(control_y, control_x);
  ref_joint.q[1] =3.1415926-atan2(control_z, sqrt(control_x*control_x+control_y*control_y))
       - acos((-l2*l2+l1*l1+control_x*control_x+control_y*control_y+control_z*control_z)
           /(2*l1*sqrt(control_x*control_x+control_y*control_y+control_z*control_z)));

  ref_joint.q[2] =
      acos((-control_x*control_x-control_y*control_y-control_z*control_z+l1*l1+l2*l2)/(2*l1*l2));

  //theta 1-3 and pose to theta 4-6
  Posture_matrix T60;//from in 6 to in 0
  T60.a11=cos(ref_pose.yaw)*cos(ref_pose.pitch);
  T60.a12=cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)-sin(ref_pose.yaw)*cos(ref_pose.roll);
  T60.a13=cos(ref_pose.yaw)*sin(ref_pose.pitch)*cos(ref_pose.roll)+sin(ref_pose.yaw)*sin(ref_pose.roll);
  T60.a21=sin(ref_pose.yaw)*cos(ref_pose.pitch);
  T60.a22=sin(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)+cos(ref_pose.yaw)*cos(ref_pose.roll);
  T60.a23=sin(ref_pose.yaw)*sin(ref_pose.pitch)*cos(ref_pose.roll)-cos(ref_pose.yaw)*sin(ref_pose.roll);
  T60.a31=-sin(ref_pose.pitch);
  T60.a32=cos(ref_pose.pitch)*sin(ref_pose.roll);
  T60.a33=cos(ref_pose.pitch)*cos(ref_pose.roll);




  Posture_matrix T31;
  T31.a11=cos(ref_joint.q[2]-ref_joint.q[1]);
  T31.a12=-sin(ref_joint.q[2]-ref_joint.q[1]);
  T31.a13=0;
  T31.a21=sin(ref_joint.q[2]-ref_joint.q[1]);
  T31.a22=cos(ref_joint.q[2]-ref_joint.q[1]);
  T31.a23=0;
  T31.a31=0;
  T31.a32=0;
  T31.a33=1;

  Posture_matrix T10;
  T10.a11=cos(ref_joint.q[0]);
  T10.a12=0;
  T10.a13=sin(ref_joint.q[0]);
  T10.a21=0;
  T10.a22=1;
  T10.a23=0;
  T10.a31=-sin(ref_joint.q[0]);
  T10.a32=0;
  T10.a33=cos(ref_joint.q[0]);

  Posture_matrix T30=Posture_Matrix_Multiply(&T10,&T31);

  Posture_matrix T03=Posture_Matrix_Tr(&T30);

  Posture_matrix T63=Posture_Matrix_Multiply(&T03,&T60);

  //caculate X'Z'X' back
  ref_joint.q[3] = atan2(T63.a31,T63.a21);
  ref_joint.q[5] = atan2(T63.a13,-T63.a12);
  ref_joint.q[4] = atan2(T63.a21,T63.a11*cos(ref_joint.q[3]));
}

#define ANGLE_INCREMENT 0.1
#define PITCH_YAW_INCREMENT 0.01
extern RC rc;
void Arm::updateRefPose() {
    if (rc.switch_.l == 0) {
        ref_pose.x += rc.channel_.r_col * ANGLE_INCREMENT;
        ref_pose.y -= rc.channel_.r_row * ANGLE_INCREMENT;
        ref_pose.z += rc.channel_.l_col * ANGLE_INCREMENT;
    } else if (rc.switch_.l == 1) {
        ref_pose.pitch += rc.channel_.r_col * PITCH_YAW_INCREMENT;
        ref_pose.yaw += rc.channel_.r_row * PITCH_YAW_INCREMENT;
        ref_pose.roll += rc.channel_.l_row * PITCH_YAW_INCREMENT;
    }
}

Joint ref_joint;
void Arm::handle() {

    updateRefPose();

    //由末端姿态得到1-6角度

    ikine(ref_pose, ref_joint);

    //轨迹插值
    // get_joint();
    // Trajectory ref_trajectory(arm_joint, ref_joint);
    // ref_trajectory.handle();

    //设置角度

    for(int i=0; i<6; i++) ref_joint.q[i]*=57.3;

    m1.setAngle(ref_joint.q[0],0);
    m2.setAngle(-ref_joint.q[1],0);
    m3.setAngle(-ref_joint.q[2],0);
    m4.setAngle(ref_joint.q[3],0);
    m5.setAngle(-ref_joint.q[4],0);
    m6.setAngle(ref_joint.q[5],0);
}

