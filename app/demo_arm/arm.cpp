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
        PID(150, 0, 0, 1000, 16384), Motor::None, 0);
Motor m2(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(150, 0, 0, 1000, 16384), Motor::None, 0);
Motor m3(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(150, 0, 0, 1000, 16384), Motor::None, 0);
Motor m4(Motor::M3508, 19.2, Motor::POSITION_SPEED,
        PID(10, 0, 0, 1000, 5000),
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

Posture_matrix T63,T30,T60,T31,T10;

void Arm::ikine(){
  float dist=sqrt(ref_pose.x*ref_pose.x+ref_pose.y*ref_pose.y+ref_pose.z*ref_pose.z);//距离过大时，指向要达到的那个点
  float max_dist=0.99*(l1+l2);
  float control_x,control_y,control_z;

  control_x=ref_pose.x;
  control_y=ref_pose.y;
  control_z=ref_pose.z;

  if(dist>max_dist) {
    control_x=ref_pose.x*max_dist/dist;
    control_y=ref_pose.y*max_dist/dist;
    control_z=ref_pose.z*max_dist/dist;
  }

  if(control_x<0.01&&control_x>-0.01) control_x=0.01;
  if(control_y<0.01&&control_y>-0.01) control_y=0.01;
  if(control_z<0.01&&control_z>-0.01) control_z=0.01;

  ref_joint.q[0] = atan2(control_y, control_x);
  ref_joint.q[1] =3.1415926-atan2(control_z, sqrt(control_x*control_x+control_y*control_y))
       - acos((-l2*l2+l1*l1+control_x*control_x+control_y*control_y+control_z*control_z)
           /(2*l1*sqrt(control_x*control_x+control_y*control_y+control_z*control_z)));

  ref_joint.q[2] =
      acos((-control_x*control_x-control_y*control_y-control_z*control_z+l1*l1+l2*l2)/(2*l1*l2));

    //from in 6 to in 0
    /*

        Z   Y
        |  /
        | /
        |/
        .______ X

    coord 0, in which target x,y,z and roll,pitch,yaw are defined

       ___________\()/ ~~[Ouch!]
        \)q[2]     ||
         \        /  \  <-lkx
          \
       q[1]\
     _ _ _(_\_____

    how q[1] and q[2] are defined

    */

    //q0~2 and rpy to q4~6
    T60.a11=cos(ref_pose.yaw)*cos(ref_pose.pitch);
    T60.a12=cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)-sin(ref_pose.yaw)*cos(ref_pose.roll);
    T60.a13=cos(ref_pose.yaw)*sin(ref_pose.pitch)*cos(ref_pose.roll)+sin(ref_pose.yaw)*sin(ref_pose.roll);
    T60.a21=sin(ref_pose.yaw)*cos(ref_pose.pitch);
    T60.a22=sin(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)+cos(ref_pose.yaw)*cos(ref_pose.roll);
    T60.a23=sin(ref_pose.yaw)*sin(ref_pose.pitch)*cos(ref_pose.roll)-cos(ref_pose.yaw)*sin(ref_pose.roll);
    T60.a31=-sin(ref_pose.pitch);
    T60.a32=cos(ref_pose.pitch)*sin(ref_pose.roll);
    T60.a33=cos(ref_pose.pitch)*cos(ref_pose.roll);


    T31.a11=cos(ref_joint.q[1]-ref_joint.q[2]);
    T31.a12=0;
    T31.a13=sin(ref_joint.q[1]-ref_joint.q[2]);
    T31.a21=0;
    T31.a22=1;
    T31.a23=0;
    T31.a31=-sin(ref_joint.q[1]-ref_joint.q[2]);
    T31.a32=0;
    T31.a33=cos(ref_joint.q[1]-ref_joint.q[2]);

    T10.a11=cos(ref_joint.q[0]);
    T10.a12=-sin(ref_joint.q[0]);
    T10.a13=0;
    T10.a21=sin(ref_joint.q[0]);
    T10.a22=cos(ref_joint.q[0]);
    T10.a23=0;
    T10.a31=0;
    T10.a32=0;
    T10.a33=1;

    T30=Posture_Matrix_Multiply(&T10,&T31);

    Posture_matrix T03=Posture_Matrix_Tr(&T30);

    T63=Posture_Matrix_Multiply(&T03,&T60);

    //caculate X'Y'X' back
    ref_joint.q[3] = atan2(T63.a21,-T63.a31);
    ref_joint.q[5] = atan2(T63.a12,T63.a13);
    ref_joint.q[4] = acos(T63.a11);




  ref_joint.q[1]*=-1;
  ref_joint.q[2]*=-1;

    for(int i=0; i<6; i++) ref_joint.q[i]*=57.295781;
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
    } else if (rc.switch_.l == 2) {
        ref_pose.x=1;
        ref_pose.y=0;
        ref_pose.z=10;
        ref_pose.pitch=0;
        ref_pose.yaw=0;
        ref_pose.roll=0;
    }
}
void Arm::get_joint(){
  m1.control_data_.fdb_angle=math::degNormalize180(m1.control_data_.fdb_angle);
  m2.control_data_.fdb_angle=math::degNormalize180(m2.control_data_.fdb_angle);
  m3.control_data_.fdb_angle=math::degNormalize180(m3.control_data_.fdb_angle);
  m4.control_data_.fdb_angle=math::degNormalize180(m4.control_data_.fdb_angle);
  m5.control_data_.fdb_angle=math::degNormalize180(m5.control_data_.fdb_angle);
  m6.control_data_.fdb_angle=math::degNormalize180(m6.control_data_.fdb_angle);

  arm_joint.q[0]=m1.control_data_.fdb_angle;
  arm_joint.q[1]=m2.control_data_.fdb_angle;
  arm_joint.q[2]=m3.control_data_.fdb_angle;
  arm_joint.q[3]=m4.control_data_.fdb_angle;
  arm_joint.q[4]=m5.control_data_.fdb_angle;
  arm_joint.q[5]=m6.control_data_.fdb_angle;
}

void Arm::calc_ctrl_Joint(){
    for (int i=0; i<6; i++) {
      if(abs(arm_joint.q[i]-(ref_joint.q[i]-360))<abs(arm_joint.q[i]-ref_joint.q[i])) {
        ctrl_joint.q[i]=ref_joint.q[i]-360;
      }
      else if(abs(arm_joint.q[i]-(ref_joint.q[i]+360))<abs(arm_joint.q[i]-ref_joint.q[i])) {
        ctrl_joint.q[i]=ref_joint.q[i]+360;
      }
      else ctrl_joint.q[i]=ref_joint.q[i];
    }
}

void Arm::set_joint_angle(){
    m1.setAngle(ctrl_joint.q[0],0);
    m2.setAngle(ctrl_joint.q[1],0);
    m3.setAngle(ctrl_joint.q[2],0);
    m4.setAngle(ctrl_joint.q[3],0);
    m5.setAngle(ctrl_joint.q[4],0);
    m6.setAngle(ctrl_joint.q[5],0);
}

void Arm::handle() {

    updateRefPose();

    get_joint();

    ikine();

    // Trajectory ref_trajectory(arm_joint, ref_joint);
    // ref_trajectory.handle();

    calc_ctrl_Joint();

    set_joint_angle();
}

