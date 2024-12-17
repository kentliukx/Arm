//
// Created by h on 2024/12/16.
//

#include "arm.h"

#include <stm32f4xx_hal.h>
#include <tgmath.h>
#include <base/remote/remote.h>

#include "rc_to_theta.h"


//pid 待调
Motor m1(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(0, 0, 0, 1000, 1000),
        PID(0, 0, 0, 1000, 1000), Motor::None, 0);
Motor m2(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(0, 0, 0, 1000, 1000),
        PID(0, 0, 0, 1000, 1000), Motor::None, 0);
Motor m3(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(0, 0, 0, 1000, 1000),
        PID(0, 0, 0, 1000, 1000), Motor::None, 0);
Motor m4(Motor::M3508, 19.2, Motor::POSITION_SPEED,
        PID(0, 0, 0, 1000, 1000),
        PID(0, 0, 0, 1000, 1000), Motor::None, 0);
Motor m5(Motor::M3508, 1, Motor::POSITION_SPEED,
        PID(0, 0, 0, 1000, 1000),
        PID(0, 0, 0, 1000, 1000), Motor::None, 0);
Motor m6(Motor::M3508, 1, Motor::POSITION_SPEED,
        PID(0, 0, 0, 1000, 1000),
        PID(0, 0, 0, 1000, 1000), Motor::None, 0);


uint16_t can_ids[6] = {0x201, 0x202, 0x203, 0x204, 0x205, 0x206};
Arm arm(m1, m2, m3, m4, m5, m6, can_ids);



void Trajectory::handle() {
    start_ticks = HAL_GetTick();
    sigma = (float)(HAL_GetTick() - start_ticks) / (float)ticks;
    for (int i = 0; i < 6; ++i) {
        interpolate_joint.q[i] = sigma * end_joint.q[i] + (1 - sigma) * start_joint.q[i];
    }
}

void Arm::canId(uint16_t can_id[6]) {
    m1.CANIdConfig(1,can_id[0]);
    m2.CANIdConfig(1,can_id[1]);
    m3.CANIdConfig(1,can_id[2]);
    m4.CANIdConfig(1,can_id[3]);
    m5.CANIdConfig(1,can_id[4]);
    m6.CANIdConfig(1,can_id[5]);
}

void Arm::ikine(Pose &ref_pose, Joint &ref_joint) {
    //to do
    //
    //
    //position to theta 1-3
    ref_joint.q[0] = atan2(-ref_pose.y, ref_pose.x);
    ref_joint.q[1] =
        ref_joint.q[0]
         + acos((l1*l1-l2*l2-ref_pose.x*ref_pose.x-ref_pose.y*ref_pose.y-ref_pose.z*ref_pose.z)
             /2*l2*sqrt(ref_pose.x*ref_pose.x+ref_pose.y*ref_pose.y+ref_pose.z*ref_pose.z))
         - atan2(ref_pose.z, sqrt(ref_pose.x*ref_pose.x+ref_pose.y*ref_pose.y));
    ref_joint.q[2] =
        acos(ref_pose.x*ref_pose.x+ref_pose.y*ref_pose.y+ref_pose.z*ref_pose.z-l1*l1-l2*l2)/(2*l1*l2);
    ref_joint.q[3] = 0;
    ref_joint.q[4] = 0;
    ref_joint.q[5] = 0;

    //theta 1-3 and pose to theta 4-6

    Posture_matrix T70;//from in 7 to in 0
    T70.a11=cos(ref_pose.yaw)*cos(ref_pose.pitch);
    T70.a12=cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)-sin(ref_pose.yaw)*cos(ref_pose.roll);
    T70.a13=cos(ref_pose.yaw)*sin(ref_pose.pitch)*cos(ref_pose.roll)+sin(ref_pose.yaw)*sin(ref_pose.roll);
    T70.a21=sin(ref_pose.yaw)*cos(ref_pose.pitch);
    T70.a22=sin(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)+cos(ref_pose.yaw)*cos(ref_pose.roll);
    T70.a23=sin(ref_pose.yaw)*sin(ref_pose.pitch)*cos(ref_pose.roll)-cos(ref_pose.yaw)*sin(ref_pose.roll);
    T70.a31=-sin(ref_pose.pitch);
    T70.a32=cos(ref_pose.pitch)*sin(ref_pose.roll);
    T70.a33=cos(ref_pose.pitch)*cos(ref_pose.roll);


    Posture_matrix T41;
    T41.a11=cos(ref_joint.q[1]-ref_joint.q[0]);
    T41.a12=0;
    T41.a13=-sin(ref_joint.q[1]-ref_joint.q[0]);
    T41.a21=0;
    T41.a22=1;
    T41.a23=0;
    T41.a31=sin(ref_joint.q[1]-ref_joint.q[0]);
    T41.a32=0;
    T41.a33=cos(ref_joint.q[1]-ref_joint.q[0]);

    Posture_matrix T10;
    T10.a11=cos(ref_joint.q[0]);
    T10.a12=sin(ref_joint.q[0]);
    T10.a13=0;
    T10.a21=-sin(ref_joint.q[0]);
    T10.a22=cos(ref_joint.q[0]);
    T10.a23=0;
    T10.a31=0;
    T10.a32=0;
    T10.a33=1;

    Posture_matrix T40=Posture_Matrix_Multiply(&T10,&T41);

    Posture_matrix T04=Posture_Matrix_Tr(&T40);

    Posture_matrix T74=Posture_Matrix_Multiply(&T04,&T70);

    ref_joint.q[4]=acos(T74.a11);
    ref_joint.q[5]=acos(-T74.a13/sqrt(1-sin(ref_joint.q[4])*sin(ref_joint.q[4])));
    ref_joint.q[3]=acos(T74.a31/sqrt(1-sin(ref_joint.q[4])*sin(ref_joint.q[4])));

}

void Arm::get_joint() {
    arm_joint.q[0] = m1.motor_data_.angle;
    arm_joint.q[1] = m2.motor_data_.angle;
    arm_joint.q[2] = m3.motor_data_.angle;
    arm_joint.q[3] = m4.motor_data_.angle;
    arm_joint.q[4] = m5.motor_data_.angle;
    arm_joint.q[5] = m6.motor_data_.angle;
}

extern RC rc;

void Arm::handle() {

    if (rc.switch_.l==0)
    {
        ref_pose.x+=rc.channel_.r_col*10;//cm
        ref_pose.y+=rc.channel_.r_row*10;
        ref_pose.z+=rc.channel_.l_col*10;
    }
    else if (rc.switch_.l==1)
    {
        ref_pose.pitch+=rc.channel_.r_col/4;//rad
        ref_pose.yaw+=rc.channel_.r_row/4;
        ref_pose.roll+=rc.channel_.l_row/4;
    }

    Joint ref_joint;

    //由末端姿态得到1-6角度
    ikine(ref_pose, ref_joint);
    get_joint();

    //轨迹插值
    Trajectory ref_trajectory(arm_joint, ref_joint);
    ref_trajectory.handle();

    //发包
    m1.setAngle(ref_trajectory.interpolate_joint.q[0], 0);
    m2.setAngle(ref_trajectory.interpolate_joint.q[1], 0);
    m3.setAngle(ref_trajectory.interpolate_joint.q[2], 0);
    m4.setAngle(ref_trajectory.interpolate_joint.q[3], 0);
    m5.setAngle(ref_trajectory.interpolate_joint.q[4], 0);
    m6.setAngle(ref_trajectory.interpolate_joint.q[5], 0);
}

