//
// Created by 98383 on 24-10-8.
//

#include "chassis.h"

void Chassis::SetVx(float vx_) { // 考虑加低通滤波
    ref_.vx = vx_;
}

void Chassis::SetVy(float vy_) {
    ref_.vy = vy_;
}

void Chassis::SetWz(float wz_) {
    ref_.wz = wz_;
}

void Chassis::SetSpeed(float vx_, float vy_, float wz_) {
    SetVx(vx_);
    SetVy(vy_);
    SetWz(wz_);
}

void Chassis::SetAngle(float angle_) {
    ref_.angle = angle_;
}
