//
// Created by kentl on 2024/12/16.
//

#ifndef RC_TO_THETA_H
#define RC_TO_THETA_H

#ifdef __cplusplus
extern "C" {
#endif

struct Pose_matrix
{
    float a11, a12, a13, a21, a22, a23, a31, a32, a33, x, y, z;
};
struct Posture_matrix
{
    float a11, a12, a13, a21, a22, a23, a31, a32, a33;
};

Pose_matrix Pose_Matrix_Multiply(Pose_matrix* m,Pose_matrix* n);
Posture_matrix Posture_Matrix_Multiply(Posture_matrix* m,Posture_matrix* n);
Posture_matrix Posture_Matrix_Tr(Posture_matrix* m);






#ifdef __cplusplus
}
#endif
#endif //RC_TO_THETA_H
