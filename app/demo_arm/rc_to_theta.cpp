#include "../../base/remote/remote.h"
#include "rc_to_theta.h"
#include "arm.h"

#include <cmath>
#include <math.h>
extern RC rc;

Pose_matrix Pose_Matrix_Multiply(Pose_matrix* m,Pose_matrix* n)//4阶位姿
{
    Pose_matrix ans;

    ans.a11=m->a11*n->a11+m->a12*n->a21+m->a13*n->a31;
    ans.a12=m->a11*n->a12+m->a12*n->a22+m->a13*n->a32;
    ans.a13=m->a11*n->a13+m->a12*n->a23+m->a13*n->a33;
    ans.a21=m->a21*n->a11+m->a22*n->a21+m->a23*n->a31;
    ans.a22=m->a21*n->a12+m->a22*n->a22+m->a23*n->a32;
    ans.a23=m->a21*n->a13+m->a22*n->a23+m->a23*n->a33;
    ans.a31=m->a31*n->a11+m->a32*n->a21+m->a33*n->a31;
    ans.a32=m->a31*n->a12+m->a32*n->a22+m->a33*n->a32;
    ans.a33=m->a31*n->a13+m->a32*n->a23+m->a33*n->a33;

    ans.x=m->a11*n->x+m->a12*n->y+m->a13*n->z+m->x;
    ans.y=m->a21*n->x+m->a22*n->y+m->a23*n->z+m->y;
    ans.z=m->a31*n->x+m->a32*n->y+m->a33*n->z+m->z;

    return ans;
}

Posture_matrix Posture_Matrix_Multiply(Posture_matrix* m,Posture_matrix* n)//3阶姿态
{
    Posture_matrix ans;

    ans.a11=m->a11*n->a11+m->a12*n->a21+m->a13*n->a31;
    ans.a12=m->a11*n->a12+m->a12*n->a22+m->a13*n->a32;
    ans.a13=m->a11*n->a13+m->a12*n->a23+m->a13*n->a33;
    ans.a21=m->a21*n->a11+m->a22*n->a21+m->a23*n->a31;
    ans.a22=m->a21*n->a12+m->a22*n->a22+m->a23*n->a32;
    ans.a23=m->a21*n->a13+m->a22*n->a23+m->a23*n->a33;
    ans.a31=m->a31*n->a11+m->a32*n->a21+m->a33*n->a31;
    ans.a32=m->a31*n->a12+m->a32*n->a22+m->a33*n->a32;
    ans.a33=m->a31*n->a13+m->a32*n->a23+m->a33*n->a33;

    return ans;
}

Posture_matrix Posture_Matrix_Tr(Posture_matrix* m)//3阶姿态
{
    Posture_matrix ans;

    ans.a11=m->a11;
    ans.a12=m->a21;
    ans.a13=m->a31;
    ans.a21=m->a12;
    ans.a22=m->a22;
    ans.a23=m->a32;
    ans.a31=m->a13;
    ans.a32=m->a23;
    ans.a33=m->a33;
    return ans;
}
