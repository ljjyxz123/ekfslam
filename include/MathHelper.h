//
//  MathHelper.h
//  ekf_final
//
//  Created by 谭智丹 on 17/2/8.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#ifndef MathHelper_h
#define MathHelper_h

#include <stdio.h>
#include <Eigen/Dense>

void vec2quat(Eigen::Vector3d& v, Eigen::Quaterniond& q)
{
    double normv = v.norm();
    
    if (normv < 1e-7) {
        q.setIdentity();
    } else {
        double hnormv = normv / 2.0;
        q.w() = cos(hnormv);
        q.vec() = (sin(hnormv)/normv) * v;
    }
    
}


void dqp_by_dq(Eigen::Quaterniond& p, Eigen::Matrix4d& m)
{
    double p1, p2, p3, p4;
    p1 = p.w();
    p2 = p.x();
    p3 = p.y();
    p4 = p.z();
    
    m <<  p1,  -p2,  -p3,  -p4,
    p2,   p1,   p4,  -p3,
    p3,  -p4,   p1,   p2,
    p4,   p3,  -p2,   p1;
}


void dqp_by_dp(Eigen::Quaterniond& q, Eigen::Matrix4d& m)
{
    double q1, q2, q3, q4;
    q1 = q.w();
    q2 = q.x();
    q3 = q.y();
    q4 = q.z();
    
    m << q1,  -q2,  -q3,  -q4,
    q2,   q1,  -q4,   q3,
    q3,   q4,   q1,  -q2,
    q4,  -q3,   q2,   q1;
}


void dq_by_dv(Eigen::Vector3d& v, Eigen::Matrix<double,4,3>& m)
{
    double normv = v.norm();
    double invv = 1.0 / normv;
    double invv2 = invv * invv;
    
    double sv = sin(normv/2.0);
    double cv = cos(normv/2.0);
    
    Eigen::Matrix3d a = Eigen::Matrix3d::Identity();
    for (int i=0; i<3; i++) {
        for (int j=i; j<3; j++) {
            double vij_by_v2 = v(i) * v(j) * invv2;
            a(i,j) = 0.5*cv*vij_by_v2 + sv*invv*(a(i,j)-vij_by_v2);
            a(j,i) = a(i,j);
        }
    }
    
    Eigen::Vector3d tempv = -0.5 * sv * invv * v;
    m << tempv.transpose(),
    a;
    
}


void dR_by_dq(Eigen::Quaterniond& q, Eigen::Matrix3d& m0, Eigen::Matrix3d& mx, Eigen::Matrix3d& my, Eigen::Matrix3d& mz)
{
    // Notice that R : R_Wc, q: q_Cw
    double q0 = 2 * q.w();
    double qx = 2 * q.x();
    double qy = 2 * q.y();
    double qz = 2 * q.z();
    
    m0 << q0,  -qz,   qy,
    qz,   q0,  -qx,
    -qy,   qx,   q0;
    
    mx << qx,  qy,   qz,
    qy,  -qx, -q0,
    qz,  q0,  -qx;
    
    my << -qy,  qx,  q0,
    qx,  qy,  qz,
    -q0,  qz, -qy;
    
    mz << -qz, -q0, qx,
    q0, -qz, qy,
    qx,  qy, qz;
}


#endif /* MathHelper_h */
