//
//  EigenHelper.h
//  ekf_final
//
//  Created by 谭智丹 on 17/2/7.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#ifndef EigenHelper_h
#define EigenHelper_h

#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef Eigen::Vector2f EgV2f;
typedef Eigen::Vector3f EgV3f;
typedef Eigen::Vector4f EgV4f;
typedef Eigen::VectorXf EgVxf;

typedef Eigen::Vector2d EgV2d;
typedef Eigen::Vector3d EgV3d;
typedef Eigen::Vector4d EgV4d;
typedef Eigen::VectorXd EgVxd;

typedef Eigen::Matrix3f EgM3f;
typedef Eigen::Matrix4f EgM4f;
typedef Eigen::MatrixXf EgMxf;

typedef Eigen::Matrix3d EgM3d;
typedef Eigen::Matrix4d EgM4d;
typedef Eigen::MatrixXd EgMxd;

typedef Eigen::Quaterniond EgQuatd;



#endif /* EigenHelper_h */
