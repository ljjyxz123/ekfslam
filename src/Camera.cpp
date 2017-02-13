//
//  Camera.cpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/8.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#include "Camera.hpp"

void Camera::SetIntrinsics(int width, int height, double focalx, double focaly, double ppx, double ppy)
{
    nCols = width;
    nRows = height;
    fx = focalx;
    fy = focaly;
    Cx = ppx;
    Cy = ppy;
}


void Camera::SetK(Mat &m)
{
	double mf1, mf2, mC1, mC2;
	mf1 = m.at<double>(0,0);
	mf2 = m.at<double>(1,1);
	mC1 = m.at<double>(0,2);
	mC2 = m.at<double>(1,2);

	K = (Mat_<float>(3,3)<<mf1, 0, mC1, 0, mf2, mC2, 0, 0, 1);
}


void Camera::SetDistortion(Mat &kkppk)
{
	double k1, k2, p1, p2, k3;
	k1 = kkppk.at<double>(0,0);
	k2 = kkppk.at<double>(1,0);
	p1 = kkppk.at<double>(2,0);
	p2 = kkppk.at<double>(3,0);
	k3 = kkppk.at<double>(4,0);

	distCoeffs = (Mat_<float>(5,1)<<k1, k2, p1, p2, k3);
}
