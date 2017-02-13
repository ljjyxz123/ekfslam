//
//  Camera.hpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/8.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#ifndef Camera_hpp
#define Camera_hpp

#include <opencv2/opencv.hpp>

using namespace cv;

class Camera
{
public:
    void SetIntrinsics(int width, int height, double focalx, double focaly, double ppx, double ppy);
	void SetK(Mat &m);
	void SetDistortion(Mat &kkppk);   

    int nCols;
    int nRows;
    double fx;
    double fy;
    double Cx;
    double Cy;

	Mat K;
	Mat distCoeffs;
};

#endif /* Camera_hpp */
