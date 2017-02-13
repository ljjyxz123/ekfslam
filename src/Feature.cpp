//
//  Feature.cpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/8.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#include "Feature.hpp"

Feature::Feature(cv::Mat& patch_ini, cv::Mat& patch_mat, int iStep):
        times_predicted(0),
        times_measured(0),
        times_notmeasured(0),
        low_innovation_inlier(0),
        high_innovation_inlier(0),
        converged(0),
        hok(0),
        zok(0),
        too_uncertain(0),
        compatible(0)
{
    patch_ini.copyTo(patch_when_initialized);
    patch_mat.copyTo(patch_when_matching);
    FrameID = iStep;
}