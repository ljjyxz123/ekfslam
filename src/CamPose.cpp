//
//  CamPose.cpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/8.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#include "CamPose.hpp"

CamPose::CamPose(int iStep, int num_features) {
    
    FrameID = iStep;
    Count   = num_features;
    
}