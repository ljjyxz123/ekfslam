//
//  CamPose.hpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/8.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#ifndef CamPose_hpp
#define CamPose_hpp

class CamPose {
public:
    CamPose(int iStep, int num_features);
    
    int FrameID;
    int Count;
    
};


#endif /* CamPose_hpp */
