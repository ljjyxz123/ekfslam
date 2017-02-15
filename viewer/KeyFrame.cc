

#include <iostream>
#include <vector>
#include "KeyFrame.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;


namespace YX_SLAM{      
    KeyFrame::KeyFrame(){
        
        
    }
    KeyFrame::KeyFrame(const cv::Mat& _t){
        
    }
    
    cv::Mat KeyFrame::getPose(){
        
        return Tcw;
        
    }
    
    void KeyFrame::setPose(const cv::Mat& _t){
        
        _t.copyTo(Tcw);
    }
}