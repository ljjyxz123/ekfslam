
#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <iostream>
#include <vector>
#include<mutex>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;


 namespace YX_SLAM{     
    class KeyFrame{
        
        public:
            KeyFrame();
            KeyFrame(const cv::Mat& _t);
            ~KeyFrame(){};
        public:
        
            cv::Mat getPose();
            void setPose(const cv::Mat& _t);
            
        private:
            cv::Mat Tcw;    
            
        
    };
    
 }    
#endif