
#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>
#include<mutex>
#include "KeyFrame.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;
    
namespace YX_SLAM{  
    
    class Map{
        
        public:
            Map();
            ~Map(){}
            
        public:
            vector<KeyFrame*> GetAllKeyFrames();
            void AddKeyFrame(KeyFrame* kf);
            int size();
            
            std::mutex mMutexMap;
        private:
            int N;
            vector<KeyFrame*> kfs;    
    };
}    
#endif    