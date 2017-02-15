

#include <iostream>
#include <vector>
#include "Map.h"
#include "KeyFrame.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;


namespace YX_SLAM{      
    Map::Map(){
       kfs.clear(); 
       N = 0; 
    }
    
    vector<YX_SLAM::KeyFrame*> Map::GetAllKeyFrames(){
        unique_lock<mutex> lock(mMutexMap);
        return kfs;
    }
    
    void Map::AddKeyFrame(KeyFrame* kf){
        unique_lock<mutex> lock(mMutexMap);
        kfs.push_back(kf);
        N ++;
    }
    
    int Map::size(){
        return N;
    }
    
}