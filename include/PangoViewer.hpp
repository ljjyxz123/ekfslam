//
//  PangoViewer.hpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/9.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#ifndef PangoViewer_hpp
#define PangoViewer_hpp

#include <stdio.h>
#include <pangolin/pangolin.h>
#include <Eigen/Geometry>


class PangoViewer {
    
public:

	PangoViewer();
    
    void DrawMapPoints(std::vector<Eigen::Vector3d>& mPoints);
    
    void DrawActivePoints(std::vector<Eigen::Vector3d>& aPoints);

   	void DrawTrajectory(std::vector<Eigen::Vector3d>& vPos);

	void DrawCamera(Eigen::Quaterniond& q_Cw, double scale);
    
    void GetOpenGlMatrix(Eigen::VectorXd &rq, pangolin::OpenGlMatrix &M);

private:

	std::vector<Eigen::Vector3d> camModel;
    
};

#endif /* PangoViewer_hpp */
