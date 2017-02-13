//
//  PangoViewer.cpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/9.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#include "PangoViewer.hpp"

PangoViewer::PangoViewer()
{
	camModel.clear();
	
	/*
	camModel.push_back(Eigen::Vector3d(0,0,0));
	camModel.push_back(Eigen::Vector3d(-0.5, 0.5, 1));
	camModel.push_back(Eigen::Vector3d(0.5, 0.5, 1));
	camModel.push_back(Eigen::Vector3d(0.5, -0.5, 1));
	camModel.push_back(Eigen::Vector3d(-0.5, -0.5, 1));
	*/

	camModel.push_back(Eigen::Vector3d(-0.5, -0.1,  0.3));
	camModel.push_back(Eigen::Vector3d(-0.5,  0.1,  0.3));
	camModel.push_back(Eigen::Vector3d( 0.5,  0.1,  0.3));
	camModel.push_back(Eigen::Vector3d( 0.5, -0.1,  0.3));
	camModel.push_back(Eigen::Vector3d(-0.5, -0.1, -0.3));
	camModel.push_back(Eigen::Vector3d(-0.5,  0.1, -0.3));
	camModel.push_back(Eigen::Vector3d( 0.5,  0.1, -0.3));
	camModel.push_back(Eigen::Vector3d( 0.5, -0.1, -0.3));
}


void PangoViewer::DrawMapPoints(std::vector<Eigen::Vector3d>& mPoints)
{
    glPointSize(4);
    glBegin(GL_POINTS);
    glColor3f(0.2, 0.2, 0.1);
    
    for (size_t i=0; i<mPoints.size(); i++) {
        pangolin::glVertex(mPoints[i]);
    }
    
    glEnd();
}


void PangoViewer::DrawActivePoints(std::vector<Eigen::Vector3d>& aPoints)
{
    glPointSize(6);
    glBegin(GL_POINTS);
    glColor3f(0.2, 0.8, 0.1);
    
    for (size_t i=0; i<aPoints.size(); i++) {
        pangolin::glVertex(aPoints[i]);
    }
    
    glEnd();
}


void PangoViewer::DrawTrajectory(std::vector<Eigen::Vector3d>& vPos)
{
	glPointSize(3);
	glColor3f(0.8, 0.3, 0.5);
	glBegin(GL_POINTS);
	for(size_t i=0; i<vPos.size(); i++)
	{
		pangolin::glVertex(vPos[i]);
	}
	glEnd();

	glPointSize(3);
	glColor3f(0.0, 1.0, 1.0);
	glBegin(GL_POINTS);
	pangolin::glVertex(vPos.back());
	glEnd();
}


void PangoViewer::DrawCamera(Eigen::Quaterniond& q_Cw, double scale)
{
	Eigen::Matrix3d R_Wc = q_Cw.matrix();

	std::vector<Eigen::Vector3d> vs;
	for(size_t i=0; i<camModel.size(); i++)
	{
		vs.push_back(scale * R_Wc * camModel[i]);
	}

	glLineWidth(3);
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);

	for(int i=0; i<4; i++)
	{
		int j = i % 4;
		pangolin::glVertex(vs[i]);
		pangolin::glVertex(vs[j]);

		pangolin::glVertex(vs[i+4]);
		pangolin::glVertex(vs[j+4]);

		pangolin::glVertex(vs[i]);
		pangolin::glVertex(vs[i+4]);
	}

	glEnd();
	
}


void PangoViewer::GetOpenGlMatrix(Eigen::VectorXd &rq, pangolin::OpenGlMatrix &Twc)
{
    Eigen::Vector3d t_Wc = rq.head(3);
    Eigen::Quaterniond q_Cw(rq(3), rq(4), rq(5), rq(6));
    Eigen::Matrix3d R_Wc = q_Cw.matrix();
    
    Twc.m[0] = R_Wc(0,0);
    Twc.m[1] = R_Wc(1,0);
    Twc.m[2] = R_Wc(2,0);
    Twc.m[3] = 0.0;
    
    Twc.m[4] = R_Wc(0,1);
    Twc.m[5] = R_Wc(1,1);
    Twc.m[6] = R_Wc(2,1);
    Twc.m[7] = 0.0;
    
    Twc.m[8] = R_Wc(0,2);
    Twc.m[9] = R_Wc(1,2);
    Twc.m[10] = R_Wc(2,2);
    Twc.m[11] = 0.0;
    
    Twc.m[12] = t_Wc(0);
    Twc.m[13] = t_Wc(1);
    Twc.m[14] = t_Wc(2);
    Twc.m[15] = 1.0;
    
}
