//
//  main.cpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/7.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <cyusb.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include "MsgQueue.h"
#include "CaptureData.h"
#include "CMutex.h"

#include "EigenHelper.h"
#include "EKFSystem.hpp"
#include "PangoViewer.hpp"

using namespace cv;
using namespace std;

int main(int argc, const char * argv[])
{
    
    // ------------- Pangolin ----------------------------------------
    pangolin::CreateWindowAndBind("Main", 852, 640);
    glEnable(GL_DEPTH_TEST);
    
    pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(852,640,200,400,426,320,0.1,1000);
    pangolin::OpenGlRenderState s_cam(proj, pangolin::ModelViewLookAt(-3, 0.0, 1, 0,0,0, 1.0, 0.0, 0.0));
    
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -852.0f/640.0f)
    .SetHandler(&handler);
    
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

	PangoViewer pango;

    
    // -------------- Create a EKF-SLAM EKFSystem & Load parameters ----
	// Create
	bool bHalfResolution = false;
	int NcamX = 19;
	EKFSystem ekfTracker(bHalfResolution, NcamX);

	// Configurate
	string config_file = "/home/tan/aslam/configLeft.yaml";
	ekfTracker.LoadParameters(config_file);
	
    
    
	// --------------- About Sensors ------------------------------------
	// Data grabber
	CaptureData *pCaptureData = new CaptureData;
	USBElements sensorCapture;

	// IMU Data container
	vector<double> tss;
	vector<EgV3d> ams;
	vector<EgV3d> wms;


    //================= Main Function ====================================
	int fastRun = 0;
	long long noUseTimeStamp = 0;
	long long callTimeStamp = 0, callTimeStamp_last = 0;
	int ret = 0;

	bool bHaveGyroBias = false;
	EgV3d sumGyroBias(0,0,0);
	int	  numGyroBias = 0;
	EgV3d estimatedGyroBias0(0,0,0), estimatedGyroBias(0,0,0);
	// Quote these two lines if you'd like to estimate the gyro bias
	//bHaveGyroBias = true;
	//estimatedGyroBias << -0.050682 -0.0688238 -0.0201648;

	bool bHaveGravity = false;
	EgV3d sumGravity(0,0,0);
	int   numGravity = 0;
	EgV3d estimatedGravity0(0,0,0), estimatedGravity(0,0,0);

	vector<EgV3d> staticGravities;

	bool bEkfInitialized = false;
	Mat prevI;
	int iStep = 0;
	
	namedWindow("img", 0);
	
	
	int nSave = 0;
	while(!pangolin::ShouldQuit())
	{
		// In the very beginning, let's sleep a little while
		if(fastRun == 0)
		{
			sleep(1);
			fastRun = 1;
		}

		// Try to get data every 50 ms
		struct timeval tv;
		gettimeofday(&tv, NULL);
		callTimeStamp = tv.tv_sec*1000 + tv.tv_usec/1000;
		if(callTimeStamp - callTimeStamp_last < 50)
		{
			continue;
		}
		callTimeStamp_last = callTimeStamp;
		
		// Capture data
		ret = pCaptureData->Get_CaptureData(&sensorCapture, noUseTimeStamp);
		
		if(ret == 0)
		{
			//cout << "ret = 0, No data comes in!" << endl;
		}
		else
		{	
			clock_t t_begin = clock();			
			
			//cout << "ret = " << ret << endl;

			// --- IMAGE ---------------------------------------------------------
			// Image & It's timestamp
			//Mat oI(480, 640, CV_8UC1, sensorCapture.IMG_Frame_Buffer.back().RightImgBuffer);
			Mat oI(480, 640, CV_8UC1, sensorCapture.RightImgBuffer);
			double img_time = (double)sensorCapture.ImgTimeStamp * 1e-9;
			
			/*
			if(nSave % 20 == 0)
			{
				char imgName[30];
				sprintf(imgName, "/home/tan/Calib/%06d.png", nSave);
				imwrite(imgName, oI);
			}
			nSave++;
			*/

						
			// Undistort ( to be deleted later )
			Mat uI;
			undistort(oI, uI, ekfTracker.Cam.K, ekfTracker.Cam.distCoeffs, Mat());
			
			// Resize if neccessary
			if(bHalfResolution)
			{
				resize(uI, uI, Size(uI.cols/2, uI.rows/2));
			}
			
			if (!bEkfInitialized)
			{
				imshow("img", uI);
				waitKey(1);
			}

			
			// --- IMU -----------------------------------------------------------
			tss.clear();	ams.clear();	wms.clear();

			int imuDataNum = sensorCapture.DataNum;
			for(int i=0; i<imuDataNum; i++)
			{
				double tmi = (double)sensorCapture.ImuTimeStamp[i]*1e-9;
				double amx = (double)sensorCapture.ImuValue[i][0];
				double amy = (double)sensorCapture.ImuValue[i][1];
				double amz = (double)sensorCapture.ImuValue[i][2];
				double wmx = (double)sensorCapture.ImuValue[i][3];
				double wmy = (double)sensorCapture.ImuValue[i][4];
				double wmz = (double)sensorCapture.ImuValue[i][5];

				tss.push_back(tmi);
				ams.push_back(EgV3d(amx, amy, amz));
				wms.push_back(EgV3d(wmx, wmy, wmz));

				// For estimating the gyroscope's bias
				if(!bHaveGyroBias)
				{
					sumGyroBias += wms.back();
					numGyroBias++;
				}
				else if(!bHaveGravity)	// For estimating the initial gravity vector
				{
					sumGravity += ams.back();
					numGravity++;
				}
			}


			// --- Preparation ---------------------------------------------
			// (Assume that we have already calibrated the Accelerometer and got appropriate bias values)

			// Estimate gyro bias if neccessary
			if(!bHaveGyroBias)
			{
				if(numGyroBias > 500)
				{
					estimatedGyroBias = sumGyroBias / (double)numGyroBias;
					numGyroBias = 0;
					sumGyroBias << 0, 0, 0;
					
					if((estimatedGyroBias - estimatedGyroBias0).norm() < 0.0003)
					{
						estimatedGyroBias = (estimatedGyroBias + estimatedGyroBias0) / 2.0;
						bHaveGyroBias = true;
						cout << "gyro bias " << estimatedGyroBias.transpose() << endl;
						waitKey();
					}
					else
					{
						cout << estimatedGyroBias << endl;
						estimatedGyroBias0 = estimatedGyroBias;
					}
				}

				continue;
				
			}

			// Estimate gravity vector
			if(!bHaveGravity)
			{
				if(numGravity < 200)
				{
					continue;
				}
				else
				{
					estimatedGravity = sumGravity / (double)numGravity;
					sumGravity << 0, 0, 0;
					numGravity = 0;
					
					if((estimatedGravity - estimatedGravity0).norm() < 0.01)
					{
						estimatedGravity = (estimatedGravity + estimatedGravity0) / 2.0;
						bHaveGravity = true;
						cout << "estimated gravity " << estimatedGravity.transpose() << endl;
						
						/*
						// For estimating accel bias
						estimatedGravity0 << 0, 0, 0;
						waitKey();
						staticGravities.push_back(estimatedGravity);
						cout << staticGravities.size() << endl;
						if(staticGravities.size() == 30)
						{
							for(size_t i=0; i<staticGravities.size(); i++)
								cout << staticGravities[i].transpose() << endl;
								waitKey();
						}
						continue;
						*/
					}
					else
					{
						estimatedGravity0 = estimatedGravity;
						cout << "mean gravity " << estimatedGravity.transpose() << endl;
						continue;
					}
				}
			}
			// go go go!!!


			//--- EKF Tracker -------------------------------------------------------------
			
			if(!bEkfInitialized)
			{
				// Initialize
				EgV3d init_p, init_v, init_ba, init_bw, init_g;
				init_p << 0, 0, 0;
				init_v << 0, 0, 0;
				init_ba << 0.5047, -0.02234, -1.72445;
				init_bw = estimatedGyroBias;
				init_g  = ekfTracker.R_Cs * (estimatedGravity - init_ba);
		
				ekfTracker.Initialize(init_p, init_g, init_v, init_ba, init_bw);
				uI.copyTo(prevI);
				bEkfInitialized = true;
				continue;
			}
			else
			{
				// Feature management (delete features, update features_info, add new features)
				ekfTracker.ManageMap(prevI, iStep);

				// Propagate (using all IMU readings between current frame and previous frame)
				ekfTracker.Propagate(tss, wms, ams);

				// Search compatible matches in current frame
				ekfTracker.SearchMatches(uI);

				// Updatemake
				ekfTracker.BigUpdate();
				
				// Prepare for next round
				uI.copyTo(prevI);
				iStep++;

				// Display
				Mat cimg;
				cvtColor(uI, cimg, CV_GRAY2BGR);
				ekfTracker.DrawOnImage(cimg);
				imshow("img", cimg);
				waitKey(1);

				clock_t t_end = clock();
				cout << "time " << (double)(t_end-t_begin)/CLOCKS_PER_SEC << endl;

			}


			//--- Pangolin ----------------------------------------------------
			// Clear screen
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			// Get Twc
			EgVxd Xrqv = ekfTracker.x_k_k.head(10);
			pango.GetOpenGlMatrix(Xrqv, Twc);

			// Activate
			//s_cam.Follow(Twc);
			d_cam.Activate(s_cam);

			// Draw
			glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
			
			//pango.DrawMapPoints(ekfTracker.map_points);	// map points
			pango.DrawTrajectory(ekfTracker.trajectory);	// trajectory

			EgV4d qq = ekfTracker.x_k_k.segment(3,4);
			EgQuatd q(qq(0), qq(1), qq(2), qq(3));
			pango.DrawCamera(q, 0.5);

			// Finish
			pangolin::FinishFrame();
			
		}

	}
	


    
	cout << "Test..." << endl	;

    waitKey();
    
    return 0;
    
    
}

