#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>


#include <boost/foreach.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <iostream>
#include <cyusb.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "MsgQueue.h"
#include "CaptureData.h"
#include "CMutex.h"

#define zhk_debug 0

using namespace std;
using namespace cv;

class CaptureData;

int main()
{
	int FastRun=0,FastRun_n=0;

	CaptureData *CaptureData_p = new CaptureData;
	
	USBElements sensorCapture;

	long long OutImgTimeStamp = 0;
	long long calltimestamp = 0, calltimestamp_last = 0;
	int ret = 0;

	namedWindow("LeftImg", 0);
	namedWindow("UnLeft", 0);

	// IMU data
	vector<double> tss;
	vector<Eigen::Vector3f> ams;
	vector<Eigen::Vector3f> wms;

	// Camera intrinsics
	FileStorage fsconfig("euroc.yaml", FileStorage::READ);
	if(!fsconfig.isOpened())
	{
		cout << "Can not open config file!" << endl;
		return -1;
	}
	float fx, fy, Cx, Cy, k1, k2, p1, p2;
	fsconfig["Camera.fx"] >> fx;
	fsconfig["Camera.fy"] >> fy;
        fsconfig["Camera.cx"] >> Cx;
	fsconfig["Camera.cy"] >> Cy;
	fsconfig["Camera.k1"] >> k1;
	fsconfig["Camera.k2"] >> k2;
	fsconfig["Camera.p1"] >> p1;
	fsconfig["Camera.p2"] >> p2;
	
	Mat cam_K = (Mat_<float>(3,3)<<fx, 0, Cx, 0, fy, Cy, 0, 0, 1.0);
	Mat cam_DistorCoeffs = (Mat_<float>(4,1)<<k1, k2, p1, p2);

	// Gyroscope bias estimation
	int n_gyo = 0;
	Eigen::Vector3f sum_gyo;
	sum_gyo << 0, 0, 0;

    Eigen::Vector3f Estimated_gyro_bias(-0.0510142 -0.0650584 -0.0204192);

	while(1)
	{
		if(FastRun==0)
		{
		    sleep(1);
		    FastRun_n++;
		    FastRun=1;
		    if(FastRun_n>1)
		    {
		        FastRun=1;
		        FastRun_n=0;
		    }
		}

		struct timeval tv;
		gettimeofday(&tv, NULL);
		calltimestamp = tv.tv_sec * 1000 + tv.tv_usec / 1000;
		if(calltimestamp - calltimestamp_last <= 50)
		{
			continue;
		}
		calltimestamp_last = calltimestamp;
		
		ret = CaptureData_p->Get_CaptureData(&sensorCapture, OutImgTimeStamp);

		if(ret == 0)
		{
			cout << "ret = 0" << endl;
		}
		else
		{
			clock_t load_img_begin = clock();

			cout << "ret = " << ret << endl;
			cv::Mat leftImage(480, 640, CV_8UC1, sensorCapture.RightImgBuffer);
			//cv::Mat rightImage(480, 640, CV_8UC1, sensorCapture.RightImgBuffer);

			double img_time = (double)sensorCapture.ImgTimeStamp * 1e-9;

			cout << "img_time " << img_time << endl;
			
			Mat unIl;
			undistort(leftImage, unIl, cam_K, cam_DistorCoeffs, Mat());
			
			clock_t load_img_end = clock();
			cout << "Load & undistort an image need " << (double)(load_img_end - load_img_begin) / CLOCKS_PER_SEC * 1000 << " ms" << endl;	

			imshow("LeftImg", leftImage);
			imshow("UnLeft", unIl);

			// IMU
			tss.clear();	ams.clear();	wms.clear();
			int DataNum = sensorCapture.DataNum;
			for(int i=0; i<DataNum; i++)
			{
				float acc_x = sensorCapture.ImuValue[i][0];
				float acc_y = sensorCapture.ImuValue[i][1];
				float acc_z = sensorCapture.ImuValue[i][2];

				float gyo_x = sensorCapture.ImuValue[i][3];
				float gyo_y = sensorCapture.ImuValue[i][4];
				float gyo_z = sensorCapture.ImuValue[i][5];
				
				double imu_time = sensorCapture.ImuTimeStamp[i] * 1e-9;
				
				tss.push_back(imu_time);
				ams.push_back(Eigen::Vector3f(acc_x, acc_y, acc_z));
				wms.push_back(Eigen::Vector3f(gyo_x, gyo_y, gyo_z));
			}

			cout << "IMU time - IMG time " << tss.back() - img_time << endl;

			/*
			// Gyroscope bias estimation
			if(n_gyo < 500)
			{
				n_gyo += DataNum;
				for(int i=0; i<DataNum-1; i++)
				{	
					sum_gyo += wms[i];
				}
			}
			else
			{
				cout << "gyro bias: " << sum_gyo.transpose() / n_gyo << endl;
				n_gyo = 0;
				sum_gyo << 0, 0, 0;
				waitKey(5000);
			}
			*/
			
			waitKey(1);
		}
		
	}


}

