//
//  EKFSystem.hpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/8.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#ifndef EKFSystem_hpp
#define EKFSystem_hpp

#include "EigenHelper.h"
#include "Camera.hpp"
#include "CamPose.hpp"
#include "Feature.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;

class EKFSystem
{
public:
    
    EKFSystem(bool half_res, int n_camx);
    
    int imu_begin;
    int grt_begin;
    int first_frame;
    int total_frames;
    
    EgQuatd q_Sc;
    EgQuatd q_Cs;
    EgM3d   R_Cs;
    
    std::vector<EgV3d> map_points;
    std::vector<EgV3d> trajectory;
    
    void LoadParameters(std::string& dir);
    
    void Initialize(EgV3d& init_p, EgV3d& init_g, EgV3d& init_v, EgV3d& init_ba, EgV3d& init_bw);
    
    void ManageMap(Mat& I, int iStep);
    
    void Propagate(std::vector<double> &vts, std::vector<EgV3d>& wms, std::vector<EgV3d>& ams);
    
    void SearchMatches(Mat& I);
    
    void BigUpdate();
    
    void DrawOnImage(Mat &I);
    
    void GetActivePoints(std::vector<EgV3d>& active_points);
    
public:
    
    int NcamX;
    
    bool bHalfRes;
    
    int NWanted;
    
    int fast_threshold;
    
    double sigma_gyro;
    double sigma_accel;
    double sigma_pixel;
    double imu_rate;
    double deltat;
    double gravity_magnitude;
    
    Camera Cam;
    
    double ncc_threshold;
    double zncc_threshold;
    double ssd_threshold;
    
    int hps_ini;
    int hps_mat;
    int edge_band;
    double max_angle;
    
    double median_inverse_depth;
    double max_inverse_depth;
    double min_inverse_depth;
    double prior_inverse_depth;
    bool well_estimated;
    
    Eigen::VectorXd x_k_k;
    Eigen::MatrixXd P_k_k;
    
    std::vector<Feature> features_info;
    std::vector<CamPose> poses_info;
    std::vector<int> entries[2];
    
private:
    
    void SearchNewFeatures(Mat& I, int numWanted, std::vector<Point2f>& newFeatures);
    
    void AddToFilter(std::vector<Point2f>& newFeatures, double initial_rho, double std_rho);
    
    void AddToInfos(std::vector<Point2f>& newFeatures, Mat& I, int iStep);
    
    void DeleteFeatures();
    
    void UpdateEntries();
    
    int UpdateFeaturesInfo();
    
    void EstimateMedianInverseDepth();
    
    void Calculate_little_hs();
    
    void Calculate_big_Hs();
    
    void Calculate_Ss();
    
    void NccMatching(Mat& I);
    
    void OnePointRansac(double palosf);
    
    void OnePointUpdateState( Eigen::VectorXd &xNew, int iFeature);
    
    void ComeOnGuys(Eigen::VectorXd &xNew, std::vector<int>& posis_cp, std::vector<int>& iInliers);
    
    void EkfUpdateLowInnos();
    
    void EkfUpdateHighInnos();
    
    void EkfUpdate(std::vector<int>& iFeatures);
    
    void RescueHighInnos();
    
    void PreIntegrate(std::vector<double>& ts,
                      std::vector<EgV3d>& wms,
                      std::vector<EgV3d>& ams,
                      EgV3d& pp, EgQuatd& qq, EgV3d& vv,
                      Eigen::Matrix<double, 10, 10> &RR);
    
    
    
};

#endif /* EKFSystem_hpp */
