//
//  EKFSystem.cpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/8.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#include "EKFSystem.hpp"
#include "MathHelper.h"

#define CHI2_095 5.9915
#define CHI2_099 9.2103

EKFSystem::EKFSystem(bool half_res, int n_camx)
{
    bHalfRes = half_res;
    NcamX    = n_camx;
}


void EKFSystem::LoadParameters(std::string &config_file)
{
    FileStorage fs(config_file, FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "Can not open config file!\n";
        waitKey();
    }
    
    fs["IWantYou"] >> NWanted;
    fs["fast_threshold"] >> fast_threshold;
    
    double c_sigma_g, c_sigma_a;
    fs["sigma_gyro"]  >> c_sigma_g;  // continuous
    fs["sigma_accel"] >> c_sigma_a;
    fs["imu_rate"]    >> imu_rate;
    deltat = 1.0 / imu_rate;
    sigma_gyro = c_sigma_g * sqrt(imu_rate); // discrete
    sigma_accel = c_sigma_a * sqrt(imu_rate);
    
    fs["sigma_pixel"] >> sigma_pixel;
    
    fs["gravity"] >> gravity_magnitude;
    fs["hps_ini"] >> hps_ini;
    fs["hps_mat"] >> hps_mat;
    edge_band = MAX(21, hps_ini+2);
    
    fs["ncc_threshold"] >> ncc_threshold;
    fs["zncc_threshold"] >>zncc_threshold;
    fs["ssd_threshold"] >> ssd_threshold;
    
    fs["prior_inverse_depth"] >> prior_inverse_depth;
    fs["max_inverse_depth"] >> max_inverse_depth;
    fs["min_inverse_depth"] >> min_inverse_depth;
    well_estimated = false;
    median_inverse_depth = prior_inverse_depth;
    
    int imWidth, imHeight;
    double fx, fy, cx, cy;
    fs["max_angle"] >> max_angle;
    fs["image_width"]  >> imWidth;
    fs["image_height"] >> imHeight;
    fs["fx"] >> fx;
    fs["fy"] >> fy;
    fs["cx"] >> cx;
    fs["cy"] >> cy;
    
    if (bHalfRes) {
        fx /= 2.0;
        fy /= 2.0;
        cx /= 2.0;
        cy /= 2.0;
        imWidth  /= 2;
        imHeight /= 2;
        hps_mat  /= 2;
        hps_ini  /= 2;
    }
    
    Cam.SetIntrinsics(imWidth, imHeight, fx, fy, cx, cy);

	Mat mK;
	fs["cameraMatrix"] >> mK;
	Cam.SetK(mK);
	std::cout << "camera K:\n";
	std::cout << Cam.K << "\n";

	Mat mDistCoeffs;
	fs["distCoeffs"] >> mDistCoeffs;
	Cam.SetDistortion(mDistCoeffs);
	std::cout << "camera DistCoeffs:\n";
	std::cout << Cam.distCoeffs << "\n";
    
    Mat q;
    fs["q_Sc"] >> q;
    q_Sc = Eigen::Quaterniond(q.at<double>(0,0), q.at<double>(1,0), q.at<double>(2,0), q.at<double>(3,0));
    q_Sc.normalize();
	//q_Sc = q_Sc.inverse();
    q_Cs = q_Sc.inverse();
    R_Cs = q_Sc.matrix();
    
}


void EKFSystem::Initialize(EgV3d& init_p, EgV3d &init_g, EgV3d &init_v, EgV3d &init_ba, EgV3d &init_bw)
{
    x_k_k.resize(NcamX);
    x_k_k << init_p, 1,0,0,0, init_v, init_ba, init_bw, init_g;
    
    P_k_k.resize(NcamX, NcamX);
    P_k_k.setZero();
    
    for (int i=0; i<3; i++)  P_k_k(i , i) = 1e-8;
    for (int i=3; i<7; i++)  P_k_k(i , i) = 1e-8;
    for (int i=7; i<10; i++) P_k_k(i , i) = 9e-4;
    for (int i=10; i<13; i++) P_k_k(i, i) = 2e-2;
    for (int i=13; i<16; i++) P_k_k(i, i) = 2e-4;
    for (int i=16; i<19; i++) P_k_k(i, i) = 9e-2;

	trajectory.push_back(init_p);
}


void EKFSystem::ManageMap(cv::Mat &I, int iStep)
{
    // Estimate median depth
    EstimateMedianInverseDepth();
    
    // Delete Features
    // Notice that the following terms will change:
    // x_k_k, P_k_k, entries, features_info, poses_info
    DeleteFeatures();
    UpdateEntries();
    
    
    // Count well-tracked features & update features_info
    int numMeasured = UpdateFeaturesInfo();
    int numWanted = NWanted - numMeasured;
    if (numWanted < 1)
    {
        return;
    }
    std::cout << "NumWanted:\t" << numWanted << "\n";
    
    
    // Add new features
    std::vector<Point2f> newFeatures;
    newFeatures.reserve(numWanted);
    SearchNewFeatures(I, numWanted, newFeatures);
    
    if (newFeatures.size() > 0)
    {
        // Add new features into filter, features_info and poses_info
        // AddToFilter(newFeatures, median_inverse_depth, median_inverse_depth);
		AddToFilter(newFeatures, 1.0, 1.0);
        AddToInfos(newFeatures, I, iStep);
        UpdateEntries();
    }
    
    std::cout << "New features:\t" << newFeatures.size() << "\n";
    
    
}



void EKFSystem::Propagate(std::vector<double> &vts, std::vector<EgV3d> &wms, std::vector<EgV3d> &ams)
{
    double cov_w = sigma_gyro * sigma_gyro;
    double cov_a = sigma_accel * sigma_accel;
    Eigen::Matrix<double, 6, 6> P_wa;
    P_wa.setZero();
    for (int i=0; i<3; i++) P_wa(i,i) = cov_w;
    for (int i=3; i<6; i++) P_wa(i,i) = cov_a;
    
    
    for (size_t i=0; i<vts.size(); i++)
    {
        // Old state
        EgV3d r_W = x_k_k.head(3);
        EgQuatd q_Cw(x_k_k(3), x_k_k(4), x_k_k(5), x_k_k(6));
        EgV3d v_W = x_k_k.segment(7, 3);
        EgV3d ba  = x_k_k.segment(10, 3);
        EgV3d bw  = x_k_k.segment(13, 3);
        EgV3d gvec= x_k_k.segment(16, 3);
        
        EgQuatd q_Sw = q_Cw * q_Sc;
        EgM3d R_Ws = q_Sw.matrix();
        
        // IMU
        double dt = 0.005;
        EgV3d a = ams[i] - ba;
        EgV3d w = wms[i] - bw;
        EgV3d wdt = w * dt;
        
        EgV3d a_W = R_Ws * a - gvec;
        EgQuatd dq;
        vec2quat(wdt, dq);
        
        // State propagation
        r_W = r_W + v_W * dt;
        v_W = v_W + a_W * dt;
        EgQuatd q_Sw_new = q_Sw * dq;
        EgQuatd q_Cw_new = q_Sw_new * q_Cs;
        
        // Covariance propagation
        EgM4d temp1, temp2, temp3, temp4;
        dqp_by_dq(q_Cs, temp1);
        dqp_by_dp(q_Sw, temp4);
        
        EgQuatd q_Cnew_c = q_Sc * dq * q_Cs;
        EgM4d q_par_q;
        dqp_by_dq(q_Cnew_c, q_par_q);
        
        EgM3d Rq0, Rqx, Rqy, Rqz;
        dR_by_dq(q_Cw, Rq0, Rqx, Rqy, Rqz);
        
        EgV3d a_Cdt = R_Cs * a * dt;
        
        Eigen::Matrix<double, 3, 4> v_par_q;
        v_par_q << Rq0 * a_Cdt, Rqx * a_Cdt, Rqy * a_Cdt, Rqz * a_Cdt;
        
        Eigen::Matrix<double, 4, 3> q_par_bg, temp5;
        dq_by_dv(wdt, temp5);
        q_par_bg = -temp1 * temp4 * temp5 * dt;
        
        EgM3d v_par_ba = -R_Ws * dt;
        
        
        EgMxd F = EgMxd::Identity(19, 19);
        F.block(0, 7, 3, 3) = dt * EgM3d::Identity();
        
        F.block(3, 3, 4, 4) = q_par_q;
        F.block(3, 13, 4, 3) = q_par_bg;
        F.block(7, 3, 3, 4) = v_par_q;
        F.block(7, 10, 3, 3) = v_par_ba;
        F.block(7, 16, 3, 3) = -dt * EgM3d::Identity();
        
        Eigen::Matrix<double, 19, 6> G;
        G.setZero();
        G.block(3, 0, 4, 3) = -q_par_bg;
        G.block(7, 3, 3, 3) = R_Ws * dt;
        
        x_k_k.head(10) << r_W, q_Cw_new.w(), q_Cw_new.vec(), v_W;
        
        if (x_k_k.size() > 19)
        {
            EgMxd Pxx = P_k_k.topLeftCorner(19, 19);
            EgMxd Pxy = P_k_k.topRightCorner(19, x_k_k.size()-19);
            EgMxd Pyx = P_k_k.bottomLeftCorner(x_k_k.size()-19, 19);
            
            P_k_k.topLeftCorner(19,19) = F * Pxx * F.transpose() + G * P_wa * G.transpose();
            P_k_k.topRightCorner(19, x_k_k.size()-19) = F * Pxy;
            P_k_k.bottomLeftCorner(x_k_k.size()-19, 19) = Pyx * F.transpose();
        }
        else
        {
            P_k_k = F * P_k_k * F.transpose() + G * P_wa * G.transpose();
        }
        
    }
    
    EgMxd P_k_k_T = P_k_k.transpose();
    P_k_k = 0.5 * (P_k_k + P_k_k_T);
    
}



void EKFSystem::SearchMatches(cv::Mat &I)
{
    //----------------------------------
    // 1. Predict camera measurements
    Calculate_little_hs();
    
    // 2. Calculate derivatives
    Calculate_big_Hs();
    
    // 3. Calculate innovations
    Calculate_Ss();
    
    
    //---------------------------------------------------
    // TO DO: warp patches (predict features' appearance)
    //---------------------------------------------------
    
    //------- Matching-------
    NccMatching(I);
    
}



void EKFSystem::BigUpdate() {
    std::cout << "state size:\t" << x_k_k.size() << "\n";
    
    // 1-point ransac
    OnePointRansac(0.999);
    
    // Update using low innovation inliers
    EkfUpdateLowInnos();
    
    // Rescue high innovation inliers
    RescueHighInnos();
    
    // Partial update using the high innovation inliers
    EkfUpdateHighInnos();
    
    // Iterate----------
    // EkfUpdateAll();
    
	EgV3d p = x_k_k.head(3);
	trajectory.push_back(p);
}



void EKFSystem::DrawOnImage(cv::Mat &cimg) {
    
    for (size_t i=0; i<features_info.size(); i++) {
        if (features_info[i].zok) {
            Point2f pt(features_info[i].z(0), features_info[i].z(1));
            
            if (features_info[i].low_innovation_inlier) {
                circle(cimg, pt, 5, Scalar(0,255,0), -1);
            }
            else if(features_info[i].high_innovation_inlier) {
                circle(cimg, pt, 5, Scalar(255,0,0), -1);
            }
            else {
                circle(cimg, pt, 5, Scalar(0,0,255), -1);
            }
        }
    }
}


void EKFSystem::GetActivePoints(std::vector<EgV3d> &active_points) {
    
    active_points.reserve(features_info.size());
    
    for (size_t i=0; i<features_info.size(); i++) {
        if (features_info[i].low_innovation_inlier || features_info[i].high_innovation_inlier) {
            int et0 = entries[0][i], et1 = entries[1][i];
            EgV3d ri = x_k_k.segment(et0, 3);
            EgV3d tpr = x_k_k.segment(et1, 3);
            double cosp = cos(tpr(1));
            EgV3d mi(cosp * sin(tpr(0)), -sin(tpr(1)), cosp * cos(tpr(0)));
            EgV3d rW = ri + mi / tpr(2);
            active_points.push_back(rW);
        }
    }
}




//=============================================================
//          Private Functions
//=============================================================

void EKFSystem::SearchNewFeatures(cv::Mat &I, int numWanted, std::vector<Point2f> &new_pts)
{
    if (numWanted < 1)
    {
        return;
    }
    
    int max_attempts = 100, attempts = 0;
    int initialized = 0;
    
    int box_w = 60, box_h = 40;     // box size
    int half_box_w = box_w / 2, half_box_h = box_h / 2;
    int inner_w = Cam.nCols - 2*edge_band - box_w;
    int inner_h = Cam.nRows - 2*edge_band - box_h;
    
    
    //-----Features already exist
    Calculate_little_hs();
    std::vector<Point2f> uv_pred;
    for (size_t i=0; i<features_info.size(); i++)
    {
        if (features_info[i].hok)
        {
            uv_pred.push_back(Point2f(features_info[i].h(0), features_info[i].h(1)));
        }
    }
    
    // Clear the predicted hs
    for (size_t i=0; i<features_info.size(); i++)
    {
        features_info[i].hok = false;
    }
    
    
    // It seems that we don't need to assign features into grids,
    // because for deciding whether the neighborhood is clean or not,
    // the computational cost is rather low.
    
    std::vector<KeyPoint> key_pts;
    int rand_times = 0;
    while (initialized < numWanted && attempts < max_attempts && rand_times<100) {
        
        // Generate a random point
        rand_times++;
        double ran1 = (double)rand() / RAND_MAX;
        double ran2 = (double)rand() / RAND_MAX;
        int centerx = ran1 * inner_w + edge_band + half_box_w;
        int centery = ran2 * inner_h + edge_band + half_box_h;
        
        // Is the neighborhood clean?
        bool isClean = true;
        for (size_t i=0; i<uv_pred.size(); i++) {
            float distx = abs(uv_pred[i].x - centerx);
            float disty = abs(uv_pred[i].y - centery);
            if (distx < half_box_w && disty < half_box_h) {
                isClean = false;
                break;
            }
        }
        
        // If clean, detect fast corners and select the strongest one
        if (isClean) {
            attempts++;
            
            int topleftx = centerx - half_box_w - 3;
            int toplefty = centery - half_box_h - 3;
            Rect roi(topleftx, toplefty, box_w+6, box_h+6);
            
            
            key_pts.clear();
            FAST(I(roi), key_pts, fast_threshold, true);   // detect FAST corners
            
            if (!key_pts.empty()) {  // Select a keypoint with the highest Shi-Tomashi score
                int besti = 0;
                float bestScore = key_pts[0].response;
                for (size_t i=1; i<key_pts.size(); i++) {
                    float score = key_pts[i].response;
                    if (score > bestScore) {
                        bestScore = score;
                        besti = i;
                    }
                }
                Point2f bestPt = key_pts[besti].pt + Point2f(topleftx, toplefty);
                new_pts.push_back(bestPt);
                uv_pred.push_back(bestPt);
                initialized++;
            }
            
        } // if clean...
    } // while...
    
}



void EKFSystem::AddToFilter(std::vector<Point2f> &newFeatures, double initial_rho, double std_rho)
{
    // 1. x_k_k
    // 2. P_k_k
    
    if (newFeatures.empty())
    {
        return;
    }
    int N = (int)newFeatures.size();
    int N3 = N * 3;
    
    // Camera intrinsics
    double Cx, Cy, ifx, ify;
    Cx = Cam.Cx;  Cy = Cam.Cy;
    ifx = 1.0 / Cam.fx;
    ify = 1.0 / Cam.fy;
    
    // Camera pose
    EgV3d r_Wc = x_k_k.head(3);
    EgQuatd q_Cw(x_k_k(3), x_k_k(4), x_k_k(5), x_k_k(6));
    EgM3d R_Wc = q_Cw.matrix();
    EgM3d Rq0, Rqx, Rqy, Rqz;
    dR_by_dq(q_Cw, Rq0, Rqx, Rqy, Rqz);
    
    // New (theta, phi, rho) s
    EgVxd new_tprs(N3);
    EgMxd dnewY_drq = EgMxd::Zero(3+N3, 7);
    dnewY_drq(0,0) = 1.0;
    dnewY_drq(1,1) = 1.0;
    dnewY_drq(2,2) = 1.0;
    
    Eigen::MatrixXd P_add = Eigen::MatrixXd::Zero(N3+3, N3+3);
    double cov_rho = std_rho * std_rho;
    double cov_pixel = sigma_pixel * sigma_pixel;
    
    Eigen::Matrix<double, 3, 4> dgw_dq;
    Eigen::Matrix<double, 2, 4> dthetaphi_dq;
    Eigen::Matrix<double, 2, 3> dthetaphi_dgw;
    Eigen::Matrix2d dthetaphi_duv;
    Eigen::Matrix<double, 3, 2> dgw_duv;
    
    Eigen::Matrix<double, 3, 2> tempM32;
    tempM32 << ifx, 0, 0, ify, 0, 0;
    dgw_duv = R_Wc * tempM32;
    
    //
    for (int i=0; i<N; i++) {
        
        // Bearing vector in camera frame
        EgV3d vcbear;
        vcbear << (newFeatures[i].x-Cx)*ifx,
                  (newFeatures[i].y-Cy)*ify,
                  1;
        
        // Bearing vector transformed to world frame
        EgV3d vwbear = R_Wc * vcbear;
        
        // Inverse parameters
        double theta, phi;
        theta = atan2(vwbear(0), vwbear(2));
        phi   = atan2(-vwbear(1), sqrt(pow(vwbear(0), 2) + pow(vwbear(2), 2)));
        
        //
        int i3 = i*3;
        new_tprs.segment(i3, 3) << theta, phi, initial_rho;
        
        //
        double nx = vwbear(0), ny = vwbear(1), nz = vwbear(2);
        double nxz2 = nx * nx + nz * nz;
        double nnn  = 1.0 / ((nxz2 + ny*ny)*sqrt(nxz2));
        dthetaphi_dgw << nz/nxz2,           0,     -nx/nxz2,
        nx*ny*nnn,  -nxz2*nnn,   ny*nz*nnn;
        
        dgw_dq << Rq0*vcbear, Rqx*vcbear, Rqy*vcbear, Rqz*vcbear;
        
        dthetaphi_duv = dthetaphi_dgw * dgw_duv;
        
        dnewY_drq.block(i3+3, 3, 2, 4) = (dthetaphi_dgw * dgw_dq); // dthetaphi_dq
        
        P_add.block(i3+3, i3+3, 2, 2) = cov_pixel * dthetaphi_duv * dthetaphi_duv.transpose();
        P_add(i3+5,i3+5) = cov_rho;
        
    }
    
    // State vector
    int old_size = int(x_k_k.size());
    x_k_k.conservativeResize(old_size+3+N3);
    x_k_k.tail(3+N3) << r_Wc, new_tprs;
    
    // Covariance matrix
    Eigen::MatrixXd P_rq = P_k_k.topLeftCorner(7, 7);
    Eigen::MatrixXd P_temp;
    P_temp = dnewY_drq * (P_k_k.topRows(7));
    
    P_k_k.conservativeResize(x_k_k.size(), x_k_k.size());
    P_k_k.bottomLeftCorner(3+N3, old_size) = P_temp;
    P_k_k.topRightCorner(old_size, 3+N3) = P_temp.transpose();
    P_k_k.bottomRightCorner(3+N3, 3+N3) = dnewY_drq * P_rq * dnewY_drq.transpose() + P_add;
}



void EKFSystem::AddToInfos(std::vector<Point2f> &newFeatures, cv::Mat &I, int iStep)
{
    if (newFeatures.empty()) return;
    
    int N = int(newFeatures.size());
    
    // poses_info-----------------------------
    poses_info.reserve(poses_info.size()+1);
    poses_info.push_back(CamPose(iStep, N));
    
    
    // features_info------------------------------
    features_info.reserve(features_info.size()+N);
    
    int l_ini = 2 * hps_ini + 1;
    int l_mat = 2 * hps_mat + 1;
    
    for (int i=0; i<N; i++) {
        //
        int u = newFeatures[i].x;
        int v = newFeatures[i].y;
        
        Mat patch_ini = I(Rect(u-hps_ini, v-hps_ini, l_ini, l_ini));
        Mat patch_mat = I(Rect(u-hps_mat, v-hps_mat, l_mat, l_mat));
        
        features_info.push_back(Feature(patch_ini, patch_mat, iStep));
        
    }
    
}



void EKFSystem::DeleteFeatures()
{
    std::vector<int> i_feature_godie;
    std::vector<int> i_pose_godie;
    Eigen::VectorXi isBad = Eigen::VectorXi::Zero(x_k_k.size());
    Eigen::Vector3i one3(1,1,1);
    
    int k=NcamX, l=0, pose_k;
    for (size_t i=0; i<poses_info.size(); i++)
    {
        int count = poses_info[i].Count;
        int ID = poses_info[i].FrameID;
        pose_k = k;
        k += 3;
        
        for (int j=0; j<count; j++)
        {
            if (features_info[l].FrameID != ID)
            {
                std::cout << "Error: In DeleteFeatures, ID mismatch\n";
                waitKey();
            }
            
            if ((features_info[l].times_predicted > 5 && features_info[l].times_measured<features_info[l].times_predicted) || features_info[l].times_notmeasured > 2 || features_info[i].too_uncertain)
            {
                i_feature_godie.push_back(l);
                poses_info[i].Count--;
                isBad.segment(k,3) = one3;
            }
            
            l++;
            k += 3;
        }
        
        if (poses_info[i].Count < 1)
        {
            i_pose_godie.push_back(i);
            isBad.segment(pose_k, 3) = one3;
        }
    }
    std::cout << "godie:\t" << i_feature_godie.size() << "\n";
    
    // -----------------------------------
    
    // features_info
    for (size_t i=i_feature_godie.size(); i>0; i--)
    {
        int i_f = i_feature_godie[i-1];
        features_info.erase(features_info.begin()+i_f);
        
        int et0 = entries[0][i_f], et1 = entries[1][i_f];
        
        EgV3d ri = x_k_k.segment(et0, 3);
        EgV3d tpr = x_k_k.segment(et1, 3);
        double std_rho = sqrt(P_k_k(et1+2, et1+2));
        if (std_rho > 0.2*tpr(2) || tpr(2)<0.05 ) {
            continue;
        }
        double cosp = cos(tpr(1));
        EgV3d mi(cosp * sin(tpr(0)), -sin(tpr(1)), cosp * cos(tpr(0)));
        EgV3d rW = ri + mi / tpr(2);
        
        map_points.push_back(rW);
        
    }
    
    // poses_info
    for (size_t i=i_pose_godie.size(); i>0; i--)
    {
        int i_p = i_pose_godie[i-1];
        poses_info.erase(poses_info.begin()+i_p);
    }
    
    // x_k_k
    int N = (int)x_k_k.size()-3;
    for (int i=N; i>NcamX-1; i-=3)
    {
        if (isBad(i)>0)
        {
            int S = (int)x_k_k.size();
            int L = S-i-3;
            if (L > 0) {
                x_k_k.segment(i, L) = x_k_k.segment(i+3, L);
            }
            x_k_k.conservativeResize(S-3);
        }
    }
    
    // P_k_k
    for (int i=N; i>NcamX-1; i-=3)
    {
        if (isBad(i)>0)
        {
            int H = (int)P_k_k.cols();
            int L = H-i-3;
            if (L > 0)
            {
                P_k_k.block(i, 0, L, H) = P_k_k.block(i+3,0, L,H);
                P_k_k.block(0, i, H-3, L) = P_k_k.block(0, i+3, H-3, L);
            }
            P_k_k.conservativeResize(H-3,H-3);
        }
    }
    
}



void EKFSystem::UpdateEntries()
{
    if (features_info.empty())
    {
        return;
    }
    
    int N = (int)features_info.size();
    entries[0].clear();     entries[1].clear();
    entries[0].reserve(N);  entries[1].reserve(N);
    
    int k=NcamX, pose_k, l=0;
    for (size_t i=0; i<poses_info.size(); i++) {
        
        pose_k = k;
        k += 3;
        int ID = poses_info[i].FrameID;
        
        for (int j=0; j<poses_info[i].Count; j++) {
            
            if (features_info[l].FrameID != ID) {
                std::cout << "Error : In UpdateEntries function, FrameID mismatch\n";
                waitKey();
            } else {
                entries[0].push_back(pose_k);
                entries[1].push_back(k);
            }
            
            l++;
            k += 3;
        }
    }
    
    if (entries[0].size() != N) {
        std::cout << "Error : In UpdateEntries function, entries & features_info size mismatch\n";
        waitKey();
    }
    
}



int EKFSystem::UpdateFeaturesInfo()
{
    int numMeasured = 0;
    for (size_t i=0; i<features_info.size(); i++)
    {
        if (features_info[i].hok)
        {
            features_info[i].times_predicted++;
        }
        
        if (features_info[i].low_innovation_inlier || features_info[i].high_innovation_inlier)
        {
            numMeasured++;
            features_info[i].times_measured++;
        }
        else
        {
            features_info[i].times_notmeasured++;
        }
        
        features_info[i].compatible = 0;
        features_info[i].hok = 0;
        features_info[i].zok = 0;
        features_info[i].low_innovation_inlier  = 0;
        features_info[i].high_innovation_inlier = 0;
    }
    
    return numMeasured;
}



void EKFSystem::EstimateMedianInverseDepth()
{
    // Get all valid features' inverse depth
    std::vector<double> vRhos;
    vRhos.reserve(features_info.size());
    
    for (size_t i=0; i<features_info.size(); i++)
    {
        if (features_info[i].low_innovation_inlier
            || features_info[i].high_innovation_inlier)
        {
            int et = entries[1][i] + 2;
            vRhos.push_back(x_k_k(et));
        }
    }
    
    // Calculate new median (fused with prior information)
    double median;
    size_t size = vRhos.size();
    if (size > 20)
    {
        std::sort(vRhos.begin(), vRhos.end());
        
        if (size % 2 == 0)
        {
            median = (vRhos[size/2-1] + vRhos[size/2]) / 2.0;
        }
        else
        {
            median = vRhos[size/2];
        }
        
        if (median<max_inverse_depth && median>min_inverse_depth)
        {
            median = (median*(double)size + prior_inverse_depth*25.0) / ((double)size+25.0);
            well_estimated = true;
        }
        else
        {
            well_estimated = false;
            median = prior_inverse_depth;
        }
    }
    else
    {
        well_estimated = false;
        median = prior_inverse_depth;
    }
    
    median_inverse_depth = (median_inverse_depth + median) / 2.0;
    
    std::cout << "Median inverse depth:\t" << median_inverse_depth << "\n";
}



void EKFSystem::Calculate_little_hs()
{
    // Camera intrinsics
    double fx = Cam.fx;
    double fy = Cam.fy;
    double Cx = Cam.Cx;
    double Cy = Cam.Cy;
    int umax = Cam.nCols-1;
    int vmax = Cam.nRows-1;
    
    // Camera pose
    EgV3d t_Wc = x_k_k.head(3);
    EgQuatd q_Cw(x_k_k(3), x_k_k(4), x_k_k(5), x_k_k(6));
    EgM3d R_Cw = q_Cw.matrix().transpose();
    
    // Predict features' position in image plane
    EgV3d ri, mi;
    double theta, phi, rho;
    for (size_t i=0; i<features_info.size(); i++)
    {
        int et1 = entries[0][i];
        int et2 = entries[1][i];
        
        // Get inverse-depth parameters
        ri << x_k_k[et1], x_k_k[et1+1], x_k_k[et1+2];
        theta = x_k_k[et2];
        phi   = x_k_k[et2+1];
        rho   = x_k_k[et2+2];
        if (rho < min_inverse_depth)
        {
            continue;
        }
        
        // 3D location in camera-frame (up to a scale factor rho)
        double cphi = cos(phi);
        mi(0) = cphi * sin(theta);
        mi(1) = -sin(phi);
        mi(2) = cphi * cos(theta);
        EgV3d xc = R_Cw * (rho*(ri-t_Wc) + mi);
        
        // Is it in front of the camera?
        if (xc(2)<0.05 || xc(2)>30)
        {
            continue;
        }
        
        double angle1 = fabs(atan2(xc(0), xc(2)));
        double angle2 = fabs(atan2(xc(1), xc(2)));
        if (angle1>max_angle || angle2>max_angle)
        {
            continue;
        }
        
        // Project to the image plane
        double invz = 1.0 / xc(2);
        double u = fx * xc(0) * invz + Cx;
        double v = fy * xc(1) * invz + Cy;
        
        // Is it visible in this camera?
        if (u>0 && u<umax && v>0 && v<vmax)
        {
            features_info[i].h << u,v;
            features_info[i].hok = true;
        }
    }
}



void EKFSystem::Calculate_big_Hs() {
    
    double fx=Cam.fx, fy=Cam.fy;
    
    EgV3d t_Wc = x_k_k.head(3);
    Eigen::Quaterniond q_Cw(x_k_k(3), x_k_k(4), x_k_k(5), x_k_k(6));
    Eigen::Quaterniond q_Wc = q_Cw.inverse();
    Eigen::Matrix3d R_Cw = q_Wc.matrix();
    
    Eigen::Matrix3d Rq0, Rqx, Rqy, Rqz;
    dR_by_dq(q_Wc, Rq0, Rqx, Rqy, Rqz);
    
    
    //------
    EgV3d ri, mi, xw, xwm, xc;
    double theta, phi, rho;
    Eigen::Matrix<double, 2, 3> dh_by_dxc, tempM23;
    Eigen::Matrix<double, 3, 4> tempM34;
    Eigen::Matrix3d dxwm_by_dtpr;
    
    //----------------------------------------------------
    for (size_t i=0; i<features_info.size(); i++) {
        if (!features_info[i].hok) {
            continue;
        }
        
        // Entry
        int et1 = entries[0][i];
        int et2 = entries[1][i];
        
        // Get inverse-depth parameters
        ri << x_k_k[et1], x_k_k[et1+1], x_k_k[et1+2];
        theta = x_k_k[et2];
        phi   = x_k_k[et2+1];
        rho   = x_k_k[et2+2];
        
        // Some temporary variable
        double cp, sp, ct, st;
        cp = cos(phi);
        sp = sin(phi);
        ct = cos(theta);
        st = sin(theta);
        mi << cp*st, -sp, cp*ct;
        
        // Point 3D in world/camera frame (up to a scale factor)
        xw  = ri * rho + mi;
        xwm = xw - rho * t_Wc;
        xc  = R_Cw * xwm;
        double icz  = 1.0/xc(2);
        double icz2 = icz * icz;
        
        
        // --------Intermediate Derivatives----------------
        dh_by_dxc << fx*icz,    0,      -fx*icz2*xc(0),
        0,      fy*icz,    -fy*icz2*xc(1);
        
        tempM23 = dh_by_dxc * R_Cw;
        
        tempM34 << Rq0*xwm, Rqx*(-xwm), Rqy*(-xwm), Rqz*(-xwm);
        
        dxwm_by_dtpr << cp*ct,  -sp*st,     ri(0)-t_Wc(0),
        0,      -cp,        ri(1)-t_Wc(1),
        -cp*st,  -sp*ct,     ri(2)-t_Wc(2);
        
        
        // --------Derivatives-----------------------------------
        features_info[i].H_rq  << (-rho)*tempM23, dh_by_dxc*tempM34;
        
        features_info[i].H_ri  = rho*tempM23;
        features_info[i].H_tpr = tempM23 * dxwm_by_dtpr;
    }
}



void EKFSystem::Calculate_Ss() {
    
    double cov_pixel = sigma_pixel * sigma_pixel;
    Eigen::Matrix2d Ri;
    Ri << cov_pixel,    0,
            0,    cov_pixel;
    
    int lx = (int)x_k_k.size();
    
    Eigen::MatrixXd P_x_rq = P_k_k.leftCols(7);
    Eigen::Matrix<double, 2, 7> Hi_rq;
    Eigen::Matrix<double, 2, 3> Hi_ri, Hi_tpr;
    Eigen::MatrixXd P_x_ri(lx, 3), P_x_tpr(lx, 3);
    
    //--------------------------------------------------
    for (size_t i=0; i<features_info.size(); i++) {
        if (!features_info[i].hok) {
            continue;
        }
        
        // Entry
        int et1 = entries[0][i];
        int et2 = entries[1][i];
        
        // Get corresponding matries--------------
        P_x_ri  = P_k_k.block(0, et1, lx, 3);
        P_x_tpr = P_k_k.block(0, et2, lx, 3);
        
        Hi_rq  = features_info[i].H_rq;
        Hi_ri  = features_info[i].H_ri;
        Hi_tpr = features_info[i].H_tpr;
        
        
        // Calculat P*H'--------------------------
        Eigen::MatrixXd phT;
        phT = P_x_rq*(Hi_rq.transpose()) + P_x_ri*(Hi_ri.transpose()) + P_x_tpr*(Hi_tpr.transpose());
        
        
        // Then calculate H*phT + R
        features_info[i].S = (Hi_rq  * phT.topRows(7)
                              + Hi_ri  * phT.block(et1, 0, 3, 2)
                              + Hi_tpr * phT.block(et2, 0, 3, 2)
                              + Ri);
        
    }
}



void EKFSystem::NccMatching(cv::Mat& I) {
    int im_width  = I.cols;
    int im_height = I.rows;
    int ps_mat = 2*hps_mat + 1;
    
    Eigen::Vector2d h;
    Eigen::Matrix2d S;
    int hsrsx, hsrsy;  // half search region size
    int srs;
    int minsx, minsy, maxsx, maxsy;
    
    for (size_t i=0; i<features_info.size(); i++) {
        if (!features_info[i].hok) {
            continue;
        }
        
        h = features_info[i].h;
        S = features_info[i].S;
        
        if (S(0,0) < 1 || S(1,1) < 1) {
            features_info[i].hok = false;
            std::cout << "how can S be negative?\n";
            waitKey();
        }
        
        hsrsx = ceil(2*sqrt(S(0,0)));
        hsrsy = ceil(2*sqrt(S(1,1)));
        srs = (2*hsrsx+1) * (2*hsrsy+1);
        
        if (hsrsx>100 || hsrsy>100 || hsrsx*hsrsy>4900) {
            features_info[i].too_uncertain = true;
            std::cout << "too uncertain!\n";
            continue;
        }
        
        minsx = MAX(round(h(0))-hsrsx, hps_mat);
        minsy = MAX(round(h(1))-hsrsy, hps_mat);
        maxsx = MIN(round(h(0))+hsrsx, im_width -hps_mat-1);
        maxsy = MIN(round(h(1))+hsrsy, im_height-hps_mat-1);
        if (maxsx < minsx || maxsy < minsy) {
            continue;
        }
        
        // Template
        cv::Mat patch_T = features_info[i].patch_when_matching;
        
        // Image patch to be searched
        cv::Mat patch_I = I(Rect(minsx-hps_mat, minsy-hps_mat, maxsx-minsx+ps_mat, maxsy-minsy+ps_mat));
        
        // ---Matching---
        cv::Mat scores(maxsy-minsy+1, maxsx-minsx+1, CV_32FC1);
        matchTemplate(patch_I, patch_T, scores, CV_TM_CCOEFF_NORMED);
        
        double minScore, maxScore;
        Point minLoc, maxLoc;
        minMaxLoc(scores, &minScore, &maxScore, &minLoc, &maxLoc);
        
        if (maxScore > ncc_threshold) {
            Eigen::Vector2d z(minsx+maxLoc.x, minsy+maxLoc.y);
            features_info[i].z = z;
            features_info[i].zok = true;
            
            // ---Chi-Square Error Checking---
            Eigen::Vector2d nu = z - h;
            if (nu.transpose() * S.inverse() * nu < CHI2_099) {
                features_info[i].compatible = true;
            }
        }
        
    } // for...
}



void EKFSystem::OnePointRansac(double palosf) {
    
    // Max number of iteration, will be updated
    int N_hyp = 500;
    
    // Supporters' count
    int max_support = 0;
    
    //Compatible features' position in features_info
    std::vector<int> posis_cp;
    posis_cp.reserve(features_info.size());
    
    std::cout << "features_info\t" << features_info.size() << "\n";
    
    for (size_t i=0; i<features_info.size(); i++) {
        if (features_info[i].compatible) {
            posis_cp.push_back(i);
        }
    }
    
    int N_cp = (int)posis_cp.size();
    if (N_cp < 2) {
        return;
    }
    
    
    //----Ransac------------------------
    Eigen::VectorXd x_new(x_k_k.size());
    std::vector<int> i_inliers_best;
    
    int k=0;
    while (k<N_hyp) {
        k++;
        
        // --Randomly select a match
        double ran = (double)rand() / RAND_MAX;
        int i_rand = floor(ran*N_cp);
        int i_feature = posis_cp[i_rand];
        
        // --1-match EKF state update
        OnePointUpdateState(x_new, i_feature);
        
        // --Count supporters
        std::vector<int> i_inliers;
        i_inliers.reserve(N_cp);
        ComeOnGuys(x_new, posis_cp, i_inliers);
        int N_inliers = (int)i_inliers.size();
        
        // --Refresh
        if (N_inliers > max_support) {
            max_support = N_inliers;
            i_inliers_best = i_inliers;
            
            double epsilon = 1.0 - (double)N_inliers/N_cp;
            N_hyp = ceil(log(1-palosf) / log(epsilon));
            
        }
    } // while...
    
    //--Finally, we set the innovation inliers
    for (size_t i=0; i<i_inliers_best.size(); i++) {
        int i_feature = i_inliers_best[i];
        features_info[i_feature].low_innovation_inlier = 1;
    }
    
    std::cout << "low innovation inliers:\t" << max_support << "\n";
    
}



void EKFSystem::OnePointUpdateState(Eigen::VectorXd &xNew, int iFeature) {
    if (!features_info[iFeature].zok) {
        std::cout << "Error: OnePointUpdateState, feature not measured\n";
        waitKey();
    }
    
    int lx = (int)x_k_k.size();
    
    // Entry
    int et1 = entries[0][iFeature];
    int et2 = entries[1][iFeature];
    
    // Get corresponding covariance block
    Eigen::MatrixXd P_x_rq = P_k_k.leftCols(7);
    Eigen::MatrixXd P_x_ri = P_k_k.block(0, et1, lx, 3);
    Eigen::MatrixXd P_x_tpr = P_k_k.block(0, et2, lx, 3);
    
    // Calculat P*H'--------------------------
    Eigen::MatrixXd phT;
    phT = P_x_rq  * (features_info[iFeature].H_rq).transpose()
    + P_x_ri  * (features_info[iFeature].H_ri).transpose()
    + P_x_tpr * (features_info[iFeature].H_tpr).transpose();
    
    Eigen::Matrix2d S = features_info[iFeature].S;
    Eigen::MatrixXd K = phT * S.inverse();
    
    xNew = x_k_k + K * (features_info[iFeature].z - features_info[iFeature].h);
    
    // Quaternion normalization
    xNew.segment(3, 4).normalize();
    
}


void EKFSystem::ComeOnGuys(Eigen::VectorXd &xNew, std::vector<int>& posis_cp, std::vector<int> &iInliers)
{
    EgV3d t_Wc = xNew.head(3);
    Eigen::Quaterniond q_Cw(xNew(3), xNew(4), xNew(5), xNew(6));
    Eigen::Matrix3d R_Cw = q_Cw.matrix().transpose();
    
    double fx = Cam.fx, fy = Cam.fy;
    double Cx = Cam.Cx, Cy = Cam.Cy;
    
    //-------------
    EgV3d ri, mi;
    double theta, phi, rho;
    //------------------------------------------------
    for (int k=0; k<posis_cp.size(); k++) {
        int i_feature = posis_cp[k];
        
        // Entry
        int et1 = entries[0][i_feature];
        int et2 = entries[1][i_feature];
        
        // Get inverse-depth parameters
        ri = xNew.segment(et1, 3);
        theta = xNew[et2];
        phi   = xNew[et2+1];
        rho   = xNew[et2+2];
        
        // Point3D in camera-frame (up to a scale factor : rho)
        double cphi = cos(phi);
        mi << cphi * sin(theta),
        -sin(phi),
        cphi * cos(theta);
        EgV3d xc = R_Cw * (rho*(ri-t_Wc) + mi);
        
        // Projection
        double invz = 1.0 / xc(2);
        Eigen::Vector2d h;
        h << fx * xc(0) * invz + Cx,
        fy * xc(1) * invz + Cy;
        
        // Error
        double dist_pixel = (h-features_info[i_feature].z).norm();
        
        if (dist_pixel < 2) {
            iInliers.push_back(i_feature);
        }
    }
}



void EKFSystem::EkfUpdateLowInnos() {
    
    // Get low innovation inlier indices in features_info
    std::vector<int> i_low_innos;
    i_low_innos.reserve(features_info.size());
    
    for (size_t i=0; i<features_info.size(); i++) {
        if (features_info[i].low_innovation_inlier) {
            i_low_innos.push_back(i);
        }
    }
    
    // Update
    EkfUpdate(i_low_innos);
    
}

void EKFSystem::EkfUpdateHighInnos() {
    
    // Get high innovation inlier indices in features_info
    std::vector<int> i_high_innos;
    i_high_innos.reserve(features_info.size());
    
    for (size_t i=0; i<features_info.size(); i++) {
        if (features_info[i].high_innovation_inlier) {
            i_high_innos.push_back(i);
        }
    }
    
    // Update
    EkfUpdate(i_high_innos);
    
    std::cout << "high inliers:\t" << i_high_innos.size() << "\n";
}



void EKFSystem::EkfUpdate(std::vector<int>& iFeatures)
{
    if (iFeatures.empty()) return;
    int M = (int)x_k_k.size();
    int N  = (int)iFeatures.size();
    int N2 = 2*N;
    
    // Get z & h----------------------
    Eigen::VectorXd z(N2), h(N2);
    
    for (int k=0; k<iFeatures.size(); k++) {
        int i_feature = iFeatures[k];
        int k2 = 2*k;
        
        z.segment(k2, 2) = features_info[i_feature].z;
        h.segment(k2, 2) = features_info[i_feature].h;
    }
    
    
    // Compute P*H'----------------------
    Eigen::MatrixXd PHT(M, N2);
    
    for (int k=0; k<iFeatures.size(); k++) {
        int i_feature = iFeatures[k];
        int k2 = 2*k;
        
        // Entry
        int et1 = entries[0][i_feature];
        int et2 = entries[1][i_feature];
        
        //
        Eigen::MatrixXd P_HiT;
        P_HiT = P_k_k.leftCols(7)      * (features_info[i_feature].H_rq).transpose()
        + P_k_k.block(0,et1,M,3) * (features_info[i_feature].H_ri).transpose()
        + P_k_k.block(0,et2,M,3) * (features_info[i_feature].H_tpr).transpose();
        PHT.block(0, k2, M, 2) = P_HiT;
    }
    
    
    // Compute H*P*H'--------------------------
    Eigen::MatrixXd S(N2, N2);
    
    for (int k=0; k<iFeatures.size(); k++) {
        int i_feature = iFeatures[k];
        int k2 = 2*k;
        
        // Entry
        int et1 = entries[0][i_feature];
        int et2 = entries[1][i_feature];
        
        //
        Eigen::MatrixXd Hi_PHT;
        Hi_PHT = features_info[i_feature].H_rq  * PHT.topRows(7)
        + features_info[i_feature].H_ri  * PHT.block(et1, 0, 3, N2)
        + features_info[i_feature].H_tpr * PHT.block(et2, 0, 3, N2);
        
        S.block(k2, 0, 2, N2) = Hi_PHT;
    }
    
    
    // Then compute S = H*P*H' + R-----
    double cov_pixel = sigma_pixel * sigma_pixel;
    for (int k=0; k<N2; k++) {
        S(k,k) += cov_pixel;
    }
    
    
    // Kalman gain K = P * H' / S--
    Eigen::MatrixXd K(M, N2);
    K = PHT * S.inverse();
    
    
    // Update state & covariance
    x_k_k += K * (z-h);
    P_k_k -= PHT * K.transpose();
    
    P_k_k = (P_k_k + P_k_k.transpose()) / 2;
    
    // Quaternion normalization
    x_k_k.segment(3, 4).normalize();
	x_k_k.segment(16, 3) = gravity_magnitude * x_k_k.segment(16,3)/x_k_k.segment(16,3).norm();	
    
    // TO DO: Covariance shall change when normalizing the quaternion
    // Now we simply omit this part
    
}


void EKFSystem::RescueHighInnos() {
    
    //-------------------------------------------
    // ------Recalculate h, H, S--------
    //---using the new x_k_k & P_k_k----------
    //--------------------------------------------
    // 1. Predict camera measurements
    Calculate_little_hs();
    
    // 2. Calculate derivatives
    Calculate_big_Hs();
    
    // 3. Calculate innovations
    Calculate_Ss();
    
    
    //---------------------------------
    //---ChiSquare Error Checking-----
    //---------------------------------
    for (size_t i=0; i<features_info.size(); i++) {
        if (features_info[i].zok && (!features_info[i].low_innovation_inlier)) {
            Eigen::Vector2d nu = features_info[i].z - features_info[i].h;
            Eigen::Matrix2d S = features_info[i].S;
            
            if (nu.transpose() * S.inverse() * nu < CHI2_095) {
                features_info[i].high_innovation_inlier = true;
            }
        }
    }
}



void EKFSystem::PreIntegrate(std::vector<double> &ts,
                          std::vector<EgV3d> &wms,
                          std::vector<EgV3d> &ams,
                          EgV3d &pp, EgQuatd &qq, EgV3d &vv,
                          Eigen::Matrix<double, 10, 10> &RR)
{
    // ----- Set F
    Eigen::Matrix<double, 10, 19> F;
    
    
    
    
    
    
}

