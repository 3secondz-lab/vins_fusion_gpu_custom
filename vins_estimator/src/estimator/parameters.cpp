/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
int USE_GPU;
int USE_GPU_ACC_FLOW;
int PUB_RECTIFY;
Eigen::Matrix3d rectify_R_left;
Eigen::Matrix3d rectify_R_right;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

// Crop parameters
int CROP_TOP = 0;
int CROP_BOTTOM = 0;

// Intrinsic modification helper
bool modifyIntrinsicParams(std::string path_in, std::string path_out, int image_height_orig, int crop_top, int crop_bottom){
    // Adjust intrinsic parameters with crop parameters
    cv::FileStorage fs(path_in, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cerr << "ERROR: Wrong path to cam0 parameters : " << path_in << std::endl;
        return false;
    }

    // Read original parameters
    std::string model_type = fs["model_type"];
    std::string camera_name = fs["camera_name"];
    int image_width = fs["image_width"];
    int image_height = fs["image_height"];

    double k1 = fs["distortion_parameters"]["k1"];
    double k2 = fs["distortion_parameters"]["k2"];
    double p1 = fs["distortion_parameters"]["p1"];
    double p2 = fs["distortion_parameters"]["p2"];

    double fx = fs["projection_parameters"]["fx"];
    double fy = fs["projection_parameters"]["fy"];
    double cx = fs["projection_parameters"]["cx"];
    double cy = fs["projection_parameters"]["cy"];

    fs.release();

    ROS_INFO_STREAM("Original parameters :");
    ROS_INFO_STREAM("[width, height] : [" << image_width << ", " << image_height << "]");
    ROS_INFO_STREAM("[k1, k2, p1, p2] : [" << k1 << ", " << k2 << ", " << p1 << ", " << p2 << "]");
    ROS_INFO_STREAM("[fx, fy, cx, cy] : [" << fx << ", " << fy << ", " << cx << ", " << cy << "]");

    // Apply crop
    cy = cy - (double)crop_top;

    image_height = image_height - crop_top - crop_bottom;

    if(image_height_orig != image_height){
        ROS_ERROR_STREAM("Inconsistent image height : " << image_height_orig << " and " << image_height);
        return false;
    };

    // Save adjusted parameters
    cv::FileStorage fs_out(path_out, cv::FileStorage::WRITE);
    if(!fs_out.isOpened())
    {
        std::cerr << "ERROR: Failed to open : " << path_out << std::endl;
        return false;
    };

    fs_out << "model_type" << model_type;
    fs_out << "camera_name" << camera_name;

    fs_out << "image_width" << image_width;
    fs_out << "image_height" << image_height;

    fs_out << "distortion_parameters";
    fs_out << "{" << "k1" << k1;
    fs_out << "k2" << k2;
    fs_out << "p1" << p1;
    fs_out << "p2" << p2 << "}";

    fs_out << "projection_parameters";
    fs_out << "{" << "fx" << fx;
    fs_out << "fy" << fy;
    fs_out << "cx" << cx;
    fs_out << "cy" << cy << "}";

    fs_out.release();

    ROS_INFO_STREAM("Modified parameters :");
    ROS_INFO_STREAM("[width, height] : [" << image_width << ", " << image_height << "]");
    ROS_INFO_STREAM("[k1, k2, p1, p2] : [" << k1 << ", " << k2 << ", " << p1 << ", " << p2 << "]");
    ROS_INFO_STREAM("[fx, fy, cx, cy] : [" << fx << ", " << fy << ", " << cx << ", " << cy << "]");

    return true;
}


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_GPU = fsSettings["use_gpu"];
    USE_GPU_ACC_FLOW = fsSettings["use_gpu_acc_flow"];

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }

    // Crop parameters
    CROP_TOP = fsSettings["crop_top"];
    CROP_BOTTOM = fsSettings["crop_bottom"];

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];

    ROW = ROW - CROP_TOP - CROP_BOTTOM;

    if(ROW <= 0){
        ROS_ERROR_STREAM("Invalid image height after crop : " << ROW << endl
                         << ", CROP : [" << CROP_TOP << ", " << CROP_BOTTOM << "]");
        return;
    }

    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    std::string cam0PathModified = configPath + "/cam0_modified.yaml";

    ROS_INFO_STREAM("\nAdjust cam0 intrinsic parameters...\n");
    if(!modifyIntrinsicParams(cam0Path, cam0PathModified, ROW, CROP_TOP, CROP_BOTTOM)){
        ROS_ERROR_STREAM("Failed to adjust cam0 parameters.");
        return;
    }
    CAM_NAMES.push_back(cam0PathModified);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        std::string cam1PathModified = configPath + "/cam1_modified.yaml";
        //printf("%s cam1 path\n", cam1Path.c_str() );

        ROS_INFO_STREAM("\nAdjust cam1 intrinsic parameters...\n");
        if(!modifyIntrinsicParams(cam1Path, cam1PathModified, ROW, CROP_TOP, CROP_BOTTOM)){
            ROS_ERROR_STREAM("Failed to adjust cam1 parameters.");
            return;
        }
        CAM_NAMES.push_back(cam1PathModified);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
        fsSettings["publish_rectify"] >> PUB_RECTIFY;
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }
    if(PUB_RECTIFY)
    {
        cv::Mat rectify_left;
        cv::Mat rectify_right;
        fsSettings["cam0_rectify"] >> rectify_left;
        fsSettings["cam1_rectify"] >> rectify_right;
        cv::cv2eigen(rectify_left, rectify_R_left);
        cv::cv2eigen(rectify_right, rectify_R_right);

    }

    fsSettings.release();
}
