//
// Created by xchu on 27/5/2022.
//

#include "base_type.hpp"
#include "runtime_config.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <opencv2/core.hpp>
#include <sstream>

namespace {

constexpr const char *kColorInfo = "\033[32m";
constexpr const char *kColorWarn = "\033[33m";
constexpr const char *kColorError = "\033[31m";
constexpr const char *kColorReset = "\033[0m";
constexpr const char *kCalibrationParamRosKey = "/ms_mapping/runtime_calibration_param_path";

std::string FormatVector(const Eigen::Vector3d &vec) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "[" << vec.x() << ", " << vec.y() << ", " << vec.z() << "]";
    return oss.str();
}

std::string FormatMatrix(const Eigen::Matrix3d &mat) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << "[";
    for (int r = 0; r < 3; ++r) {
        if (r != 0) {
            oss << " ";
        }
        oss << "[";
        for (int c = 0; c < 3; ++c) {
            if (c != 0) {
                oss << ", ";
            }
            oss << mat(r, c);
        }
        oss << "]";
        if (r != 2) {
            oss << ",";
        }
    }
    oss << "]";
    return oss.str();
}

std::string InferExtrinsicPrefixFromTopic(const std::string &lid_topic) {
    if (lid_topic.find("main_left") != std::string::npos ||
        lid_topic.find("pandar") != std::string::npos) {
        return "imu_in_main_left";
    }
    if (lid_topic.find("top_front") != std::string::npos) {
        return "imu_in_top_front";
    }
    return "";
}

bool LoadInverseExtrinsicFromCalibration(const std::string &file_path,
                                         const std::string &rotation_key,
                                         const std::string &translation_key,
                                         Eigen::Matrix3d &R_inv,
                                         Eigen::Vector3d &t_inv,
                                         std::string &error_msg) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        error_msg = "failed to open file";
        return false;
    }

    cv::Mat rot_cv, trans_cv;
    fs[rotation_key] >> rot_cv;
    fs[translation_key] >> trans_cv;

    if (rot_cv.empty() || trans_cv.empty()) {
        error_msg = "missing " + rotation_key + " or " + translation_key;
        return false;
    }

    cv::Mat rot_cv64, trans_cv64;
    rot_cv.convertTo(rot_cv64, CV_64F);
    trans_cv.convertTo(trans_cv64, CV_64F);

    if (rot_cv64.rows != 3 || rot_cv64.cols != 3) {
        error_msg = "rotation matrix size is not 3x3";
        return false;
    }

    if (trans_cv64.total() != 3) {
        error_msg = "translation vector does not contain 3 elements";
        return false;
    }

    Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            rot(r, c) = rot_cv64.at<double>(r, c);
        }
    }

    Eigen::Vector3d trans = Eigen::Vector3d::Zero();
    if (trans_cv64.rows == 3 && trans_cv64.cols == 1) {
        for (int r = 0; r < 3; ++r) {
            trans(r) = trans_cv64.at<double>(r, 0);
        }
    } else if (trans_cv64.rows == 1 && trans_cv64.cols == 3) {
        for (int c = 0; c < 3; ++c) {
            trans(c) = trans_cv64.at<double>(0, c);
        }
    } else {
        error_msg = "translation matrix is not 3x1 or 1x3";
        return false;
    }

    Eigen::Matrix3d rot_inv = rot.transpose();
    Eigen::Vector3d trans_inv = -rot_inv * trans;

    R_inv = rot_inv;
    t_inv = trans_inv;
    return true;
}

}  // namespace


std::string toString(Pose6D &pose)
{
    return "(" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ", " +
           std::to_string(pose.z) + ", " + std::to_string(pose.roll) + ", " +
           std::to_string(pose.pitch) + ", " + std::to_string(pose.yaw) + ")";
}

extern Pose6D getOdom(nav_msgs::Odometry _odom)
{
    auto tx = _odom.pose.pose.position.x;
    auto ty = _odom.pose.pose.position.y;
    auto tz = _odom.pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom.pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w))
        .getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw};
    //    return Pose6D{tx, ty, tz, roll, pitch, yaw, _odom.header.seq};
}

extern Pose6D diffTransformation(const Pose6D &_p1, const Pose6D &_p2)
{
    Eigen::Affine3f SE3_p1 =
        pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 =
        pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta;
    SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)),
                  double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
}

extern Eigen::Matrix4d Pose6D2Matrix(const Pose6D &p)
{
    Eigen::Translation3d tf_trans(p.x, p.y, p.z);
    Eigen::AngleAxisd rot_x(p.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(p.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(p.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d mat = (tf_trans * rot_z * rot_y * rot_x).matrix();
    return mat;
}

extern Pose6D Pose6D2Body(const Pose6D &p, const Eigen::Matrix4d &matrix)
{
    Eigen::Translation3d tf_trans(p.x, p.y, p.z);
    Eigen::AngleAxisd rot_x(p.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(p.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(p.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d mat = (tf_trans * rot_z * rot_y * rot_x).matrix();

    // lidar -> imu T_imu_lidar * Pose_lidar
    Eigen::Matrix4d imu_pose = matrix * mat;
    Eigen::Vector3d pos = imu_pose.block<3, 1>(0, 3).matrix();
    Eigen::Matrix3d rot = imu_pose.block<3, 3>(0, 0).matrix();
    Eigen::Quaterniond quat(rot);

    Pose6D p2;
    p2.x = pos(0);
    p2.y = pos(1);
    p2.z = pos(2);
    tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()))
        .getRPY(p2.roll, p2.pitch, p2.yaw);
    return p;
}

extern Pose6D Matrix2Pose6D(const Eigen::Matrix4d &matrix)
{
    Eigen::Vector3d pos = matrix.block<3, 1>(0, 3).matrix();
    Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0).matrix();
    Eigen::Quaterniond quat(rot);

    Pose6D p;
    p.x = pos(0);
    p.y = pos(1);
    p.z = pos(2);
    tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()))
        .getRPY(p.roll, p.pitch, p.yaw);
    return p;
}

extern Pose6D Isometry3d2Pose6D(const Eigen::Isometry3d &matrix)
{
    Eigen::Matrix3d rot = matrix.rotation().matrix();
    Eigen::Quaterniond quat(rot);
    // quat.normalize();

    Pose6D p;
    p.x = matrix.translation().x();
    p.y = matrix.translation().y();
    p.z = matrix.translation().z();
    tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()))
        .getRPY(p.roll, p.pitch, p.yaw);
    return p;
}

extern Eigen::Isometry3d Pose6D2sometry3d(const Pose6D &p)
{
    Eigen::Vector3d tf_trans(p.x, p.y, p.z);
    //  Eigen::AngleAxisd rollAngle1(p.roll, Eigen::Vector3d::UnitX());
    //  Eigen::AngleAxisd pitchAngle1(p.pitch, Eigen::Vector3d::UnitY());
    //  Eigen::AngleAxisd yawAngle1(p.yaw, Eigen::Vector3d::UnitZ());
    //  Eigen::Quaterniond quat = yawAngle1 * pitchAngle1 * rollAngle1;

    tf::Quaternion quat = tf::createQuaternionFromRPY(p.roll, p.pitch, p.yaw);
    Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
    mat.rotate(Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z())
                   .toRotationMatrix());
    mat.pretranslate(tf_trans);

    return mat;
}

extern Eigen::Isometry3d OdomToIsometry3d(const nav_msgs::Odometry &pose_geo)
{
    Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
    pose_eigen.rotate(Eigen::Quaterniond(
        pose_geo.pose.pose.orientation.w, pose_geo.pose.pose.orientation.x,
        pose_geo.pose.pose.orientation.y, pose_geo.pose.pose.orientation.z));
    pose_eigen.pretranslate(Eigen::Vector3d(pose_geo.pose.pose.position.x,
                                            pose_geo.pose.pose.position.y,
                                            pose_geo.pose.pose.position.z));
    return pose_eigen;
}

extern Eigen::Isometry3d GeoposeToIsometry3d(
    const geometry_msgs::PoseWithCovarianceStamped &pose_geo)
{
    Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
    pose_eigen.rotate(Eigen::Quaterniond(
        pose_geo.pose.pose.orientation.w, pose_geo.pose.pose.orientation.x,
        pose_geo.pose.pose.orientation.y, pose_geo.pose.pose.orientation.z));
    pose_eigen.pretranslate(Eigen::Vector3d(pose_geo.pose.pose.position.x,
                                            pose_geo.pose.pose.position.y,
                                            pose_geo.pose.pose.position.z));
    return pose_eigen;
}

extern Eigen::Matrix4d GeoposeToMatrix4d(
    const geometry_msgs::PoseWithCovarianceStamped &pose_geo)
{
    Eigen::Quaterniond test_q(
        pose_geo.pose.pose.orientation.w, pose_geo.pose.pose.orientation.x,
        pose_geo.pose.pose.orientation.y, pose_geo.pose.pose.orientation.z);

    Eigen::Matrix4d test_guess = Eigen::Matrix4d::Identity();
    test_guess.block<3, 3>(0, 0) = test_q.toRotationMatrix();
    test_guess.block<3, 1>(0, 3) = Eigen::Vector3d(pose_geo.pose.pose.position.x,
                                                   pose_geo.pose.pose.position.y,
                                                   pose_geo.pose.pose.position.z);
    return test_guess;
}

extern Pose6D GtsamPose2Pose6D(gtsam::Pose3 pose)
{
    auto tx = pose.translation().x();
    auto ty = pose.translation().y();
    auto tz = pose.translation().z();

    double roll, pitch, yaw;
    roll = pose.rotation().roll();
    pitch = pose.rotation().pitch();
    yaw = pose.rotation().yaw();

    return Pose6D{tx, ty, tz, roll, pitch, yaw};
}

extern Eigen::Affine3f Pose6dToAffine3f(Pose6D pose) {
    return pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch,
                                  pose.yaw);
}

extern gtsam::Pose3 Pose6dTogtsamPose3(Pose6D pose) {
    return gtsam::Pose3(
            gtsam::Rot3::RzRyRx(double(pose.roll), double(pose.pitch),
                                double(pose.yaw)),
            gtsam::Point3(double(pose.x), double(pose.y), double(pose.z)));
}

extern gtsam::Pose3 Matrix4dToPose3(const Eigen::Matrix4d &matrix) {
    // Extract rotation block
    Eigen::Matrix3d rotation_matrix = matrix.block<3, 3>(0, 0);
    gtsam::Rot3 rot(rotation_matrix);
    // Extract translation block
    Eigen::Vector3d translation_vector = matrix.block<3, 1>(0, 3);
    gtsam::Point3 trans(translation_vector);
    return gtsam::Pose3(rot, trans);
}

pcl::PointCloud<PointT>::Ptr TransformPointCloud(
    const pcl::PointCloud<PointT>::Ptr &cloudIn, const Pose6D &tf)
{
    pcl::PointCloud<PointT>::Ptr cloudOut(new pcl::PointCloud<PointT>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur =
        pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

    int numberOfCores = 16;
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
                                transCur(0, 1) * pointFrom.y +
                                transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
                                transCur(1, 1) * pointFrom.y +
                                transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
                                transCur(2, 1) * pointFrom.y +
                                transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

pcl::PointCloud<PointT>::Ptr TransformPointCloud(
    const pcl::PointCloud<PointT>::Ptr &cloudIn,
    const Eigen::Matrix4d &transformIn)
{
    pcl::PointCloud<PointT>::Ptr cloudOut(new pcl::PointCloud<PointT>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    //  Eigen::Affine3f transCur = pcl::getTransformation(
    //      transformIn->x, transformIn->y, transformIn->z, transformIn->roll,
    //      transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(8)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transformIn(0, 0) * pointFrom.x +
                                transformIn(0, 1) * pointFrom.y +
                                transformIn(0, 2) * pointFrom.z + transformIn(0, 3);
        cloudOut->points[i].y = transformIn(1, 0) * pointFrom.x +
                                transformIn(1, 1) * pointFrom.y +
                                transformIn(1, 2) * pointFrom.z + transformIn(1, 3);
        cloudOut->points[i].z = transformIn(2, 0) * pointFrom.x +
                                transformIn(2, 1) * pointFrom.y +
                                transformIn(2, 2) * pointFrom.z + transformIn(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc(
    const std::vector<Pose6D> vectorPose6d)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto p : vectorPose6d)
    {
        res->points.emplace_back(p.x, p.y, p.z);
    }
    return res;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc2d(
    const std::vector<Pose6D> vectorPose6d)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto p : vectorPose6d)
    {
        res->points.emplace_back(p.x, p.y, 0);
    }
    return res;
}

bool useGPS;
int gps_type;
bool useFixedOri;
bool saveKeyFrame;
int saveKeyFrameMode = 2;
bool useFixedCov;
bool useScancontext;
bool useLoopClosure;
bool saveResultBag;
bool saveResultBodyFrame;
bool align_gnss_enu_to_ros;
bool gnss_init_use_pose = true;

bool saveLoopdata;
bool useKeyframe;
bool useImuFactor;
bool useRawCloud;
bool useCompressedImage;
bool useGlobalPrior;
int baseline;
bool useMultiMode;
bool useNoMotionFactor = false;
double mea_noise;
bool showDegenercy = false;
int degenercy_type = 0;

int method;
int imuFactorType;
double gpsFrequence;
double gpsCovThreshold;
bool gpsRobustKernel = false;
bool loopRobustKernel = false;
bool gpsBatchOptimizationEnable = false;
int gpsBatchOptimizationThreshold = 5;
double keyframeMeterGap;
double keyframeDegGap;
double keyframeRadGap;
double pixelDis;
vector<double> origin_lla;
std::vector<double> runtime_origin_lla(3, 0.0);
std::vector<double> runtime_origin_initial_lla(3, 0.0);
std::atomic<bool> runtime_origin_valid{false};
std::atomic<bool> runtime_origin_initial_valid{false};
std::mutex runtime_origin_mutex;
double pointCropRange = -1.0;
Eigen::Vector3d pointCropBox(60.0, 60.0, 60.0);
bool pointCropBoxEnabled = true;
int mapSorMeanK = 20;
double mapSorStdMul = 1.0;
double mapRorRadius = -1.0;
int mapRorMinNeighbors = 3;
// 默认关闭导出阶段的地图异常点过滤（SOR/ROR），
// 因为旧图和新图在采集/建图阶段已经做过严格过滤，
// 且导出时需要完整点云用于生成 grid map 和后续定位。
bool mapOutlierFilterEnabled = false;
std::string mapOutlierFilterMode = "sor";
bool useRefinedLidarCloud = false;
double zuptTranslationThreshold = 0.01;
double zuptRotationThreshold = 0.015;

double scDistThres;
int initialPoseType;
double scMaximumRadius;

double historyKeyframeSearchRadius;
double historyKeyframeSearchTimeDiff;
double loopFitnessScoreThreshold;
double loopMinOverlapThreshold = 0.7;
double loopMaxCandidatePlanarDistance = -1.0;

// 默认值与当前硬编码逻辑保持一致：
// - M2F 全局 ICP 局部地图裁剪半径：80m
// - ICP 搜索距离：1.0m
// - ICP RMSE 阈值：0.3
// - ICP fitness 阈值：0.8
double m2fLocalMapRadius = 80.0;
double m2fIcpSearchRadius = 1.0;
double m2fIcpScoreThreshold = 0.3;
double m2fIcpFitnessThreshold = 0.8;
bool   m2fUseHandcraftedICP = false;

bool useRPGO;
int historyKeyframeSearchNum;
int accumulatedFactorSize;
int gnss_constraint_type;
double loopClosureFrequency;
bool loopBackfillEnable = true;
double loopBackfillMoveDist = 1.5;
double loopBackfillMoveYawDeg = 3.0;
int loopBackfillPerCycle = 2;
double loopBackfillRetryWaitSec = 5.0;
int loopBackfillQueueMaxSize = 256;
bool loopPCMEnable = true;
double loopPCMSelfTranslationThresh = 0.6;
double loopPCMSelfRotationThreshDeg = 4.0;
double loopPCMTranslationThresh = 1.0;
double loopPCMRotationThreshDeg = 6.0;

int lidar_type;

double filterDis;
double LOOP_Z_OFFSET;
int filterNodeNum;
int SKIP_FRAMES;
int GPS_SKIP_NUMBER;
std::string configDirectory;
std::string saveDirectory;
std::string mapDirectory;
std::string sequence;
std::string odom_link;
std::string gps_topic;
std::string imu_topic;
std::string left_image_topic;
std::string right_image_topic;

Eigen::Quaterniond q_body_sensor;
Eigen::Vector3d t_body_sensor;
Eigen::Matrix3d rot_body_sensor;
Eigen::Matrix4d T_body_lidar;

vector<double> camera_matrix0_vec;
vector<double> distCoeffs0_vec;
vector<double> rot_camera2imu0_vec;
vector<double> trans_camera2imu0_vec;
Eigen::Matrix3d camera_matrix0;
Eigen::Quaterniond q_camera0_body;
Eigen::Matrix3d rot_camera0_body;
Eigen::Vector3d t_camera0_body;

Eigen::Vector4d distCoeffs0, distCoeffs1;

vector<double> camera_matrix1_vec;
vector<double> distCoeffs1_vec;
vector<double> rot_camera2imu1_vec;
vector<double> trans_camera2imu1_vec;
Eigen::Matrix3d camera_matrix1;
Eigen::Quaterniond q_camera1_body;
Eigen::Matrix3d rot_camera1_body;
Eigen::Vector3d t_camera1_body;

double scan_filter_size;
double map_viewer_size;
double map_saved_size;
double grid_map_downsample_size;
double localization_map_grid_size;
double intensity_image_resolution;
bool save_grid_map;
bool save_intensity_image;

// Visualization and export defaults
double pgoMapPubHz = 0.05;       // /pgo_map publish rate (Hz), default every 20s
double pathPubHz = 1.0;          // /pgo_path publish rate (Hz)
double markerPubPeriod = 10.0;    // MarkerArray throttle period (seconds)
bool exportCovariance = false;    // compute Marginals only when true

double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
vector<double> b_gyr_cov_n;
vector<double> b_acc_cov_n;

vector<double> extrinT;
vector<double> extrinR;

void LoadRosParams(ros::NodeHandle &nh, bool load_extrinsic)
{
    t_body_sensor = Eigen::Vector3d::Identity();
    q_body_sensor = Eigen::Quaterniond::Identity();

    nh.param<std::string>("config_directory", configDirectory,   "~/Download/");
    nh.param<std::string>("save_directory", saveDirectory, "~/Download/fl2sam");
    nh.param<std::string>("map_directory", mapDirectory, "~/Download/fl2sam");

    nh.param<std::string>("common/gps_topic", gps_topic, "/fix");
    nh.param<std::string>("common/imu_topic", imu_topic, "/fix");

    nh.param<std::string>("common/sequence", sequence, "exp05");
    std::string bag_path;
    nh.param<std::string>("common/bag_path", bag_path, "");
    std::string lid_topic;
    nh.param<std::string>("common/lid_topic", lid_topic, "");
    nh.param<int>("common/initial_pose_type", initialPoseType, 0);

    // Console log level (controls ROS_* output in terminal).
    // Supported values: DEBUG, INFO, WARN, ERROR, FATAL (case-insensitive).
    std::string console_log_level_str;
    nh.param<std::string>("common/console_log_level", console_log_level_str, "INFO");
    std::string console_log_level_upper = console_log_level_str;
    std::transform(console_log_level_upper.begin(),
                   console_log_level_upper.end(),
                   console_log_level_upper.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    ros::console::Level console_level = ros::console::levels::Info;
    if (console_log_level_upper == "DEBUG")
    {
        console_level = ros::console::levels::Debug;
    }
    else if (console_log_level_upper == "INFO")
    {
        console_level = ros::console::levels::Info;
    }
    else if (console_log_level_upper == "WARN")
    {
        console_level = ros::console::levels::Warn;
    }
    else if (console_log_level_upper == "ERROR")
    {
        console_level = ros::console::levels::Error;
    }
    else if (console_log_level_upper == "FATAL")
    {
        console_level = ros::console::levels::Fatal;
    }
    else
    {
        ROS_WARN_STREAM("[ParameterLoader] common/console_log_level=\""
                        << console_log_level_str
                        << "\" is invalid. Falling back to INFO.");
        console_log_level_upper = "INFO";
        console_level = ros::console::levels::Info;
    }

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ROS_INFO_STREAM("[ParameterLoader] console_log_level set to " << console_log_level_upper);

    if (!saveDirectory.empty() && saveDirectory.back() != '/')
    {
        saveDirectory.push_back('/');
    }
    if (!mapDirectory.empty() && mapDirectory.back() != '/')
    {
        mapDirectory.push_back('/');
    }
    SetLogDirectory(ComposeLogDirectory(saveDirectory, sequence));
    nh.param<std::string>("common/odom_link", odom_link, "camera_init");
    nh.param<bool>("common/useLoopClosure", useLoopClosure, false);
    nh.param<bool>("pgo/use_no_motion_factor", useNoMotionFactor, false);
    nh.param<bool>("pgo/showDegenercy", showDegenercy, false);
    nh.param<int>("pgo/degenercy_type", degenercy_type, 0);
    nh.param<double>("common/zupt_translation_threshold", zuptTranslationThreshold, zuptTranslationThreshold);
    nh.param<double>("common/zupt_rotation_threshold", zuptRotationThreshold, zuptRotationThreshold);

    // if we use scan context
    nh.param<bool>("common/useScancontext", useScancontext, false);
    nh.param<double>("common/sc_dist_thres", scDistThres, 0.2);
    nh.param<double>("common/sc_max_radius", scMaximumRadius, 40.0);

    // color map

    // gps params
    nh.param<bool>("common/useGPS", useGPS, false);
    nh.param<int>("common/gps_type", gps_type, 1);
    nh.param<bool>("common/useFixedOri", useFixedOri, false);
    nh.param<bool>("common/align_gnss_enu_to_ros", align_gnss_enu_to_ros, true);
    nh.param<bool>("common/gnss_init_use_pose", gnss_init_use_pose, true);
    nh.param<double>("common/gpsFrequence", gpsFrequence, 10);
    nh.param<double>("common/gpsCovThreshold", gpsCovThreshold, 2.0);
    nh.param<bool>("pgo/gps_robust_kernel", gpsRobustKernel, gpsRobustKernel);
    nh.param<bool>("pgo/loop_robust_kernel", loopRobustKernel, loopRobustKernel);
    nh.param<bool>("common/gps_batch_enable", gpsBatchOptimizationEnable, false);
    nh.param<int>("common/gps_batch_threshold", gpsBatchOptimizationThreshold, 5);
    nh.param<int>("common/accumulatedFactorSize", accumulatedFactorSize, 10);
    nh.param<int>("common/gnss_constraint_type", gnss_constraint_type, 0);


    nh.param<int>("common/GPS_SKIP_NUMBER", GPS_SKIP_NUMBER, 5.0);
    nh.param<vector<double>>("common/origin_lla", origin_lla, vector<double>(3));

    // get extrinsics between  lidar to imu
    nh.param<double>("common/gyr_cov", gyr_cov, 0.01);
    nh.param<double>("common/acc_cov", acc_cov, 0.001);
    nh.param<double>("common/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("common/b_acc_cov", b_acc_cov, 0.0000001);
    nh.param<vector<double>>("common/extrinsic_T", extrinT, vector<double>(3));
    nh.param<vector<double>>("common/extrinsic_R", extrinR, vector<double>(9));
    nh.param<vector<double>>("common/b_gyr_cov_n", b_gyr_cov_n,  vector<double>(3));
    nh.param<vector<double>>("common/b_acc_cov_n", b_acc_cov_n,  vector<double>(3));

    ros::param::set(kCalibrationParamRosKey, "");

    bool extrinsic_overridden = false;
    if (load_extrinsic) {
        const std::string extrinsic_prefix = InferExtrinsicPrefixFromTopic(lid_topic);
        const std::string rotation_key = extrinsic_prefix.empty() ? "" : extrinsic_prefix + "_rotation";
        const std::string translation_key = extrinsic_prefix.empty() ? "" : extrinsic_prefix + "_translation";

        if (!bag_path.empty()) {
            std::filesystem::path bag_path_fs(bag_path);
            std::error_code fs_ec;
            std::filesystem::path base_dir;
            if (std::filesystem::is_directory(bag_path_fs, fs_ec)) {
                base_dir = bag_path_fs;
            } else {
                base_dir = bag_path_fs.parent_path();
            }
            if (base_dir.empty()) {
                base_dir = bag_path_fs;
            }
            const std::filesystem::path calibration_file = base_dir / "calibration_param.yaml";
            ROS_INFO_STREAM(kColorInfo << "[ExtrinsicLoader] bag_path: " << bag_path << kColorReset);
            ROS_INFO_STREAM(kColorInfo << "[ExtrinsicLoader] Looking for calibration_param.yaml at: "
                                       << calibration_file.string() << kColorReset);

            if (extrinsic_prefix.empty()) {
                ROS_WARN_STREAM(kColorWarn
                                << "[ExtrinsicLoader] Unable to infer calibration keys from lid topic \""
                                << lid_topic
                                << "\". Falling back to default extrinsic."
                                << kColorReset);
            } else {
                std::error_code exists_ec;
                if (std::filesystem::exists(calibration_file, exists_ec)) {
                    ROS_INFO_STREAM(kColorInfo << "[ExtrinsicLoader] Selected calibration keys: "
                                               << rotation_key << " / " << translation_key << kColorReset);
                    Eigen::Matrix3d calib_rot_inv;
                    Eigen::Vector3d calib_trans_inv;
                    std::string error_msg;
                    if (LoadInverseExtrinsicFromCalibration(calibration_file.string(),
                                                            rotation_key,
                                                            translation_key,
                                                            calib_rot_inv,
                                                            calib_trans_inv,
                                                            error_msg)) {
                        extrinsic_overridden = true;
                        extrinR.resize(9);
                        for (int r = 0; r < 3; ++r) {
                            for (int c = 0; c < 3; ++c) {
                                extrinR[r * 3 + c] = calib_rot_inv(r, c);
                            }
                        }
                        extrinT = {calib_trans_inv.x(), calib_trans_inv.y(), calib_trans_inv.z()};
                        ROS_INFO_STREAM(kColorInfo
                                        << "[ExtrinsicLoader] Loaded extrinsic from calibration_param.yaml (inverse applied)."
                                        << kColorReset);
                        ros::param::set(kCalibrationParamRosKey, calibration_file.string());
                    } else {
                        ROS_WARN_STREAM(kColorWarn << "[ExtrinsicLoader] Failed to parse calibration_param.yaml: "
                                                   << error_msg
                                                   << ". Falling back to default extrinsic."
                                                   << kColorReset);
                    }
                } else {
                    if (exists_ec) {
                        ROS_WARN_STREAM(kColorWarn << "[ExtrinsicLoader] Error while checking calibration_param.yaml: "
                                                   << exists_ec.message()
                                                   << ". Falling back to default extrinsic."
                                                   << kColorReset);
                    } else {
                        ROS_WARN_STREAM(kColorWarn << "[ExtrinsicLoader] calibration_param.yaml not found at: "
                                                   << calibration_file.string()
                                                   << ". Falling back to default extrinsic."
                                                   << kColorReset);
                    }
                }
            }
        } else {
            ROS_WARN_STREAM(kColorWarn << "[ExtrinsicLoader] bag_path not provided. Using default extrinsic from configuration."
                                       << kColorReset);
        }
    }

    nh.param<int>("lio/lidar_type", lidar_type, 5);

    // std::cout << "----------------------------Read extrinsics
    // lidar2imu---------------------------" << std::endl;
    rot_body_sensor = Eigen::Matrix3d::Identity();
    rot_body_sensor =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            extrinR.data(), 3, 3);
    q_body_sensor = Eigen::Quaterniond(1, 0, 0, 0);
    q_body_sensor = Eigen::Quaterniond(rot_body_sensor);
    t_body_sensor = Eigen::Vector3d(extrinT[0], extrinT[1], extrinT[2]);
    if (load_extrinsic) {
        ROS_INFO_STREAM(kColorInfo << "[ExtrinsicLoader] Final extrinsic R (lidar->imu): "
                                   << FormatMatrix(rot_body_sensor) << kColorReset);
        ROS_INFO_STREAM(kColorInfo << "[ExtrinsicLoader] Final extrinsic T (lidar->imu): "
                                   << FormatVector(t_body_sensor) << kColorReset);
        if (extrinsic_overridden) {
            ROS_INFO_STREAM(kColorInfo << "[ExtrinsicLoader] Extrinsic source: calibration_param.yaml." << kColorReset);
        } else {
            ROS_INFO_STREAM(kColorInfo << "[ExtrinsicLoader] Extrinsic source: configuration default." << kColorReset);
        }
    }

    // lidar -> body imu
    T_body_lidar.block<3, 1>(0, 3) = t_body_sensor.matrix();
    T_body_lidar.block<3, 3>(0, 0) = rot_body_sensor.matrix();

    // save map data
    nh.param<bool>("common/saveResultBag", saveResultBag, false);
    nh.param<bool>("common/saveResultBodyFrame", saveResultBodyFrame, false);
    nh.param<bool>("common/saveLoopdata", saveLoopdata, false);
    nh.param<bool>("common/saveKeyFrame", saveKeyFrame, false);
    nh.param<int>("common/saveKeyFrameMode", saveKeyFrameMode, 2);
    if (saveKeyFrameMode < 0 || saveKeyFrameMode > 2)
    {
        ROS_WARN_STREAM("[ParameterLoader] common/saveKeyFrameMode out of range (" << saveKeyFrameMode
                        << "), clamping to [0,2].");
        saveKeyFrameMode = std::max(0, std::min(saveKeyFrameMode, 2));
    }
    nh.param<bool>("common/useFixedCov", useFixedCov, false);

    // voxigrid size
    nh.param<double>("common/scan_filter_size", scan_filter_size, 0.1);
    nh.param<double>("common/map_viewer_size", map_viewer_size, 0.5);
    nh.param<double>("common/map_saved_size", map_saved_size, 0.2);
    nh.param<double>("common/grid_map_downsample_size", grid_map_downsample_size, 0.2);
    nh.param<double>("common/localization_map_grid_size", localization_map_grid_size, 20.0);
    nh.param<double>("common/intensity_image_resolution", intensity_image_resolution, 0.2);
    nh.param<bool>("common/save_grid_map", save_grid_map, true);
    nh.param<bool>("common/save_intensity_image", save_intensity_image, true);

    // Visualization and export controls
    nh.param<double>("common/pgo_map_pub_hz", pgoMapPubHz, pgoMapPubHz);
    nh.param<double>("common/path_pub_hz", pathPubHz, pathPubHz);
    nh.param<double>("common/marker_pub_period", markerPubPeriod, markerPubPeriod);
    nh.param<bool>("common/export_covariance", exportCovariance, exportCovariance);

    nh.param<bool>("common/point_crop_box_enabled", pointCropBoxEnabled, pointCropBoxEnabled);
    nh.param<double>("common/point_crop_box", pointCropRange, pointCropRange);
    if (pointCropRange > 0.0)
    {
        pointCropBox = Eigen::Vector3d::Constant(pointCropRange);
    }

    nh.param<bool>("common/map_outlier_filter_enabled", mapOutlierFilterEnabled, mapOutlierFilterEnabled);
    nh.param<std::string>("common/map_outlier_filter_mode", mapOutlierFilterMode, mapOutlierFilterMode);
    std::transform(mapOutlierFilterMode.begin(), mapOutlierFilterMode.end(), mapOutlierFilterMode.begin(), ::tolower);
    nh.param<int>("common/map_sor_mean_k", mapSorMeanK, mapSorMeanK);
    nh.param<double>("common/map_sor_std_mul", mapSorStdMul, mapSorStdMul);
    nh.param<double>("common/map_ror_radius", mapRorRadius, mapRorRadius);
    nh.param<int>("common/map_ror_min_neighbors", mapRorMinNeighbors, mapRorMinNeighbors);

    // if we need to save key frame
    nh.param<int>("pgo/SKIP_FRAMES", SKIP_FRAMES, 5);
    nh.param<bool>("pgo/useRawCloud", useRawCloud, false);
    nh.param<bool>("pgo/useKeyframe", useKeyframe, true);
    nh.param<bool>("pgo/useGlobalPrior", useGlobalPrior, true);
    nh.param<bool>("pgo/use_refined_cloud", useRefinedLidarCloud, useRefinedLidarCloud);
    nh.param<int>("pgo/baseline", baseline, 0);
    nh.param<bool>("pgo/useMultiMode", useMultiMode, true);
    nh.param<bool>("pgo/use_imu_factor", useImuFactor, false);
    nh.param<int>("pgo/imu_factor_type", imuFactorType, 0);
    nh.param<int>("pgo/method", method, 2);
    nh.param<double>("pgo/mea_noise", mea_noise, 10.0);

    nh.param<double>("pgo/keyframe_meter_gap", keyframeMeterGap, 0.1);
    nh.param<double>("pgo/keyframe_deg_gap", keyframeDegGap, 10.0);
    keyframeRadGap = pcl::deg2rad(keyframeDegGap);

    nh.param<double>("pgo/filterDis", filterDis, 12.0);
    nh.param<double>("pgo/loopZOffset", LOOP_Z_OFFSET, 20.0);
    nh.param<int>("pgo/filterNodeNum", filterNodeNum, 10);
    nh.param<bool>("pgo/rpgo", useRPGO, true);

    // for loop closure detection
    nh.param<double>("pgo/historyKeyframeSearchRadius",
                     historyKeyframeSearchRadius, 20.0);
    nh.param<double>("pgo/historyKeyframeSearchTimeDiff",
                     historyKeyframeSearchTimeDiff, 10.0);
    nh.param<int>("pgo/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<double>("pgo/loopClosureFrequency", loopClosureFrequency, 5);
    nh.param<double>("pgo/loopFitnessScoreThreshold", loopFitnessScoreThreshold,  0.9);
    nh.param<double>("pgo/loop_min_overlap_threshold", loopMinOverlapThreshold, loopMinOverlapThreshold);
    nh.param<double>("pgo/loop_max_candidate_planar_distance",
                     loopMaxCandidatePlanarDistance,
                     loopMaxCandidatePlanarDistance);
    nh.param<bool>("pgo/loop_backfill_enable", loopBackfillEnable, loopBackfillEnable);
    nh.param<double>("pgo/loop_backfill_move_dist_m", loopBackfillMoveDist, loopBackfillMoveDist);
    nh.param<double>("pgo/loop_backfill_move_yaw_deg", loopBackfillMoveYawDeg, loopBackfillMoveYawDeg);
    nh.param<int>("pgo/loop_backfill_per_cycle", loopBackfillPerCycle, loopBackfillPerCycle);
    nh.param<double>("pgo/loop_backfill_retry_wait_sec", loopBackfillRetryWaitSec, loopBackfillRetryWaitSec);
    nh.param<int>("pgo/loop_backfill_queue_max", loopBackfillQueueMaxSize, loopBackfillQueueMaxSize);
    nh.param<bool>("pgo/loop_pcm_enable", loopPCMEnable, loopPCMEnable);
    nh.param<double>("pgo/loop_pcm_self_trans_thresh_m", loopPCMSelfTranslationThresh, loopPCMSelfTranslationThresh);
    nh.param<double>("pgo/loop_pcm_self_rot_thresh_deg", loopPCMSelfRotationThreshDeg, loopPCMSelfRotationThreshDeg);
    nh.param<double>("pgo/loop_pcm_trans_thresh_m", loopPCMTranslationThresh, loopPCMTranslationThresh);
    nh.param<double>("pgo/loop_pcm_rot_thresh_deg", loopPCMRotationThreshDeg, loopPCMRotationThreshDeg);

    // M2F 模式：地图匹配（map-to-frame）ICP 参数
    // 仅在需要单独调节 M2F 匹配行为时在 yaml 中配置；其他 yaml 可以不包含该段。
    nh.param<double>("m2f/local_map_radius", m2fLocalMapRadius, m2fLocalMapRadius);
    nh.param<double>("m2f/icp_search_radius", m2fIcpSearchRadius, m2fIcpSearchRadius);
    nh.param<double>("m2f/icp_score_threshold", m2fIcpScoreThreshold, m2fIcpScoreThreshold);
    nh.param<double>("m2f/icp_fitness_threshold", m2fIcpFitnessThreshold, m2fIcpFitnessThreshold);
    nh.param<bool>("m2f/use_handcrafted_icp", m2fUseHandcraftedICP, m2fUseHandcraftedICP);
}
