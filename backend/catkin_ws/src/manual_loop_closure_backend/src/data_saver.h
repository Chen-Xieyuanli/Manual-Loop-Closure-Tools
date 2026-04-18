
#ifndef FAST_LIO_SRC_PGO_SRC_DATASAVER_H_
#define FAST_LIO_SRC_PGO_SRC_DATASAVER_H_

#include "base_type.hpp"
#include "tic_toc.h"

#include <filesystem>
#include <fstream>
#include <mutex>
#include <cstdint>
#include <optional>

#include <ros/ros.h>

#include <Eigen/StdVector>

using namespace std;
using namespace gtsam;

class DataSaver
{
public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        DataSaver();

        ~DataSaver();

        DataSaver(string _base_dir, string _sequence_name);

        void setDir(string _base_dir, string _sequence_name);

        const std::string &GetLogsPath() const { return logs_path_; }

        void ResetLocalizationOutputs();
        void UpdateVersionManifest();

        void setConfigDir(string _configDirectory);

        void setMapDir(string _config_dir);

        void setKeyframe(bool _save_key_frame, int keyframe_mode);
        void ImportExistingKeyframes(const std::string &source_dir);

        void setExtrinc(bool _saveResultBodyFrame,
                        Eigen::Vector3d _t_body_sensor,
                        Eigen::Quaterniond _q_body_sensor);

        void saveOptimizedVerticesKITTI(gtsam::Values _estimates);

        void saveOptimizedVerticesKITTI(std::vector<Vector7> _estimates);

        void saveOdometryVerticesKITTI(std::string _filename);

        void saveOriginGPS(Eigen::Vector3d gps_point);

        void saveOriginPositionCfg(const Eigen::Vector3d &gps_point,
                                   const std::string &address,
                                   const std::string &filename = "origin_position.cfg");

        void saveTrajectoryKML(const std::vector<Eigen::Vector3d> &enu_positions,
                               const Eigen::Vector3d &origin_lla,
                               const std::string &filename = "trajectory.kml",
                               const std::string &color_hex = "ff0000ff",
                               const std::string &output_directory_override = "");

        void saveTimes(vector<double> keyframeTimes);

        void saveOptimizedVerticesTUM(gtsam::Values _estimates);

        void saveOptimizedVerticesTUM(std::vector<Vector7> _estimates);

        void saveOptimizedVerticesTUM(std::vector<Vector7> _estimates,
                                      std::string file_name);

        void SaveSessionSubsetOutputs(const std::string &tag,
                                      const std::vector<nav_msgs::Odometry> &odoms,
                                      const std::vector<pcl::PointCloud<PointT>::Ptr> &clouds,
                                      const std::vector<Vector7> &poses,
                                      const std::vector<double> &times,
                                      const std::vector<Eigen::Vector3d> &trajectory_enu,
                                      const std::optional<Eigen::Vector3d> &origin_lla,
                                      const std::string &kml_color_hex);

        void saveOdomCov(std::vector<Eigen::Matrix<double, 6, 6>> &cov_vec,
                         std::string file_name);

        void saveOdometryVerticesTUM(
            std::vector<nav_msgs::Odometry> keyframePosesOdom);

        // 提供先验地图缓存，用于保存时直接合并旧图。
        void SetPriorGlobalMap(const pcl::PointCloud<PointT>::Ptr &map);

        void saveEdgeErrors(const std::string &filename, gtsam::ISAM2 *isam, gtsam::Values &estimate);

        void saveGraphGtsam(gtsam::NonlinearFactorGraph gtSAMgraph,
                            gtsam::ISAM2 *isam, gtsam::Values isamCurrentEstimate);
        bool WriteGraphG2o(const std::string &filepath,
                           const gtsam::NonlinearFactorGraph &graph,
                           const gtsam::Values &estimates) const;

        void saveGraph(std::vector<nav_msgs::Odometry> keyframePosesOdom);

        void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                           std::vector<nav_msgs::Odometry> updatedOdometryVec,
                           std::vector<sensor_msgs::PointCloud2> allResVec);

        void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                           std::vector<sensor_msgs::PointCloud2> allResVec);

        void saveLogBag(std::vector<Vector12> logVec);

        void writeDeskedFrame(pcl::PointCloud<PointT>::Ptr pc, int size);

        void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                           std::vector<pcl::PointCloud<PointT>::Ptr> allResVec);

        void saveLoopandImagePair(
            std::map<int, int> loopIndexCheckedMap,
            std::vector<std::vector<int>> all_camera_corre_match_pair);

        void savePointCloudMap(const std::vector<nav_msgs::Odometry> &allOdometryVec,
                               const std::vector<pcl::PointCloud<PointT>::Ptr> &allResVec);

        void savePointCloudMap(const std::vector<nav_msgs::Odometry> &allOdometryVec,
                               const std::vector<pcl::PointCloud<PointT>::Ptr> &allResVec,
                               int start_index,
                               bool full_export);

        void savePointCloudMap(const std::vector<Eigen::Isometry3d> &allOdometryVec,
                               const std::vector<pcl::PointCloud<PointT>::Ptr> &allResVec);

        void savePointCloudMapLIO(
            const std::vector<nav_msgs::Odometry> &allOdometryVec,
            const std::vector<pcl::PointCloud<PointT>::Ptr> &allResVec);

        void ReadPosesAndPointClouds(const std::string &tum_file, const std::string &cloud_directory,
                                     std::vector<Measurement> &measurements, pcl::PointCloud<PointT>::Ptr globalmap_ptr,
                                     bool keyframes_in_body_frame,
                                     bool use_keyframe_map = true);

        gtsam::NonlinearFactorGraph BuildFactorGraph(const std::string &g2o_file, gtsam::Values &initial_values);

        void DumpRuntimeConfig(const std::string &directory) const;

        void ArchiveEssentialOutputs();

        std::string FindG2oFile(const std::string &directory) const;

        // 公开：在无g2o场景下导出旧轨迹用于可视化
        bool ExportLegacyTrajectoryMeasurements(std::vector<Measurement> &measurements);

        // 标识当前先验是否处于 legacy 模式：
        // - true  : 通过 legacy 轨迹 / global_*.pcd 等构造的简化先验（无完备 g2o+关键帧）
        // - false : 正常的 g2o+TUM+关键帧先验
        bool IsLegacyModeEnabled() const { return legacy_mode_enabled_; }

public:
        string save_directory, config_directory, map_directory;

private:
        void GenerateVersionJson(const std::string &output_path);
        std::string FindTumTrajectoryFile(const std::string &directory) const;
        std::string FindKeyframeBinFile(const std::string &directory) const;
        std::string FindBackendOdomFile(const std::string &directory) const;
        bool LoadBackendOdomLog(const std::string &file_path,
                                std::vector<Pose6D, Eigen::aligned_allocator<Pose6D>> &trajectory_out) const;
        bool LoadGlobalMapFromBin(const std::string &bin_file, pcl::PointCloud<PointT>::Ptr globalmap_ptr);
        bool LoadGlobalMapFromPCDDirectory(const std::string &cloud_directory,
                                           pcl::PointCloud<PointT>::Ptr globalmap_ptr,
                                           bool keyframes_in_body_frame);
        bool LoadGridMapTiles(const std::string &map_dir, pcl::PointCloud<PointT>::Ptr globalmap_ptr) const;
        bool ShouldSaveKeyframePCD() const;
        bool ShouldSaveKeyframeBin() const;
        bool EnsureLegacyTrajectoryLoaded();
        bool LoadLegacyTrajectory();
        bool LoadLegacyGlobalMap(pcl::PointCloud<PointT>::Ptr globalmap_ptr,
                                 bool apply_downsample = true);
        bool LoadLegacyTrajectoryPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &trajectory_out);

        string base_dir, sequence_name;
        string keyFrmaePath;

        vector<string> configParameter;

        bool saveResultBodyFrame = false;
        bool save_key_frame = false;

        Eigen::Quaterniond q_body_sensor;
        Eigen::Vector3d t_body_sensor;

        vector<double> keyframeTimes;
        bool InitializeKeyframeBinWriter();
        void FinalizeKeyframeBinWriter();
        bool AppendKeyframeBin(const pcl::PointCloud<PointT> &cloud,
                               double timestamp,
                               const Eigen::Matrix4d &pose);
        void ExportImuPosesCsvToMapping(const std::filesystem::path &tum_source);
        bool WriteImuPosesCsv(const std::filesystem::path &tum_source,
                              const std::filesystem::path &csv_destination);

        bool keyframe_bin_active_ = false;
        uint64_t keyframe_bin_frame_count_ = 0;
        uint64_t keyframe_bin_point_count_ = 0;
        std::filesystem::path keyframe_bin_output_path_;
        std::ofstream keyframe_bin_stream_;
        int keyframe_save_mode_ = 2;

        std::string PcdmapFile(const std::string &filename) const;

    void saveTrajectoryPointCloud(const std::vector<Vector7> &estimates,
                       const std::string &file_path);

        pcl::PointCloud<PointT>::Ptr prior_global_map_cache_;

        struct GridInfo
        {
            int grid_id = -1;
            int grid_id_x = 0;
            int grid_id_y = 0;
            float lower_bound_x = 0.0f;
            float lower_bound_y = 0.0f;
            float upper_bound_x = 0.0f;
            float upper_bound_y = 0.0f;
            std::string name;
            std::string filename;
            pcl::PointCloud<pcl::PointXYZI> cloud;
        };

        void InitializeLocalizationOutputs();
        pcl::PointCloud<pcl::PointXYZI>::Ptr DownSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const;
        pcl::PointCloud<pcl::PointXYZI>::Ptr DownSampleWithLeaf(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
            float leaf_size_m) const;
        bool PrepareGridDirectory() const;
        void SaveLocalizationMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_cloud);
        bool SavePointCloudAsImage(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                   const std::string &filename,
                                   float resolution,
                                   const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &trajectory = nullptr);
        bool WriteCSV(const std::vector<GridInfo> &grids);

        pcl::PointCloud<PointT>::Ptr FilterMapOutliers(const pcl::PointCloud<PointT>::Ptr &cloud) const;

        std::string localization_map_path_;
        std::string pcdmap_root_path_;
        std::string logs_path_;
        std::string sensor_data_path_;
        std::string logs_directory_name_;
        double localization_grid_size_ = 20.0;
        double localization_downsample_resolution_ = 0.2;
        double intensity_image_resolution_ = 0.2;
        bool export_grid_map_ = true;
        bool export_intensity_image_ = true;
        float min_x_ = 0.0f;
        float max_x_ = 0.0f;
        float min_y_ = 0.0f;
        float max_y_ = 0.0f;

        void UpdateTransformCache();
        void EnsureTransformCache() const;
        const Eigen::Matrix4f &BodyToLidarMatrixF() const;
        Eigen::Matrix4d BodyToLidarMatrix() const;

        bool legacy_mode_enabled_ = false;
        std::vector<Pose6D, Eigen::aligned_allocator<Pose6D>> legacy_trajectory_;

        template <typename PointTType>
        bool WritePointCloudBinary(const std::string &path, const pcl::PointCloud<PointTType> &cloud) const
        {
            if (cloud.empty())
            {
                return false;
            }

            std::filesystem::path target(path);
            std::error_code ec;
            const auto parent = target.parent_path();
            if (!parent.empty())
            {
                std::filesystem::create_directories(parent, ec);
                if (ec)
                {
                    ROS_ERROR_STREAM("[DataSaver] 创建目录失败: " << parent << ", " << ec.message());
                    return false;
                }
            }

            const std::string temp_path = path + ".tmp";
            {
                std::lock_guard<std::mutex> lock(writer_mutex_);
                if (pcd_writer_.writeBinaryCompressed(temp_path, cloud) != 0)
                {
                    std::filesystem::remove(temp_path, ec);
                    ROS_ERROR_STREAM("[DataSaver] 写入临时PCD失败: " << temp_path);
                    return false;
                }
            }

            std::filesystem::rename(temp_path, target, ec);
            if (ec)
            {
                std::filesystem::remove(target, ec);
                std::filesystem::rename(temp_path, target, ec);
                if (ec)
                {
                    ROS_ERROR_STREAM("[DataSaver] 替换PCD失败: " << temp_path << " -> " << target << ", " << ec.message());
                    std::filesystem::remove(temp_path, ec);
                    return false;
                }
            }
            return true;
        }

        mutable pcl::PCDWriter pcd_writer_;
        mutable std::mutex writer_mutex_;

        // 缓存变换数据
        mutable bool body_to_lidar_cached_ = false;
        mutable Eigen::Matrix4d body_to_lidar_matrix_d_ = Eigen::Matrix4d::Identity();
        mutable Eigen::Matrix4f body_to_lidar_matrix_f_ = Eigen::Matrix4f::Identity();

        // 临时点云缓冲
        pcl::PointCloud<PointT>::Ptr scratch_pointi_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr scratch_traj_xyz_;

        pcl::PointCloud<PointT>::Ptr AcquirePointIScratch(bool clear = true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr AcquireTrajectoryScratch(bool clear = true);
};

#endif
