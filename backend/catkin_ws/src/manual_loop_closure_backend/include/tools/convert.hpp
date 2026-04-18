#ifndef __COMMON_CONVERT__
#define __COMMON_CONVERT__


#include <map>
#include <queue>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <filesystem>
#include <mutex>
#include <atomic>
#include <thread>
#include <signal.h>
#include <pthread.h>
#include <condition_variable>

// 根据ROS版本条件编译
#if defined(ROS_VERSION_1)
    // ROS1头文件
    #include <ros/ros.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <sensor_msgs/Imu.h>
    #include <sensor_msgs/Image.h>
#elif defined(ROS_VERSION_2)
    // ROS2头文件
    #include <rclcpp/rclcpp.hpp>

    #include <sensor_msgs/msg/point_cloud2.hpp>
    #include <sensor_msgs/msg/imu.hpp>
    #include <sensor_msgs/msg/image.hpp>
    #include <sensor_msgs/msg/compressed_image.hpp>

    #include "shm_msgs/msg/shm_point_cloud2m.hpp"
    #include "shm_msgs/msg/shm_image6m.hpp"
    #include "msfl_msgs/msg/inspvax_data.hpp"
    #include "sensor_data/fixposition_data.hpp" // 只有ROS2有这个inspvax的自定义类型

    #include <rosbag2_cpp/writer.hpp>
    #include <rosbag2_storage/storage_options.hpp>
    #include <rosbag2_cpp/writers/sequential_writer.hpp>
    #include <rosbag2_storage/serialized_bag_message.hpp>
#endif
// Eigen
#include <Eigen/Core>
// PCL
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// OpenCV头文件
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "sensor_data/imu_data.hpp"
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/camera_data.hpp"
// 数据压缩
#include "data_struct.hpp"  // 添加 Frame 类型的定义
#include "lidar_compress_module.hpp"
#include "image_compress_module.hpp"

#include "../../common/log.hpp"

namespace common {

class Convert {
public:
    // ================= PointCloud =================
    #if defined(ROS_VERSION_1)
    static bool Msg2CloudData(const sensor_msgs::PointCloud2& cloud_msg, CloudDataMID360& cloud_data) {
        pcl::fromROSMsg(cloud_msg, *cloud_data.cloud_ptr);
        cloud_data.time = cloud_msg.header.stamp.toSec();
        return true;
    }

    static bool Msg2CloudData(const sensor_msgs::PointCloud2& cloud_msg, CloudDataRobosense& cloud_data) {
        pcl::fromROSMsg(cloud_msg, *cloud_data.cloud_ptr);
        cloud_data.time = cloud_msg.header.stamp.toSec();
        return true;
    }
    #elif defined(ROS_VERSION_2)
    static bool Msg2CloudData(const sensor_msgs::msg::PointCloud2& cloud_msg, CloudDataMID360& cloud_data) {
        pcl::fromROSMsg(cloud_msg, *cloud_data.cloud_ptr);
        cloud_data.time = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9;
        return true;
    }

    static bool Msg2CloudData(const sensor_msgs::msg::PointCloud2& cloud_msg, CloudDataRobosense& cloud_data) {
        pcl::fromROSMsg(cloud_msg, *cloud_data.cloud_ptr);
        cloud_data.time = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9;
        return true;
    }

    // 自定义类型点云
    static bool Msg2CloudData(const shm_msgs::msg::ShmPointCloud2m& cloud_msg, CloudDataMID360& cloud_data) {
        // 将共享内存数据转换成ROS格式的 PointCloud2 消息
        sensor_msgs::msg::PointCloud2 ros_cloud;

        // 注意：假设 msg->header.frame_id 是自定义的字符串类型，
        // 其内部字段 data 为 std::array<unsigned char,256> 类型，size 为有效字符数。
        ros_cloud.header.frame_id =
        std::string(reinterpret_cast<const char*>(cloud_msg.header.frame_id.data.data()),
                    cloud_msg.header.frame_id.size);
        ros_cloud.header.stamp = cloud_msg.header.stamp;
        ros_cloud.height       = cloud_msg.height;
        ros_cloud.width        = cloud_msg.width;

        // 解析字段：逐个将 shm_msgs 内的字段转换为 sensor_msgs::msg::PointField
        for (const auto &field : cloud_msg.fields)
        {
            sensor_msgs::msg::PointField point_field;
            point_field.name     = std::string(reinterpret_cast<const char*>(field.name.data.data()),
                                                field.name.size);
            point_field.offset   = field.offset;
            point_field.datatype = field.datatype;
            point_field.count    = field.count;
            ros_cloud.fields.push_back(point_field);
        }
        ros_cloud.is_bigendian = cloud_msg.is_bigendian;
        ros_cloud.point_step   = cloud_msg.point_step;
        ros_cloud.row_step     = cloud_msg.row_step;
        ros_cloud.is_dense     = cloud_msg.is_dense;

        // 复制点云数据
        size_t data_size = cloud_msg.row_step * cloud_msg.height;
        ros_cloud.data.resize(data_size);
        std::memcpy(ros_cloud.data.data(), cloud_msg.data.data(), data_size);

        pcl::fromROSMsg(ros_cloud, *cloud_data.cloud_ptr);
        cloud_data.time = ros_cloud.header.stamp.sec + ros_cloud.header.stamp.nanosec * 1e-9;
        return true;
    }

    // 自定义类型点云
    static bool Msg2CloudData(const shm_msgs::msg::ShmPointCloud2m& cloud_msg, CloudDataRobosense& cloud_data) {
        // 将共享内存数据转换成ROS格式的 PointCloud2 消息
        sensor_msgs::msg::PointCloud2 ros_cloud;

        ros_cloud.header.frame_id =
        std::string(reinterpret_cast<const char*>(cloud_msg.header.frame_id.data.data()),
                    cloud_msg.header.frame_id.size);
        ros_cloud.header.stamp = cloud_msg.header.stamp;
        ros_cloud.height       = cloud_msg.height;
        ros_cloud.width        = cloud_msg.width;

        for (const auto &field : cloud_msg.fields)
        {
        sensor_msgs::msg::PointField point_field;
        point_field.name     = std::string(reinterpret_cast<const char*>(field.name.data.data()),
                                            field.name.size);
        point_field.offset   = field.offset;
        point_field.datatype = field.datatype;
        point_field.count    = field.count;
        ros_cloud.fields.push_back(point_field);
        }
        ros_cloud.is_bigendian = cloud_msg.is_bigendian;
        ros_cloud.point_step   = cloud_msg.point_step;
        ros_cloud.row_step     = cloud_msg.row_step;
        ros_cloud.is_dense     = cloud_msg.is_dense;

        // 复制点云数据
        size_t data_size = cloud_msg.row_step * cloud_msg.height;
        ros_cloud.data.resize(data_size);
        std::memcpy(ros_cloud.data.data(), cloud_msg.data.data(), data_size);

        pcl::fromROSMsg(ros_cloud, *cloud_data.cloud_ptr);
        cloud_data.time = ros_cloud.header.stamp.sec + ros_cloud.header.stamp.nanosec * 1e-9;
        return true;
    }

    /**
     * @brief 将共享内存数据转换成Frame
     * 
     */
    static bool Msg2Frame(const shm_msgs::msg::ShmPointCloud2m& cloud_msg, Frame& frame) {
        // 将共享内存数据转换成ROS格式的 PointCloud2 消息
        sensor_msgs::msg::PointCloud2 ros_cloud;

        ros_cloud.header.frame_id =
        std::string(reinterpret_cast<const char*>(cloud_msg.header.frame_id.data.data()),
                    cloud_msg.header.frame_id.size);
        ros_cloud.header.stamp = cloud_msg.header.stamp;
        ros_cloud.height       = cloud_msg.height;
        ros_cloud.width        = cloud_msg.width;

        for (const auto &field : cloud_msg.fields)
        {
            sensor_msgs::msg::PointField point_field;
            point_field.name     = std::string(reinterpret_cast<const char*>(field.name.data.data()),
                                                field.name.size);
            point_field.offset   = field.offset;
            point_field.datatype = field.datatype;
            point_field.count    = field.count;
            ros_cloud.fields.push_back(point_field);
        }
        ros_cloud.is_bigendian = cloud_msg.is_bigendian;
        ros_cloud.point_step   = cloud_msg.point_step;
        ros_cloud.row_step     = cloud_msg.row_step;
        ros_cloud.is_dense     = cloud_msg.is_dense;

        // 复制点云数据
        size_t data_size = cloud_msg.row_step * cloud_msg.height;
        ros_cloud.data.resize(data_size);
        std::memcpy(ros_cloud.data.data(), cloud_msg.data.data(), data_size);
        CloudDataRobosense cloud_data;
        pcl::fromROSMsg(ros_cloud, *cloud_data.cloud_ptr);
        cloud_data.time = ros_cloud.header.stamp.sec + ros_cloud.header.stamp.nanosec * 1e-9; 

        // 创建CompressModule对象并调用Encode
        CompressModule compressor;
        compressor.Encode(frame, cloud_data.cloud_ptr);

        return true;
    }

    static bool Msg2Frame(const shm_msgs::msg::ShmPointCloud6m& cloud_msg, Frame64& frame) {
        // 将共享内存数据转换成ROS格式的 PointCloud2 消息
        sensor_msgs::msg::PointCloud2 ros_cloud;

        ros_cloud.header.frame_id =
        std::string(reinterpret_cast<const char*>(cloud_msg.header.frame_id.data.data()),
                    cloud_msg.header.frame_id.size);
        ros_cloud.header.stamp = cloud_msg.header.stamp;
        ros_cloud.height       = cloud_msg.height;
        ros_cloud.width        = cloud_msg.width;

        for (const auto &field : cloud_msg.fields)
        {
            sensor_msgs::msg::PointField point_field;
            point_field.name     = std::string(reinterpret_cast<const char*>(field.name.data.data()),
                                                field.name.size);
            point_field.offset   = field.offset;
            point_field.datatype = field.datatype;
            point_field.count    = field.count;
            ros_cloud.fields.push_back(point_field);
        }
        ros_cloud.is_bigendian = cloud_msg.is_bigendian;
        ros_cloud.point_step   = cloud_msg.point_step;
        ros_cloud.row_step     = cloud_msg.row_step;
        ros_cloud.is_dense     = cloud_msg.is_dense;

        // 复制点云数据
        size_t data_size = cloud_msg.row_step * cloud_msg.height;
        ros_cloud.data.resize(data_size);
        std::memcpy(ros_cloud.data.data(), cloud_msg.data.data(), data_size);
        CloudDataRobosense cloud_data;
        pcl::fromROSMsg(ros_cloud, *cloud_data.cloud_ptr);
        cloud_data.time = ros_cloud.header.stamp.sec + ros_cloud.header.stamp.nanosec * 1e-9; 

        // 创建CompressModule对象并调用Encode
        CompressModule compressor;
        compressor.Encode(frame, cloud_data.cloud_ptr);

        return true;
    }

    static bool Msg2Frame(const  sensor_msgs::msg::PointCloud2& cloud_msg, Frame& frame) {
        // 将共享内存数据转换成ROS格式的 PointCloud2 消息
        sensor_msgs::msg::PointCloud2 ros_cloud;
        ros_cloud = cloud_msg;
        CloudDataRobosense cloud_data;
        pcl::fromROSMsg(ros_cloud, *cloud_data.cloud_ptr);
        cloud_data.time = ros_cloud.header.stamp.sec + ros_cloud.header.stamp.nanosec * 1e-9; 

        // 创建CompressModule对象并调用Encode
        CompressModule compressor;
        compressor.Encode(frame, cloud_data.cloud_ptr);

        return true;
    }

    static bool Msg2Frame(pcl::PointCloud<pcl::PointXYZI>& cloud, double timestamp, Eigen::Matrix4d& pose, FrameWithOutRT& frame)
    {
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud);
        CompressModule compressor;
        compressor.EncodeWithOutRT(frame, cloud_ptr, timestamp, pose);
        return true;
    }
    #endif

    #if defined(ROS_VERSION_1)
    static bool CloudData2Msg(const CloudDataMID360& cloud_data, sensor_msgs::PointCloud2& cloud_msg, const std::string& frame_id = "base_link") {
        pcl::toROSMsg(*cloud_data.cloud_ptr, cloud_msg);
        cloud_msg.header.stamp = ros::Time(cloud_data.time);
        cloud_msg.header.frame_id = frame_id;
        return true;
    }

    static bool CloudData2Msg(const CloudDataRobosense& cloud_data, sensor_msgs::PointCloud2& cloud_msg, const std::string& frame_id = "base_link") {
        pcl::toROSMsg(*cloud_data.cloud_ptr, cloud_msg);
        cloud_msg.header.stamp = ros::Time(cloud_data.time);
        cloud_msg.header.frame_id = frame_id;
        return true;
    }
    #elif defined(ROS_VERSION_2)
    static bool CloudData2Msg(const CloudDataMID360& cloud_data, sensor_msgs::msg::PointCloud2& cloud_msg, const std::string& frame_id = "base_link") {
        pcl::toROSMsg(*cloud_data.cloud_ptr, cloud_msg);
        // 设置时间戳，分解为秒和纳秒
        double int_part;
        double frac_part = std::modf(cloud_data.time, &int_part);
        cloud_msg.header.stamp.sec = static_cast<int32_t>(int_part);
        cloud_msg.header.stamp.nanosec = static_cast<uint32_t>(frac_part * 1e9);
        cloud_msg.header.frame_id = frame_id;
        return true;
    }

    static bool CloudData2Msg(const CloudDataRobosense& cloud_data, sensor_msgs::msg::PointCloud2& cloud_msg, const std::string& frame_id = "base_link") {
        pcl::toROSMsg(*cloud_data.cloud_ptr, cloud_msg);
        // 设置时间戳，分解为秒和纳秒
        double int_part;
        double frac_part = std::modf(cloud_data.time, &int_part);
        cloud_msg.header.stamp.sec = static_cast<int32_t>(int_part);
        cloud_msg.header.stamp.nanosec = static_cast<uint32_t>(frac_part * 1e9);
        cloud_msg.header.frame_id = frame_id;
        return true;
    }

    static bool Frame2Msg(Frame& frame, sensor_msgs::msg::PointCloud2& cloud_msg, const std::string& frame_id = "base_link") {
        // 创建CompressModule对象并调用Decode
        CompressModule decompressor;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZIRT>> cloud_ptr(new pcl::PointCloud<pcl::PointXYZIRT>());    
        double timestamp = decompressor.Decode(frame, cloud_ptr);
        pcl::toROSMsg(*cloud_ptr, cloud_msg);
        double int_part;
        double frac_part = std::modf(timestamp, &int_part);
        cloud_msg.header.stamp.sec = static_cast<int32_t>(int_part);
        cloud_msg.header.stamp.nanosec = static_cast<uint32_t>(frac_part * 1e9);
        cloud_msg.header.frame_id = frame_id;
        return true;
    }

    static bool Frame2Msg(Frame64& frame, sensor_msgs::msg::PointCloud2& cloud_msg, const std::string& frame_id = "base_link") {
        // 创建CompressModule对象并调用Decode
        CompressModule decompressor;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZIRT>> cloud_ptr(new pcl::PointCloud<pcl::PointXYZIRT>());    
        double timestamp = decompressor.Decode(frame, cloud_ptr);
        pcl::toROSMsg(*cloud_ptr, cloud_msg);
        double int_part;
        double frac_part = std::modf(timestamp, &int_part);
        cloud_msg.header.stamp.sec = static_cast<int32_t>(int_part);
        cloud_msg.header.stamp.nanosec = static_cast<uint32_t>(frac_part * 1e9);
        cloud_msg.header.frame_id = frame_id;
        return true;
    }

    static bool Frame2Cloud(const FrameWithOutRT& frame, pcl::PointCloud<pcl::PointXYZI>& cloud, double& timestamp, Eigen::Matrix4d& pose)
    {
        CompressModule decompressor;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        decompressor.DecodeWithOutRt(frame, cloud_ptr, timestamp, pose);
        cloud = *cloud_ptr;
        return true;
    }
    #endif

    // ================= Camera =================
    #if defined(ROS_VERSION_1)
    static bool Msg2CameraData(const sensor_msgs::Image& img_msg, CameraData& camera_data) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            camera_data.image = cv_ptr->image;
            camera_data.time = img_msg.header.stamp.toSec();
            return true;
        } catch (cv_bridge::Exception& e) {
            return false;
        }
    }
    #elif defined(ROS_VERSION_2)
    static bool Msg2CameraData(const sensor_msgs::msg::Image& img_msg, CameraData& camera_data) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            camera_data.image = cv_ptr->image;
            camera_data.time = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9;
            return true;
        } catch (cv_bridge::Exception& e) {
            return false;
        }
    }

    static bool Msg2CameraData(const shm_msgs::msg::ShmImage6m& img_msg, CameraData& camera_data) {
        auto sensor_msg = std::make_shared<sensor_msgs::msg::Image>();
        sensor_msg->height = img_msg.height;
        sensor_msg->width = img_msg.width;
        auto encoding_data = img_msg.encoding.data;
        std::string encoding_string(encoding_data.begin(), encoding_data.end());
        encoding_string.erase(
            std::find(encoding_string.begin(), encoding_string.end(), '\0'),
            encoding_string.end());
        sensor_msg->encoding = encoding_string;
        sensor_msg->is_bigendian = img_msg.is_bigendian;
        sensor_msg->step = img_msg.step;
        sensor_msg->data.insert(sensor_msg->data.end(), img_msg.data.begin(),
                                img_msg.data.begin() +
                                    (img_msg.height * img_msg.step));
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(sensor_msg, sensor_msg->encoding);
        camera_data.image = cv_ptr->image;
        camera_data.time = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9;
        return true;
    }
    #endif

    // 将OpenCV图像转换为ROS图像消息
    #if defined(ROS_VERSION_1)
    static bool CameraData2Msg(const CameraData& camera_data, sensor_msgs::Image& img_msg, const std::string& frame_id = "camera") {
        cv_bridge::CvImage cv_image;
        cv_image.image = camera_data.image;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.header.stamp = ros::Time(camera_data.time);
        cv_image.header.frame_id = frame_id;
        cv_image.toImageMsg(img_msg);
        return true;
    }
    #elif defined(ROS_VERSION_2)
    static bool CameraData2Msg(const CameraData& camera_data, sensor_msgs::msg::Image& img_msg, const std::string& frame_id = "camera") {
        cv_bridge::CvImage cv_image;
        cv_image.image = camera_data.image;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        // 设置时间戳，分解为秒和纳秒
        double int_part;
        double frac_part = std::modf(camera_data.time, &int_part);
        cv_image.header.stamp.sec = static_cast<int32_t>(int_part);
        cv_image.header.stamp.nanosec = static_cast<uint32_t>(frac_part * 1e9);
        cv_image.header.frame_id = frame_id;
        img_msg = *cv_image.toImageMsg();
        return true;
    }
    
    // 转换成压缩格式图像
    // static bool CameraData2Msg(const CameraData& camera_data, sensor_msgs::msg::CompressedImage& compressed_msg, const std::string& format = "png", const int quality = 0, const std::string& frame_id = "camera") {
    //     try {
    //         // 创建cv_bridge对象
    //         cv_bridge::CvImage cv_image;
    //         cv_image.image = camera_data.image;
    //         cv_image.encoding = sensor_msgs::image_encodings::BGR8;
            
    //         // 设置时间戳，分解为秒和纳秒
    //         double int_part;
    //         double frac_part = std::modf(camera_data.time, &int_part);
    //         cv_image.header.stamp.sec = static_cast<int32_t>(int_part);
    //         cv_image.header.stamp.nanosec = static_cast<uint32_t>(frac_part * 1e9);
    //         cv_image.header.frame_id = frame_id;
            
    //         try {
    //             compressed_msg = *cv_image.toCompressedImageMsg();
    //             return true;
    //         } catch (const std::exception& e) {
    //             std::cerr << "使用cv_bridge压缩图像失败: " << e.what() << std::endl;
    //             std::cerr << "尝试使用OpenCV直接压缩..." << std::endl;
    //         }
    //         std::vector<uchar> buffer;
    //         std::vector<int> params;
    //         params.push_back(cv::IMWRITE_JPEG_QUALITY);
    //         params.push_back(95); // 默认高质量
            
    //         if (!cv::imencode(".jpg", cv_image.image, buffer, params)) {
    //             std::cerr << "OpenCV压缩JPEG失败!" << std::endl;
    //             return false;
    //         }
    //         compressed_msg.format = "jpeg";
    //         compressed_msg.data = std::vector<uint8_t>(buffer.begin(), buffer.end());
    //         return true;
    //     } catch (const std::exception& e) {
    //         std::cerr << "压缩图像时发生错误: " << e.what() << std::endl;
    //         return false;
    //     }
    // }
    #endif

    // ================= IMU =================
    #if defined(ROS_VERSION_1)
    static bool Msg2IMUData(const sensor_msgs::Imu& imu_msg, IMUData& imu_data) {
        imu_data.time = imu_msg.header.stamp.toSec();
        
        // 线性加速度
        imu_data.linear_acceleration.x = imu_msg.linear_acceleration.x;
        imu_data.linear_acceleration.y = imu_msg.linear_acceleration.y;
        imu_data.linear_acceleration.z = imu_msg.linear_acceleration.z;
        
        // 角速度
        imu_data.angular_velocity.x = imu_msg.angular_velocity.x;
        imu_data.angular_velocity.y = imu_msg.angular_velocity.y;
        imu_data.angular_velocity.z = imu_msg.angular_velocity.z;
        
        // 姿态四元数
        imu_data.orientation.x = imu_msg.orientation.x;
        imu_data.orientation.y = imu_msg.orientation.y;
        imu_data.orientation.z = imu_msg.orientation.z;
        imu_data.orientation.w = imu_msg.orientation.w;
        
        return true;
    }
    #elif defined(ROS_VERSION_2)
    static bool Msg2IMUData(const sensor_msgs::msg::Imu& imu_msg, IMUData& imu_data) {
        imu_data.time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;
        
        // 线性加速度
        imu_data.linear_acceleration.x = imu_msg.linear_acceleration.x;
        imu_data.linear_acceleration.y = imu_msg.linear_acceleration.y;
        imu_data.linear_acceleration.z = imu_msg.linear_acceleration.z;
        
        // 角速度
        imu_data.angular_velocity.x = imu_msg.angular_velocity.x;
        imu_data.angular_velocity.y = imu_msg.angular_velocity.y;
        imu_data.angular_velocity.z = imu_msg.angular_velocity.z;
        
        // 姿态四元数
        imu_data.orientation.x = imu_msg.orientation.x;
        imu_data.orientation.y = imu_msg.orientation.y;
        imu_data.orientation.z = imu_msg.orientation.z;
        imu_data.orientation.w = imu_msg.orientation.w;
        
        return true;
    }
    #endif

    // 将自定义IMU数据转换为ROS的IMU消息
    #if defined(ROS_VERSION_1)
    static bool IMUData2Msg(const IMUData& imu_data, sensor_msgs::Imu& imu_msg, const std::string& frame_id = "imu_link") {
        imu_msg.header.stamp = ros::Time(imu_data.time);
        imu_msg.header.frame_id = frame_id;
        
        // 线性加速度
        imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x;
        imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y;
        imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z;
        
        // 角速度
        imu_msg.angular_velocity.x = imu_data.angular_velocity.x;
        imu_msg.angular_velocity.y = imu_data.angular_velocity.y;
        imu_msg.angular_velocity.z = imu_data.angular_velocity.z;
        
        // 姿态四元数
        imu_msg.orientation.x = imu_data.orientation.x;
        imu_msg.orientation.y = imu_data.orientation.y;
        imu_msg.orientation.z = imu_data.orientation.z;
        imu_msg.orientation.w = imu_data.orientation.w;
        
        return true;
    }
    #elif defined(ROS_VERSION_2)
    static bool IMUData2Msg(const IMUData& imu_data, sensor_msgs::msg::Imu& imu_msg, const std::string& frame_id = "imu_link") {
        // 设置时间戳，分解为秒和纳秒
        double int_part;
        double frac_part = std::modf(imu_data.time, &int_part);
        imu_msg.header.stamp.sec = static_cast<int32_t>(int_part);
        imu_msg.header.stamp.nanosec = static_cast<uint32_t>(frac_part * 1e9);
        imu_msg.header.frame_id = frame_id;
        
        // 线性加速度
        imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x;
        imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y;
        imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z;
        
        // 角速度
        imu_msg.angular_velocity.x = imu_data.angular_velocity.x;
        imu_msg.angular_velocity.y = imu_data.angular_velocity.y;
        imu_msg.angular_velocity.z = imu_data.angular_velocity.z;
        
        // 姿态四元数
        imu_msg.orientation.x = imu_data.orientation.x;
        imu_msg.orientation.y = imu_data.orientation.y;
        imu_msg.orientation.z = imu_data.orientation.z;
        imu_msg.orientation.w = imu_data.orientation.w;
        
        return true;
    }
    #endif

    // ================= GPS =================
    #if defined(ROS_VERSION_2)
    // 注意方向
    static bool Msg2GPSData(const msfl_msgs::msg::INSPVAXData& fromMsg, INSPVAXData& inspvax_data) {
        inspvax_data.time = fromMsg.header.stamp.sec + fromMsg.header.stamp.nanosec * 1e-9;
        inspvax_data.head.head[0] = fromMsg.head.head[0];
        inspvax_data.head.head[1] = fromMsg.head.head[1];
        inspvax_data.head.head[2] = fromMsg.head.head[2];

        inspvax_data.head.head_lenth = fromMsg.head.head_length; // 注意拼写差异：head_lenth vs head_length
        inspvax_data.head.msg_id = fromMsg.head.msg_id;
        inspvax_data.head.msg_type = static_cast<char>(fromMsg.head.msg_type);
        inspvax_data.head.port_address = fromMsg.head.port_address;

        inspvax_data.head.msg_len = fromMsg.head.msg_len;
        inspvax_data.head.sequence = fromMsg.head.sequence;
        inspvax_data.head.idle_time = fromMsg.head.idle_time;
        inspvax_data.head.time_status = fromMsg.head.time_status;
        inspvax_data.head.week = fromMsg.head.week;
        inspvax_data.head.gps_ms = fromMsg.head.gps_ms;
        inspvax_data.head.receiver_status = fromMsg.head.receiver_status;
        inspvax_data.head.reserved = fromMsg.head.reserved;
        inspvax_data.head.receiver_sw_version = fromMsg.head.receiver_sw_version;
        inspvax_data.ins_status = fromMsg.ins_status;
        inspvax_data.pos_type = fromMsg.pos_type;
        inspvax_data.latitude = fromMsg.latitude;
        inspvax_data.longitude = fromMsg.longitude;
        inspvax_data.height = fromMsg.height;
        inspvax_data.undulation = fromMsg.undulation;
        inspvax_data.north_velocity = fromMsg.north_velocity;
        inspvax_data.east_velocity = fromMsg.east_velocity;
        inspvax_data.up_velocity = fromMsg.up_velocity;
        inspvax_data.roll = fromMsg.roll;
        inspvax_data.pitch = fromMsg.pitch;
        inspvax_data.azimuth = fromMsg.azimuth;
        inspvax_data.latitude_deviation = fromMsg.latitude_deviation;
        inspvax_data.longitude_deviation = fromMsg.longitude_deviation;
        inspvax_data.height_deviation = fromMsg.height_deviation;
        inspvax_data.north_velocity_deviation = fromMsg.north_velocity_deviation;
        inspvax_data.east_velocity_deviation = fromMsg.east_velocity_deviation;
        inspvax_data.up_velocity_deviation = fromMsg.up_velocity_deviation;
        inspvax_data.roll_deviation = fromMsg.roll_deviation;
        inspvax_data.pitch_deviation = fromMsg.pitch_deviation;
        inspvax_data.azimuth_deviation = fromMsg.azimuth_deviation;
        inspvax_data.ext_sol_stat = fromMsg.ext_sol_stat;
        inspvax_data.time_since_update = fromMsg.time_since_update;
        inspvax_data.updateFlag = fromMsg.update_flag;
        return true;
    }

    static bool GPSData2Msg(const INSPVAXData& inspvax_data, msfl_msgs::msg::INSPVAXData& toMsg, const std::string& frame_id = "fixposition") {
        // 设置Header
        double int_part;
        double frac_part = std::modf(inspvax_data.time, &int_part);
        toMsg.header.stamp.sec = static_cast<int32_t>(int_part);
        toMsg.header.stamp.nanosec = static_cast<uint32_t>(frac_part * 1e9);
        toMsg.header.frame_id = frame_id;
        
        // 复制头部数据
        toMsg.head.head[0] = inspvax_data.head.head[0];
        toMsg.head.head[1] = inspvax_data.head.head[1];
        toMsg.head.head[2] = inspvax_data.head.head[2];
        
        toMsg.head.head_length = inspvax_data.head.head_lenth; // 注意拼写差异：head_length vs head_lenth
        toMsg.head.msg_id = inspvax_data.head.msg_id;
        toMsg.head.msg_type = static_cast<uint8_t>(inspvax_data.head.msg_type);
        toMsg.head.port_address = inspvax_data.head.port_address;
        
        toMsg.head.msg_len = inspvax_data.head.msg_len;
        toMsg.head.sequence = inspvax_data.head.sequence;
        toMsg.head.idle_time = inspvax_data.head.idle_time;
        toMsg.head.time_status = inspvax_data.head.time_status;
        toMsg.head.week = inspvax_data.head.week;
        toMsg.head.gps_ms = inspvax_data.head.gps_ms;
        toMsg.head.receiver_status = inspvax_data.head.receiver_status;
        toMsg.head.reserved = inspvax_data.head.reserved;
        toMsg.head.receiver_sw_version = inspvax_data.head.receiver_sw_version;
        
        // 复制GNSS/INS状态和位置数据
        toMsg.ins_status = inspvax_data.ins_status;
        toMsg.pos_type = inspvax_data.pos_type;
        toMsg.latitude = inspvax_data.latitude;
        toMsg.longitude = inspvax_data.longitude;
        toMsg.height = inspvax_data.height;
        toMsg.undulation = inspvax_data.undulation;
        
        // 复制速度数据
        toMsg.north_velocity = inspvax_data.north_velocity;
        toMsg.east_velocity = inspvax_data.east_velocity;
        toMsg.up_velocity = inspvax_data.up_velocity;
        
        // 复制姿态数据
        toMsg.roll = inspvax_data.roll;
        toMsg.pitch = inspvax_data.pitch;
        toMsg.azimuth = inspvax_data.azimuth;
        
        // 复制偏差数据
        toMsg.latitude_deviation = inspvax_data.latitude_deviation;
        toMsg.longitude_deviation = inspvax_data.longitude_deviation;
        toMsg.height_deviation = inspvax_data.height_deviation;
        toMsg.north_velocity_deviation = inspvax_data.north_velocity_deviation;
        toMsg.east_velocity_deviation = inspvax_data.east_velocity_deviation;
        toMsg.up_velocity_deviation = inspvax_data.up_velocity_deviation;
        
        return true;
    }
#endif
};
}

#endif