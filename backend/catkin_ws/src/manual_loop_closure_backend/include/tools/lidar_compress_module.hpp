#ifndef COMPRESS_MODULE_HPP
#define COMPRESS_MODULE_HPP

#include "data_struct.hpp"
#include "common/runtime_logger.h"
#include <zstd.h>

class CompressModule {
public:
    CompressModule(){}
    ~CompressModule(){}

#ifdef MS_MAPPING_ENABLE_RING_LIDAR_COMPRESSION
    struct TimestampComparator {
        bool operator()(const pcl::PointXYZIRT& a, const pcl::PointXYZIRT& b) {
            return a.timestamp > b.timestamp; // 小顶堆，时间戳小的在顶部
        }
    };

    /**
     * @brief 压缩点云数据
     * 
     * @param frame     输出编码后的frame
     * @param cloud_ptr 输入点云数据
     */
    void Encode(Frame& frame, std::shared_ptr<pcl::PointCloud<pcl::PointXYZIRT>> cloud_ptr)
    {
        if(cloud_ptr->empty()) {
            return;
        }
        std::vector<std::priority_queue<pcl::PointXYZIRT, 
                                        std::vector<pcl::PointXYZIRT>, 
                                        TimestampComparator>> ring_queues(32);

        for(const auto& point : *cloud_ptr) {
            if(point.ring >= 0 && point.ring < 32) {
                if(point.timestamp == 0)
                {
                    continue;
                }
                ring_queues[point.ring].push(point);
            }
        }
        // memset 0 offset_table
        memset(frame.offset_table, 0, sizeof(frame.offset_table));

        for(int i = 0; i < 32; i++)
        {
            Ring& ring = frame.rings[i];
            if(ring_queues[i].empty())
            {
                continue;
            }
            ring.timestamp = ring_queues[i].top().timestamp * 1e8;
            while(!ring_queues[i].empty())
            {
                pcl::PointXYZIRT point = ring_queues[i].top();
                ring_queues[i].pop();
                int index = std::round(((point.timestamp * 1e8) - frame.rings[i].timestamp) / 5556.0);
                if (index >= 0 && index < 1800) {
                    frame.offset_table[index] |= (1 << i);
                }
            
                Point p;
                pack(point.x, point.y, point.z, point.intensity, p.p);
                ring.Points.push_back(p);
            }
        }                                
    }

    double Decode(Frame& frame, std::shared_ptr<pcl::PointCloud<pcl::PointXYZIRT>> cloud_ptr)
    {
        double mini_timestamp = 1e20;
        // 遍历帧中的所有环
        for (int i = 0; i < 1800; i++) {
            // 遍历环中的所有点
            for (int j = 0; j < 32; j++) {
                if(frame.offset_table[i] & (1 << j)) {
                    // 解包数据
                    float x, y, z, intensity;
                    if (frame.rings[j].Points.empty()) {
                        continue;
                    }
                    Point point = frame.rings[j].Points.front();
                    
                    // 使用erase替换pop_front
                    frame.rings[j].Points.erase(frame.rings[j].Points.begin());
                    unpack(point.p, x, y, z, intensity);
                    // 创建PCL点并添加到点云
                    pcl::PointXYZIRT pcl_point;
                    pcl_point.x = x;
                    pcl_point.y = y;
                    pcl_point.z = z;
                    pcl_point.intensity = intensity;
                    pcl_point.ring = 0;
                    long long timestamp = frame.rings[j].timestamp + i * 5556;
                    pcl_point.timestamp = timestamp / 1e8;
                    cloud_ptr->points.push_back(pcl_point);

                    if(pcl_point.timestamp < mini_timestamp) {
                        mini_timestamp = pcl_point.timestamp;
                    }
                }
            }
        }
        
        // 设置点云的其他属性
        cloud_ptr->width = cloud_ptr->points.size();
        cloud_ptr->height = 1;
        cloud_ptr->is_dense = true;

        return mini_timestamp;
    }

    void Encode(Frame64& frame, std::shared_ptr<pcl::PointCloud<pcl::PointXYZIRT>> cloud_ptr)
    {

        // std::cout << "encode 64 size: " << cloud_ptr->points.size() << std::endl;
        if(cloud_ptr->empty()) {
            return;
        }
        std::vector<std::priority_queue<pcl::PointXYZIRT, 
                                        std::vector<pcl::PointXYZIRT>, 
                                        TimestampComparator>> ring_queues(64);

        for(const auto& point : *cloud_ptr) {
            if(point.ring >= 0 && point.ring < 64) {
                if(point.timestamp == 0)
                {
                    std::cout << "point.timestamp == 0" << std::endl;
                    continue;
                }
                ring_queues[point.ring].push(point);
            }
        }
        // memset 0 offset_table
        memset(frame.offset_table, 0, sizeof(frame.offset_table));

        for(int i = 0; i < 64; i++)
        {
            Ring& ring = frame.rings[i];
            if(ring_queues[i].empty())
            {
                continue;
            }
            ring.timestamp = ring_queues[i].top().timestamp * 1e8;
            while(!ring_queues[i].empty())
            {
                pcl::PointXYZIRT point = ring_queues[i].top();
                ring_queues[i].pop();
                // std::cout << std::fixed << std::setprecision(20) << "i :" << i << " point.timestamp: " << point.timestamp << " first point timestamp: " << frame.rings[i].timestamp << std::endl;
                int index = std::round(((point.timestamp * 1e8) - frame.rings[i].timestamp) / 5556.0);
                if (index >= 0 && index < 1800) {
                    frame.offset_table[index] |= (1 << i);
                }
            
                Point p;
                pack(point.x, point.y, point.z, point.intensity, p.p);
                ring.Points.push_back(p);
            }
        }                                
    }

    double Decode(Frame64& frame, std::shared_ptr<pcl::PointCloud<pcl::PointXYZIRT>> cloud_ptr)
    {
        double mini_timestamp = 1e20;
        // 遍历帧中的所有环
        for (int i = 0; i < 1800; i++) {
            // 遍历环中的所有点
            for (int j = 0; j < 64; j++) {
                if(frame.offset_table[i] & (1 << j)) {
                    // 解包数据
                    float x, y, z, intensity;
                    if (frame.rings[j].Points.empty()) {
                        continue;
                    }
                    Point point = frame.rings[j].Points.front();
                    
                    // 使用erase替换pop_front
                    frame.rings[j].Points.erase(frame.rings[j].Points.begin());
                    unpack(point.p, x, y, z, intensity);
                    // 创建PCL点并添加到点云
                    pcl::PointXYZIRT pcl_point;
                    pcl_point.x = x;
                    pcl_point.y = y;
                    pcl_point.z = z;
                    pcl_point.intensity = intensity;
                    pcl_point.ring = 0;
                    long long timestamp = frame.rings[j].timestamp + i * 5556;
                    pcl_point.timestamp = timestamp / 1e8;
                    cloud_ptr->points.push_back(pcl_point);

                    if(pcl_point.timestamp < mini_timestamp) {
                        mini_timestamp = pcl_point.timestamp;
                    }
                }
            }
        }
        
        // 设置点云的其他属性
        cloud_ptr->width = cloud_ptr->points.size();
        cloud_ptr->height = 1;
        cloud_ptr->is_dense = true;

        return mini_timestamp;
    }

#endif  // MS_MAPPING_ENABLE_RING_LIDAR_COMPRESSION

    void EncodeWithOutRT(FrameWithOutRT& frame, 
                         std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_ptr, 
                         double timestamp, 
                         Eigen::Matrix4d& pose)
    {
        if(cloud_ptr->empty()) {
            return;
        }
        frame.timestamp = timestamp * 1e8; 
        for(int i = 0; i < 16; i++) {
            frame.pose[i] = pose(i/4, i%4);  // 修改为正确的矩阵索引
        }
        for(const auto& point : *cloud_ptr) {
            Point p;
            pack(point.x, point.y, point.z, point.intensity, p.p);
            frame.Points.push_back(p);
        }
    }
    
    void DecodeWithOutRt(const FrameWithOutRT& frame, 
                         std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_ptr, 
                         double& timestamp,
                         Eigen::Matrix4d& pose)
    {
        if(frame.Points.empty()) {
            return;
        }
        
        timestamp = frame.timestamp / 1e8;
        for(int i = 0; i < 16; i++) {
            pose(i/4, i%4) = frame.pose[i];  // 修改为正确的矩阵索引
        }
        cloud_ptr->clear();
        
        for(const auto& p : frame.Points) {
            pcl::PointXYZI point;
            unpack(p.p, point.x, point.y, point.z, point.intensity);
            cloud_ptr->push_back(point);
        }
    }

#ifdef MS_MAPPING_ENABLE_RING_LIDAR_COMPRESSION

    static bool SaveToBin(const Frame& frame, std::ofstream& file) {
        // 首先将数据序列化到内存中
        std::vector<char> buffer;
        
        // 写入offset_table
        buffer.insert(buffer.end(), 
                     reinterpret_cast<const char*>(frame.offset_table), 
                     reinterpret_cast<const char*>(frame.offset_table) + sizeof(frame.offset_table));
        
        // 写入环的数量（固定为32）
        const uint32_t num_rings = 32;
        buffer.insert(buffer.end(), 
                     reinterpret_cast<const char*>(&num_rings), 
                     reinterpret_cast<const char*>(&num_rings) + sizeof(num_rings));
        
        // 遍历所有环
        for (const auto& ring : frame.rings) {
            // 写入时间戳
            buffer.insert(buffer.end(), 
                         reinterpret_cast<const char*>(&ring.timestamp), 
                         reinterpret_cast<const char*>(&ring.timestamp) + sizeof(ring.timestamp));
            
            // 写入点的数量
            uint32_t num_points = static_cast<uint32_t>(ring.Points.size());
            buffer.insert(buffer.end(), 
                         reinterpret_cast<const char*>(&num_points), 
                         reinterpret_cast<const char*>(&num_points) + sizeof(num_points));
            
            // 写入所有点
            for (const auto& point : ring.Points) {
                // 写入p值（包含所有数据）
                buffer.insert(buffer.end(), 
                             reinterpret_cast<const char*>(&point.p), 
                             reinterpret_cast<const char*>(&point.p) + sizeof(point.p));
            }
        }
        
        // 使用Zstd压缩数据，设置压缩级别6以获得高压缩率
        size_t max_compressed_size = ZSTD_compressBound(buffer.size());
        std::vector<char> compressed_buffer(max_compressed_size);
        
        // 执行压缩，使用高压缩级别
        size_t compressed_size = ZSTD_compress(compressed_buffer.data(), max_compressed_size,
                                             buffer.data(), buffer.size(), 6);
        
        if (ZSTD_isError(compressed_size)) {
            std::cerr << "Zstd压缩失败: " << ZSTD_getErrorName(compressed_size) << std::endl;
            return false;
        }
        
        // 调整压缩缓冲区大小到实际压缩后的大小
        compressed_buffer.resize(compressed_size);
        

        // 检查文件是否打开
        if (!file.is_open()) {
            std::cerr << "文件未打开" << std::endl;
            return false;
        }
        
        // 写入原始数据大小（用于解压缩时分配内存）
        uint64_t origin_size = buffer.size();
        uint64_t compressed_size_uint64 = compressed_buffer.size();
        file.write(reinterpret_cast<const char*>(&origin_size), sizeof(origin_size));
        file.write(reinterpret_cast<const char*>(&compressed_size_uint64), sizeof(compressed_size_uint64));
        file.write(reinterpret_cast<const char*>(compressed_buffer.data()), compressed_size_uint64 * sizeof(char));
        
        return true;
    }
    
    static bool SaveToBin(const Frame64& frame, std::ofstream& file) {
        // 首先将数据序列化到内存中
        std::vector<char> buffer;
        
        // 写入offset_table
        buffer.insert(buffer.end(), 
                     reinterpret_cast<const char*>(frame.offset_table), 
                     reinterpret_cast<const char*>(frame.offset_table) + sizeof(frame.offset_table));
        
        // 写入环的数量（固定为64）
        const uint32_t num_rings = 64;
        buffer.insert(buffer.end(), 
                     reinterpret_cast<const char*>(&num_rings), 
                     reinterpret_cast<const char*>(&num_rings) + sizeof(num_rings));
        
        // 遍历所有环
        for (const auto& ring : frame.rings) {
            // 写入时间戳
            buffer.insert(buffer.end(), 
                         reinterpret_cast<const char*>(&ring.timestamp), 
                         reinterpret_cast<const char*>(&ring.timestamp) + sizeof(ring.timestamp));
            
            // 写入点的数量
            uint32_t num_points = static_cast<uint32_t>(ring.Points.size());
            buffer.insert(buffer.end(), 
                         reinterpret_cast<const char*>(&num_points), 
                         reinterpret_cast<const char*>(&num_points) + sizeof(num_points));
            
            // 写入所有点
            for (const auto& point : ring.Points) {
                // 写入p值（包含所有数据）
                buffer.insert(buffer.end(), 
                             reinterpret_cast<const char*>(&point.p), 
                             reinterpret_cast<const char*>(&point.p) + sizeof(point.p));
            }
        }
        
        // 使用Zstd压缩数据，设置压缩级别6以获得高压缩率
        size_t max_compressed_size = ZSTD_compressBound(buffer.size());
        std::vector<char> compressed_buffer(max_compressed_size);
        
        // 执行压缩，使用高压缩级别
        size_t compressed_size = ZSTD_compress(compressed_buffer.data(), max_compressed_size,
                                             buffer.data(), buffer.size(), 6);
        
        if (ZSTD_isError(compressed_size)) {
            std::cerr << "Zstd压缩失败: " << ZSTD_getErrorName(compressed_size) << std::endl;
            return false;
        }
        
        // 调整压缩缓冲区大小到实际压缩后的大小
        compressed_buffer.resize(compressed_size);
        

        // 检查文件是否打开
        if (!file.is_open()) {
            std::cerr << "文件未打开" << std::endl;
            return false;
        }
        
        // 写入原始数据大小（用于解压缩时分配内存）
        uint64_t origin_size = buffer.size();
        uint64_t compressed_size_uint64 = compressed_buffer.size();
        file.write(reinterpret_cast<const char*>(&origin_size), sizeof(origin_size));
        file.write(reinterpret_cast<const char*>(&compressed_size_uint64), sizeof(compressed_size_uint64));
        file.write(reinterpret_cast<const char*>(compressed_buffer.data()), compressed_size_uint64 * sizeof(char));
        
        return true;
    }

#endif  // MS_MAPPING_ENABLE_RING_LIDAR_COMPRESSION

    static bool SaveToBin(const FrameWithOutRT& frame, std::ofstream& file) {
        // 首先将数据序列化到内存中
        std::vector<char> buffer;
        
        // 写入时间戳
        buffer.insert(buffer.end(), 
                    reinterpret_cast<const char*>(&frame.timestamp), 
                    reinterpret_cast<const char*>(&frame.timestamp) + sizeof(frame.timestamp));
        
        // 写入pose数据
        buffer.insert(buffer.end(), 
                    reinterpret_cast<const char*>(frame.pose), 
                    reinterpret_cast<const char*>(frame.pose) + sizeof(frame.pose));
        
        // 写入点的数量
        uint32_t num_points = static_cast<uint32_t>(frame.Points.size());
        buffer.insert(buffer.end(), 
                    reinterpret_cast<const char*>(&num_points), 
                    reinterpret_cast<const char*>(&num_points) + sizeof(num_points));
        
        // 写入所有点
        for (const auto& point : frame.Points) {
            // 写入p值（包含所有数据）
            buffer.insert(buffer.end(), 
                        reinterpret_cast<const char*>(&point.p), 
                        reinterpret_cast<const char*>(&point.p) + sizeof(point.p));
        }
        
        // 使用Zstd压缩数据，设置压缩级别6以获得高压缩率
        size_t max_compressed_size = ZSTD_compressBound(buffer.size());
        std::vector<char> compressed_buffer(max_compressed_size);
        
        // 执行压缩，使用高压缩级别
        size_t compressed_size = ZSTD_compress(compressed_buffer.data(), max_compressed_size,
                                             buffer.data(), buffer.size(), 6);
        
        if (ZSTD_isError(compressed_size)) {
            std::cerr << "Zstd压缩失败: " << ZSTD_getErrorName(compressed_size) << std::endl;
            return false;
        }
        
        // 调整压缩缓冲区大小到实际压缩后的大小
        compressed_buffer.resize(compressed_size);
        
        // 检查文件是否打开
        if (!file.is_open()) {
            std::cerr << "文件未打开" << std::endl;
            return false;
        }
        
        // 写入原始数据大小和压缩后的大小
        uint64_t origin_size = buffer.size();
        uint64_t compressed_size_uint64 = compressed_buffer.size();
        file.write(reinterpret_cast<const char*>(&origin_size), sizeof(origin_size));
        file.write(reinterpret_cast<const char*>(&compressed_size_uint64), sizeof(compressed_size_uint64));
        file.write(reinterpret_cast<const char*>(compressed_buffer.data()), compressed_size_uint64 * sizeof(char));
        
        return true;
    }

    // 从二进制文件加载Frame结构（使用Zstd解压缩）
    // static Frame LoadFromBin(std::ifstream& file) {
    //     Frame frame;
        
    //     if (!file.is_open()) {
    //         std::cerr << "文件未打开" << std::endl;
    //         return frame;
    //     }
        
    //     // 读取原始数据大小
    //     uint64_t original_size;
    //     file.read(reinterpret_cast<char*>(&original_size), sizeof(original_size));
    //     std::cout << "original_size:" << original_size << std::endl;
    //     // 读取压缩后的数据
    //     std::vector<char> compressed_data;
    //     file.seekg(0, std::ios::end);
    //     size_t compressed_size = static_cast<size_t>(file.tellg()) - sizeof(original_size);
    //     file.seekg(sizeof(original_size), std::ios::beg);
        
    //     compressed_data.resize(compressed_size);
    //     file.read(compressed_data.data(), compressed_size);
        
    //     // 使用Zstd解压缩数据
    //     ZSTD_DCtx* dctx = ZSTD_createDCtx();
        
    //     // 初始化解压缩流
    //     size_t actual_decompressed_size = ZSTD_decompress(dctx, frame.offset_table, sizeof(frame.offset_table), compressed_data.data(), compressed_size);
    //     if (actual_decompressed_size == 0) {
    //         std::cerr << "Zstd解压缩失败" << std::endl;
    //         return frame;
    //     }
        
    //     // 计算实际解压大小
    //     std::cout << "期望解压大小: " << original_size << " bytes" << std::endl;
    //     std::cout << "实际解压大小: " << actual_decompressed_size << " bytes" << std::endl;
        
    //     // 结束解压缩
    //     ZSTD_freeDCtx(dctx);
        
    //     // 从解压缩后的数据中读取Frame结构
    //     size_t offset = 0;
        
    //     // 读取环的数量
    //     uint32_t num_rings;
    //     memcpy(&num_rings, frame.offset_table + offset, sizeof(num_rings));
    //     offset += sizeof(num_rings);
        
    //     // 确保环的数量不超过32
    //     num_rings = std::min(num_rings, static_cast<uint32_t>(32));
        
    //     // 遍历所有环
    //     for (uint32_t i = 0; i < num_rings; ++i) {
    //         // 读取时间戳
    //         memcpy(&frame.rings[i].timestamp, frame.offset_table + offset, sizeof(frame.rings[i].timestamp));
    //         offset += sizeof(frame.rings[i].timestamp);
            
    //         // 读取点的数量
    //         uint32_t num_points;
    //         memcpy(&num_points, frame.offset_table + offset, sizeof(num_points));
    //         offset += sizeof(num_points);
            
    //         // 预分配点的空间
    //         frame.rings[i].Points.resize(num_points);
            
    //         // 读取所有点
    //         for (uint32_t j = 0; j < num_points; ++j) {
    //             Point& point = frame.rings[i].Points[j];
                
    //             // 读取p值（包含所有数据）
    //             memcpy(&point.p, frame.offset_table + offset, sizeof(point.p));
    //             offset += sizeof(point.p);
    //         }
    //     }
        
    //     return frame;
    // }

    #ifdef MS_MAPPING_ENABLE_RING_LIDAR_COMPRESSION
    static void LoadFromAllBin(std::ifstream& file, std::vector<Frame>& frames) {
        if (!file.is_open()) {
            std::cerr << "文件未打开" << std::endl;
            return;
        }
        
        static int count = 0;
        while(true) {
            // 检查是否到达文件末尾
            if(file.eof()) {
                std::cout << "到达文件末尾" << std::endl;
                break;
            }

            // 保存起始位置
            size_t frame_start_pos = file.tellg();
            
            // 直接读取大小信息（read会自动移动文件指针）
            uint64_t original_size;
            uint64_t compressed_size;
            file.read(reinterpret_cast<char*>(&original_size), sizeof(uint64_t));
            file.read(reinterpret_cast<char*>(&compressed_size), sizeof(uint64_t));
            if(file.eof()) {
                std::cout << "读取大小后到达文件末尾" << std::endl;
                break;
            }
            
            // 直接读取压缩数据（不需要额外的seekg）
            std::vector<char> compressed_data;
            compressed_data.resize(compressed_size);
            file.read(compressed_data.data(), compressed_size * sizeof(char));

            // 计算下一帧位置
            size_t next_frame_start_pos = frame_start_pos + sizeof(uint64_t) + sizeof(uint64_t) + compressed_size * sizeof(char);
            file.seekg(next_frame_start_pos, std::ios::beg);

            // 使用Zstd解压缩数据
            std::vector<char> decompressed_buffer(original_size);
            
            // 执行解压缩
            size_t actual_decompressed_size = ZSTD_decompress(decompressed_buffer.data(), original_size,
                                                            compressed_data.data(), compressed_size);
            
            if (ZSTD_isError(actual_decompressed_size)) {
                std::cerr << "Zstd解压缩失败: " << ZSTD_getErrorName(actual_decompressed_size) << std::endl;
                continue;
            }

            // 从解压缩后的数据中读取Frame结构
            size_t offset = 0;
            
            Frame frame;
            // 读取offset_table
            memcpy(frame.offset_table, decompressed_buffer.data() + offset, sizeof(frame.offset_table));
            offset += sizeof(frame.offset_table);
            
            // 读取环的数量
            uint32_t num_rings;
            memcpy(&num_rings, decompressed_buffer.data() + offset, sizeof(num_rings));
            offset += sizeof(num_rings);
            
            // 确保环的数量不超过32
            num_rings = std::min(num_rings, static_cast<uint32_t>(32));
            
            // 遍历所有环
            for (uint32_t i = 0; i < num_rings; ++i) {
                // 读取时间戳
                memcpy(&frame.rings[i].timestamp, decompressed_buffer.data() + offset, sizeof(frame.rings[i].timestamp));
                offset += sizeof(frame.rings[i].timestamp);
                
                // 读取点的数量
                uint32_t num_points;
                memcpy(&num_points, decompressed_buffer.data() + offset, sizeof(num_points));
                offset += sizeof(num_points);
                
                // 预分配点的空间
                frame.rings[i].Points.resize(num_points);
                
                // 读取所有点
                for (uint32_t j = 0; j < num_points; ++j) {
                    Point& point = frame.rings[i].Points[j];
                    
                    // 读取p值（包含所有数据）
                    memcpy(&point.p, decompressed_buffer.data() + offset, sizeof(point.p));
                    offset += sizeof(point.p);
                }
            }
            frames.push_back(frame);
            std::cout << "加载帧: " << count++ << std::endl;
        }
    }

    static void LoadFromAllBin(std::ifstream& file, std::vector<Frame64>& frames) {
        if (!file.is_open()) {
            std::cerr << "文件未打开" << std::endl;
            return;
        }
        
        static int count = 0;
        while(true) {
            // 检查是否到达文件末尾
            if(file.eof()) {
                std::cout << "到达文件末尾" << std::endl;
                break;
            }

            // 保存起始位置
            size_t frame_start_pos = file.tellg();
            
            // 直接读取大小信息（read会自动移动文件指针）
            uint64_t original_size;
            uint64_t compressed_size;
            file.read(reinterpret_cast<char*>(&original_size), sizeof(uint64_t));
            file.read(reinterpret_cast<char*>(&compressed_size), sizeof(uint64_t));
            if(file.eof()) {
                std::cout << "读取大小后到达文件末尾" << std::endl;
                break;
            }
            
            // 直接读取压缩数据（不需要额外的seekg）
            std::vector<char> compressed_data;
            compressed_data.resize(compressed_size);
            file.read(compressed_data.data(), compressed_size * sizeof(char));

            // 计算下一帧位置
            size_t next_frame_start_pos = frame_start_pos + sizeof(uint64_t) + sizeof(uint64_t) + compressed_size * sizeof(char);
            file.seekg(next_frame_start_pos, std::ios::beg);

            // 使用Zstd解压缩数据
            std::vector<char> decompressed_buffer(original_size);
            
            // 执行解压缩
            size_t actual_decompressed_size = ZSTD_decompress(decompressed_buffer.data(), original_size,
                                                            compressed_data.data(), compressed_size);
            
            if (ZSTD_isError(actual_decompressed_size)) {
                std::cerr << "Zstd解压缩失败: " << ZSTD_getErrorName(actual_decompressed_size) << std::endl;
                continue;
            }

            // 从解压缩后的数据中读取Frame结构
            size_t offset = 0;
            
            Frame64 frame;
            // 读取offset_table
            memcpy(frame.offset_table, decompressed_buffer.data() + offset, sizeof(frame.offset_table));
            offset += sizeof(frame.offset_table);
            
            // 读取环的数量
            uint32_t num_rings;
            memcpy(&num_rings, decompressed_buffer.data() + offset, sizeof(num_rings));
            offset += sizeof(num_rings);
            
            // 确保环的数量不超过32
            num_rings = std::min(num_rings, static_cast<uint32_t>(64));
            
            // 遍历所有环
            for (uint32_t i = 0; i < num_rings; ++i) {
                // 读取时间戳
                memcpy(&frame.rings[i].timestamp, decompressed_buffer.data() + offset, sizeof(frame.rings[i].timestamp));
                offset += sizeof(frame.rings[i].timestamp);
                
                // 读取点的数量
                uint32_t num_points;
                memcpy(&num_points, decompressed_buffer.data() + offset, sizeof(num_points));
                offset += sizeof(num_points);
                
                // 预分配点的空间
                frame.rings[i].Points.resize(num_points);
                
                // 读取所有点
                for (uint32_t j = 0; j < num_points; ++j) {
                    Point& point = frame.rings[i].Points[j];
                    
                    // 读取p值（包含所有数据）
                    memcpy(&point.p, decompressed_buffer.data() + offset, sizeof(point.p));
                    offset += sizeof(point.p);
                }
            }
            frames.push_back(frame);
            std::cout << "加载帧: " << count++ << std::endl;
        }
    }
    #endif  // MS_MAPPING_ENABLE_RING_LIDAR_COMPRESSION

    static void LoadFromAllBin(std::ifstream& file, std::vector<FrameWithOutRT>& frames) 
    {
        if (!file.is_open()) {
            std::cerr << "文件未打开" << std::endl;
            return;
        }
        
        while(true) {
            // 检查是否到达文件末尾
            if(file.eof()) {
                std::cout << "到达文件末尾" << std::endl;
                break;
            }

            // 保存起始位置
            size_t frame_start_pos = file.tellg();
            
            // 读取大小信息
            uint64_t original_size;
            uint64_t compressed_size;
            file.read(reinterpret_cast<char*>(&original_size), sizeof(uint64_t));
            file.read(reinterpret_cast<char*>(&compressed_size), sizeof(uint64_t));
            if(file.eof()) {
                std::cout << "读取大小后到达文件末尾" << std::endl;
                break;
            }
            
            // 读取压缩数据
            std::vector<char> compressed_data;
            compressed_data.resize(compressed_size);
            file.read(compressed_data.data(), compressed_size * sizeof(char));

            // 计算下一帧位置
            size_t next_frame_start_pos = frame_start_pos + sizeof(uint64_t) + sizeof(uint64_t) + compressed_size * sizeof(char);
            file.seekg(next_frame_start_pos, std::ios::beg);

            // 使用Zstd解压缩数据
            std::vector<char> decompressed_buffer(original_size);
            
            // 执行解压缩
            size_t actual_decompressed_size = ZSTD_decompress(decompressed_buffer.data(), original_size,
                                                            compressed_data.data(), compressed_size);
            
            if (ZSTD_isError(actual_decompressed_size)) {
                std::cerr << "Zstd解压缩失败: " << ZSTD_getErrorName(actual_decompressed_size) << std::endl;
                continue;
            }

            // 从解压缩后的数据中读取FrameWithOutRT结构
            size_t offset = 0;
            
            FrameWithOutRT frame;
            
            // 读取时间戳
            memcpy(&frame.timestamp, decompressed_buffer.data() + offset, sizeof(frame.timestamp));
            offset += sizeof(frame.timestamp);
            
            // 读取pose数据
            memcpy(frame.pose, decompressed_buffer.data() + offset, sizeof(frame.pose));
            offset += sizeof(frame.pose);
            
            // 读取点的数量
            uint32_t num_points;
            memcpy(&num_points, decompressed_buffer.data() + offset, sizeof(num_points));
            offset += sizeof(num_points);
            
            // 预分配点的空间
            frame.Points.resize(num_points);
            
            // 读取所有点
            for (uint32_t i = 0; i < num_points; ++i) {
                Point& point = frame.Points[i];
                
                // 读取p值（包含所有数据）
                memcpy(&point.p, decompressed_buffer.data() + offset, sizeof(point.p));
                offset += sizeof(point.p);
            }
            
            frames.push_back(frame);
        }
    }
};
#endif
