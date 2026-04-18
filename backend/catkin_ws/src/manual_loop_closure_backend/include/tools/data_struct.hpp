#ifndef __DATA_STRUCT_HPP__
#define __DATA_STRUCT_HPP__

#include <iostream>
#include <vector>
#include <cmath>

#pragma pack(push, 1)
struct Point
{
    uint64_t p;
};

struct Ring
{
    uint64_t timestamp;
    std::vector<Point> Points;
};

struct Frame
{
    uint32_t offset_table[1800];
    Ring rings[32];
};

struct Frame64
{
    uint64_t offset_table[1800];
    Ring rings[64];
};

/**
 * @brief 关键帧，不包含每个点的时间戳和ring值
 * 
 */
struct FrameWithOutRT
{
    uint64_t timestamp; // 帧的时间戳
    double pose[16]; // 位姿
    std::vector<Point> Points; // 点云数据
};

#pragma pack(pop)

inline bool pack(float x, float y, float z, float intensity, uint64_t& out) {
    int32_t xi = static_cast<int32_t>(std::round(x * 1000.0f));
    int32_t yi = static_cast<int32_t>(std::round(y * 1000.0f));
    int32_t zi = static_cast<int32_t>(std::round(z * 1000.0f));

    // z 超出 17 位范围（无符号） => 忽略此点
    if (std::abs(zi) >= (1 << 17)) return false;

    uint64_t sign_bits = 0;
    sign_bits |= (xi < 0 ? 1 : 0) << 2;
    sign_bits |= (yi < 0 ? 1 : 0) << 1;
    sign_bits |= (zi < 0 ? 1 : 0);

    uint64_t x_bits = static_cast<uint64_t>(std::abs(xi)) & ((1ULL << 18) - 1);
    uint64_t y_bits = static_cast<uint64_t>(std::abs(yi)) & ((1ULL << 18) - 1);
    uint64_t z_bits = static_cast<uint64_t>(std::abs(zi)) & ((1ULL << 17) - 1);
    uint64_t i_bits = static_cast<uint64_t>(std::min(std::max(int(intensity), 0), 255));

    // 位拼接：从高位到低位
    out = 0;
    out |= (i_bits & 0xFFULL) << 56;
    out |= (sign_bits & 0x7ULL) << 53;
    out |= (x_bits & 0x3FFFFULL) << 35;
    out |= (y_bits & 0x3FFFFULL) << 17;
    out |= (z_bits & 0x1FFFFULL);

    return true;
}

inline void unpack(uint64_t packed, float& x, float& y, float& z, float& intensity) {
    uint8_t i_bits = (packed >> 56) & 0xFF;
    uint8_t sign_bits = (packed >> 53) & 0x7;
    uint32_t x_bits = (packed >> 35) & 0x3FFFF;
    uint32_t y_bits = (packed >> 17) & 0x3FFFF;
    uint32_t z_bits = packed & 0x1FFFF;

    intensity = static_cast<float>(i_bits);
    x = ((sign_bits & 0b100) ? -1.0f : 1.0f) * x_bits / 1000.0f;
    y = ((sign_bits & 0b010) ? -1.0f : 1.0f) * y_bits / 1000.0f;
    z = ((sign_bits & 0b001) ? -1.0f : 1.0f) * z_bits / 1000.0f;
}

#endif // __DATA_STRUCT_HPP__