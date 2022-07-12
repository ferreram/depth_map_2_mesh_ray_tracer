#pragma once

#include <chrono>

#include "sophus/se3.hpp"

typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > VectorAlignSE3d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > VectorAlignV3d;

struct float3 { float x, y, z; };
struct float6 { float u1, v1, u2, v2, u3, v3; };
struct uint3 { uint32_t v1, v2, v3; };

class manual_timer
{
    std::chrono::high_resolution_clock::time_point t0;
    float timestamp{ 0.0 };
public:
    void start() { t0 = std::chrono::high_resolution_clock::now(); }
    void stop() { timestamp = std::chrono::duration<float,std::milli>(std::chrono::high_resolution_clock::now() - t0).count(); }
    const float & get() { return timestamp; }
};

