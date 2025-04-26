#pragma once
#include <cmath>

namespace robotics {

/**
 * @brief 表示三维空间中的点或向量
 */
struct Vector3 {
    double x { 0.0 };
    double y { 0.0 };
    double z { 0.0 };

    Vector3() = default;
    Vector3(double x, double y, double z)
        : x(x)
        , y(y)
        , z(z)
    {
    }

    // 向量加法
    Vector3 operator+(const Vector3& other) const
    {
        return { x + other.x, y + other.y, z + other.z };
    }

    // 向量减法
    Vector3 operator-(const Vector3& other) const
    {
        return { x - other.x, y - other.y, z - other.z };
    }

    // 标量乘法
    Vector3 operator*(double scalar) const
    {
        return { x * scalar, y * scalar, z * scalar };
    }
};

/**
 * @brief 表示三维旋转的四元数
 * 使用w, x, y, z表示，其中w是实部，x, y, z是虚部
 */
struct Quaternion {
    double w { 1.0 }; // 实部，默认为单位四元数
    double x { 0.0 }; // 虚部i
    double y { 0.0 }; // 虚部j
    double z { 0.0 }; // 虚部k

    Quaternion() = default;
    Quaternion(double w, double x, double y, double z)
        : w(w)
        , x(x)
        , y(y)
        , z(z)
    {
    }

    // 归一化四元数
    void normalize()
    {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm > 1e-10) {
            w /= norm;
            x /= norm;
            y /= norm;
            z /= norm;
        } else {
            // 如果四元数接近零，设置为单位四元数
            w = 1.0;
            x = y = z = 0.0;
        }
    }

    // 四元数乘法
    Quaternion operator*(const Quaternion& q) const
    {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    // 标量乘法
    Quaternion operator*(double scalar) const
    {
        return { w * scalar, x * scalar, y * scalar, z * scalar };
    }

    // 四元数加法
    Quaternion operator+(const Quaternion& q) const
    {
        return { w + q.w, x + q.x, y + q.y, z + q.z };
    }
};

/**
 * @brief 表示6自由度位姿
 * 包含位置(position)和方向(orientation)
 */
struct Pose {
    Vector3 position; // 位置
    Quaternion orientation; // 方向（使用四元数表示）

    Pose() = default;
    Pose(const Vector3& pos, const Quaternion& orient)
        : position(pos)
        , orientation(orient)
    {
    }
};

/**
 * @brief 带时间戳的位姿
 */
struct TimedPose {
    double time_stamp { 0.0 }; // 时间戳（单位：秒）
    Pose pose; // 位姿

    TimedPose() = default;
    TimedPose(double time, const Pose& p)
        : time_stamp(time)
        , pose(p)
    {
    }
};

} // namespace robotics