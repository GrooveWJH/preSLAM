#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "pose.hpp"

using namespace robotics;

/**
 * @brief 在时间序列中查找最接近目标时间的两个位姿索引
 * @param poses 时间戳位姿序列
 * @param target_time 目标时间
 * @return std::pair<size_t, size_t> 相邻的两个时间戳索引，第一个小于等于目标时间，第二个大于目标时间
 * @throw std::out_of_range 如果目标时间超出范围
 */
std::pair<size_t, size_t> findNeighborPoseIndices(const std::vector<TimedPose>& poses, double target_time)
{
    // 检查输入有效性
    if (poses.empty()) {
        throw std::invalid_argument("Pose sequence is empty");
    }

    // 检查是否在时间范围内
    if (target_time < poses.front().time_stamp || target_time > poses.back().time_stamp) {
        throw std::out_of_range("Target time is outside the range of pose timestamps");
    }

    // 如果刚好等于第一个时间戳，直接返回第一个位姿
    if (target_time == poses.front().time_stamp) {
        return { 0, 0 };
    }

    // 如果刚好等于最后一个时间戳，直接返回最后一个位姿
    if (target_time == poses.back().time_stamp) {
        return { poses.size() - 1, poses.size() - 1 };
    }

    // 遍历查找相邻时间戳
    for (size_t i = 0; i < poses.size() - 1; ++i) {
        if (poses[i].time_stamp <= target_time && target_time < poses[i + 1].time_stamp) {
            return { i, i + 1 };
        }
    }

    // 理论上不应该到达这里，因为我们已经处理了所有情况
    throw std::runtime_error("Failed to find neighbor poses");
}

/**
 * @brief 线性插值两个位姿
 * @param pose1 第一个位姿
 * @param pose2 第二个位姿
 * @param t 插值因子 (0.0-1.0)，0表示完全是pose1，1表示完全是pose2
 * @return Pose 插值后的位姿
 */
Pose interpolatePose(const Pose& pose1, const Pose& pose2, double t)
{
    // 确保插值因子在有效范围内
    t = std::max(0.0, std::min(1.0, t));

    // 位置进行线性插值
    Vector3 interp_position = pose1.position * (1.0 - t) + pose2.position * t;

    // 四元数进行球面线性插值 (SLERP)
    // 计算四元数的点积
    double dot = pose1.orientation.w * pose2.orientation.w + pose1.orientation.x * pose2.orientation.x + pose1.orientation.y * pose2.orientation.y + pose1.orientation.z * pose2.orientation.z;

    // 确保走最短路径
    Quaternion q2 = pose2.orientation;
    if (dot < 0.0) {
        q2.w = -q2.w;
        q2.x = -q2.x;
        q2.y = -q2.y;
        q2.z = -q2.z;
        dot = -dot;
    }

    // 对于非常接近的四元数，使用线性插值
    Quaternion interp_orientation;
    if (dot > 0.9995) {
        interp_orientation.w = pose1.orientation.w * (1.0 - t) + q2.w * t;
        interp_orientation.x = pose1.orientation.x * (1.0 - t) + q2.x * t;
        interp_orientation.y = pose1.orientation.y * (1.0 - t) + q2.y * t;
        interp_orientation.z = pose1.orientation.z * (1.0 - t) + q2.z * t;
        interp_orientation.normalize();
    } else {
        // 使用球面线性插值公式计算SLERP
        double angle = std::acos(dot);
        double sin_angle = std::sin(angle);
        double factor1 = std::sin((1.0 - t) * angle) / sin_angle;
        double factor2 = std::sin(t * angle) / sin_angle;

        interp_orientation.w = pose1.orientation.w * factor1 + q2.w * factor2;
        interp_orientation.x = pose1.orientation.x * factor1 + q2.x * factor2;
        interp_orientation.y = pose1.orientation.y * factor1 + q2.y * factor2;
        interp_orientation.z = pose1.orientation.z * factor1 + q2.z * factor2;
    }

    return { interp_position, interp_orientation };
}

/**
 * @brief 根据时间插值位姿
 * @param poses 按时间戳排序的位姿序列
 * @param target_time 目标时间
 * @return TimedPose 插值后的带时间戳位姿
 */
TimedPose interpolateTimedPose(const std::vector<TimedPose>& poses, double target_time)
{
    // 获取相邻的位姿索引
    auto [idx1, idx2] = findNeighborPoseIndices(poses, target_time);

    // 如果目标时间刚好等于某个时间戳，直接返回对应的位姿
    if (idx1 == idx2) {
        return poses[idx1];
    }

    // 计算插值因子
    double t1 = poses[idx1].time_stamp;
    double t2 = poses[idx2].time_stamp;
    double t = (target_time - t1) / (t2 - t1);

    // 插值位姿
    Pose interp_pose = interpolatePose(poses[idx1].pose, poses[idx2].pose, t);

    // 返回带有目标时间戳的插值位姿
    return { target_time, interp_pose };
}

int main()
{
    // 创建一个位姿序列
    std::vector<TimedPose> poses;

    poses.push_back({ 0.0,
        { Vector3 { 0.0, 0.0, 0.0 },
            Quaternion { 1.0, 0.0, 0.0, 0.0 } } });

    poses.push_back({ 1.0,
        { Vector3 { 1.0, 0.0, 0.0 },
            Quaternion { 0.7071, 0.0, 0.7071, 0.0 } } });

    poses.push_back({ 2.0,
        { Vector3 { 1.0, 1.0, 0.0 },
            Quaternion { 0.0, 0.0, 1.0, 0.0 } } });

    poses.push_back({ 3.0,
        { Vector3 { 0.0, 1.0, 0.0 },
            Quaternion { 0.0, 0.0, 0.7071, 0.7071 } } });

    poses.push_back({ 4.0,
        { Vector3 { 0.0, 0.0, 1.0 },
            Quaternion { 0.0, 0.0, 0.0, 1.0 } } });

    try {
        const std::string green_color = "\033[32m";
        const std::string reset_color = "\033[0m";

        std::vector<double> test_times = { 0.0, 0.5, 1.0, 1.75, 2.5, 3.5, 4.0 };

        for (double time : test_times) {
            TimedPose interp_pose = interpolateTimedPose(poses, time);

            // 检查当前时间是否是原始时间戳
            bool is_original_timestamp = false;
            for (const auto& p : poses) {
                if (std::fabs(p.time_stamp - time) < 1e-9) { // 使用浮点数比较
                    is_original_timestamp = true;
                    break;
                }
            }

            // 如果是插值出来的结果，则使用绿色
            if (!is_original_timestamp) {
                std::cout << green_color; 
            }

            std::cout << "Time: " << interp_pose.time_stamp << std::endl;
            std::cout << "Position: [" << interp_pose.pose.position.x << ", "
                      << interp_pose.pose.position.y << ", "
                      << interp_pose.pose.position.z << "]" << std::endl;
            std::cout << "Orientation: [" << interp_pose.pose.orientation.w << ", "
                      << interp_pose.pose.orientation.x << ", "
                      << interp_pose.pose.orientation.y << ", "
                      << interp_pose.pose.orientation.z << "]" << std::endl;
            
            // 如果设置了颜色，则重置
            if (!is_original_timestamp) {
                std::cout << reset_color;
            }
            std::cout << "----------------------------------------" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
