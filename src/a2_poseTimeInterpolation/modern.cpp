#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

#include "pose.hpp"

using namespace robotics;

/**
 * @brief 使用二分查找找到最接近目标时间的两个位姿
 * @param poses 时间戳位姿序列
 * @param target_time 目标时间
 * @return std::optional<std::pair<size_t, size_t>> 相邻的两个时间戳索引，如果找不到返回std::nullopt
 */
std::optional<std::pair<size_t, size_t>> findNeighborPoseIndicesModern(
    const std::vector<TimedPose>& poses, double target_time) {
    
    // 使用异常替代错误码，提前检查异常情况
    if (poses.empty()) {
        throw std::invalid_argument("Pose sequence is empty");
    }
    
    // 检查目标时间是否在时间范围内
    if (target_time < poses.front().time_stamp || target_time > poses.back().time_stamp) {
        throw std::out_of_range("Target time is outside the range of pose timestamps");
    }
    
    // 使用STL算法，代替手动实现的循环
    // std::lower_bound 查找第一个大于等于目标值的元素
    auto comp = [](const TimedPose& pose, double time) { return pose.time_stamp < time; };
    auto it = std::lower_bound(poses.begin(), poses.end(), target_time, comp);
    
    // 如果找到的是第一个元素，且时间戳等于目标时间
    if (it == poses.begin() && it->time_stamp == target_time) {
        size_t idx = std::distance(poses.begin(), it);
        return std::make_pair(idx, idx);
    }
    
    // 如果找到的是最后一个元素，且时间戳等于目标时间
    if (it == poses.end() - 1 && it->time_stamp == target_time) {
        size_t idx = std::distance(poses.begin(), it);
        return std::make_pair(idx, idx);
    }
    
    // 如果找到的元素时间戳正好等于目标时间
    if (it != poses.end() && it->time_stamp == target_time) {
        size_t idx = std::distance(poses.begin(), it);
        return std::make_pair(idx, idx);
    }
    
    // 否则，找到相邻的两个时间戳
    if (it != poses.begin()) {
        auto prev_it = std::prev(it);
        size_t idx1 = std::distance(poses.begin(), prev_it);
        size_t idx2 = std::distance(poses.begin(), it);
        return std::make_pair(idx1, idx2);
    }
    
    // 理论上不应该到达这里
    return std::nullopt;
}

/**
 * @brief 线性插值两个位姿（使用C++的函数式编程思想）
 * @param pose1 第一个位姿
 * @param pose2 第二个位姿
 * @param t 插值因子 (0.0-1.0)
 * @return Pose 插值后的位姿
 */
Pose interpolatePoseModern(const Pose& pose1, const Pose& pose2, double t) {
    // 使用模板化的lambda和std::clamp来限制t的范围
    auto clamp = [](double val, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, val));
    };
    
    // 确保插值因子在有效范围内
    double t_clamped = clamp(t, 0.0, 1.0);
    
    // 线性插值函数模板
    auto lerp = [](const auto& a, const auto& b, double factor) {
        return a * (1.0 - factor) + b * factor;
    };
    
    // 位置进行线性插值
    Vector3 interp_position{
        lerp(pose1.position.x, pose2.position.x, t_clamped),
        lerp(pose1.position.y, pose2.position.y, t_clamped),
        lerp(pose1.position.z, pose2.position.z, t_clamped)
    };
    
    // 四元数插值 - SLERP实现
    // 点积计算
    auto dot_product = [](const Quaternion& q1, const Quaternion& q2) {
        return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    };
    
    double dot = dot_product(pose1.orientation, pose2.orientation);
    
    // 四元数插值函数 - 处理最短路径
    auto slerp = [&dot_product](Quaternion q1, Quaternion q2, double factor) {
        double dot = dot_product(q1, q2);
        
        // 确保走最短路径
        if (dot < 0.0) {
            q2.w = -q2.w;
            q2.x = -q2.x;
            q2.y = -q2.y;
            q2.z = -q2.z;
            dot = -dot;
        }
        
        // 对于非常接近的四元数，使用线性插值
        if (dot > 0.9995) {
            Quaternion result{
                q1.w * (1.0 - factor) + q2.w * factor,
                q1.x * (1.0 - factor) + q2.x * factor,
                q1.y * (1.0 - factor) + q2.y * factor,
                q1.z * (1.0 - factor) + q2.z * factor
            };
            
            // 归一化
            double norm = std::sqrt(result.w * result.w + result.x * result.x + 
                                   result.y * result.y + result.z * result.z);
            if (norm > 1e-10) {
                result.w /= norm;
                result.x /= norm;
                result.y /= norm;
                result.z /= norm;
            }
            
            return result;
        }
        
        // SLERP公式
        double angle = std::acos(dot);
        double sin_angle = std::sin(angle);
        double factor1 = std::sin((1.0 - factor) * angle) / sin_angle;
        double factor2 = std::sin(factor * angle) / sin_angle;
        
        return Quaternion{
            q1.w * factor1 + q2.w * factor2,
            q1.x * factor1 + q2.x * factor2,
            q1.y * factor1 + q2.y * factor2,
            q1.z * factor1 + q2.z * factor2
        };
    };
    
    // 进行四元数球面线性插值
    Quaternion interp_orientation = slerp(pose1.orientation, pose2.orientation, t_clamped);
    
    return {interp_position, interp_orientation};
}

/**
 * @brief 根据时间插值位姿（使用现代C++特性）
 * @param poses 按时间戳排序的位姿序列
 * @param target_time 目标时间
 * @return TimedPose 插值后的带时间戳位姿
 */
TimedPose interpolateTimedPoseModern(const std::vector<TimedPose>& poses, double target_time) {
    // 使用可选类型处理结果
    auto indices_opt = findNeighborPoseIndicesModern(poses, target_time);
    
    if (!indices_opt.has_value()) {
        throw std::runtime_error("Failed to find neighbor poses");
    }
    
    auto [idx1, idx2] = indices_opt.value();
    
    // 如果目标时间刚好等于某个时间戳，直接返回对应的位姿
    if (idx1 == idx2) {
        return poses[idx1];
    }
    
    // 计算插值因子
    double t1 = poses[idx1].time_stamp;
    double t2 = poses[idx2].time_stamp;
    double t = (target_time - t1) / (t2 - t1);
    
    // 插值位姿
    Pose interp_pose = interpolatePoseModern(poses[idx1].pose, poses[idx2].pose, t);
    
    // 使用初始化列表构造结果
    return {target_time, interp_pose};
}

int main() {
    // 创建一个位姿序列，使用初始化列表
    std::vector<TimedPose> poses = {
        {0.0, {Vector3{0.0, 0.0, 0.0}, Quaternion{1.0, 0.0, 0.0, 0.0}}},
        {1.0, {Vector3{1.0, 0.0, 0.0}, Quaternion{0.7071, 0.0, 0.7071, 0.0}}},
        {2.0, {Vector3{1.0, 1.0, 0.0}, Quaternion{0.0, 0.0, 1.0, 0.0}}},
        {3.0, {Vector3{0.0, 1.0, 0.0}, Quaternion{0.0, 0.0, 0.7071, 0.7071}}},
        {4.0, {Vector3{0.0, 0.0, 1.0}, Quaternion{0.0, 0.0, 0.0, 1.0}}}
    };
    
    try {
        const std::string green_color = "\033[32m";
        const std::string reset_color = "\033[0m";

        std::vector<double> test_times = { 0.0, 0.5, 1.0, 1.75, 2.5, 3.5, 4.0 };

        for (const auto& time : test_times) {
            TimedPose interp_pose = interpolateTimedPoseModern(poses, time);

            // 检查当前时间是否是原始时间戳 (使用 modern C++ 的 std::any_of)
            bool is_original_timestamp = std::any_of(poses.begin(), poses.end(), 
                [time](const TimedPose& p){ 
                    return std::fabs(p.time_stamp - time) < 1e-9; // 浮点数比较
                });

            auto& [timestamp, pose] = interp_pose;
            auto& [position, orientation] = pose;

            // 如果是插值出来的结果，则使用绿色
            if (!is_original_timestamp) {
                std::cout << green_color;
            }

            std::cout << "Time: " << timestamp << std::endl;
            std::cout << "Position: [" << position.x << ", " << position.y << ", " << position.z << "]" << std::endl;
            std::cout << "Orientation: [" << orientation.w << ", " << orientation.x << ", "
                      << orientation.y << ", " << orientation.z << "]" << std::endl;
            
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