#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator> // 添加 iterator 头文件
#include <list> // 添加 list 头文件
#include <map> // 添加 map 头文件
#include <stdexcept>
#include <vector> // 保留 vector 用于 main 函数示例
#include "pose.hpp" // 假设 pose.hpp 在同一或可包含路径下

using namespace robotics;

// Helper function to get TimedPose from iterator (works for vector/list iterator and map iterator)
template <typename Iterator>
const TimedPose& getTimedPose(Iterator it)
{
    if constexpr (std::is_same_v<typename std::iterator_traits<Iterator>::value_type, TimedPose>) {
        // For vector/list iterators
        return *it;
    } else {
        // For map iterators (value_type is std::pair<const double, TimedPose>)
        return it->second;
    }
}

// Helper function to get timestamp from iterator
template <typename Iterator>
double getTimestamp(Iterator it)
{
    if constexpr (std::is_same_v<typename std::iterator_traits<Iterator>::value_type, TimedPose>) {
        // For vector/list iterators
        return it->time_stamp;
    } else {
        // For map iterators (value_type is std::pair<const double, TimedPose>)
        return it->first; // Map key is the timestamp
    }
}

/**
 * @brief 在时间序列中查找最接近目标时间的两个位姿迭代器
 * @tparam InputIt 输入迭代器类型
 * @param first 指向序列起始的迭代器
 * @param last 指向序列结束的迭代器
 * @param target_time 目标时间
 * @return std::pair<InputIt, InputIt> 相邻的两个时间戳的迭代器，第一个小于等于目标时间，第二个大于目标时间
 * @throw std::out_of_range 如果目标时间超出范围
 * @throw std::invalid_argument 如果序列为空
 */
template <typename InputIt>
std::pair<InputIt, InputIt> findNeighborPoseIterators(InputIt first, InputIt last, double target_time)
{
    if (first == last) {
        throw std::invalid_argument("Pose sequence is empty");
    }

    // 检查是否在时间范围内
    InputIt begin_it = first;
    InputIt end_it = std::prev(last); // 指向最后一个元素

    if (target_time < getTimestamp(begin_it) || target_time > getTimestamp(end_it)) {
        throw std::out_of_range("Target time is outside the range of pose timestamps");
    }

    // 如果刚好等于第一个时间戳
    if (target_time == getTimestamp(begin_it)) {
        return { begin_it, begin_it };
    }

    // 如果刚好等于最后一个时间戳
    if (target_time == getTimestamp(end_it)) {
        return { end_it, end_it };
    }

    // 遍历查找相邻时间戳
    InputIt prev_it = first;
    InputIt curr_it = std::next(first);
    while (curr_it != last) {
        if (getTimestamp(prev_it) <= target_time && target_time < getTimestamp(curr_it)) {
            return { prev_it, curr_it };
        }
        prev_it = curr_it;
        ++curr_it;
    }

    // 理论上不应该到达这里，因为我们已经处理了所有情况
    throw std::runtime_error("Failed to find neighbor poses");
}

/**
 * @brief 线性插值两个位姿 (与 a2 相同)
 * @param pose1 第一个位姿
 * @param pose2 第二个位姿
 * @param t 插值因子 (0.0-1.0)
 * @return Pose 插值后的位姿
 */
Pose interpolatePose(const Pose& pose1, const Pose& pose2, double t)
{
    t = std::max(0.0, std::min(1.0, t));
    Vector3 interp_position = pose1.position * (1.0 - t) + pose2.position * t;

    double dot = pose1.orientation.w * pose2.orientation.w + pose1.orientation.x * pose2.orientation.x + pose1.orientation.y * pose2.orientation.y + pose1.orientation.z * pose2.orientation.z;
    Quaternion q2 = pose2.orientation;
    if (dot < 0.0) {
        q2.w = -q2.w;
        q2.x = -q2.x;
        q2.y = -q2.y;
        q2.z = -q2.z;
        dot = -dot;
    }

    Quaternion interp_orientation;
    if (dot > 0.9995) {
        interp_orientation.w = pose1.orientation.w * (1.0 - t) + q2.w * t;
        interp_orientation.x = pose1.orientation.x * (1.0 - t) + q2.x * t;
        interp_orientation.y = pose1.orientation.y * (1.0 - t) + q2.y * t;
        interp_orientation.z = pose1.orientation.z * (1.0 - t) + q2.z * t;
        interp_orientation.normalize();
    } else {
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
 * @brief 根据时间插值位姿 (模板化版本)
 * @tparam Container 容器类型 (例如 std::vector<TimedPose>, std::list<TimedPose>, std::map<double, TimedPose>)
 * @param poses 按时间戳排序的位姿容器
 * @param target_time 目标时间
 * @return TimedPose 插值后的带时间戳位姿
 */
template <typename Container>
TimedPose interpolateTimedPose(const Container& poses, double target_time)
{
    auto it_pair = findNeighborPoseIterators(poses.begin(), poses.end(), target_time);
    auto it1 = it_pair.first;
    auto it2 = it_pair.second;

    // 如果目标时间刚好等于某个时间戳
    if (it1 == it2) {
        return getTimedPose(it1); // 直接返回对应的位姿
    }

    // 获取相邻位姿和时间戳
    const TimedPose& p1 = getTimedPose(it1);
    const TimedPose& p2 = getTimedPose(it2);
    double t1 = getTimestamp(it1);
    double t2 = getTimestamp(it2);

    // 计算插值因子
    double t = (target_time - t1) / (t2 - t1);

    // 插值位姿
    Pose interp_pose = interpolatePose(p1.pose, p2.pose, t);

    // 返回带有目标时间戳的插值位姿
    return { target_time, interp_pose };
}


// Helper function to print results
void printInterpolatedPose(const TimedPose& interp_pose, double time, bool is_interpolated)
{
    const std::string green_color = "\033[32m";
    const std::string reset_color = "\033[0m";

    if (is_interpolated) {
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

    if (is_interpolated) {
        std::cout << reset_color;
    }
    std::cout << "----------------------------------------" << std::endl;
}

int main()
{
    // 创建位姿数据 (用于所有容器)
    std::vector<TimedPose> pose_data = {
        { 0.0, { Vector3 { 0.0, 0.0, 0.0 }, Quaternion { 1.0, 0.0, 0.0, 0.0 } } },
        { 1.0, { Vector3 { 1.0, 0.0, 0.0 }, Quaternion { 0.7071, 0.0, 0.7071, 0.0 } } },
        { 2.0, { Vector3 { 1.0, 1.0, 0.0 }, Quaternion { 0.0, 0.0, 1.0, 0.0 } } },
        { 3.0, { Vector3 { 0.0, 1.0, 0.0 }, Quaternion { 0.0, 0.0, 0.7071, 0.7071 } } },
        { 4.0, { Vector3 { 0.0, 0.0, 1.0 }, Quaternion { 0.0, 0.0, 0.0, 1.0 } } }
    };

    // 测试时间点
    std::vector<double> test_times = { 0.0, 0.5, 1.0, 1.75, 2.5, 3.5, 4.0 };

    // --- 测试 std::vector ---
    std::cout << "========= Testing with std::vector =========" << std::endl;
    std::vector<TimedPose> poses_vec = pose_data;
    try {
        for (double time : test_times) {
            TimedPose interp_pose = interpolateTimedPose(poses_vec, time);
            bool is_interpolated = true;
             for (const auto& p : poses_vec) {
                 if (std::fabs(p.time_stamp - time) < 1e-9) {
                     is_interpolated = false;
                     break;
                 }
             }
            printInterpolatedPose(interp_pose, time, is_interpolated);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error (vector): " << e.what() << std::endl;
    }
    std::cout << std::endl;


    // --- 测试 std::list ---
    std::cout << "========= Testing with std::list =========" << std::endl;
    std::list<TimedPose> poses_list(pose_data.begin(), pose_data.end());
     try {
        for (double time : test_times) {
            TimedPose interp_pose = interpolateTimedPose(poses_list, time);
             bool is_interpolated = true;
             for (const auto& p : poses_list) {
                 if (std::fabs(p.time_stamp - time) < 1e-9) {
                     is_interpolated = false;
                     break;
                 }
             }
            printInterpolatedPose(interp_pose, time, is_interpolated);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error (list): " << e.what() << std::endl;
    }
    std::cout << std::endl;


    // --- 测试 std::map ---
    std::cout << "========= Testing with std::map =========" << std::endl;
    std::map<double, TimedPose> poses_map;
    for(const auto& p : pose_data) {
        poses_map[p.time_stamp] = p; // 使用时间戳作为键
    }
     try {
        for (double time : test_times) {
            TimedPose interp_pose = interpolateTimedPose(poses_map, time);
             bool is_interpolated = true;
             // 对于 map，可以直接使用 find
             if (poses_map.count(time)) {
                  is_interpolated = false;
             }
            printInterpolatedPose(interp_pose, time, is_interpolated);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error (map): " << e.what() << std::endl;
    }

    return 0;
}
