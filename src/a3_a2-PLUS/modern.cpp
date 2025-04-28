#include <algorithm>
#include <cmath>
#include <concepts> // 需要 C++20
#include <iostream>
#include <iterator> // 添加 iterator 头文件
#include <list> // 添加 list 头文件
#include <map> // 添加 map 头文件
#include <optional>
#include <stdexcept>
#include <type_traits> // 用于 iterator_traits 和 is_same_v
#include <vector> // 保留 vector 用于 main 函数示例

#include "pose.hpp" // 假设 pose.hpp 在同一或可包含路径下

// Helper type trait for static_assert (定义在文件顶部或 common header 中)
namespace detail {
    template <typename T>
    struct dependent_false : std::false_type {};
}

using namespace robotics;

// Helper concepts and functions (与 traditional.cpp 类似，但用于 Modern 版本)
template <typename T>
concept TimedPoseContainer =
    requires(T c) {
        typename T::value_type;
        c.begin();
        c.end();
    } &&
    (std::is_same_v<typename T::value_type, TimedPose> ||
     std::is_same_v<typename T::value_type, std::pair<const double, TimedPose>>);

template <typename Iterator>
const TimedPose& getTimedPoseModern(Iterator it)
{
    using ValueType = typename std::iterator_traits<Iterator>::value_type;
    if constexpr (std::is_same_v<ValueType, TimedPose>) {
        return *it;
    } else if constexpr (std::is_same_v<ValueType, std::pair<const double, TimedPose>>) {
        return it->second;
    } else {
        // 编译时错误，如果类型不支持
        static_assert(detail::dependent_false<Iterator>::value, "Unsupported iterator value type");
    }
}

template <typename Iterator>
double getTimestampModern(Iterator it)
{
    using ValueType = typename std::iterator_traits<Iterator>::value_type;
    if constexpr (std::is_same_v<ValueType, TimedPose>) {
        return it->time_stamp;
    } else if constexpr (std::is_same_v<ValueType, std::pair<const double, TimedPose>>) {
        return it->first; // Map key is the timestamp
    } else {
        static_assert(detail::dependent_false<Iterator>::value, "Unsupported iterator value type");
    }
}

/**
 * @brief 使用二分查找找到最接近目标时间的两个位姿迭代器 (模板化 Modern 版本)
 * @tparam Container 满足 TimedPoseContainer 概念的容器类型
 * @param poses 时间戳位姿容器
 * @param target_time 目标时间
 * @return std::optional<std::pair<typename Container::const_iterator, typename Container::const_iterator>> 相邻的两个时间戳迭代器，如果找不到或出错返回std::nullopt
 * @throw std::invalid_argument 如果容器为空
 * @throw std::out_of_range 如果目标时间超出范围
 */
template <TimedPoseContainer Container>
std::optional<std::pair<typename Container::const_iterator, typename Container::const_iterator>>
findNeighborPoseIteratorsModern(const Container& poses, double target_time)
{
    using Iterator = typename Container::const_iterator;

    if (poses.empty()) {
        throw std::invalid_argument("Pose sequence is empty");
    }

    Iterator begin_it = poses.begin();
    Iterator end_it = std::prev(poses.end()); // 指向最后一个元素

    // 检查目标时间是否在时间范围内
    if (target_time < getTimestampModern(begin_it) || target_time > getTimestampModern(end_it)) {
        throw std::out_of_range("Target time is outside the range of pose timestamps");
    }

    // 自定义比较函数，用于 lower_bound
    auto comp = [](const auto& element, double time) {
        // 根据元素类型获取时间戳
        if constexpr (std::is_same_v<std::decay_t<decltype(element)>, TimedPose>) {
            return element.time_stamp < time;
        } else { // 假定是 std::pair<const double, TimedPose>
            return element.first < time;
        }
    };

    // 使用 std::lower_bound 查找第一个时间戳不小于 target_time 的元素
    Iterator it = std::lower_bound(poses.begin(), poses.end(), target_time, comp);

    // 检查找到的迭代器情况
    double found_time = getTimestampModern(it);

    // 情况 1: 目标时间正好等于找到的时间戳
    if (found_time == target_time) {
        return std::make_pair(it, it);
    }

    // 情况 2: target_time 介于 prev_it 和 it 之间
    if (it != poses.begin()) {
        Iterator prev_it = std::prev(it);
        // 再次确认 prev_it <= target_time < it
        if (getTimestampModern(prev_it) <= target_time && target_time < found_time) {
            return std::make_pair(prev_it, it);
        }
    }

    // 理论上，由于范围检查，应该总能找到合适的区间
    // 但为了代码健壮性，可以返回 nullopt 或抛出异常
    // throw std::runtime_error("Logical error in findNeighborPoseIteratorsModern");
    return std::nullopt; // 或者返回空 optional 表示未找到（理论上不应发生）
}

/**
 * @brief 线性插值两个位姿 (与 a2 相同)
 * @param pose1 第一个位姿
 * @param pose2 第二个位姿
 * @param t 插值因子 (0.0-1.0)
 * @return Pose 插值后的位姿
 */
Pose interpolatePoseModern(const Pose& pose1, const Pose& pose2, double t) {
    auto clamp = [](double val, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, val));
    };
    double t_clamped = clamp(t, 0.0, 1.0);

    auto lerp = [](const auto& a, const auto& b, double factor) {
        return a * (1.0 - factor) + b * factor;
    };

    Vector3 interp_position{
        lerp(pose1.position.x, pose2.position.x, t_clamped),
        lerp(pose1.position.y, pose2.position.y, t_clamped),
        lerp(pose1.position.z, pose2.position.z, t_clamped)
    };

    auto dot_product = [](const Quaternion& q1, const Quaternion& q2) {
        return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    };

    auto slerp = [&dot_product, &lerp](Quaternion q1, Quaternion q2, double factor) {
        double dot = dot_product(q1, q2);
        if (dot < 0.0) {
            q2 = {-q2.w, -q2.x, -q2.y, -q2.z};
            dot = -dot;
        }
        if (dot > 0.9995) {
            Quaternion result = {
                lerp(q1.w, q2.w, factor),
                lerp(q1.x, q2.x, factor),
                lerp(q1.y, q2.y, factor),
                lerp(q1.z, q2.z, factor)
            };
            result.normalize();
            return result;
        }
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

    Quaternion interp_orientation = slerp(pose1.orientation, pose2.orientation, t_clamped);

    return {interp_position, interp_orientation};
}


/**
 * @brief 根据时间插值位姿 (模板化 Modern 版本)
 * @tparam Container 满足 TimedPoseContainer 概念的容器类型
 * @param poses 按时间戳排序的位姿容器
 * @param target_time 目标时间
 * @return TimedPose 插值后的带时间戳位姿
 * @throw std::runtime_error 如果无法找到相邻位姿
 */
template <TimedPoseContainer Container>
TimedPose interpolateTimedPoseModern(const Container& poses, double target_time)
{
    auto indices_opt = findNeighborPoseIteratorsModern(poses, target_time);

    if (!indices_opt.has_value()) {
        // findNeighborPoseIteratorsModern 内部已做范围检查，理论上不会到这里
        // 但如果 findNeighborPoseIteratorsModern 返回 nullopt，则抛出异常
        throw std::runtime_error("Failed to find neighbor poses (Modern)");
    }

    auto [it1, it2] = indices_opt.value();

    // 如果目标时间刚好等于某个时间戳
    if (it1 == it2) {
        return getTimedPoseModern(it1);
    }

    // 获取相邻位姿和时间戳
    const TimedPose& p1 = getTimedPoseModern(it1);
    const TimedPose& p2 = getTimedPoseModern(it2);
    double t1 = getTimestampModern(it1);
    double t2 = getTimestampModern(it2);

    // 计算插值因子
    // 添加分母检查避免除零
    if (std::fabs(t2 - t1) < 1e-9) {
         // 时间戳相同但迭代器不同，理论上不应发生在此逻辑点
         // 但为健壮性，返回第一个位姿
        return p1; 
    }
    double t = (target_time - t1) / (t2 - t1);

    // 插值位姿
    Pose interp_pose = interpolatePoseModern(p1.pose, p2.pose, t);

    // 返回带有目标时间戳的插值位姿
    return { target_time, interp_pose };
}

// Helper function to print results (与 traditional.cpp 相同)
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

// Helper function to check if time is original
template <TimedPoseContainer Container>
bool isOriginalTimestamp(const Container& poses, double time) {
    // 对于 map, 使用 count 效率更高
    if constexpr (std::is_same_v<Container, std::map<double, TimedPose>>) {
        return poses.count(time) > 0;
    } else {
        // 对于 vector/list, 使用 any_of
        return std::any_of(poses.begin(), poses.end(),
            [time](const auto& element){
                if constexpr (std::is_same_v<std::decay_t<decltype(element)>, TimedPose>) {
                    return std::fabs(element.time_stamp - time) < 1e-9;
                } else { // 假定是 pair
                    return std::fabs(element.first - time) < 1e-9;
                }
            });
    }
}


int main() {
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
    std::cout << "========= Testing with std::vector (Modern) =========" << std::endl;
    std::vector<TimedPose> poses_vec = pose_data;
    try {
        for (double time : test_times) {
            TimedPose interp_pose = interpolateTimedPoseModern(poses_vec, time);
            bool is_interpolated = !isOriginalTimestamp(poses_vec, time);
            printInterpolatedPose(interp_pose, time, is_interpolated);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error (vector Modern): " << e.what() << std::endl;
    }
    std::cout << std::endl;

    // --- 测试 std::list ---
    std::cout << "========= Testing with std::list (Modern) =========" << std::endl;
    std::list<TimedPose> poses_list(pose_data.begin(), pose_data.end());
     try {
        for (double time : test_times) {
            TimedPose interp_pose = interpolateTimedPoseModern(poses_list, time);
            bool is_interpolated = !isOriginalTimestamp(poses_list, time);
            printInterpolatedPose(interp_pose, time, is_interpolated);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error (list Modern): " << e.what() << std::endl;
    }
    std::cout << std::endl;

    // --- 测试 std::map ---
    std::cout << "========= Testing with std::map (Modern) =========" << std::endl;
    std::map<double, TimedPose> poses_map;
    for(const auto& p : pose_data) {
        poses_map[p.time_stamp] = p; // 使用时间戳作为键
    }
     try {
        for (double time : test_times) {
            TimedPose interp_pose = interpolateTimedPoseModern(poses_map, time);
             bool is_interpolated = !isOriginalTimestamp(poses_map, time);
            printInterpolatedPose(interp_pose, time, is_interpolated);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error (map Modern): " << e.what() << std::endl;
    }

    return 0;
}
