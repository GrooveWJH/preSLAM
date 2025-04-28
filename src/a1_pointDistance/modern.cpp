/**
 * @file modern.cpp
 * @brief 计算 N 维空间中两点间欧氏距离的现代 C++ 实现。
 */
#include <algorithm> // std::transform
#include <cmath>     // std::sqrt
#include <iostream>  // std::cout, std::cerr
#include <iterator>  // std::back_inserter
#include <numeric>   // std::accumulate
#include <stdexcept> // std::invalid_argument
#include <vector>    // std::vector

// 移除非必要的头文件 <__math/exponential_functions.h>
// #include <__math/exponential_functions.h>


/**
 * @brief 计算两个 N 维点之间的欧氏距离 (现代 C++ 风格)。
 *
 * 使用 STL 算法 (std::transform, std::accumulate) 来计算。
 * 首先计算每个维度差值的平方，然后累加求和，最后开方。
 *
 * @param p1 第一个点的坐标向量 (std::vector<double>)
 * @param p2 第二个点的坐标向量 (std::vector<double>)
 * @return double 两点之间的欧氏距离。
 * @throw std::invalid_argument 如果两个点的维度不相同。
 */
double distance_modern(const std::vector<double>& p1, const std::vector<double>& p2)
{
    if (p1.size() != p2.size()) {
        throw std::invalid_argument("Points must have the same dimension.");
    }

    if (p1.empty()) {
        return 0.0; // 空向量的点距离为0
    }

    // 预分配空间以提高效率，直接写入 diff_sq
    std::vector<double> diff_sq;
    diff_sq.reserve(p1.size()); 

    std::transform(
        p1.begin(),
        p1.end(),
        p2.begin(),
        std::back_inserter(diff_sq), // 使用 back_inserter 将结果添加到 diff_sq 末尾
        [](double val1, double val2) {
            double diff = val1 - val2;
            return diff * diff;
        });

    // 使用 std::accumulate 计算平方和
    double sum_of_squares = std::accumulate(diff_sq.begin(), diff_sq.end(), 0.0);

    return std::sqrt(sum_of_squares);
}

/**
 * @brief 主函数，演示 distance_modern 函数的用法。
 * @return int 程序退出代码 (0 表示成功)。
 */
int main()
{
    std::vector<double> p_a = { 1.0, 2.0 },
                        p_b = { 4.0, 6.0 };
    std::vector<double> p_c = { 1.0, 2.0, 3.0 },
                        p_d = { 4.0, 5.0, 6.0 };
    std::vector<double> p_e = { 1.0, 2.0, 3.0, 4.0 },
                        p_f = { 5.0, 6.0, 7.0, 8.0 };

    try {
        double dist2D = distance_modern(p_a, p_b);
        std::cout << "Modern Distance between p_a and p_b (2D): " << dist2D << std::endl; // 5
        double dist3D = distance_modern(p_c, p_d);
        std::cout << "Modern Distance between p_c and p_d (3D): " << dist3D << std::endl; // 5.19615
        double dist4D = distance_modern(p_e, p_f);
        std::cout << "Modern Distance between p_e and p_f (4D): " << dist4D << std::endl; // 8
        // 尝试不同维度的点会抛出异常
        distance_modern(p_a, p_c);
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}
