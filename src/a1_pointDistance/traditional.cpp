/**
 * @file traditional.cpp
 * @brief 计算 N 维空间中两点间欧氏距离的传统 C++ 实现。
 */
#include <cmath>
#include <stdexcept>
#include <vector>
#include <iostream> // main 函数需要

/**
 * @brief 计算两个 N 维点之间的欧氏距离 (传统 C++ 风格)。
 *
 * 使用循环遍历坐标并累加差值的平方和，最后开方。
 * @param p1 第一个点的坐标向量 (std::vector<double>)
 * @param p2 第二个点的坐标向量 (std::vector<double>)
 * @return double 两点之间的欧氏距离。
 * @throw std::invalid_argument 如果两个点的维度不相同。
 */
double distance_traditional(const std::vector<double>& p1, const std::vector<double>& p2)
{
    if (p1.size() != p2.size()) {
        throw std::invalid_argument("Points must have the same dimension.");
    }
    size_t dimension = p1.size();
    double sum_sq_diff = 0.0;
    for (size_t i = 0; i < dimension; ++i) {
        double diff = p1[i] - p2[i];
        sum_sq_diff += diff * diff;
    }
    return std::sqrt(sum_sq_diff);
}

/**
 * @brief 主函数，演示 distance_traditional 函数的用法。
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
        double dist2D = distance_traditional(p_a, p_b);
        std::cout << "Distance between p_a and p_b (2D): " << dist2D << std::endl;
        double dist3D = distance_traditional(p_c, p_d);
        std::cout << "Distance between p_c and p_d (3D): " << dist3D << std::endl;
        double dist4D = distance_traditional(p_e, p_f);
        std::cout << "Distance between p_e and p_f (4D): " << dist4D << std::endl;
        distance_traditional(p_a, p_c);
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}
