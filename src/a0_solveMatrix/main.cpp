/**
 * @file main.cpp
 * @brief 演示 Eigen 库中不同线性方程组求解器的用法。
 *
 * 该文件包含两个示例：
 * 1. 求解一个良态的对称正定方阵系统。
 * 2. 求解一个超定系统的最小二乘问题。
 *
 * 使用了来自 mid-solvers.hpp/cpp 中定义的求解函数，并打印结果。
 */

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "mid-solvers.cpp"
#include "mid-solvers.hpp"

/**
 * @brief 程序主入口点。
 *
 * 设置示例矩阵和向量，调用不同的求解器，并打印结果。
 * @return int 程序退出代码 (0 表示成功)。
 */
int main()
{
    // --- 示例 1: 一个良态的方阵系统 ---
    std::cout << "=== Example 1: Well-conditioned Square System ===" << std::endl;
    Eigen::MatrixXd A1(3, 3);
    Eigen::VectorXd b1(3);
    A1 << 4, 1, 1,
        1, 3, -1,
        1, -1, 2; // 对称正定矩阵
    b1 << 6, 3, 2;
    std::cout << "Matrix A1:\n"
              << A1 << std::endl;
    std::cout << "Vector b1:\n"
              << b1 << std::endl;

    std::vector<SolveResult> results1;
    results1.push_back(solveWithPartialPivLU(A1, b1));
    results1.push_back(solveWithLLT(A1, b1)); // A1 是对称正定的，适用
    results1.push_back(solveWithColPivHouseholderQr(A1, b1));
    results1.push_back(solveWithJacobiSVD(A1, b1));
    results1.push_back(solveWithConjugateGradient(A1, b1)); // A1 是对称正定的，适用
    results1.push_back(solveWithBiCGSTAB(A1, b1));
    results1.push_back(solveWithManualJacobi(A1, b1));

    for (const auto& res : results1) {
        std::cout << "\nMethod: " << res.method << std::endl;
        if (res.success) {
            std::cout << " Solution x:\n"
                      << res.solution << std::endl;
            if (res.iterations > 0)
                std::cout << " Iterations: " << res.iterations << std::endl;
            std::cout << " Residual Norm ||Ax-b||: " << res.error << std::endl;
        } else {
            std::cout << " Solver failed or did not converge." << std::endl;
            if (res.iterations > 0)
                std::cout << " Iterations performed: " << res.iterations << std::endl;
        }
    }

    // --- 示例 2: 最小二乘问题 (超定系统) ---
    std::cout << "\n=== Example 2: Least Squares (Overdetermined System) ===" << std::endl;
    Eigen::MatrixXd A2(4, 2); // 4 个方程, 2 个未知数
    Eigen::VectorXd b2(4);
    A2 << 1, 1,
        1, 2,
        1, 3,
        1, 4;
    b2 << 6, 5, 7, 10;
    std::cout << "Matrix A2:\n"
              << A2 << std::endl;
    std::cout << "Vector b2:\n"
              << b2 << std::endl;

    std::vector<SolveResult> results2;
    // LU 和 Cholesky 不直接适用于非方阵最小二乘
    // CG, BiCGSTAB, Jacobi 通常用于方阵
    results2.push_back(solveWithColPivHouseholderQr(A2, b2));
    results2.push_back(solveWithJacobiSVD(A2, b2));

    // 也可以用正规方程法 A^T A x = A^T b (但可能损失精度)
    Eigen::MatrixXd AtA = A2.transpose() * A2;
    Eigen::VectorXd Atb = A2.transpose() * b2;
    std::cout << "\nSolving Normal Equations A^T A x = A^T b:" << std::endl;
    std::cout << "A^T A:\n"
              << AtA << std::endl;
    std::cout << "A^T b:\n"
              << Atb << std::endl;
    results2.push_back(solveWithLLT(AtA, Atb)); // AtA 应该是对称正定的

    for (const auto& res : results2) {
        std::cout << "\nMethod: " << res.method << std::endl;
        if (res.success) {
            std::cout << " Solution x (Least Squares Sense):\n"
                      << res.solution << std::endl;
            std::cout << " Residual Norm ||Ax-b||: " << res.error << std::endl;
        } else {
            std::cout << " Solver failed." << std::endl;
        }
    }

    return 0;
}