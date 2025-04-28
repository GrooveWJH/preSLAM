#pragma once

#include <Eigen/Dense>
#include <string>

/**
 * @brief 存储线性方程组求解结果的结构体
 *
 * 包含解向量、求解状态、迭代次数（如果适用）、误差（如果适用）以及使用的求解方法名称。
 */
struct SolveResult {
    /** @brief 方程组的解向量 */
    Eigen::VectorXd solution;
    /** @brief 指示求解是否成功 */
    bool success = false;
    /** @brief 迭代法求解时使用的迭代次数 */
    int iterations = 0; // 用于迭代法
    /** @brief 迭代法的最终误差或直接法的残差 */
    double error = 0.0; // 用于迭代法/直接法残差
    /** @brief 使用的求解方法名称 */
    std::string method = "Unknown";
};

// --- 函数声明 ---

//直接法
/**
 * @brief 使用部分主元 LU 分解求解线性方程组 Ax = b
 * @param A 系数矩阵
 * @param b 常数向量
 * @return SolveResult 包含求解结果的结构体
 */
SolveResult solveWithPartialPivLU(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

/**
 * @brief 使用 LLT (Cholesky) 分解求解线性方程组 Ax = b (要求 A 为正定矩阵)
 * @param A 系数矩阵 (必须是正定矩阵)
 * @param b 常数向量
 * @return SolveResult 包含求解结果的结构体
 */
SolveResult solveWithLLT(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

/**
 * @brief 使用带列主元的 Householder QR 分解求解线性方程组 Ax = b
 * @param A 系数矩阵
 * @param b 常数向量
 * @return SolveResult 包含求解结果的结构体
 */
SolveResult solveWithColPivHouseholderQr(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

/**
 * @brief 使用 Jacobi SVD 分解求解线性方程组 Ax = b (适用于非方阵或病态矩阵)
 * @param A 系数矩阵
 * @param b 常数向量
 * @return SolveResult 包含求解结果的结构体
 */
SolveResult solveWithJacobiSVD(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

// 迭代法
/**
 * @brief 使用共轭梯度法求解线性方程组 Ax = b (要求 A 为正定矩阵)
 * @param A 系数矩阵 (必须是正定矩阵)
 * @param b 常数向量
 * @return SolveResult 包含求解结果的结构体 (包含迭代次数和误差)
 */
SolveResult solveWithConjugateGradient(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

/**
 * @brief 使用 BiCGSTAB (双共轭梯度稳定) 方法求解线性方程组 Ax = b
 * @param A 系数矩阵
 * @param b 常数向量
 * @return SolveResult 包含求解结果的结构体 (包含迭代次数和误差)
 */
SolveResult solveWithBiCGSTAB(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

/**
 * @brief 使用手动实现的 Jacobi 迭代法求解线性方程组 Ax = b
 * @param A 系数矩阵
 * @param b 常数向量
 * @param max_iterations 最大迭代次数
 * @param tolerance 收敛容差
 * @return SolveResult 包含求解结果的结构体 (包含迭代次数和误差)
 */
SolveResult solveWithManualJacobi(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, int max_iterations = 1000, double tolerance = 1e-6); 