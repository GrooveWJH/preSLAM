#pragma once

#include <Eigen/Dense>
#include <string>

// 定义一个简单的结构体来存储结果和状态
struct SolveResult {
    Eigen::VectorXd solution;
    bool success = false;
    int iterations = 0; // 用于迭代法
    double error = 0.0; // 用于迭代法/直接法残差
    std::string method = "Unknown";
};

// --- 函数声明 ---

//直接法
SolveResult solveWithPartialPivLU(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
SolveResult solveWithLLT(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
SolveResult solveWithColPivHouseholderQr(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
SolveResult solveWithJacobiSVD(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

// 迭代法
SolveResult solveWithConjugateGradient(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
SolveResult solveWithBiCGSTAB(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
SolveResult solveWithManualJacobi(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, int max_iterations = 1000, double tolerance = 1e-6); 