#include "mid-solvers.hpp"
#include <Eigen/IterativeLinearSolvers> // 包含迭代求解器
#include <Eigen/LU>       // 包含 LU 分解
#include <Eigen/Cholesky> // 包含 Cholesky 分解
#include <Eigen/QR>       // 包含 QR 分解
#include <Eigen/SVD>      // 包含 SVD 分解
#include <iostream> // 用于 std::cerr
#include <cmath>    // 用于 std::abs

// --- 直接法求解器实现 ---

/**
 * @brief 使用 LU 分解求解 (适用于一般方阵)
 */
SolveResult solveWithPartialPivLU(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    SolveResult result;
    result.method = "PartialPivLU";
    if (A.rows() != A.cols() || A.rows() != b.size()) {
        std::cerr << "Error: Matrix A must be square and dimensions must match b for LU.\n";
        return result; // success is false
    }
    Eigen::PartialPivLU<Eigen::MatrixXd> lu(A);
    // For Eigen 3.4, check validity after solve(), not before using info()
    result.solution = lu.solve(b);

    // 检查解是否包含 NaN 或 Inf
    if (!result.solution.array().isFinite().all()) {
        std::cerr << "Error: LU solve resulted in non-finite values (matrix might be singular).\n";
        result.success = false;
    } else {
        result.error = (A * result.solution - b).norm();
        result.success = true;
    }
    return result;
}

/**
 * @brief 使用 Cholesky 分解求解 (适用于对称正定矩阵)
 */
SolveResult solveWithLLT(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    SolveResult result;
    result.method = "Cholesky (LLT)";
    if (A.rows() != A.cols() || A.rows() != b.size()) {
        std::cerr << "Error: Matrix A must be square and dimensions must match b for Cholesky.\n";
        return result;
    }
    // 检查对称性 (近似检查)
    if (!A.isApprox(A.transpose())) {
         std::cerr << "Error: Matrix A is not symmetric, cannot use LLT.\n";
         return result;
    }
    Eigen::LLT<Eigen::MatrixXd> llt(A);
    if (llt.info() != Eigen::Success) {
        // LLT 分解失败通常意味着矩阵不是正定的
        std::cerr << "Error: LLT decomposition failed. Matrix might not be positive definite.\n";
        return result;
    }
    result.solution = llt.solve(b);
    result.error = (A * result.solution - b).norm();
    result.success = (llt.info() == Eigen::Success);
    return result;
}

/**
 * @brief 使用 QR 分解求解 (适用于任意矩阵，特别是最小二乘问题)
 */
SolveResult solveWithColPivHouseholderQr(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    SolveResult result;
    result.method = "Column Pivoting Householder QR";
     if (A.rows() != b.size()) {
        std::cerr << "Error: Number of rows of A must match size of b for QR solve.\n";
        return result;
    }
    // 对于非方阵，QR 分解通常用于求解最小二乘问题: min ||Ax - b||^2
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
     if (qr.info() != Eigen::Success) {
        std::cerr << "Error: QR decomposition failed.\n";
        return result;
    }
    result.solution = qr.solve(b);
    result.error = (A * result.solution - b).norm(); // 这是最小二乘解的残差范数
    result.success = true; // qr.solve() is generally robust
    return result;
}

/**
 * @brief 使用 SVD 分解求解 (适用于任意矩阵，非常鲁棒，也可用于最小二乘)
 */
SolveResult solveWithJacobiSVD(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    SolveResult result;
    result.method = "Jacobi SVD";
    if (A.rows() != b.size()) {
        std::cerr << "Error: Number of rows of A must match size of b for SVD solve.\n";
        return result;
    }
    // 使用 JacobiSVD，并请求计算 U 和 V (ComputeThinU/V 效率更高)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.info() != Eigen::Success) {
         std::cerr << "Error: SVD decomposition failed.\n";
         return result;
    }
    result.solution = svd.solve(b);
    result.error = (A * result.solution - b).norm();
    result.success = true; // svd.solve() is very robust
    return result;
}

// --- 迭代法求解器实现 ---

/**
 * @brief 使用共轭梯度法 (适用于对称正定矩阵)
 */
SolveResult solveWithConjugateGradient(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    SolveResult result;
    result.method = "Conjugate Gradient";
    if (A.rows() != A.cols() || A.rows() != b.size()) {
        std::cerr << "Error: Matrix A must be square and dimensions must match b for CG.\n";
        return result;
    }
     // 检查对称性 (近似检查)
    if (!A.isApprox(A.transpose())) {
         std::cerr << "Error: Matrix A is not symmetric, cannot use Conjugate Gradient.\n";
         return result;
    }
    // 还需要正定性，但 Eigen 的 CG 会在迭代中发现问题（如果非正定）

    Eigen::ConjugateGradient<Eigen::MatrixXd, Eigen::Lower | Eigen::Upper> cg;
    cg.compute(A);
    if (cg.info() != Eigen::Success) {
        std::cerr << "Error: CG compute failed (maybe matrix properties?).\n";
        return result;
    }
    result.solution = cg.solve(b);
    result.iterations = cg.iterations();
    result.error = cg.error(); // 这是估计的误差范数 ||Ax-b||/||b||
    result.success = (cg.info() == Eigen::Success);
    return result;
}

/**
 * @brief 使用 BiCGSTAB (适用于一般方阵)
 */
SolveResult solveWithBiCGSTAB(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    SolveResult result;
    result.method = "BiCGSTAB";
     if (A.rows() != A.cols() || A.rows() != b.size()) {
        std::cerr << "Error: Matrix A must be square and dimensions must match b for BiCGSTAB.\n";
        return result;
    }

    Eigen::BiCGSTAB<Eigen::MatrixXd> bicg;
    bicg.compute(A);
     if (bicg.info() != Eigen::Success) {
        std::cerr << "Error: BiCGSTAB compute failed.\n";
        return result;
    }
    result.solution = bicg.solve(b);
    result.iterations = bicg.iterations();
    result.error = bicg.error();
    result.success = (bicg.info() == Eigen::Success); // Check if solver converged
    return result;
}

/**
 * @brief 手动实现的 Jacobi 迭代法 (仅为演示，Eigen 无内置 Dense Jacobi 求解器)
 */
SolveResult solveWithManualJacobi(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                  int max_iterations, double tolerance) {
    SolveResult result;
    result.method = "Manual Jacobi Iteration";
     if (A.rows() != A.cols() || A.rows() != b.size()) {
        std::cerr << "Error: Matrix A must be square and dimensions must match b for Jacobi.\n";
        return result;
    }

    int n = A.rows();
    // 检查对角线元素是否为零 (Jacobi 要求 a_ii != 0)
    for (int i = 0; i < n; ++i) {
        if (std::abs(A(i, i)) < 1e-12) { // 用一个小的阈值检查
            std::cerr << "Warning: Diagonal element A(" << i << "," << i << ") is close to zero. Jacobi may fail or converge slowly.\n";
            // 不返回失败，但给出警告
        }
    }

    Eigen::VectorXd x = Eigen::VectorXd::Zero(n); // 初始猜测为 0
    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd D_inv = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd R = A; // R = L + U

    for(int i = 0; i < n; ++i) {
        if (std::abs(A(i, i)) > 1e-12) { // 避免除以零
           D_inv(i, i) = 1.0 / A(i, i);
        } else {
             // 如果对角元素接近零，Jacobi迭代可能无法进行或非常不稳定
             // 这里的处理方式可以是直接失败，或者用一个很大的数代替逆（模拟无穷大）
             // 或者保持 D_inv(i,i) 为 0，这会导致该行的 x_new(i) 始终为 0
             // 这里我们选择让 D_inv(i,i) 为 0 并继续，但这通常不是理想的
             std::cerr << "Warning: Diagonal element A(" << i << "," << i << ") is very close to zero, setting D_inv(i,i) to 0 for Jacobi iteration.\n";
             D_inv(i, i) = 0; 
        }
        R(i, i) = 0.0; // 从 A 中移除对角线元素得到 R = L+U
    }

    for (int iter = 0; iter < max_iterations; ++iter) {
        x_new = D_inv * (b - R * x);

        // 检查收敛性
        double current_error = (x_new - x).norm();
        if (current_error < tolerance) {
            result.solution = x_new;
            result.success = true;
            result.iterations = iter + 1;
            result.error = (A * result.solution - b).norm(); // 实际残差
            return result;
        }
        x = x_new; // 更新解
        result.iterations = iter + 1;
    }

    std::cerr << "Warning: Jacobi iteration did not converge within " << max_iterations << " iterations.\n";
    result.solution = x; // 返回最后一次迭代的结果
    result.error = (A * result.solution - b).norm();
    // success 保持 false
    return result;
} 