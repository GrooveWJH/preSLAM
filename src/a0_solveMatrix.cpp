#include <iostream>
#include <vector>
#include <Eigen/Dense> // 包含稠密矩阵和向量功能
#include <Eigen/IterativeLinearSolvers> // 包含迭代求解器
#include <Eigen/LU>       // 包含 LU 分解
#include <Eigen/Cholesky> // 包含 Cholesky 分解
#include <Eigen/QR>       // 包含 QR 分解
#include <Eigen/SVD>      // 包含 SVD 分解

// 定义一个简单的结构体来存储结果和状态
struct SolveResult {
    Eigen::VectorXd solution;
    bool success = false;
    int iterations = 0; // 用于迭代法
    double error = 0.0; // 用于迭代法
    std::string method = "Unknown";
};

// --- 直接法求解器 ---

// 使用 LU 分解求解 (适用于一般方阵)
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

// 使用 Cholesky 分解求解 (适用于对称正定矩阵)
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

// 使用 QR 分解求解 (适用于任意矩阵，特别是最小二乘问题)
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

// 使用 SVD 分解求解 (适用于任意矩阵，非常鲁棒，也可用于最小二乘)
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


// --- 迭代法求解器 ---

// 使用共轭梯度法 (适用于对称正定矩阵)
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

// 使用 BiCGSTAB (适用于一般方阵)
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

// 手动实现 Jacobi 迭代法 (仅为演示，Eigen 无内置 Dense Jacobi 求解器)
SolveResult solveWithManualJacobi(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                  int max_iterations = 1000, double tolerance = 1e-6) {
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
            std::cerr << "Error: Diagonal element A(" << i << "," << i << ") is close to zero. Jacobi may fail.\n";
            // return result; // 可以选择返回失败，或继续尝试
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
             D_inv(i, i) = 0; // 或者抛出错误，或者用伪逆等策略？这里设为0
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


int main() {
    // --- 示例 1: 一个良态的方阵系统 ---
    std::cout << "=== Example 1: Well-conditioned Square System ===" << std::endl;
    Eigen::MatrixXd A1(3, 3);
    Eigen::VectorXd b1(3);
    A1 << 4, 1, 1,
          1, 3, -1,
          1, -1, 2; // 对称正定矩阵
    b1 << 6, 3, 2;
    std::cout << "Matrix A1:\n" << A1 << std::endl;
    std::cout << "Vector b1:\n" << b1 << std::endl;

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
            std::cout << " Solution x:\n" << res.solution << std::endl;
            if (res.iterations > 0) std::cout << " Iterations: " << res.iterations << std::endl;
            std::cout << " Residual Norm ||Ax-b||: " << res.error << std::endl;
        } else {
            std::cout << " Solver failed or did not converge." << std::endl;
            if (res.iterations > 0) std::cout << " Iterations performed: " << res.iterations << std::endl;
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
    b2 << 6, 5, 7, 10; // 真实解可能是 y = 1.5x + 3.5 ? -> x=[3.5, 1.5]
                      // 1*3.5 + 1*1.5 = 5
                      // 1*3.5 + 2*1.5 = 6.5
                      // 1*3.5 + 3*1.5 = 8
                      // 1*3.5 + 4*1.5 = 9.5
                      // b 向量与真实解有偏差
    std::cout << "Matrix A2:\n" << A2 << std::endl;
    std::cout << "Vector b2:\n" << b2 << std::endl;

    std::vector<SolveResult> results2;
    // LU 和 Cholesky 不直接适用于非方阵最小二乘
    // CG, BiCGSTAB, Jacobi 通常用于方阵
    results2.push_back(solveWithColPivHouseholderQr(A2, b2));
    results2.push_back(solveWithJacobiSVD(A2, b2));

    // 也可以用正规方程法 A^T A x = A^T b (但可能损失精度)
    Eigen::MatrixXd AtA = A2.transpose() * A2;
    Eigen::VectorXd Atb = A2.transpose() * b2;
    std::cout << "\nSolving Normal Equations A^T A x = A^T b:" << std::endl;
    std::cout << "A^T A:\n" << AtA << std::endl;
    std::cout << "A^T b:\n" << Atb << std::endl;
    results2.push_back(solveWithLLT(AtA, Atb)); // AtA 应该是对称正定的

    for (const auto& res : results2) {
        std::cout << "\nMethod: " << res.method << std::endl;
        if (res.success) {
            std::cout << " Solution x (Least Squares Sense):\n" << res.solution << std::endl;
            std::cout << " Residual Norm ||Ax-b||: " << res.error << std::endl;
        } else {
            std::cout << " Solver failed." << std::endl;
        }
    }

    return 0;
}