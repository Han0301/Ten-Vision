#include "ctrl_common/LM.h"

// 优化主函数
Params LMOptimizer::optimize(const Params& initialParams, int maxIter, 
                double eps, double lambdaInit) {
    Params params = initialParams;
    double lambda = lambdaInit;  // 阻尼因子
    double prevError = computeTotalError(params);  // 初始误差

    for (int iter = 0; iter < maxIter; ++iter) {
        // 1. 计算残差和雅可比矩阵
        Eigen::VectorXd residuals = computeResiduals(params);
        Eigen::MatrixXd J = computeJacobian(params);

        // 2. 构建LM方程：(J^T J + lambda*diag(J^T J)) * dx = J^T r
        Eigen::MatrixXd Jt = J.transpose();
        Eigen::MatrixXd H = Jt * J;  // 近似Hessian矩阵
        Eigen::VectorXd g = Jt * residuals;  // 梯度

        // 修正：正确构建阻尼项（使用对角矩阵与原矩阵相加）
        Eigen::MatrixXd H_LM = H + lambda * Eigen::MatrixXd::Identity(H.rows(), H.cols()) 
                                * H.diagonal().asDiagonal();

        // 3. 求解增量方程
        Eigen::VectorXd delta = H_LM.ldlt().solve(-g);  // 数值稳定的Cholesky分解

        // 4. 尝试更新参数并计算新误差
        Params newParams = params;
        newParams.fromVector(params.toVector() + delta);
        double newError = computeTotalError(newParams);

        // 5. 根据误差变化调整阻尼因子和参数
        if (newError < prevError) {
            // 接受更新：减小阻尼因子，更新参数和误差
            lambda *= 0.1;
            params = newParams;
            prevError = newError;

            // 检查收敛条件
            if (delta.norm() < eps) {
                std::cout << "Converged at iter " << iter << ", error: " << prevError << std::endl;
                break;
            }
        } else {
            // 拒绝更新：增大阻尼因子
            lambda *= 10;
            if (lambda > 1e10) {  // 避免阻尼过大导致停滞
                std::cout << "Lambda too large, stop at iter " << iter << std::endl;
                break;
            }
        }
    }

    return params;
}
