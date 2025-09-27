
#pragma once
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

// 测量数据结构：IMU yaw角(度)、雷达x坐标、雷达y坐标
struct Measurement {
    float yaw_deg;   // IMU测量yaw角(度)
    float x;         // 雷达测量x坐标(m)
    float y;         // 雷达测量y坐标(m)
};

// 待优化参数：[x_c, y_c, b_y, dx, dy]
struct Params {
    double xc;  // 篮筐中心x
    double yc;  // 篮筐中心y
    double by;  // IMU yaw零偏(弧度)
    double dx;  // 雷达x偏移
    double dy;  // 雷达y偏移

    // 参数向量转换（方便矩阵运算）
    Eigen::VectorXd toVector() const {
        Eigen::VectorXd vec(5);
        vec << xc, yc, by, dx, dy;
        return vec;
    }

    // 从向量更新参数
    void fromVector(const Eigen::VectorXd& vec) {
        xc = vec[0];
        yc = vec[1];
        by = vec[2];
        dx = vec[3];
        dy = vec[4];
    }
};

// LM算法优化器
class LMOptimizer {
public:
    LMOptimizer(const std::vector<Measurement>& measurements) 
        : measurements_(measurements) {}

    // 优化主函数
    Params optimize(const Params& initialParams, int maxIter = 100, 
                   double eps = 1e-8, double lambdaInit = 1e-3);

private:
    std::vector<Measurement> measurements_;

    // 计算单个测量的残差
    double computeResidual(const Params& params, const Measurement& meas) {
        double yaw_rad = meas.yaw_deg * M_PI / 180.0;  // 测量yaw转弧度
        double yaw_true = yaw_rad - params.by;         // 修正零偏后的真实yaw
        double cos_yaw = std::cos(yaw_true);
        double sin_yaw = std::sin(yaw_true);

        // 直线方程参数（修正雷达偏移后）
        double A = -sin_yaw;
        double B = cos_yaw;
        double C = A * (meas.x - params.dx) + B * (meas.y - params.dy);

        // 残差：A*xc + B*yc - C
        return A * params.xc + B * params.yc - C;
    }

    // 计算所有测量的残差向量
    Eigen::VectorXd computeResiduals(const Params& params) {
        int n = measurements_.size();
        Eigen::VectorXd residuals(n);
        for (int i = 0; i < n; ++i) {
            residuals[i] = computeResidual(params, measurements_[i]);
        }
        return residuals;
    }

    // 计算残差平方和（目标函数）
    double computeTotalError(const Params& params) {
        Eigen::VectorXd residuals = computeResiduals(params);
        return residuals.squaredNorm();  // 残差平方和
    }

    // 计算雅可比矩阵（n行×5列，n为测量数）
    Eigen::MatrixXd computeJacobian(const Params& params) {
        int n = measurements_.size();
        Eigen::MatrixXd J(n, 5);  // 每行对应一个测量的残差对5个参数的偏导

        for (int i = 0; i < n; ++i) {
            const auto& meas = measurements_[i];
            double yaw_rad = meas.yaw_deg * M_PI / 180.0;
            double yaw_true = yaw_rad - params.by;
            double cos_yaw = std::cos(yaw_true);
            double sin_yaw = std::sin(yaw_true);

            // 直线方程参数
            double A = -sin_yaw;
            double B = cos_yaw;
            double C = A * (meas.x - params.dx) + B * (meas.y - params.dy);

            // 残差对各参数的偏导数
            double dr_dxc = A;  // 残差对xc的偏导
            double dr_dyc = B;  // 残差对yc的偏导

            // 残差对by的偏导（需用链式法则）
            double dA_dby = cos_yaw;
            double dB_dby = sin_yaw;
            double dC_dby = dA_dby * (meas.x - params.dx) + dB_dby * (meas.y - params.dy);
            double dr_dby = dA_dby * params.xc + dB_dby * params.yc - dC_dby;

            // 残差对dx和dy的偏导
            double dr_ddx = A;
            double dr_ddy = B;

            // 填充雅可比矩阵行
            J.row(i) << dr_dxc, dr_dyc, dr_dby, dr_ddx, dr_ddy;
        }

        return J;
    }
};
