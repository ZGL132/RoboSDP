#pragma once

#include <Eigen/Core>

#include <vector>

/// @brief 从 Jacobian 矩阵计算条件数和可操作度。
/// @param jacobian 6xN 空间雅可比矩阵
/// @param conditionNumber [out] 条件数 κ = σ_max / σ_min
/// @param manipulability [out] Yoshikawa 可操作度 w = σ₁·σ₂·…·σₖ
/// @param singularValues [out] 从大到小的奇异值列表
/// @return 是否奇异
bool ComputeSvdMetrics(
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian,
    double& conditionNumber,
    double& manipulability,
    std::vector<double>& singularValues);
