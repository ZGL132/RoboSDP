#pragma once

#include "core/logging/ILogger.h"
#include "modules/kinematics/adapter/IIkSolverAdapter.h"

#include <Eigen/Dense>

#include <memory>

namespace RoboSDP::Kinematics::Adapter
{

/**
 * @brief 基于 Pieper 准则的闭式解析 IK 求解器。
 * @details
 * 适用于标准 6R 工业机器人（末端三关节轴线交于一点，即 a4 = a5 = d5 = 0）。
 * 相比数值 IK：
 * 1. 计算速度在微秒级（比数值快 1000 倍以上）；
 * 2. 一次性求出全部 8 个空间构型（肩左/右 × 肘上/下 × 腕翻转/不翻转）；
 * 3. 返回所有有效解并过滤限位、按种子距离排序。
 *
 * 非标准构型（不满足 Pieper）由 Service 层自动回退到 Pinocchio 数值 IK。
 */
class AnalyticalIkSolverAdapter final : public IIkSolverAdapter
{
public:
    explicit AnalyticalIkSolverAdapter(RoboSDP::Logging::ILogger* logger = nullptr);
    ~AnalyticalIkSolverAdapter() override;

    QString SolverId() const override;
    QString SolverDescription() const override;

    /// @brief 执行闭式解析 IK 求解。
    RoboSDP::Kinematics::Dto::IkResultDto SolveIk(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::IkRequestDto& request) const override;

    /// @brief 检测模型是否满足 Pieper 准则（标准 6R + 球形腕）。
    static bool CheckPieperCriterion(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        double tolerance = 1.0e-6);

private:
    /// @brief 一组完整关节解的临时结构。
    struct JointSolution
    {
        std::vector<double> values_deg;
        double seed_distance = 0.0;
    };

    // ── DH/MDH 变换工具 ──────────────────────────────────
    /// @brief 标准 DH 连杆变换矩阵 T_i (Rz*Tz*Tx*Rx)。
    static Eigen::Matrix4d DhTransform(double a, double alphaDeg,
                                       double d, double thetaDeg);
    /// @brief 改进 DH (MDH) 连杆变换矩阵 T_i (Tx*Rx*Rz*Tz)。
    static Eigen::Matrix4d MdhTransform(double a, double alphaDeg,
                                        double d, double thetaDeg);

    /// @brief 对 6 个关节执行 FK，返回 flange 在基座标系下的 4x4 矩阵。
    static Eigen::Matrix4d ComputeFlangeTransform(
        const std::vector<double>& jointAnglesDeg,
        const std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto>& links,
        const QString& parameterConvention);

    // ── 目标位姿预处理 ──────────────────────────────────
    /// @brief 从 IK 请求目标 TCP 位姿反算目标法兰位姿（移除 base/tcp/flange 偏移）。
    Eigen::Matrix4d ComputeTargetFlangeTransform(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::IkRequestDto& request) const;

    // ── 臂部求解 (θ1, θ2, θ3) ────────────────────────────
    /// @brief 给定腕心位置，求解前三个关节角，返回最多 4 组解。
    static std::vector<std::array<double, 3>> SolveArm(
        const Eigen::Vector3d& wristCenter,
        const std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto>& links);

    // ── 腕部求解 (θ4, θ5, θ6) ────────────────────────────
    /// @brief 对每组臂解，从末端旋转矩阵提取腕关节角，返回最多 2 组腕解。
    static std::vector<JointSolution> SolveWrist(
        const std::array<double, 3>& armSolutionDeg,
        const Eigen::Matrix4d& targetFlangeTransform,
        const std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto>& links,
        const QString& parameterConvention);

    // ── 后处理 ───────────────────────────────────────────
    /// @brief 限位过滤 + 按种子距离升序排序。
    static std::vector<JointSolution> FilterAndSortSolutions(
        std::vector<JointSolution> allSolutions,
        const std::vector<RoboSDP::Kinematics::Dto::KinematicJointLimitDto>& jointLimits,
        const std::vector<double>& seedDeg);

    // ── 误差计算 ─────────────────────────────────────────
    /// @brief 计算给定关节角对应的 TCP 位置误差 [mm]。
    static double ComputePositionErrorM(
        const std::vector<double>& anglesDeg,
        const Eigen::Matrix4d& targetFlangeTransform,
        const std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto>& links,
        const QString& convention);
    /// @brief 计算给定关节角对应的 TCP 姿态误差 [rad]。
    static double ComputeOrientationErrorRad(
        const std::vector<double>& anglesDeg,
        const Eigen::Matrix4d& targetFlangeTransform,
        const std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto>& links,
        const QString& convention);

    // ── 通用工具 ─────────────────────────────────────────
    static double NormalizeAngleDeg(double angleDeg);
    static double ComputeSeedDistance(const std::vector<double>& candidate,
                                      const std::vector<double>& seed);
    static Eigen::Matrix3d ExtractRotation(const Eigen::Matrix4d& mat);
    static Eigen::Vector3d ExtractPosition(const Eigen::Matrix4d& mat);

    /// @brief CartesianPoseDto → 4x4 齐次矩阵。
    static Eigen::Matrix4d PoseToMatrix4d(
        const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose);
    /// @brief TcpFrameDto → 4x4 齐次矩阵（flange→TCP）。
    static Eigen::Matrix4d TcpFrameToMatrix4d(
        const RoboSDP::Kinematics::Dto::TcpFrameDto& tcp);

    void LogMessage(RoboSDP::Logging::LogLevel level,
                    const QString& actionName,
                    const QString& message) const;

private:
    RoboSDP::Logging::ILogger* m_logger = nullptr;
};

} // namespace RoboSDP::Kinematics::Adapter
