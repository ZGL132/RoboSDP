#include "modules/kinematics/adapter/AnalyticalIkSolverAdapter.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace RoboSDP::Kinematics::Adapter
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kSingularTolerance = 1.0e-10;

double DegToRad(double deg) { return deg * kDegToRad; }
double RadToDeg(double rad) { return rad * kRadToDeg; }

double NormalizeAngleDegAnon(double a)
{
    double n = std::fmod(a + 180.0, 360.0);
    if (n < 0.0) n += 360.0;
    return n - 180.0;
}

double SafeAcos(double v)
{
    return std::acos(std::max(-1.0, std::min(1.0, v)));
}

/// @brief RPY (ZYX extrinsic) → 旋转矩阵。与 FK 中 BuildPoseFromSe3 的 RPY 约定一致。
Eigen::Matrix3d RpyDegToMatrix(double rollDeg, double pitchDeg, double yawDeg)
{
    const double r = DegToRad(rollDeg);
    const double p = DegToRad(pitchDeg);
    const double y = DegToRad(yawDeg);
    const Eigen::AngleAxisd rz(y, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd ry(p, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rx(r, Eigen::Vector3d::UnitX());
    return (rz * ry * rx).toRotationMatrix();
}

/// @brief RPY 差分 → 轴角姿态误差向量。
Eigen::Vector3d ComputeOrientationErrorRad(
    const std::array<double, 3>& targetRpyDeg,
    const std::array<double, 3>& currentRpyDeg)
{
    const Eigen::Matrix3d R_target = RpyDegToMatrix(targetRpyDeg[0], targetRpyDeg[1], targetRpyDeg[2]);
    const Eigen::Matrix3d R_current = RpyDegToMatrix(currentRpyDeg[0], currentRpyDeg[1], currentRpyDeg[2]);
    const Eigen::Matrix3d R_err = R_target * R_current.transpose();
    const Eigen::AngleAxisd aa(R_err);
    return aa.angle() * aa.axis();
}

} // namespace

// =========================================================================
// 构造 / 析构
// =========================================================================
AnalyticalIkSolverAdapter::AnalyticalIkSolverAdapter(RoboSDP::Logging::ILogger* logger)
    : m_logger(logger)
{
}

AnalyticalIkSolverAdapter::~AnalyticalIkSolverAdapter() = default;

QString AnalyticalIkSolverAdapter::SolverId() const
{
    return QStringLiteral("analytical_closed_form_ik");
}

QString AnalyticalIkSolverAdapter::SolverDescription() const
{
    return QStringLiteral("闭式解析 IK 求解器（Pieper 准则，适用于标准 6R 球形腕机器人）。");
}

// =========================================================================
// Pieper 条件检测
// =========================================================================
bool AnalyticalIkSolverAdapter::CheckPieperCriterion(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    double tolerance)
{
    // 1. 必须为 6 轴
    if (model.links.size() != 6 || model.joint_count != 6)
        return false;

    // 2. 关节限位数量也必须匹配
    if (model.joint_limits.size() != 6)
        return false;

    // 3. 球形腕条件：a4 ≈ 0 且 a5 ≈ 0（最后三关节轴线交于一点）
    if (std::abs(model.links[3].a) > tolerance ||
        std::abs(model.links[4].a) > tolerance)
        return false;

    return true;
}

// =========================================================================
// DH / MDH 变换
// =========================================================================
Eigen::Matrix4d AnalyticalIkSolverAdapter::DhTransform(
    double a, double alphaDeg, double d, double thetaDeg)
{
    const double ct = std::cos(DegToRad(thetaDeg));
    const double st = std::sin(DegToRad(thetaDeg));
    const double ca = std::cos(DegToRad(alphaDeg));
    const double sa = std::sin(DegToRad(alphaDeg));

    // 标准 DH: RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = ct;         T(0, 1) = -st * ca;   T(0, 2) =  st * sa;   T(0, 3) = a * ct;
    T(1, 0) = st;         T(1, 1) =  ct * ca;   T(1, 2) = -ct * sa;   T(1, 3) = a * st;
    T(2, 1) = sa;         T(2, 2) =  ca;         T(2, 3) = d;
    // Row 3: 0, 0, 0, 1  (identity)
    return T;
}

Eigen::Matrix4d AnalyticalIkSolverAdapter::MdhTransform(
    double a, double alphaDeg, double d, double thetaDeg)
{
    const double ct = std::cos(DegToRad(thetaDeg));
    const double st = std::sin(DegToRad(thetaDeg));
    const double ca = std::cos(DegToRad(alphaDeg));
    const double sa = std::sin(DegToRad(alphaDeg));

    // 改进 DH: TransX(a) * RotX(alpha) * RotZ(theta) * TransZ(d)
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = ct;          T(0, 1) = -st;         T(0, 2) = 0.0;    T(0, 3) = a;
    T(1, 0) = st * ca;     T(1, 1) = ct * ca;     T(1, 2) = -sa;    T(1, 3) = -d * sa;
    T(2, 0) = st * sa;     T(2, 1) = ct * sa;     T(2, 2) = ca;     T(2, 3) = d * ca;
    return T;
}

Eigen::Matrix4d AnalyticalIkSolverAdapter::ComputeFlangeTransform(
    const std::vector<double>& jointAnglesDeg,
    const std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto>& links,
    const QString& parameterConvention)
{
    const bool isDh = (parameterConvention == QStringLiteral("DH"));
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (std::size_t i = 0; i < links.size() && i < jointAnglesDeg.size(); ++i)
    {
        const double theta = jointAnglesDeg[i] + links[i].theta_offset;
        if (isDh)
        {
            T = T * DhTransform(links[i].a, links[i].alpha, links[i].d, theta);
        }
        else
        {
            T = T * MdhTransform(links[i].a, links[i].alpha, links[i].d, theta);
        }
    }
    return T;
}

// =========================================================================
// 目标法兰位姿计算
// =========================================================================
Eigen::Matrix4d AnalyticalIkSolverAdapter::ComputeTargetFlangeTransform(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::IkRequestDto& request) const
{
    // 目标 TCP 位姿 → 4x4
    const Eigen::Matrix4d T_target = PoseToMatrix4d(request.target_pose);

    // 基坐标系偏移
    const Eigen::Matrix4d T_base = PoseToMatrix4d(model.base_frame);

    // TCP offset (flange → TCP)
    const Eigen::Matrix4d T_tcp = TcpFrameToMatrix4d(model.tcp_frame);

    // Flange offset (DH6 → 法兰中心)
    const Eigen::Matrix4d T_flange = PoseToMatrix4d(model.flange_frame);

    // T_base_to_flange_target = inv(T_base) * T_target * inv(T_tcp) * inv(T_flange)
    // 即 DH chain 从 0 到 6 需要达到的目标
    const Eigen::Matrix4d T_base_inv = T_base.inverse();
    const Eigen::Matrix4d T_tcp_inv = T_tcp.inverse();
    const Eigen::Matrix4d T_flange_inv = T_flange.inverse();

    return T_base_inv * T_target * T_tcp_inv * T_flange_inv;
}

// =========================================================================
// 臂部求解 (θ1, θ2, θ3)
// =========================================================================
std::vector<std::array<double, 3>> AnalyticalIkSolverAdapter::SolveArm(
    const Eigen::Vector3d& wristCenter,
    const std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto>& links)
{
    // 使用 standard DH 参数
    // d1 = links[0].d  (base to shoulder offset)
    // a2 = links[1].a  (upper arm length)
    // a3 = links[2].a  (forearm length)
    // d4 = links[3].d  (wrist offset along Z3)

    const double d1 = links[0].d;
    const double a2 = links[1].a;
    const double a3 = links[2].a;
    const double d4 = links[3].d;

    const double Px = wristCenter.x();
    const double Py = wristCenter.y();
    const double Pz = wristCenter.z();

    std::vector<std::array<double, 3>> solutions;

    // ── θ1: 肩部 (shoulder left/right) ──
    const double r_xy = std::sqrt(Px * Px + Py * Py);
    if (r_xy < std::abs(d4) - kSingularTolerance)
        return solutions; // 超出肩部可达范围

    const double phi = std::atan2(Py, Px);
    const double r_safe = std::max(r_xy, kSingularTolerance);
    double delta = std::asin(d4 / r_safe);
    // 防止数值越界导致的 NaN
    if (!std::isfinite(delta))
        delta = 0.0;

    const double th1_candidates[2] = {
        NormalizeAngleDeg(RadToDeg(phi + delta)),
        NormalizeAngleDeg(RadToDeg(phi + kPi - delta))
    };

    for (int i = 0; i < 2; ++i)
    {
        const double th1_deg = th1_candidates[i];
        const double c1 = std::cos(DegToRad(th1_deg));
        const double s1 = std::sin(DegToRad(th1_deg));

        // r_proj = Px * cos(θ1) + Py * sin(θ1) - d4 * (sin² + cos²) ...
        // Formula: r_proj = (Px - d4*sin(th1)) / cos(th1) or alternate
        double r_proj;
        if (std::abs(c1) > kSingularTolerance)
        {
            r_proj = (Px - d4 * s1) / c1;
        }
        else if (std::abs(s1) > kSingularTolerance)
        {
            r_proj = (Py + d4 * c1) / s1;
        }
        else
        {
            continue; // 奇异
        }

        // B = Pz - d1
        const double B = Pz - d1;

        // ── θ3: 肘部 (elbow up/down) ──
        const double a2sq = a2 * a2;
        const double a3sq = a3 * a3;
        const double cos_th3 = (r_proj * r_proj + B * B - a2sq - a3sq) / (2.0 * a2 * a3 + kSingularTolerance);

        if (cos_th3 < -1.0 - kSingularTolerance || cos_th3 > 1.0 + kSingularTolerance)
            continue; // 肘部不可达

        const double th3_rad = SafeAcos(cos_th3);
        const double th3_candidates[2] = {
            NormalizeAngleDeg(RadToDeg(th3_rad)),    // elbow up
            NormalizeAngleDeg(RadToDeg(-th3_rad))     // elbow down
        };

        for (int j = 0; j < 2; ++j)
        {
            const double th3_deg = th3_candidates[j];
            const double c3 = std::cos(DegToRad(th3_deg));
            const double s3 = std::sin(DegToRad(th3_deg));

            // ── θ2 ──
            const double C = a2 + a3 * c3;
            const double D = a3 * s3;
            const double denom = C * C + D * D;
            if (denom < kSingularTolerance)
                continue;

            const double c2 = (C * r_proj + D * B) / denom;
            const double s2 = (C * B - D * r_proj) / denom;
            const double th2_deg = NormalizeAngleDeg(RadToDeg(std::atan2(s2, c2)));

            solutions.push_back({th1_deg, th2_deg, th3_deg});
        }
    }

    return solutions;
}

// =========================================================================
// 腕部求解 (θ4, θ5, θ6)
// =========================================================================
std::vector<AnalyticalIkSolverAdapter::JointSolution> AnalyticalIkSolverAdapter::SolveWrist(
    const std::array<double, 3>& armSolutionDeg,
    const Eigen::Matrix4d& targetFlangeTransform,
    const std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto>& links,
    const QString& parameterConvention)
{
    // 计算前 3 个关节的 FK → T_03
    const std::vector<double> armAngles = {armSolutionDeg[0], armSolutionDeg[1], armSolutionDeg[2]};
    std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto> first3Links(
        links.begin(), links.begin() + 3);
    const Eigen::Matrix4d T_03 = ComputeFlangeTransform(armAngles, first3Links, parameterConvention);
    const Eigen::Matrix3d R_03 = ExtractRotation(T_03);

    // 目标方向矩阵 R_06
    const Eigen::Matrix3d R_06 = ExtractRotation(targetFlangeTransform);

    // R_3_to_6 = R_03^T * R_06
    const Eigen::Matrix3d R_36 = R_03.transpose() * R_06;

    // 使用 alpha4, alpha5, alpha6 参数
    const double a4 = DegToRad(links[3].alpha);
    const double a5 = DegToRad(links[4].alpha);
    // alpha6 通常为 0

    // 从 R_36 提取腕关节角
    // R_36 = RotZ(th4) * RotX(a4) * RotZ(th5) * RotX(a5) * RotZ(th6)
    // 当 a4 = pi/2, a5 = -pi/2 (标准球形腕):
    // R_36(0,2) = -cos(th4)*sin(th5)
    // R_36(1,2) = -sin(th4)*sin(th5)
    // R_36(2,2) = cos(th5)
    // R_36(2,0) = sin(th5)*cos(th6)
    // R_36(2,1) = -sin(th5)*sin(th6)

    const double r13 = R_36(0, 2);   // -c4*s5
    const double r23 = R_36(1, 2);   // -s4*s5
    const double r33 = R_36(2, 2);   // c5
    const double r31 = R_36(2, 0);   // s5*c6
    const double r32 = R_36(2, 1);   // -s5*s6

    const double s5_sq = r13 * r13 + r23 * r23;

    std::vector<JointSolution> solutions;

    if (s5_sq > kSingularTolerance)
    {
        // 正常情况 (非腕部奇异)
        const double s5 = std::sqrt(s5_sq);

        // 两个腕解: th5 取正或负
        for (int sign : {+1, -1})
        {
            const double th5_rad = std::atan2(sign * s5, r33);
            const double th4_rad = std::atan2(-sign * r23, -sign * r13);
            const double th6_rad = std::atan2(-sign * r32, sign * r31);

            JointSolution sol;
            sol.values_deg = {
                armSolutionDeg[0], armSolutionDeg[1], armSolutionDeg[2],
                NormalizeAngleDeg(RadToDeg(th4_rad)),
                NormalizeAngleDeg(RadToDeg(th5_rad)),
                NormalizeAngleDeg(RadToDeg(th6_rad))
            };
            solutions.push_back(sol);
        }
    }
    else
    {
        // 腕部奇异 (s5 ≈ 0): th4 和 th6 无法单独确定，只有 th4+th6 可解
        // 设置 th4 = 0，解出 th4+th6
        const double th5_rad = std::atan2(0.0, r33); // 0 或 pi
        const double th4_plus_th6 = std::atan2(-R_36(0, 1), R_36(0, 0));

        JointSolution sol;
        sol.values_deg = {
            armSolutionDeg[0], armSolutionDeg[1], armSolutionDeg[2],
            0.0,
            NormalizeAngleDeg(RadToDeg(th5_rad)),
            NormalizeAngleDeg(RadToDeg(th4_plus_th6))
        };
        solutions.push_back(sol);
    }

    return solutions;
}

// =========================================================================
// 后处理：限位过滤 + 排序
// =========================================================================
std::vector<AnalyticalIkSolverAdapter::JointSolution>
AnalyticalIkSolverAdapter::FilterAndSortSolutions(
    std::vector<JointSolution> allSolutions,
    const std::vector<RoboSDP::Kinematics::Dto::KinematicJointLimitDto>& jointLimits,
    const std::vector<double>& seedDeg)
{
    std::vector<JointSolution> valid;

    for (auto& sol : allSolutions)
    {
        // 归一化各关节角
        for (auto& v : sol.values_deg)
            v = NormalizeAngleDeg(v);

        // 限位检查
        bool withinLimits = true;
        for (std::size_t j = 0; j < sol.values_deg.size() && j < jointLimits.size(); ++j)
        {
            const double val = sol.values_deg[j];
            if (val < jointLimits[j].soft_limit[0] - 1.0 ||  // 1° 松弛容差
                val > jointLimits[j].soft_limit[1] + 1.0)
            {
                withinLimits = false;
                break;
            }
        }
        if (!withinLimits)
            continue;

        // 计算种子距离
        sol.seed_distance = ComputeSeedDistance(sol.values_deg, seedDeg);
        valid.push_back(std::move(sol));
    }

    // 按种子距离升序排序
    std::sort(valid.begin(), valid.end(),
        [](const JointSolution& a, const JointSolution& b) {
            return a.seed_distance < b.seed_distance;
        });

    return valid;
}

// =========================================================================
// 误差计算
// =========================================================================
double AnalyticalIkSolverAdapter::ComputePositionErrorM(
    const std::vector<double>& anglesDeg,
    const Eigen::Matrix4d& targetFlangeTransform,
    const std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto>& links,
    const QString& convention)
{
    const Eigen::Matrix4d T_fk = ComputeFlangeTransform(anglesDeg, links, convention);
    const Eigen::Vector3d pos_fk = ExtractPosition(T_fk);
    const Eigen::Vector3d pos_target = ExtractPosition(targetFlangeTransform);
    return (pos_fk - pos_target).norm();
}

double AnalyticalIkSolverAdapter::ComputeOrientationErrorRad(
    const std::vector<double>& anglesDeg,
    const Eigen::Matrix4d& targetFlangeTransform,
    const std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto>& links,
    const QString& convention)
{
    const Eigen::Matrix4d T_fk = ComputeFlangeTransform(anglesDeg, links, convention);
    const Eigen::Matrix3d R_fk = ExtractRotation(T_fk);
    const Eigen::Matrix3d R_target = ExtractRotation(targetFlangeTransform);
    const Eigen::Matrix3d R_err = R_target * R_fk.transpose();
    const Eigen::AngleAxisd aa(R_err);
    return aa.angle();
}

// =========================================================================
// 通用工具
// =========================================================================
double AnalyticalIkSolverAdapter::NormalizeAngleDeg(double angleDeg)
{
    return NormalizeAngleDegAnon(angleDeg);
}

double AnalyticalIkSolverAdapter::ComputeSeedDistance(
    const std::vector<double>& candidate, const std::vector<double>& seed)
{
    if (seed.empty()) return 0.0;
    double dist = 0.0;
    for (std::size_t i = 0; i < candidate.size() && i < seed.size(); ++i)
    {
        const double d = candidate[i] - seed[i];
        dist += d * d;
    }
    return std::sqrt(dist);
}

Eigen::Matrix3d AnalyticalIkSolverAdapter::ExtractRotation(const Eigen::Matrix4d& mat)
{
    return mat.block<3, 3>(0, 0);
}

Eigen::Vector3d AnalyticalIkSolverAdapter::ExtractPosition(const Eigen::Matrix4d& mat)
{
    return mat.block<3, 1>(0, 3);
}

Eigen::Matrix4d AnalyticalIkSolverAdapter::PoseToMatrix4d(
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = RpyDegToMatrix(pose.rpy_deg[0], pose.rpy_deg[1], pose.rpy_deg[2]);
    T(0, 3) = pose.position_m[0];
    T(1, 3) = pose.position_m[1];
    T(2, 3) = pose.position_m[2];
    return T;
}

Eigen::Matrix4d AnalyticalIkSolverAdapter::TcpFrameToMatrix4d(
    const RoboSDP::Kinematics::Dto::TcpFrameDto& tcp)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = RpyDegToMatrix(tcp.rpy_deg[0], tcp.rpy_deg[1], tcp.rpy_deg[2]);
    T(0, 3) = tcp.translation_m[0];
    T(1, 3) = tcp.translation_m[1];
    T(2, 3) = tcp.translation_m[2];
    return T;
}

void AnalyticalIkSolverAdapter::LogMessage(
    RoboSDP::Logging::LogLevel level,
    const QString& actionName,
    const QString& message) const
{
    if (m_logger == nullptr) return;
    m_logger->Log(
        level, message,
        RoboSDP::Errors::ErrorCode::Ok,
        {QStringLiteral("Kinematics"), actionName, QStringLiteral("AnalyticalIkSolverAdapter")});
}

// =========================================================================
// SolveIk 主入口
// =========================================================================
RoboSDP::Kinematics::Dto::IkResultDto AnalyticalIkSolverAdapter::SolveIk(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::IkRequestDto& request) const
{
    using RoboSDP::Kinematics::Dto::IkResultDto;

    const auto failWithReason = [this, &request](const QString& reason) -> IkResultDto
    {
        IkResultDto result;
        result.joint_positions_deg = request.seed_joint_positions_deg;
        result.success = false;
        result.solver_id = SolverId();
        result.message = QStringLiteral("解析 IK 求解失败：%1").arg(reason);
        LogMessage(RoboSDP::Logging::LogLevel::Warning, QStringLiteral("SolveIk"), result.message);
        return result;
    };

    // 1. Pieper 条件检测
    if (!CheckPieperCriterion(model))
    {
        return failWithReason(QStringLiteral("不满足 Pieper 准则（需要 6R 且 a4=a5=0）。"));
    }

    // 2. 计算目标法兰位姿
    const Eigen::Matrix4d T_flange_target = ComputeTargetFlangeTransform(model, request);
    const Eigen::Vector3d P_06 = ExtractPosition(T_flange_target);
    const Eigen::Matrix3d R_06 = ExtractRotation(T_flange_target);
    const Eigen::Vector3d z_06 = R_06 * Eigen::Vector3d::UnitZ();

    // 3. 腕心位置 (使用近似公式 P_wc = P_06 - (d5+d6) * R_06 * [0,0,1]^T)
    const double d5 = model.links[4].d;
    const double d6 = model.links[5].d;
    const Eigen::Vector3d wristCenter = P_06 - (d5 + d6) * z_06;

    // 4. 求解臂部 (θ1, θ2, θ3)
    const auto armSolutions = SolveArm(wristCenter, model.links);

    // 5. 对每组臂解求解腕部 (θ4, θ5, θ6)
    std::vector<JointSolution> allSolutions;
    for (const auto& armSol : armSolutions)
    {
        auto wristSolutions = SolveWrist(armSol, T_flange_target, model.links,
            model.parameter_convention);
        for (auto& ws : wristSolutions)
            allSolutions.push_back(std::move(ws));
    }

    const int totalFound = static_cast<int>(allSolutions.size());

    // 6. 限位过滤 + 排序
    auto validSolutions = FilterAndSortSolutions(
        std::move(allSolutions), model.joint_limits, request.seed_joint_positions_deg);

    if (validSolutions.empty())
    {
        return failWithReason(QStringLiteral("找到 %1 组原始解，但均被关节限位过滤。").arg(totalFound));
    }

    // 7. 取最佳解，计算误差
    const auto& best = validSolutions.front();

    const double posErrM = ComputePositionErrorM(
        best.values_deg, T_flange_target, model.links, model.parameter_convention);
    const double orientErrRad = ComputeOrientationErrorRad(
        best.values_deg, T_flange_target, model.links, model.parameter_convention);

    // 8. 组装结果
    IkResultDto result;
    result.success = true;
    result.solver_id = SolverId();
    result.joint_positions_deg = best.values_deg;
    result.position_error_mm = posErrM * 1000.0;
    result.orientation_error_deg = RadToDeg(orientErrRad);
    result.iteration_count = 1;

    // 填充多解信息
    result.total_solutions_found = totalFound;
    result.valid_solution_count = static_cast<int>(validSolutions.size());
    for (const auto& sol : validSolutions)
        result.all_solutions_deg.push_back(sol.values_deg);

    result.message = QStringLiteral("解析 IK 求解完成：共 %1 组原始解，%2 组有效，使用最佳解。")
        .arg(totalFound)
        .arg(validSolutions.size());

    LogMessage(RoboSDP::Logging::LogLevel::Info, QStringLiteral("SolveIk"), result.message);
    return result;
}

} // namespace RoboSDP::Kinematics::Adapter
