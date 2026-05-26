#include "modules/kinematics/adapter/AnalyticalIkSolverAdapter.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace RoboSDP::Kinematics::Adapter
{

namespace
{

// =========================================================================
// 匿名空间：辅助常量与工具函数
// =========================================================================
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kSingularTolerance = 1.0e-10; // 奇异值判定容差

double DegToRad(double deg) { return deg * kDegToRad; }
double RadToDeg(double rad) { return rad * kRadToDeg; }

/// @brief 将角度规整到 [-180, 180) 范围内。
double NormalizeAngleDegAnon(double a)
{
    double n = std::fmod(a + 180.0, 360.0);
    if (n < 0.0) n += 360.0;
    return n - 180.0;
}

/// @brief 安全的反余弦函数，防止因浮点数微小误差导致输入超出 [-1, 1] 范围而产生 NaN。
double SafeAcos(double v)
{
    return std::acos(std::max(-1.0, std::min(1.0, v)));
}

/// @brief RPY (ZYX 外规旋转) -> 旋转矩阵。
/// 旋转顺序：先绕固定轴 X 旋转 Roll，再绕固定轴 Y 旋转 Pitch，最后绕固定轴 Z 旋转 Yaw。
/// 该定义与正向运动学 (FK) 中 BuildPoseFromSe3 的 RPY 约定一致。
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

/// @brief 计算目标 RPY 与当前 RPY 之间的姿态误差，返回以弧度表示的轴角姿态误差向量。
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
    return QStringLiteral("闭式解析 IK 求解器（标准 DH、6R、球形腕、|α₀|=90°、α₁=α₂=0）。");
}

// =========================================================================
// Pieper 条件检测
// =========================================================================
bool AnalyticalIkSolverAdapter::CheckPieperCriterion(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    double tolerance)
{
    // 1. 关节及连杆数量必须为 6（六轴机器人）
    if (model.links.size() != 6 || model.joint_count != 6)
        return false;

    // 2. 关节限位数据数量也必须为 6
    if (model.joint_limits.size() != 6)
        return false;

    // 3. 球形腕条件：最后三个关节的轴线必须交于一点。
    // 在标准 DH 参数下，这通常对应 a4 ≈ 0, a5 ≈ 0 且 d5 ≈ 0。
    if (std::abs(model.links[3].a) > tolerance ||
        std::abs(model.links[4].a) > tolerance ||
        std::abs(model.links[4].d) > tolerance)
        return false;

    // 4. 腕部相邻关节轴线的夹角 (alpha4, alpha5) 必须为 ±90°，以保证闭式解析法能够正常提取腕部角度
    const double alpha4 = std::abs(model.links[3].alpha);
    const double alpha5 = std::abs(model.links[4].alpha);
    if (std::abs(alpha4 - 90.0) > tolerance ||
        std::abs(alpha5 - 90.0) > tolerance)
        return false;

    // 5. 仅支持标准 DH（MDH 下臂部闭式解推导不适用）
    if (model.parameter_convention != QStringLiteral("DH"))
        return false;

    // 6. 臂型假设：α₁ ≈ 0、α₂ ≈ 0（肩关节与肘关节轴线平行，构成平面 2R 臂）
    if (std::abs(model.links[1].alpha) > tolerance ||
        std::abs(model.links[2].alpha) > tolerance)
        return false;

    // 7. 基座与肩关节轴线需近似垂直：|α₀| ≈ 90°
    //    保证臂部投影到水平面可用当前闭式推导
    if (std::abs(std::abs(model.links[0].alpha) - 90.0) > tolerance)
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

    // 标准 DH 齐次变换矩阵公式: RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = ct;         T(0, 1) = -st * ca;   T(0, 2) =  st * sa;   T(0, 3) = a * ct;
    T(1, 0) = st;         T(1, 1) =  ct * ca;   T(1, 2) = -ct * sa;   T(1, 3) = a * st;
    T(2, 1) = sa;         T(2, 2) =  ca;         T(2, 3) = d;
    return T;
}

Eigen::Matrix4d AnalyticalIkSolverAdapter::MdhTransform(
    double a, double alphaDeg, double d, double thetaDeg)
{
    const double ct = std::cos(DegToRad(thetaDeg));
    const double st = std::sin(DegToRad(thetaDeg));
    const double ca = std::cos(DegToRad(alphaDeg));
    const double sa = std::sin(DegToRad(alphaDeg));

    // 改进 DH (Modified DH) 齐次变换矩阵公式: TransX(a) * RotX(alpha) * RotZ(theta) * TransZ(d)
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
        // 计算时累加关节的初始偏差偏移量 (theta_offset)
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
    // 获取目标 TCP 的 4x4 齐次变换矩阵
    const Eigen::Matrix4d T_target = PoseToMatrix4d(request.target_pose);

    // 基坐标系变换矩阵
    const Eigen::Matrix4d T_base = PoseToMatrix4d(model.base_frame);

    // 工具中心点偏移量 (Flange -> TCP)
    const Eigen::Matrix4d T_tcp = TcpFrameToMatrix4d(model.tcp_frame);

    // 法兰坐标系偏移量 (DH 最后一个连杆坐标系 -> 法兰中心)
    const Eigen::Matrix4d T_flange = PoseToMatrix4d(model.flange_frame);

    // 逆向求解目标连杆链的末端位姿 (从基座 0 到连杆 6 的变换)：
    // T_base_to_flange_target = inv(T_base) * T_target * inv(T_tcp) * inv(T_flange)
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
    // 标准 DH 参数提取（0-indexed）：
    //   links[0]: a₀, α₀=±90°, d₁     — 基座偏置
    //   links[1]: a₁, α₁=0,    d₂=0   — 上臂长度
    //   links[2]: a₂, α₂=0,    d₃=0   — 前臂 DH a 参数
    //   links[3]: a₃=0, α₃=±90°, d₄   — 腕心沿 Z₃ 方向的偏移
    const double d1 = links[0].d;
    const double a0 = links[0].a;
    const double a1 = links[1].a;      // 上臂长度
    const double a2 = links[2].a;      // 前臂 DH a 参数
    const double d4 = links[3].d;      // 腕心在 Z₃ 方向的偏移

    if (a1 <= kSingularTolerance || a2 <= kSingularTolerance)
        return {};

    const double Px = wristCenter.x();
    const double Py = wristCenter.y();
    const double Pz = wristCenter.z();
    const double rho = std::hypot(Px, Py);
    if (rho <= kSingularTolerance)
        return {};  // 腕心在 Z₀ 轴上，退化

    // ────────────────────────────────────────────────────────────────────
    // 标准 DH 正向运动学（α₀=±90°, α₁=α₂=0）：
    //
    //   Px = (x₁+a₀)*cθ₁ + d₄*sθ₁
    //   Py = (x₁+a₀)*sθ₁ - d₄*cθ₁
    //   Pz = y₁ + d₁
    //
    // 其中 x₁ = a₁*cθ₂ + a₂*cos(θ₂+θ₃), y₁ = a₁*sθ₂ + a₂*sin(θ₂+θ₃)
    //       z₁ = d₄ （腕心在 Z₃ 方向偏移，垂直于臂平面）
    //
    // 由 Px*sθ₁ - Py*cθ₁ = d₄ 可解 θ₁
    // ────────────────────────────────────────────────────────────────────

    // d₄ 校正后的径向距离（腕心在臂平面内的投影半径）
    const double rho_sq = rho * rho;
    const double d4_sq = d4 * d4;
    const double radial = std::sqrt(std::max(0.0, rho_sq - d4_sq));

    // θ₁ 基准解（对应臂向前的构型）
    //   θ₁ = φ + atan2(-d₄, radial), 其中 φ = atan2(Py, Px)
    const double th1_rad_base = std::atan2(Py * radial - Px * d4,
                                            Px * radial + Py * d4);

    std::vector<std::array<double, 3>> solutions;

    for (const double shoulderSign : {1.0, -1.0})
    {
        const double th1_rad = th1_rad_base + (shoulderSign < 0.0 ? kPi : 0.0);
        const double ct1 = std::cos(th1_rad);
        const double st1 = std::sin(th1_rad);

        // 臂平面内腕心相对于关节 2 原点的位置
        //   x₁ = Px*cθ₁ + Py*sθ₁ - a₀   （径向分量）
        //   y₁ = Pz - d₁                 （垂直分量，即臂平面内高度）
        const double x1 = Px * ct1 + Py * st1 - a0;
        const double y1 = Pz - d1;

        // ── 余弦定理求 θ₃ ──
        //   L² = x₁² + y₁² + d₄² = a₁² + a₂² + d₄² + 2*a₁*a₂*cos(θ₃)
        const double L_sq = x1 * x1 + y1 * y1;
        const double cos_th3 = (L_sq - a1 * a1 - a2 * a2) / (2.0 * a1 * a2);
        if (cos_th3 < -1.0 - kSingularTolerance ||
            cos_th3 > 1.0 + kSingularTolerance)
            continue;

        const double th3_abs = SafeAcos(cos_th3);

        for (const double elbowSign : {1.0, -1.0})
        {
            const double th3 = elbowSign * th3_abs;

            // ── 2R 臂平面正解求 θ₂ ──
            //   x₁ = A*cθ₂ - B*sθ₂
            //   y₁ = A*sθ₂ + B*cθ₂
            //   其中 A = a₁ + a₂*cos(θ₃), B = a₂*sin(θ₃)
            const double c3 = std::cos(th3);
            const double s3 = std::sin(th3);
            const double A = a1 + a2 * c3;
            const double B = a2 * s3;
            const double r2_sq = A * A + B * B;
            if (r2_sq <= kSingularTolerance)
                continue;

            const double s2 = (A * y1 - B * x1) / r2_sq;
            const double c2 = (A * x1 + B * y1) / r2_sq;
            const double th2 = std::atan2(s2, c2);

            solutions.push_back({
                NormalizeAngleDeg(RadToDeg(th1_rad) - links[0].theta_offset),
                NormalizeAngleDeg(RadToDeg(th2) - links[1].theta_offset),
                NormalizeAngleDeg(RadToDeg(th3) - links[2].theta_offset)
            });
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
    // 1. 基于已知的臂部解计算前 3 个关节的正向运动学矩阵 T_03
    const std::vector<double> armAngles = {armSolutionDeg[0], armSolutionDeg[1], armSolutionDeg[2]};
    std::vector<RoboSDP::Kinematics::Dto::KinematicLinkParameterDto> first3Links(
        links.begin(), links.begin() + 3);
    const Eigen::Matrix4d T_03 = ComputeFlangeTransform(armAngles, first3Links, parameterConvention);
    const Eigen::Matrix3d R_03 = ExtractRotation(T_03);

    // 2. 提取目标末端总方向 R_06
    const Eigen::Matrix3d R_06 = ExtractRotation(targetFlangeTransform);

    // 3. 计算从连杆 3 到连杆 6 的相对旋转：R_3_to_6 = R_03^T * R_06
    const Eigen::Matrix3d R_36 = R_03.transpose() * R_06;

    // 获取相邻连杆旋向符号。由于已满足 Pieper 准则，|alpha4| 和 |alpha5| 接近 90°
    const double sA4 = std::sin(DegToRad(links[3].alpha)); // sin(α4), 值为 ±1
    const double sA5 = std::sin(DegToRad(links[4].alpha)); // sin(α5), 值为 ±1

    // 4. 从 R_36 提取腕关节角
    // R_36 展开式 (|α4|=|α5|=90°):
    //   R_36(0,2) =  cos(θ4) * sin(θ5) * sA5
    //   R_36(1,2) =  sin(θ4) * sin(θ5) * sA5
    //   R_36(2,2) = -sA4 * sA5 * cos(θ5)
    //   R_36(2,0) =  sA4 * sin(θ5) * cos(θ6)
    //   R_36(2,1) = -sA4 * sin(θ5) * sin(θ6)

    const double r13 = R_36(0, 2);
    const double r23 = R_36(1, 2);
    const double r33 = R_36(2, 2);
    const double r31 = R_36(2, 0);
    const double r32 = R_36(2, 1);

    // 计算 sin²(θ5)
    const double s5_sq = r13 * r13 + r23 * r23;

    std::vector<JointSolution> solutions;

    if (s5_sq > kSingularTolerance)
    {
        // ================= 非奇异情况 =================
        const double s5Mag = std::sqrt(s5_sq);           // |sin(θ5)|
        const double cosTh5 = -sA4 * sA5 * r33;          // cos(θ5)

        // sin(θ5) 可取正值或负值，对应两种不同的手腕姿态（翻转状态）
        for (int sign : {+1, -1})
        {
            const double th5_rad = std::atan2(sign * s5Mag, cosTh5);
            const double th4_rad = std::atan2(sign * sA5 * r23, sign * sA5 * r13);
            const double th6_rad = std::atan2(-sign * sA4 * r32, sign * sA4 * r31);

            JointSolution sol;
            sol.values_deg = {
                armSolutionDeg[0], armSolutionDeg[1], armSolutionDeg[2],
                NormalizeAngleDeg(RadToDeg(th4_rad) - links[3].theta_offset),
                NormalizeAngleDeg(RadToDeg(th5_rad) - links[4].theta_offset),
                NormalizeAngleDeg(RadToDeg(th6_rad) - links[5].theta_offset)
            };
            solutions.push_back(sol);
        }
    }
    else
    {
        // ================= 腕部奇异情况 (sin(θ5) ≈ 0) =================
        // 此时第 4 轴与第 6 轴共线，它们具有无穷多组解。
        // 按通用约定：令 θ4 = 0，从而将相对旋转全部合并给 θ6。
        const double cosTh5 = -sA4 * sA5 * r33;
        const double th5_rad = (cosTh5 >= 0.0) ? 0.0 : kPi;

        double th6_composite_rad = 0.0;
        if (cosTh5 >= 0.0)
        {
            // 当 θ5 ≈ 0 时：R_36 ≈ Rz(θ4) * Rx(α4+α5) * Rz(θ6)
            // 根据公式简化并用 R_36(0,0) 和 R_36(0,1) 求解复合角 θ6
            th6_composite_rad = std::atan2(-R_36(0, 1), R_36(0, 0));
        }
        else
        {
            // 当 θ5 ≈ π 时：R_36 ≈ Rz(θ4) * Rx(α4) * Rz(π) * Rx(α5) * Rz(θ6)
            th6_composite_rad = std::atan2(R_36(0, 1), -R_36(0, 0));
        }

        JointSolution sol;
        sol.values_deg = {
            armSolutionDeg[0], armSolutionDeg[1], armSolutionDeg[2],
            NormalizeAngleDeg(-links[3].theta_offset), // θ4 设为 0
            NormalizeAngleDeg(RadToDeg(th5_rad) - links[4].theta_offset),
            NormalizeAngleDeg(RadToDeg(th6_composite_rad) - links[5].theta_offset)
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
        // 将各关节角度规整到标准区间
        for (auto& v : sol.values_deg)
            v = NormalizeAngleDeg(v);

        // 软限位检查
        bool withinLimits = true;
        for (std::size_t j = 0; j < sol.values_deg.size() && j < jointLimits.size(); ++j)
        {
            const double val = sol.values_deg[j];
            // 允许给关节软限位提供 1.0 度的冗余宽容度
            if (val < jointLimits[j].soft_limit[0] - 1.0 ||
                val > jointLimits[j].soft_limit[1] + 1.0)
            {
                withinLimits = false;
                break;
            }
        }
        if (!withinLimits)
            continue;

        // 计算当前解相比于种子关节状态 (seed joint state) 的累计位移距离
        sol.seed_distance = ComputeSeedDistance(sol.values_deg, seedDeg);
        valid.push_back(std::move(sol));
    }

    // 按与种子状态的距离升序排序，使最接近当前姿态的解排在最前面
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

    // 统一定义失败退出的返回辅助闭包
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

    // 1. Pieper 准则几何条件检测
    if (!CheckPieperCriterion(model))
    {
        return failWithReason(QStringLiteral("不满足 Pieper 准则（需要 DH 标准、6R、α₁=α₂=0、|α₀|=|α₄|=|α₅|=90°、a₄=a₅=d₅=0）。"));
    }

    // 2. 计算目标法兰中心的目标位姿 R_06, P_06
    const Eigen::Matrix4d T_flange_target = ComputeTargetFlangeTransform(model, request);
    const Eigen::Vector3d P_06 = ExtractPosition(T_flange_target);
    const Eigen::Matrix3d R_06 = ExtractRotation(T_flange_target);
    const Eigen::Vector3d z_06 = R_06 * Eigen::Vector3d::UnitZ();

    // 3. 计算腕心位置 (Wrist Center)：标准球形手腕其第 4、5、6 轴线交于一点
    // 因此只需将法兰端点坐标沿末端 Z 轴方向反向缩减一个 d6 连杆长度。
    const double d6 = model.links[5].d;
    const Eigen::Vector3d wristCenter = P_06 - d6 * z_06;

    // 4. 求解臂部关节角度 (θ1, θ2, θ3)
    const auto armSolutions = SolveArm(wristCenter, model.links);

    // 5. 针对前一步得到的每组可用臂部解，进一步求出对应的手腕解 (θ4, θ5, θ6)
    std::vector<JointSolution> allSolutions;
    for (const auto& armSol : armSolutions)
    {
        auto wristSolutions = SolveWrist(armSol, T_flange_target, model.links,
            model.parameter_convention);
        for (auto& ws : wristSolutions)
            allSolutions.push_back(std::move(ws));
    }

    const int totalFound = static_cast<int>(allSolutions.size());

    // 6. 进行限位过滤与基于种子距离的重排序
    auto validSolutions = FilterAndSortSolutions(
        std::move(allSolutions), model.joint_limits, request.seed_joint_positions_deg);
    const int limitValidCount = static_cast<int>(validSolutions.size());
    
    // 通过正向运动学 (FK) 计算每个候选解的位姿误差，剔除误差超出容差范围的无效解。
    // [修复此处]: 在捕获列表中显式添加 "this" 指针，以调用类成员函数 ComputePositionErrorM 等
    validSolutions.erase(
        std::remove_if(validSolutions.begin(), validSolutions.end(),
            [this, &model, &T_flange_target](const JointSolution& solution) {
                const double positionErrorM = ComputePositionErrorM(
                    solution.values_deg, T_flange_target, model.links, model.parameter_convention);
                const double orientationErrorRad = ComputeOrientationErrorRad(
                    solution.values_deg, T_flange_target, model.links, model.parameter_convention);
                // 限制容差：位置误差不得大于 5 毫米，姿态误差不得大于 5 度
                return positionErrorM > 0.005 || orientationErrorRad > DegToRad(5.0);
            }),
        validSolutions.end());

    if (validSolutions.empty())
    {
        return failWithReason(QStringLiteral("解析 IK 生成 %1 组原始候选，限位过滤后 %2 组，FK 反验后 0 组有效。")
            .arg(totalFound)
            .arg(limitValidCount));
    }

    // 7. 提取最优解并评估误差精度
    const auto& best = validSolutions.front();

    const double posErrM = ComputePositionErrorM(
        best.values_deg, T_flange_target, model.links, model.parameter_convention);
    const double orientErrRad = ComputeOrientationErrorRad(
        best.values_deg, T_flange_target, model.links, model.parameter_convention);

    // 8. 组装并返回最终求解结果结构体
    IkResultDto result;
    result.success = true;
    result.solver_id = SolverId();
    result.joint_positions_deg = best.values_deg;
    result.position_error_mm = posErrM * 1000.0;
    result.orientation_error_deg = RadToDeg(orientErrRad);
    result.iteration_count = 1;

    // 输出多解统计信息
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
