#pragma once

#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"

#include <QString>

#include <memory>
#include <vector>

#if defined(ROBOSDP_HAVE_PINOCCHIO)
#include "core/kinematics/SharedRobotKernelRegistry.h"
#include <pinocchio/multibody/frame.hpp>
#endif

namespace RoboSDP::Kinematics::Adapter
{
/**
 * @brief 后端适配器的内部私有状态声明。
 *
 * @details
 * 定义了 NativeKernelState 结构，
 * 在 .cpp 文件内部持有 Pinocchio 相关的原生重对象（如 Model、Data 以及业务 Frame ID 缓存）。
 * 通过该头文件将 Pinocchio 的依赖隔离在适配器实现层，避免暴露给系统其他模块。
 */
struct PinocchioKinematicBackendAdapter::NativeKernelState
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    std::shared_ptr<const RoboSDP::Core::Kinematics::SharedPinocchioModel> model;
    std::unique_ptr<RoboSDP::Core::Kinematics::SharedPinocchioData> data;
    pinocchio::FrameIndex base_frame_id = 0;
    pinocchio::FrameIndex flange_frame_id = 0;
    pinocchio::FrameIndex tcp_frame_id = 0;
    std::vector<double> native_position_offsets_deg;
    std::vector<pinocchio::FrameIndex> link_frame_ids;
#endif
    QString cache_key;
    int build_count = 0;
    int native_joint_count = 0;
    int native_frame_count = 0;
    bool ready = false;
    QString last_message;
    QString last_warning_message;
};

} // namespace RoboSDP::Kinematics::Adapter
