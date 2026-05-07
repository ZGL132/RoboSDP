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
