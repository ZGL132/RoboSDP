#pragma once

#include "modules/selection/dto/MotorSelectionDto.h"
#include "modules/selection/dto/ReducerSelectionDto.h"

#include <QString>

#include <vector>

namespace RoboSDP::Selection::Dto
{

/// 单个驱动链组合候选 DTO。
struct DriveTrainMatchCandidateDto
{
    QString joint_id;
    QString motor_id;
    QString reducer_id;
    QString motor_name;
    QString reducer_name;
    double combined_ratio = 1.0;
    double total_efficiency = 0.0;
    double output_rated_torque_nm = 0.0;
    double output_peak_torque_nm = 0.0;
    double output_peak_speed_deg_s = 0.0;
    bool brake_check_passed = false;
    double score = 0.0;
    QString recommendation_reason;
};

/// 单关节驱动链选型结果 DTO。
struct DriveTrainJointSelectionDto
{
    QString joint_id;
    std::vector<DriveTrainMatchCandidateDto> candidates;
    bool has_recommendation = false;
    DriveTrainMatchCandidateDto recommended;
    QString message;
};

/// 联合驱动链结果 DTO。
struct DriveTrainSelectionResultDto
{
    QString dynamic_ref;
    QString kinematic_ref;
    QString topology_ref;
    QString requirement_ref;
    std::vector<DriveTrainJointSelectionDto> joint_selections;
    bool success = false;
    QString message;
};

/**
 * @brief Selection 模块持久化工作态 DTO。
 *
 * 当前工作态只保存电机结果、减速器结果和联合驱动链结果，
 * 便于无复杂 UI 的情况下直接落盘和回读。
 */
struct SelectionWorkspaceStateDto
{
    std::vector<MotorSelectionResultDto> motor_results;
    std::vector<ReducerSelectionResultDto> reducer_results;
    DriveTrainSelectionResultDto drive_train_result;

    static SelectionWorkspaceStateDto CreateDefault()
    {
        return {};
    }
};

} // namespace RoboSDP::Selection::Dto
