#pragma once

#include "core/infrastructure/ProjectSaveCoordinator.h"
#include "core/logging/ConsoleLogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/requirement/service/RequirementService.h"

#include <QHash>
#include <QString>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QLabel;

namespace RoboSDP::Requirement::Ui
{

/**
 * @brief Requirement 最小可用页面。
 *
 * 页面只承担字段录入、保存/加载和基础校验结果展示，
 * 不做视觉美化扩展，也不接入三维联动和复杂结果窗口。
 */
class RequirementWidget : public QWidget, public RoboSDP::Infrastructure::IProjectSaveParticipant
{
    Q_OBJECT

public:
    explicit RequirementWidget(QWidget* parent = nullptr);

    /// @brief 返回全局保存日志使用的模块名称。
    QString ModuleName() const override;

    /// @brief 保存当前 Requirement 草稿，供模块按钮和全局保存共用。
    RoboSDP::Infrastructure::ProjectSaveItemResult SaveCurrentDraft() override;

    /// @brief 返回当前 Requirement 表单是否存在未保存变更。
    bool HasUnsavedChanges() const override;

    /// @brief 由 Ribbon 按钮触发校验（职责分离：Widget 不持有 action 按钮）。
    void TriggerValidate() { OnValidateClicked(); }

signals:
    /// 将 Requirement 操作消息抛给主窗口底部日志面板。
    void LogMessageGenerated(const QString& message);

private:
    void BuildUi();
    QGroupBox* CreateProjectMetaGroup();
    QGroupBox* CreateLoadRequirementsGroup();
    QGroupBox* CreateWorkspaceGroup();
    QGroupBox* CreateMotionGroup();
    QGroupBox* CreateAccuracyGroup();
    QGroupBox* CreateReliabilityGroup();
    QGroupBox* CreateValidationGroup();
    QWidget* CreateScrollableTab(QWidget* contentWidget);

    QDoubleSpinBox* CreateDoubleSpinBox(double minimum, double maximum, int decimals, double step);
    QSpinBox* CreateIntegerSpinBox(int minimum, int maximum, int step);
    void RegisterFieldWidget(const QString& fieldPath, QWidget* widget);
    void ConnectDirtyTracking();
    void MarkDirty();
    void MarkClean();

    /// 从当前表单收集 Requirement 模型，同时保留界面未直接编辑的透传字段。
    RoboSDP::Requirement::Dto::RequirementModelDto CollectModelFromForm();
    void PopulateForm(const RoboSDP::Requirement::Dto::RequirementModelDto& model);

    /// 刷新关键工位列表，确保列表项与当前工作模型保持一致。
    void RefreshKeyPoseList();

    /// 将当前关键工位编辑器的值回写到工作模型，避免切换列表项时丢失编辑内容。
    void SaveCurrentKeyPoseEdits();

    /// 根据当前选中的工位索引刷新右侧编辑器。
    void LoadCurrentKeyPoseToEditor();

    /// 收集关键工位编辑器中的单条工位数据。
    RoboSDP::Requirement::Dto::RequirementKeyPoseDto CollectKeyPoseFromEditor() const;

    /// 将单条关键工位数据填充到编辑器中。
    void PopulateKeyPoseEditor(const RoboSDP::Requirement::Dto::RequirementKeyPoseDto& keyPose);

    /// 构建关键工位列表项文本，便于用户区分多工位草稿。
    QString BuildKeyPoseListLabel(
        const RoboSDP::Requirement::Dto::RequirementKeyPoseDto& keyPose,
        int index) const;

    /// 根据勾选状态启用或禁用工具方向要求编辑器。
    void SetRequiredDirectionEditorsEnabled(bool enabled);

    void ClearValidationState();
    void ApplyValidationResult(const RoboSDP::Requirement::Validation::RequirementValidationResult& result);
    void SetOperationMessage(const QString& message, bool success);

    void OnValidateClicked();
    void OnSaveDraftClicked();
    void OnLoadClicked();
    void OnAddKeyPoseClicked();
    void OnRemoveKeyPoseClicked();
    void OnKeyPoseSelectionChanged(int currentRow);

private:
    RoboSDP::Logging::ConsoleLogger m_logger;
    RoboSDP::Repository::LocalJsonRepository m_repository;
    RoboSDP::Requirement::Validation::RequirementValidator m_validator;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage m_storage;
    RoboSDP::Requirement::Service::RequirementService m_service;
    RoboSDP::Requirement::Dto::RequirementModelDto m_working_model;

    QHash<QString, QWidget*> m_field_widgets;
    int m_current_key_pose_index = -1;
    bool m_has_unsaved_changes = false;

    QLabel* m_operation_label = nullptr;
    QLabel* m_validation_summary_label = nullptr;
    QListWidget* m_validation_issue_list = nullptr;

    QLineEdit* m_project_name_edit = nullptr;
    QComboBox* m_scenario_type_combo = nullptr;
    QLineEdit* m_description_edit = nullptr;

    QDoubleSpinBox* m_rated_payload_spin = nullptr;
    QDoubleSpinBox* m_max_payload_spin = nullptr;
    QDoubleSpinBox* m_tool_mass_spin = nullptr;
    QDoubleSpinBox* m_fixture_mass_spin = nullptr;
    QDoubleSpinBox* m_payload_cog_x_spin = nullptr;
    QDoubleSpinBox* m_payload_cog_y_spin = nullptr;
    QDoubleSpinBox* m_payload_cog_z_spin = nullptr;
    QDoubleSpinBox* m_payload_inertia_xx_spin = nullptr;
    QDoubleSpinBox* m_payload_inertia_yy_spin = nullptr;
    QDoubleSpinBox* m_payload_inertia_zz_spin = nullptr;
    QDoubleSpinBox* m_payload_inertia_xy_spin = nullptr;
    QDoubleSpinBox* m_payload_inertia_yz_spin = nullptr;
    QDoubleSpinBox* m_payload_inertia_xz_spin = nullptr;
    QCheckBox* m_off_center_load_check = nullptr;
    QDoubleSpinBox* m_cable_drag_load_spin = nullptr;

    QDoubleSpinBox* m_max_radius_spin = nullptr;
    QDoubleSpinBox* m_min_radius_spin = nullptr;
    QDoubleSpinBox* m_max_height_spin = nullptr;
    QDoubleSpinBox* m_min_height_spin = nullptr;
    QComboBox* m_base_mount_type_combo = nullptr;
    QComboBox* m_hollow_wrist_requirement_combo = nullptr;
    QDoubleSpinBox* m_reserved_channel_diameter_spin = nullptr;
    QListWidget* m_key_pose_list = nullptr;
    QPushButton* m_add_key_pose_button = nullptr;
    QPushButton* m_remove_key_pose_button = nullptr;
    QLineEdit* m_key_pose_id_edit = nullptr;
    QLineEdit* m_key_pose_name_edit = nullptr;
    QDoubleSpinBox* m_key_pose_x_spin = nullptr;
    QDoubleSpinBox* m_key_pose_y_spin = nullptr;
    QDoubleSpinBox* m_key_pose_z_spin = nullptr;
    QDoubleSpinBox* m_key_pose_rx_spin = nullptr;
    QDoubleSpinBox* m_key_pose_ry_spin = nullptr;
    QDoubleSpinBox* m_key_pose_rz_spin = nullptr;
    QDoubleSpinBox* m_key_pose_position_tol_spin = nullptr;
    QDoubleSpinBox* m_key_pose_orientation_tol_spin = nullptr;
    QCheckBox* m_key_pose_required_direction_check = nullptr;
    QDoubleSpinBox* m_key_pose_direction_x_spin = nullptr;
    QDoubleSpinBox* m_key_pose_direction_y_spin = nullptr;
    QDoubleSpinBox* m_key_pose_direction_z_spin = nullptr;

    QDoubleSpinBox* m_max_linear_speed_spin = nullptr;
    QDoubleSpinBox* m_max_angular_speed_spin = nullptr;
    QDoubleSpinBox* m_max_acceleration_spin = nullptr;
    QDoubleSpinBox* m_max_angular_acceleration_spin = nullptr;
    QDoubleSpinBox* m_jerk_limit_spin = nullptr;
    QDoubleSpinBox* m_takt_time_spin = nullptr;

    QDoubleSpinBox* m_absolute_accuracy_spin = nullptr;
    QDoubleSpinBox* m_repeatability_spin = nullptr;
    QDoubleSpinBox* m_tracking_accuracy_spin = nullptr;
    QDoubleSpinBox* m_orientation_accuracy_spin = nullptr;
    QDoubleSpinBox* m_tcp_position_tol_spin = nullptr;
    QDoubleSpinBox* m_tcp_orientation_tol_spin = nullptr;

    QDoubleSpinBox* m_design_life_spin = nullptr;
    QSpinBox* m_cycle_count_spin = nullptr;
    QDoubleSpinBox* m_duty_cycle_spin = nullptr;
    QDoubleSpinBox* m_operating_hours_per_day_spin = nullptr;
    QDoubleSpinBox* m_mtbf_target_spin = nullptr;
};

} // namespace RoboSDP::Requirement::Ui
