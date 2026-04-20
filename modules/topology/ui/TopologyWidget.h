#pragma once

#include "core/logging/ConsoleLogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"
#include "modules/topology/service/TopologyService.h"
#include "modules/topology/service/TopologyTemplateLoader.h"

#include <QHash>
#include <QWidget>

#include <array>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;

namespace RoboSDP::Topology::Ui
{

/**
 * @brief Topology 最小可用页面。
 *
 * 页面负责模板选择、候选生成、推荐展示以及保存/加载，
 * 不承载真实三维实体、复杂对比窗口和二期结构设计能力。
 */
class TopologyWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TopologyWidget(QWidget* parent = nullptr);

signals:
    /// 将 Topology 操作消息抛给主窗口底部日志面板。
    void LogMessageGenerated(const QString& message);

private:
    void BuildUi();
    void RefreshTemplateOptions();

    QGroupBox* CreateTopologyGroup();
    QGroupBox* CreateCandidateGroup();
    QGroupBox* CreateValidationGroup();

    QDoubleSpinBox* CreateDoubleSpinBox(double minimum, double maximum, int decimals, double step);
    void RegisterFieldWidget(const QString& fieldPath, QWidget* widget);

    RoboSDP::Topology::Dto::RobotTopologyModelDto CollectModelFromForm() const;
    void PopulateForm(const RoboSDP::Topology::Dto::RobotTopologyModelDto& model);
    void RenderCandidates();
    void RenderRecommendation();

    void ClearValidationState();
    void ApplyValidationResult(const RoboSDP::Topology::Validation::TopologyValidationResult& result);
    void SetOperationMessage(const QString& message, bool success);

    void OnBrowseProjectRootClicked();
    void OnRefreshTemplatesClicked();
    void OnGenerateClicked();
    void OnValidateClicked();
    void OnSaveDraftClicked();
    void OnLoadClicked();

private:
    RoboSDP::Logging::ConsoleLogger m_logger;
    RoboSDP::Repository::LocalJsonRepository m_repository;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage m_requirement_storage;
    RoboSDP::Topology::Persistence::TopologyJsonStorage m_topology_storage;
    RoboSDP::Topology::Validation::TopologyValidator m_validator;
    RoboSDP::Topology::Service::TopologyTemplateLoader m_template_loader;
    RoboSDP::Topology::Service::TopologyService m_service;

    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto m_state;
    QHash<QString, QWidget*> m_field_widgets;

    QLineEdit* m_project_root_edit = nullptr;
    QPushButton* m_browse_button = nullptr;
    QComboBox* m_template_combo = nullptr;
    QPushButton* m_refresh_template_button = nullptr;
    QPushButton* m_generate_button = nullptr;
    QPushButton* m_validate_button = nullptr;
    QPushButton* m_save_button = nullptr;
    QPushButton* m_load_button = nullptr;
    QLabel* m_operation_label = nullptr;

    QLineEdit* m_topology_name_edit = nullptr;
    QComboBox* m_base_mount_combo = nullptr;
    QDoubleSpinBox* m_base_height_spin = nullptr;
    QDoubleSpinBox* m_base_orientation_x_spin = nullptr;
    QDoubleSpinBox* m_base_orientation_y_spin = nullptr;
    QDoubleSpinBox* m_base_orientation_z_spin = nullptr;
    QDoubleSpinBox* m_j1_range_min_spin = nullptr;
    QDoubleSpinBox* m_j1_range_max_spin = nullptr;
    QLineEdit* m_shoulder_type_edit = nullptr;
    QLineEdit* m_elbow_type_edit = nullptr;
    QLineEdit* m_wrist_type_edit = nullptr;
    QCheckBox* m_wrist_intersection_check = nullptr;
    QCheckBox* m_wrist_offset_check = nullptr;
    QCheckBox* m_internal_routing_check = nullptr;
    QCheckBox* m_hollow_wrist_check = nullptr;
    QDoubleSpinBox* m_reserved_channel_spin = nullptr;
    QCheckBox* m_seventh_axis_reserved_check = nullptr;
    QLineEdit* m_hollow_joint_ids_edit = nullptr;
    QPlainTextEdit* m_axis_relations_edit = nullptr;
    std::array<QLineEdit*, 6> m_joint_role_edits {};

    QLabel* m_candidate_summary_label = nullptr;
    QListWidget* m_candidate_list = nullptr;
    QLabel* m_recommendation_summary_label = nullptr;
    QListWidget* m_recommendation_reason_list = nullptr;

    QLabel* m_validation_summary_label = nullptr;
    QListWidget* m_validation_issue_list = nullptr;
};

} // namespace RoboSDP::Topology::Ui
