#include "modules/requirement/ui/RequirementWidget.h"

#include "core/infrastructure/ProjectManager.h"

#include <algorithm>
#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSignalBlocker>
#include <QScrollArea>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>

namespace RoboSDP::Requirement::Ui
{

RequirementWidget::RequirementWidget(QWidget* parent)
    : QWidget(parent)
    , m_storage(m_repository)
    , m_service(m_storage, m_validator, &m_logger)
{
    BuildUi();
    PopulateForm(m_service.CreateDefaultModel());
    ConnectDirtyTracking();
    MarkClean();
}

QString RequirementWidget::ModuleName() const
{
    return QStringLiteral("Requirement");
}

bool RequirementWidget::HasUnsavedChanges() const
{
    return m_has_unsaved_changes;
}

RoboSDP::Infrastructure::ProjectSaveItemResult RequirementWidget::SaveCurrentDraft()
{
    const auto model = CollectModelFromForm();
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto saveResult = m_service.SaveDraft(projectRootPath, model);
    ApplyValidationResult(saveResult.validation_result);
    SetOperationMessage(saveResult.message, saveResult.IsSuccess());

    if (saveResult.IsSuccess() && !saveResult.validation_result.IsValid())
    {
        SetOperationMessage(
            QStringLiteral("草稿已保存，但当前仍有 %1 条校验问题。").arg(saveResult.validation_result.issues.size()),
            true);
    }
    if (saveResult.IsSuccess())
    {
        MarkClean();
    }

    return {ModuleName(), saveResult.IsSuccess(), saveResult.message};
}

void RequirementWidget::ConnectDirtyTracking()
{
    for (QLineEdit* editor : findChildren<QLineEdit*>())
    {
        connect(editor, &QLineEdit::textEdited, this, [this]() { MarkDirty(); });
    }
    for (QComboBox* editor : findChildren<QComboBox*>())
    {
        connect(editor, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [this](int) {
            MarkDirty();
        });
    }
    for (QDoubleSpinBox* editor : findChildren<QDoubleSpinBox*>())
    {
        connect(editor, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [this](double) {
            MarkDirty();
        });
    }
    for (QSpinBox* editor : findChildren<QSpinBox*>())
    {
        connect(editor, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, [this](int) {
            MarkDirty();
        });
    }
    for (QCheckBox* editor : findChildren<QCheckBox*>())
    {
        connect(editor, &QCheckBox::toggled, this, [this](bool) { MarkDirty(); });
    }
}

void RequirementWidget::MarkDirty()
{
    m_has_unsaved_changes = true;
}

void RequirementWidget::MarkClean()
{
    m_has_unsaved_changes = false;
}

void RequirementWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    m_operation_label = new QLabel(QStringLiteral("就绪：请录入 Requirement 基础字段。"), this);
    m_operation_label->setWordWrap(true);

    auto* tabs = new QTabWidget(this);
    tabs->setDocumentMode(true);
    // 中文说明：Requirement 表单按业务域拆分，字段控件仍由原有分组函数创建，避免影响 DTO 收集和保存路径。
    tabs->addTab(CreateScrollableTab(CreateProjectMetaGroup()), QStringLiteral("项目信息"));
    tabs->addTab(CreateScrollableTab(CreateLoadRequirementsGroup()), QStringLiteral("负载需求"));
    tabs->addTab(CreateScrollableTab(CreateWorkspaceGroup()), QStringLiteral("工作空间"));
    tabs->addTab(CreateScrollableTab(CreateMotionGroup()), QStringLiteral("运动性能"));
    tabs->addTab(CreateScrollableTab(CreateAccuracyGroup()), QStringLiteral("精度需求"));
    tabs->addTab(CreateScrollableTab(CreateReliabilityGroup()), QStringLiteral("可靠性"));
    tabs->addTab(CreateScrollableTab(CreateValidationGroup()), QStringLiteral("校验结果"));

    rootLayout->addWidget(m_operation_label);
    rootLayout->addWidget(tabs, 1);
}

QWidget* RequirementWidget::CreateScrollableTab(QWidget* contentWidget)
{
    auto* tabPage = new QWidget(this);
    auto* tabLayout = new QVBoxLayout(tabPage);
    tabLayout->setContentsMargins(0, 0, 0, 0);
    tabLayout->setSpacing(0);

    auto* scrollArea = new QScrollArea(tabPage);
    scrollArea->setWidgetResizable(true);

    auto* scrollContent = new QWidget(scrollArea);
    auto* contentLayout = new QVBoxLayout(scrollContent);
    contentLayout->setContentsMargins(4, 4, 4, 4);
    contentLayout->setSpacing(8);
    if (contentWidget != nullptr)
    {
        contentLayout->addWidget(contentWidget);
    }
    contentLayout->addStretch();
    scrollArea->setWidget(scrollContent);

    tabLayout->addWidget(scrollArea, 1);
    return tabPage;
}

QGroupBox* RequirementWidget::CreateProjectMetaGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("项目信息"), this);
    auto* layout = new QFormLayout(groupBox);

    m_project_name_edit = new QLineEdit(groupBox);
    m_scenario_type_combo = new QComboBox(groupBox);
    m_description_edit = new QLineEdit(groupBox);

    m_scenario_type_combo->addItem(QStringLiteral("搬运"), QStringLiteral("handling"));
    m_scenario_type_combo->addItem(QStringLiteral("焊接"), QStringLiteral("welding"));
    m_scenario_type_combo->addItem(QStringLiteral("装配"), QStringLiteral("assembly"));
    m_scenario_type_combo->addItem(QStringLiteral("打磨"), QStringLiteral("grinding"));
    m_scenario_type_combo->addItem(QStringLiteral("喷涂"), QStringLiteral("spraying"));
    m_scenario_type_combo->addItem(QStringLiteral("自定义"), QStringLiteral("custom"));

    layout->addRow(QStringLiteral("项目名称"), m_project_name_edit);
    layout->addRow(QStringLiteral("场景类型"), m_scenario_type_combo);
    layout->addRow(QStringLiteral("描述"), m_description_edit);

    RegisterFieldWidget(QStringLiteral("project_meta.project_name"), m_project_name_edit);
    RegisterFieldWidget(QStringLiteral("project_meta.scenario_type"), m_scenario_type_combo);

    return groupBox;
}

QGroupBox* RequirementWidget::CreateLoadRequirementsGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("负载需求"), this);
    auto* layout = new QFormLayout(groupBox);

    m_rated_payload_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_max_payload_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_tool_mass_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_fixture_mass_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_payload_cog_x_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_payload_cog_y_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_payload_cog_z_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_payload_inertia_xx_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 6, 0.001);
    m_payload_inertia_yy_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 6, 0.001);
    m_payload_inertia_zz_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 6, 0.001);
    m_payload_inertia_xy_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 6, 0.001);
    m_payload_inertia_yz_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 6, 0.001);
    m_payload_inertia_xz_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 6, 0.001);
    m_off_center_load_check = new QCheckBox(QStringLiteral("是否偏载"), groupBox);
    m_cable_drag_load_spin = CreateDoubleSpinBox(0.0, 1.0e9, 3, 1.0);

    layout->addRow(QStringLiteral("额定负载 [kg]"), m_rated_payload_spin);
    layout->addRow(QStringLiteral("最大负载 [kg]"), m_max_payload_spin);
    layout->addRow(QStringLiteral("工具质量 [kg]"), m_tool_mass_spin);
    layout->addRow(QStringLiteral("工装质量 [kg]"), m_fixture_mass_spin);
    layout->addRow(QStringLiteral("负载质心 X [m]"), m_payload_cog_x_spin);
    layout->addRow(QStringLiteral("负载质心 Y [m]"), m_payload_cog_y_spin);
    layout->addRow(QStringLiteral("负载质心 Z [m]"), m_payload_cog_z_spin);
    layout->addRow(QStringLiteral("偏载"), m_off_center_load_check);
    layout->addRow(QStringLiteral("电缆拖曳负载 [N]"), m_cable_drag_load_spin);

    layout->addRow(QStringLiteral("负载惯量 Ixx [kg*m^2]"), m_payload_inertia_xx_spin);
    layout->addRow(QStringLiteral("负载惯量 Iyy [kg*m^2]"), m_payload_inertia_yy_spin);
    layout->addRow(QStringLiteral("负载惯量 Izz [kg*m^2]"), m_payload_inertia_zz_spin);
    layout->addRow(QStringLiteral("负载惯量 Ixy [kg*m^2]"), m_payload_inertia_xy_spin);
    layout->addRow(QStringLiteral("负载惯量 Iyz [kg*m^2]"), m_payload_inertia_yz_spin);
    layout->addRow(QStringLiteral("负载惯量 Ixz [kg*m^2]"), m_payload_inertia_xz_spin);

    RegisterFieldWidget(QStringLiteral("load_requirements.rated_payload"), m_rated_payload_spin);
    RegisterFieldWidget(QStringLiteral("load_requirements.max_payload"), m_max_payload_spin);
    RegisterFieldWidget(QStringLiteral("load_requirements.tool_mass"), m_tool_mass_spin);
    RegisterFieldWidget(QStringLiteral("load_requirements.fixture_mass"), m_fixture_mass_spin);
    RegisterFieldWidget(QStringLiteral("load_requirements.payload_inertia[0]"), m_payload_inertia_xx_spin);
    RegisterFieldWidget(QStringLiteral("load_requirements.payload_inertia[1]"), m_payload_inertia_yy_spin);
    RegisterFieldWidget(QStringLiteral("load_requirements.payload_inertia[2]"), m_payload_inertia_zz_spin);
    RegisterFieldWidget(QStringLiteral("load_requirements.payload_inertia[3]"), m_payload_inertia_xy_spin);
    RegisterFieldWidget(QStringLiteral("load_requirements.payload_inertia[4]"), m_payload_inertia_yz_spin);
    RegisterFieldWidget(QStringLiteral("load_requirements.payload_inertia[5]"), m_payload_inertia_xz_spin);
    RegisterFieldWidget(QStringLiteral("load_requirements.cable_drag_load"), m_cable_drag_load_spin);

    return groupBox;
}

QGroupBox* RequirementWidget::CreateWorkspaceGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("工作空间与关键工位"), this);
    auto* layout = new QFormLayout(groupBox);

    m_max_radius_spin = CreateDoubleSpinBox(0.0, 1.0e6, 4, 0.01);
    m_min_radius_spin = CreateDoubleSpinBox(0.0, 1.0e6, 4, 0.01);
    m_max_height_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_min_height_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_base_mount_type_combo = new QComboBox(groupBox);
    m_hollow_wrist_requirement_combo = new QComboBox(groupBox);
    m_reserved_channel_diameter_spin = CreateDoubleSpinBox(0.0, 1.0e6, 2, 1.0);
    m_key_pose_list = new QListWidget(groupBox);
    m_key_pose_list->setSelectionMode(QAbstractItemView::SingleSelection);
    m_key_pose_list->setMinimumHeight(120);
    m_add_key_pose_button = new QPushButton(QStringLiteral("新增工位"), groupBox);
    m_remove_key_pose_button = new QPushButton(QStringLiteral("删除工位"), groupBox);

    /**
     * @brief 显式录入与 Topology 直接相关的 base_constraints 字段。
     *
     * 本轮只暴露 Topology 已消费的最小约束，避免 Requirement 页面提前承载二期结构设计字段。
     */
    m_base_mount_type_combo->addItem(QStringLiteral("未指定"), QString());
    m_base_mount_type_combo->addItem(QStringLiteral("落地"), QStringLiteral("floor"));
    m_base_mount_type_combo->addItem(QStringLiteral("壁挂"), QStringLiteral("wall"));
    m_base_mount_type_combo->addItem(QStringLiteral("顶装"), QStringLiteral("ceiling"));
    m_base_mount_type_combo->addItem(QStringLiteral("立柱"), QStringLiteral("pedestal"));

    m_hollow_wrist_requirement_combo->addItem(QStringLiteral("未指定"), QString());
    m_hollow_wrist_requirement_combo->addItem(QStringLiteral("需要中空腕"), QStringLiteral("true"));
    m_hollow_wrist_requirement_combo->addItem(QStringLiteral("不要求中空腕"), QStringLiteral("false"));

    auto* keyPoseToolbarWidget = new QWidget(groupBox);
    auto* keyPoseToolbarLayout = new QHBoxLayout(keyPoseToolbarWidget);
    keyPoseToolbarLayout->setContentsMargins(0, 0, 0, 0);
    keyPoseToolbarLayout->addWidget(m_add_key_pose_button);
    keyPoseToolbarLayout->addWidget(m_remove_key_pose_button);
    keyPoseToolbarLayout->addStretch();

    m_key_pose_id_edit = new QLineEdit(groupBox);
    m_key_pose_name_edit = new QLineEdit(groupBox);
    m_key_pose_x_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_key_pose_y_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_key_pose_z_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_key_pose_rx_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_key_pose_ry_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_key_pose_rz_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_key_pose_position_tol_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_key_pose_orientation_tol_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_key_pose_required_direction_check = new QCheckBox(QStringLiteral("启用工具方向要求"), groupBox);
    m_key_pose_direction_x_spin = CreateDoubleSpinBox(-1.0, 1.0, 4, 0.1);
    m_key_pose_direction_y_spin = CreateDoubleSpinBox(-1.0, 1.0, 4, 0.1);
    m_key_pose_direction_z_spin = CreateDoubleSpinBox(-1.0, 1.0, 4, 0.1);

    layout->addRow(QStringLiteral("最大半径 [m]"), m_max_radius_spin);
    layout->addRow(QStringLiteral("最小半径 [m]"), m_min_radius_spin);
    layout->addRow(QStringLiteral("最大高度 [m]"), m_max_height_spin);
    layout->addRow(QStringLiteral("最小高度 [m]"), m_min_height_spin);
    layout->addRow(QStringLiteral("基座安装偏好"), m_base_mount_type_combo);
    layout->addRow(QStringLiteral("中空腕需求"), m_hollow_wrist_requirement_combo);
    layout->addRow(QStringLiteral("预留通道直径 [mm]"), m_reserved_channel_diameter_spin);
    layout->addRow(QStringLiteral("关键工位 ID"), m_key_pose_id_edit);
    layout->addRow(QStringLiteral("关键工位名称"), m_key_pose_name_edit);
    layout->addRow(QStringLiteral("工位 X [m]"), m_key_pose_x_spin);
    layout->addRow(QStringLiteral("工位 Y [m]"), m_key_pose_y_spin);
    layout->addRow(QStringLiteral("工位 Z [m]"), m_key_pose_z_spin);
    layout->addRow(QStringLiteral("工位 RX [deg]"), m_key_pose_rx_spin);
    layout->addRow(QStringLiteral("工位 RY [deg]"), m_key_pose_ry_spin);
    layout->addRow(QStringLiteral("工位 RZ [deg]"), m_key_pose_rz_spin);
    layout->addRow(QStringLiteral("位置容限 [mm]"), m_key_pose_position_tol_spin);
    layout->addRow(QStringLiteral("姿态容限 [deg]"), m_key_pose_orientation_tol_spin);

    layout->addRow(QStringLiteral("关键工位操作"), keyPoseToolbarWidget);
    layout->addRow(QStringLiteral("关键工位列表"), m_key_pose_list);
    layout->addRow(QStringLiteral("方向要求"), m_key_pose_required_direction_check);
    layout->addRow(QStringLiteral("方向 X"), m_key_pose_direction_x_spin);
    layout->addRow(QStringLiteral("方向 Y"), m_key_pose_direction_y_spin);
    layout->addRow(QStringLiteral("方向 Z"), m_key_pose_direction_z_spin);

    RegisterFieldWidget(QStringLiteral("workspace_requirements.max_radius"), m_max_radius_spin);
    RegisterFieldWidget(QStringLiteral("workspace_requirements.min_radius"), m_min_radius_spin);
    RegisterFieldWidget(QStringLiteral("workspace_requirements.max_height"), m_max_height_spin);
    RegisterFieldWidget(QStringLiteral("workspace_requirements.min_height"), m_min_height_spin);
    RegisterFieldWidget(
        QStringLiteral("workspace_requirements.base_constraints.base_mount_type"),
        m_base_mount_type_combo);
    RegisterFieldWidget(
        QStringLiteral("workspace_requirements.base_constraints.hollow_wrist_required"),
        m_hollow_wrist_requirement_combo);
    RegisterFieldWidget(
        QStringLiteral("workspace_requirements.base_constraints.reserved_channel_diameter_mm"),
        m_reserved_channel_diameter_spin);
    RegisterFieldWidget(QStringLiteral("workspace_requirements.key_poses[0].pose_id"), m_key_pose_id_edit);
    RegisterFieldWidget(QStringLiteral("workspace_requirements.key_poses[0].name"), m_key_pose_name_edit);
    RegisterFieldWidget(
        QStringLiteral("workspace_requirements.key_poses[0].position_tol"),
        m_key_pose_position_tol_spin);
    RegisterFieldWidget(
        QStringLiteral("workspace_requirements.key_poses[0].orientation_tol"),
        m_key_pose_orientation_tol_spin);
    RegisterFieldWidget(
        QStringLiteral("workspace_requirements.key_poses[0].required_direction"),
        m_key_pose_required_direction_check);

    connect(
        m_key_pose_required_direction_check,
        &QCheckBox::toggled,
        this,
        [this](bool checked) { SetRequiredDirectionEditorsEnabled(checked); });
    connect(m_add_key_pose_button, &QPushButton::clicked, this, [this]() { OnAddKeyPoseClicked(); });
    connect(m_remove_key_pose_button, &QPushButton::clicked, this, [this]() { OnRemoveKeyPoseClicked(); });
    connect(
        m_key_pose_list,
        &QListWidget::currentRowChanged,
        this,
        [this](int currentRow) { OnKeyPoseSelectionChanged(currentRow); });

    SetRequiredDirectionEditorsEnabled(false);

    return groupBox;
}

QGroupBox* RequirementWidget::CreateMotionGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("运动性能"), this);
    auto* layout = new QFormLayout(groupBox);

    m_max_linear_speed_spin = CreateDoubleSpinBox(0.0, 1.0e6, 4, 0.01);
    m_max_angular_speed_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 1.0);
    m_max_acceleration_spin = CreateDoubleSpinBox(0.0, 1.0e6, 4, 0.01);
    m_max_angular_acceleration_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 1.0);
    m_jerk_limit_spin = CreateDoubleSpinBox(0.0, 1.0e6, 4, 0.01);
    m_takt_time_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);

    layout->addRow(QStringLiteral("最大线速度 [m/s]"), m_max_linear_speed_spin);
    layout->addRow(QStringLiteral("最大角速度 [deg/s]"), m_max_angular_speed_spin);
    layout->addRow(QStringLiteral("最大线加速度 [m/s^2]"), m_max_acceleration_spin);
    layout->addRow(QStringLiteral("最大角加速度 [deg/s^2]"), m_max_angular_acceleration_spin);
    layout->addRow(QStringLiteral("jerk 限制"), m_jerk_limit_spin);
    layout->addRow(QStringLiteral("节拍时间 [s]"), m_takt_time_spin);

    RegisterFieldWidget(QStringLiteral("motion_requirements.max_linear_speed"), m_max_linear_speed_spin);
    RegisterFieldWidget(QStringLiteral("motion_requirements.max_angular_speed"), m_max_angular_speed_spin);
    RegisterFieldWidget(QStringLiteral("motion_requirements.max_acceleration"), m_max_acceleration_spin);
    RegisterFieldWidget(
        QStringLiteral("motion_requirements.max_angular_acceleration"),
        m_max_angular_acceleration_spin);
    RegisterFieldWidget(QStringLiteral("motion_requirements.jerk_limit"), m_jerk_limit_spin);
    RegisterFieldWidget(QStringLiteral("motion_requirements.takt_time"), m_takt_time_spin);

    return groupBox;
}

QGroupBox* RequirementWidget::CreateAccuracyGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("精度需求"), this);
    auto* layout = new QFormLayout(groupBox);

    m_absolute_accuracy_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_repeatability_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_tracking_accuracy_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_orientation_accuracy_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_tcp_position_tol_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_tcp_orientation_tol_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);

    layout->addRow(QStringLiteral("绝对定位精度 [mm]"), m_absolute_accuracy_spin);
    layout->addRow(QStringLiteral("重复定位精度 [mm]"), m_repeatability_spin);
    layout->addRow(QStringLiteral("跟踪精度 [mm]"), m_tracking_accuracy_spin);
    layout->addRow(QStringLiteral("姿态精度 [deg]"), m_orientation_accuracy_spin);
    layout->addRow(QStringLiteral("TCP 位置容限 [mm]"), m_tcp_position_tol_spin);
    layout->addRow(QStringLiteral("TCP 姿态容限 [deg]"), m_tcp_orientation_tol_spin);

    RegisterFieldWidget(QStringLiteral("accuracy_requirements.absolute_accuracy"), m_absolute_accuracy_spin);
    RegisterFieldWidget(QStringLiteral("accuracy_requirements.repeatability"), m_repeatability_spin);
    RegisterFieldWidget(QStringLiteral("accuracy_requirements.tracking_accuracy"), m_tracking_accuracy_spin);
    RegisterFieldWidget(QStringLiteral("accuracy_requirements.orientation_accuracy"), m_orientation_accuracy_spin);
    RegisterFieldWidget(QStringLiteral("accuracy_requirements.tcp_position_tol"), m_tcp_position_tol_spin);
    RegisterFieldWidget(QStringLiteral("accuracy_requirements.tcp_orientation_tol"), m_tcp_orientation_tol_spin);

    return groupBox;
}

QGroupBox* RequirementWidget::CreateReliabilityGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("可靠性需求"), this);
    auto* layout = new QFormLayout(groupBox);

    m_design_life_spin = CreateDoubleSpinBox(0.0, 1.0e9, 2, 1.0);
    m_cycle_count_spin = CreateIntegerSpinBox(0, 1000000000, 1);
    m_duty_cycle_spin = CreateDoubleSpinBox(0.0, 1.0, 3, 0.05);
    m_operating_hours_per_day_spin = CreateDoubleSpinBox(0.0, 24.0, 2, 0.5);
    m_mtbf_target_spin = CreateDoubleSpinBox(0.0, 1.0e9, 2, 1.0);

    layout->addRow(QStringLiteral("设计寿命 [h]"), m_design_life_spin);
    layout->addRow(QStringLiteral("循环次数"), m_cycle_count_spin);
    layout->addRow(QStringLiteral("占空比 [0~1]"), m_duty_cycle_spin);
    layout->addRow(QStringLiteral("每日运行时长 [h]"), m_operating_hours_per_day_spin);
    layout->addRow(QStringLiteral("MTBF 目标 [h]"), m_mtbf_target_spin);

    RegisterFieldWidget(QStringLiteral("reliability_requirements.design_life"), m_design_life_spin);
    RegisterFieldWidget(QStringLiteral("reliability_requirements.cycle_count"), m_cycle_count_spin);
    RegisterFieldWidget(QStringLiteral("reliability_requirements.duty_cycle"), m_duty_cycle_spin);
    RegisterFieldWidget(
        QStringLiteral("reliability_requirements.operating_hours_per_day"),
        m_operating_hours_per_day_spin);
    RegisterFieldWidget(QStringLiteral("reliability_requirements.mtbf_target"), m_mtbf_target_spin);

    return groupBox;
}

QGroupBox* RequirementWidget::CreateValidationGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("基础校验结果"), this);
    auto* layout = new QVBoxLayout(groupBox);

    m_validation_summary_label = new QLabel(QStringLiteral("尚未执行校验。"), groupBox);
    m_validation_summary_label->setWordWrap(true);
    m_validation_issue_list = new QListWidget(groupBox);
    m_validation_issue_list->setSelectionMode(QAbstractItemView::NoSelection);

    layout->addWidget(m_validation_summary_label);
    layout->addWidget(m_validation_issue_list);

    return groupBox;
}

RoboSDP::Requirement::Dto::RequirementModelDto RequirementWidget::CollectModelFromForm()
{
    using namespace RoboSDP::Requirement::Dto;

    SaveCurrentKeyPoseEdits();

    // 从当前工作模型出发，保留 UI 未直接编辑的透传字段。
    RequirementModelDto model = m_working_model;

    model.project_meta.project_name = m_project_name_edit->text().trimmed();
    model.project_meta.scenario_type = m_scenario_type_combo->currentData().toString();
    model.project_meta.description = m_description_edit->text().trimmed();

    model.load_requirements.rated_payload = m_rated_payload_spin->value();
    model.load_requirements.max_payload = m_max_payload_spin->value();
    model.load_requirements.tool_mass = m_tool_mass_spin->value();
    model.load_requirements.fixture_mass = m_fixture_mass_spin->value();
    model.load_requirements.payload_cog = {
        m_payload_cog_x_spin->value(),
        m_payload_cog_y_spin->value(),
        m_payload_cog_z_spin->value()};
    model.load_requirements.payload_inertia = {
        m_payload_inertia_xx_spin->value(),
        m_payload_inertia_yy_spin->value(),
        m_payload_inertia_zz_spin->value(),
        m_payload_inertia_xy_spin->value(),
        m_payload_inertia_yz_spin->value(),
        m_payload_inertia_xz_spin->value()};
    model.load_requirements.off_center_load = m_off_center_load_check->isChecked();
    model.load_requirements.cable_drag_load = m_cable_drag_load_spin->value();

    model.workspace_requirements.max_radius = m_max_radius_spin->value();
    model.workspace_requirements.min_radius = m_min_radius_spin->value();
    model.workspace_requirements.max_height = m_max_height_spin->value();
    model.workspace_requirements.min_height = m_min_height_spin->value();

    QJsonObject baseConstraints = model.workspace_requirements.base_constraints;
    baseConstraints.remove(QStringLiteral("base_mount_type"));
    baseConstraints.remove(QStringLiteral("hollow_wrist_required"));
    baseConstraints.remove(QStringLiteral("reserved_channel_diameter_mm"));

    const QString baseMountType = m_base_mount_type_combo->currentData().toString();
    if (!baseMountType.isEmpty())
    {
        baseConstraints.insert(QStringLiteral("base_mount_type"), baseMountType);
    }

    const QString hollowWristRequirement = m_hollow_wrist_requirement_combo->currentData().toString();
    if (hollowWristRequirement == QStringLiteral("true"))
    {
        baseConstraints.insert(QStringLiteral("hollow_wrist_required"), true);
    }
    else if (hollowWristRequirement == QStringLiteral("false"))
    {
        baseConstraints.insert(QStringLiteral("hollow_wrist_required"), false);
    }

    if (m_reserved_channel_diameter_spin->value() > 0.0)
    {
        baseConstraints.insert(
            QStringLiteral("reserved_channel_diameter_mm"),
            m_reserved_channel_diameter_spin->value());
    }
    model.workspace_requirements.base_constraints = baseConstraints;

    if (model.workspace_requirements.key_poses.empty())
    {
        model.workspace_requirements.key_poses.push_back(RequirementKeyPoseDto {});
    }

    const int currentIndex = std::clamp(
        m_current_key_pose_index,
        0,
        static_cast<int>(model.workspace_requirements.key_poses.size()) - 1);
    model.workspace_requirements.key_poses[static_cast<std::size_t>(currentIndex)] =
        CollectKeyPoseFromEditor();

    model.motion_requirements.max_linear_speed = m_max_linear_speed_spin->value();
    model.motion_requirements.max_angular_speed = m_max_angular_speed_spin->value();
    model.motion_requirements.max_acceleration = m_max_acceleration_spin->value();
    model.motion_requirements.max_angular_acceleration = m_max_angular_acceleration_spin->value();
    model.motion_requirements.jerk_limit = m_jerk_limit_spin->value();
    model.motion_requirements.takt_time = m_takt_time_spin->value();

    model.accuracy_requirements.absolute_accuracy = m_absolute_accuracy_spin->value();
    model.accuracy_requirements.repeatability = m_repeatability_spin->value();
    model.accuracy_requirements.tracking_accuracy = m_tracking_accuracy_spin->value();
    model.accuracy_requirements.orientation_accuracy = m_orientation_accuracy_spin->value();
    model.accuracy_requirements.tcp_position_tol = m_tcp_position_tol_spin->value();
    model.accuracy_requirements.tcp_orientation_tol = m_tcp_orientation_tol_spin->value();

    model.reliability_requirements.design_life = m_design_life_spin->value();
    model.reliability_requirements.cycle_count = m_cycle_count_spin->value();
    model.reliability_requirements.duty_cycle = m_duty_cycle_spin->value();
    model.reliability_requirements.operating_hours_per_day = m_operating_hours_per_day_spin->value();
    model.reliability_requirements.mtbf_target = m_mtbf_target_spin->value();

    m_working_model = model;
    return model;
}

void RequirementWidget::PopulateForm(const RoboSDP::Requirement::Dto::RequirementModelDto& model)
{
    m_working_model = model;
    if (m_working_model.workspace_requirements.key_poses.empty())
    {
        m_working_model.workspace_requirements.key_poses.push_back(
            RoboSDP::Requirement::Dto::RequirementKeyPoseDto {});
    }

    m_project_name_edit->setText(m_working_model.project_meta.project_name);

    const int scenarioIndex = m_scenario_type_combo->findData(m_working_model.project_meta.scenario_type);
    m_scenario_type_combo->setCurrentIndex(scenarioIndex >= 0 ? scenarioIndex : 0);
    m_description_edit->setText(m_working_model.project_meta.description);

    m_rated_payload_spin->setValue(m_working_model.load_requirements.rated_payload);
    m_max_payload_spin->setValue(m_working_model.load_requirements.max_payload);
    m_tool_mass_spin->setValue(m_working_model.load_requirements.tool_mass);
    m_fixture_mass_spin->setValue(m_working_model.load_requirements.fixture_mass);
    m_payload_cog_x_spin->setValue(m_working_model.load_requirements.payload_cog[0]);
    m_payload_cog_y_spin->setValue(m_working_model.load_requirements.payload_cog[1]);
    m_payload_cog_z_spin->setValue(m_working_model.load_requirements.payload_cog[2]);
    m_payload_inertia_xx_spin->setValue(m_working_model.load_requirements.payload_inertia[0]);
    m_payload_inertia_yy_spin->setValue(m_working_model.load_requirements.payload_inertia[1]);
    m_payload_inertia_zz_spin->setValue(m_working_model.load_requirements.payload_inertia[2]);
    m_payload_inertia_xy_spin->setValue(m_working_model.load_requirements.payload_inertia[3]);
    m_payload_inertia_yz_spin->setValue(m_working_model.load_requirements.payload_inertia[4]);
    m_payload_inertia_xz_spin->setValue(m_working_model.load_requirements.payload_inertia[5]);
    m_off_center_load_check->setChecked(m_working_model.load_requirements.off_center_load);
    m_cable_drag_load_spin->setValue(m_working_model.load_requirements.cable_drag_load);

    m_max_radius_spin->setValue(m_working_model.workspace_requirements.max_radius);
    m_min_radius_spin->setValue(m_working_model.workspace_requirements.min_radius);
    m_max_height_spin->setValue(m_working_model.workspace_requirements.max_height);
    m_min_height_spin->setValue(m_working_model.workspace_requirements.min_height);

    const QJsonObject baseConstraints = m_working_model.workspace_requirements.base_constraints;
    const int baseMountIndex =
        m_base_mount_type_combo->findData(baseConstraints.value(QStringLiteral("base_mount_type")).toString());
    m_base_mount_type_combo->setCurrentIndex(baseMountIndex >= 0 ? baseMountIndex : 0);

    int hollowWristIndex = 0;
    if (baseConstraints.contains(QStringLiteral("hollow_wrist_required")))
    {
        hollowWristIndex = m_hollow_wrist_requirement_combo->findData(
            baseConstraints.value(QStringLiteral("hollow_wrist_required")).toBool() ? QStringLiteral("true")
                                                                                    : QStringLiteral("false"));
    }
    m_hollow_wrist_requirement_combo->setCurrentIndex(hollowWristIndex >= 0 ? hollowWristIndex : 0);
    m_reserved_channel_diameter_spin->setValue(
        baseConstraints.value(QStringLiteral("reserved_channel_diameter_mm")).toDouble(0.0));

    m_max_linear_speed_spin->setValue(m_working_model.motion_requirements.max_linear_speed);
    m_max_angular_speed_spin->setValue(m_working_model.motion_requirements.max_angular_speed);
    m_max_acceleration_spin->setValue(m_working_model.motion_requirements.max_acceleration);
    m_max_angular_acceleration_spin->setValue(m_working_model.motion_requirements.max_angular_acceleration);
    m_jerk_limit_spin->setValue(m_working_model.motion_requirements.jerk_limit);
    m_takt_time_spin->setValue(m_working_model.motion_requirements.takt_time);

    m_absolute_accuracy_spin->setValue(m_working_model.accuracy_requirements.absolute_accuracy);
    m_repeatability_spin->setValue(m_working_model.accuracy_requirements.repeatability);
    m_tracking_accuracy_spin->setValue(m_working_model.accuracy_requirements.tracking_accuracy);
    m_orientation_accuracy_spin->setValue(m_working_model.accuracy_requirements.orientation_accuracy);
    m_tcp_position_tol_spin->setValue(m_working_model.accuracy_requirements.tcp_position_tol);
    m_tcp_orientation_tol_spin->setValue(m_working_model.accuracy_requirements.tcp_orientation_tol);

    m_design_life_spin->setValue(m_working_model.reliability_requirements.design_life);
    m_cycle_count_spin->setValue(m_working_model.reliability_requirements.cycle_count);
    m_duty_cycle_spin->setValue(m_working_model.reliability_requirements.duty_cycle);
    m_operating_hours_per_day_spin->setValue(m_working_model.reliability_requirements.operating_hours_per_day);
    m_mtbf_target_spin->setValue(m_working_model.reliability_requirements.mtbf_target);

    RefreshKeyPoseList();
    m_current_key_pose_index = 0;
    {
        const QSignalBlocker blocker(m_key_pose_list);
        m_key_pose_list->setCurrentRow(m_current_key_pose_index);
    }
    LoadCurrentKeyPoseToEditor();
}

void RequirementWidget::RefreshKeyPoseList()
{
    const QSignalBlocker blocker(m_key_pose_list);
    m_key_pose_list->clear();

    for (std::size_t index = 0; index < m_working_model.workspace_requirements.key_poses.size(); ++index)
    {
        const auto& keyPose = m_working_model.workspace_requirements.key_poses.at(index);
        m_key_pose_list->addItem(BuildKeyPoseListLabel(keyPose, static_cast<int>(index)));
    }

    m_remove_key_pose_button->setEnabled(m_working_model.workspace_requirements.key_poses.size() > 1);
}

void RequirementWidget::SaveCurrentKeyPoseEdits()
{
    if (m_current_key_pose_index < 0 ||
        m_current_key_pose_index >= static_cast<int>(m_working_model.workspace_requirements.key_poses.size()))
    {
        return;
    }

    m_working_model.workspace_requirements.key_poses[static_cast<std::size_t>(m_current_key_pose_index)] =
        CollectKeyPoseFromEditor();

    if (m_key_pose_list != nullptr && m_current_key_pose_index < m_key_pose_list->count())
    {
        m_key_pose_list->item(m_current_key_pose_index)
            ->setText(
                BuildKeyPoseListLabel(
                    m_working_model.workspace_requirements.key_poses[static_cast<std::size_t>(m_current_key_pose_index)],
                    m_current_key_pose_index));
    }
}

void RequirementWidget::LoadCurrentKeyPoseToEditor()
{
    if (m_current_key_pose_index < 0 ||
        m_current_key_pose_index >= static_cast<int>(m_working_model.workspace_requirements.key_poses.size()))
    {
        PopulateKeyPoseEditor(RoboSDP::Requirement::Dto::RequirementKeyPoseDto {});
        return;
    }

    PopulateKeyPoseEditor(
        m_working_model.workspace_requirements.key_poses[static_cast<std::size_t>(m_current_key_pose_index)]);
}

RoboSDP::Requirement::Dto::RequirementKeyPoseDto RequirementWidget::CollectKeyPoseFromEditor() const
{
    RoboSDP::Requirement::Dto::RequirementKeyPoseDto keyPose;
    keyPose.pose_id = m_key_pose_id_edit->text().trimmed();
    keyPose.name = m_key_pose_name_edit->text().trimmed();
    keyPose.pose = {
        m_key_pose_x_spin->value(),
        m_key_pose_y_spin->value(),
        m_key_pose_z_spin->value(),
        m_key_pose_rx_spin->value(),
        m_key_pose_ry_spin->value(),
        m_key_pose_rz_spin->value()};
    keyPose.position_tol = m_key_pose_position_tol_spin->value();
    keyPose.orientation_tol = m_key_pose_orientation_tol_spin->value();
    keyPose.has_required_direction = m_key_pose_required_direction_check->isChecked();
    keyPose.required_direction = {
        m_key_pose_direction_x_spin->value(),
        m_key_pose_direction_y_spin->value(),
        m_key_pose_direction_z_spin->value()};
    return keyPose;
}

void RequirementWidget::PopulateKeyPoseEditor(
    const RoboSDP::Requirement::Dto::RequirementKeyPoseDto& keyPose)
{
    m_key_pose_id_edit->setText(keyPose.pose_id);
    m_key_pose_name_edit->setText(keyPose.name);
    m_key_pose_x_spin->setValue(keyPose.pose[0]);
    m_key_pose_y_spin->setValue(keyPose.pose[1]);
    m_key_pose_z_spin->setValue(keyPose.pose[2]);
    m_key_pose_rx_spin->setValue(keyPose.pose[3]);
    m_key_pose_ry_spin->setValue(keyPose.pose[4]);
    m_key_pose_rz_spin->setValue(keyPose.pose[5]);
    m_key_pose_position_tol_spin->setValue(keyPose.position_tol);
    m_key_pose_orientation_tol_spin->setValue(keyPose.orientation_tol);
    m_key_pose_direction_x_spin->setValue(keyPose.required_direction[0]);
    m_key_pose_direction_y_spin->setValue(keyPose.required_direction[1]);
    m_key_pose_direction_z_spin->setValue(keyPose.required_direction[2]);

    {
        const QSignalBlocker blocker(m_key_pose_required_direction_check);
        m_key_pose_required_direction_check->setChecked(keyPose.has_required_direction);
    }
    SetRequiredDirectionEditorsEnabled(keyPose.has_required_direction);
}

QString RequirementWidget::BuildKeyPoseListLabel(
    const RoboSDP::Requirement::Dto::RequirementKeyPoseDto& keyPose,
    int index) const
{
    const QString displayName = keyPose.name.trimmed().isEmpty()
                                    ? QStringLiteral("工位%1").arg(index + 1)
                                    : keyPose.name.trimmed();
    const QString displayId = keyPose.pose_id.trimmed().isEmpty()
                                  ? QStringLiteral("pose_%1").arg(index + 1, 3, 10, QChar('0'))
                                  : keyPose.pose_id.trimmed();
    return QStringLiteral("%1 [%2]").arg(displayName, displayId);
}

void RequirementWidget::SetRequiredDirectionEditorsEnabled(bool enabled)
{
    m_key_pose_direction_x_spin->setEnabled(enabled);
    m_key_pose_direction_y_spin->setEnabled(enabled);
    m_key_pose_direction_z_spin->setEnabled(enabled);
}

void RequirementWidget::ClearValidationState()
{
    for (QWidget* widget : m_field_widgets)
    {
        if (widget != nullptr)
        {
            widget->setStyleSheet(QString());
            widget->setToolTip(QString());
        }
    }

    m_validation_issue_list->clear();
}

void RequirementWidget::ApplyValidationResult(
    const RoboSDP::Requirement::Validation::RequirementValidationResult& result)
{
    ClearValidationState();

    const auto resolveCurrentKeyPoseWidget = [this](const QString& fieldPath) -> QWidget* {
        const QString prefix = QStringLiteral("workspace_requirements.key_poses[%1].").arg(m_current_key_pose_index);
        if (!fieldPath.startsWith(prefix))
        {
            return nullptr;
        }

        const QString suffix = fieldPath.mid(prefix.size());
        if (suffix == QStringLiteral("pose_id"))
        {
            return m_key_pose_id_edit;
        }
        if (suffix == QStringLiteral("name"))
        {
            return m_key_pose_name_edit;
        }
        if (suffix == QStringLiteral("position_tol"))
        {
            return m_key_pose_position_tol_spin;
        }
        if (suffix == QStringLiteral("orientation_tol"))
        {
            return m_key_pose_orientation_tol_spin;
        }
        return nullptr;
    };

    if (result.IsValid())
    {
        m_validation_summary_label->setStyleSheet(QStringLiteral("color: #1b7f3b;"));
        m_validation_summary_label->setText(QStringLiteral("基础校验通过，当前 Requirement 可继续保存与下游使用。"));
        return;
    }

    m_validation_summary_label->setStyleSheet(QStringLiteral("color: #b42318;"));
    m_validation_summary_label->setText(
        QStringLiteral("基础校验发现 %1 条问题，其中错误 %2 条，警告 %3 条。")
            .arg(result.issues.size())
            .arg(result.ErrorCount())
            .arg(result.WarningCount()));

    for (const auto& issue : result.issues)
    {
        auto* item = new QListWidgetItem(
            QStringLiteral("[%1] %2 - %3")
                .arg(RoboSDP::Requirement::Validation::ToString(issue.severity), issue.field, issue.message_zh),
            m_validation_issue_list);
        item->setToolTip(issue.code);

        QWidget* fieldWidget = m_field_widgets.value(issue.field, nullptr);
        if (fieldWidget == nullptr)
        {
            fieldWidget = resolveCurrentKeyPoseWidget(issue.field);
        }
        if (fieldWidget != nullptr)
        {
            fieldWidget->setStyleSheet(
                issue.severity == RoboSDP::Requirement::Validation::ValidationSeverity::Warning
                    ? QStringLiteral("border: 1px solid #d97706;")
                    : QStringLiteral("border: 1px solid #dc2626;"));
            fieldWidget->setToolTip(issue.message_zh);
        }
    }
}

void RequirementWidget::OnValidateClicked()
{
    const auto model = CollectModelFromForm();
    const auto validationResult = m_service.Validate(model);
    ApplyValidationResult(validationResult);

    SetOperationMessage(
        validationResult.IsValid()
            ? QStringLiteral("Requirement 基础校验通过。")
            : QStringLiteral("Requirement 基础校验未通过，请根据结果列表修正字段。"),
        validationResult.IsValid());

    emit LogMessageGenerated(
        validationResult.IsValid()
            ? QStringLiteral("[Requirement] 基础校验通过。")
            : QStringLiteral("[Requirement] 基础校验发现 %1 条问题。").arg(validationResult.issues.size()));
}

void RequirementWidget::OnSaveDraftClicked()
{
    const auto saveResult = SaveCurrentDraft();
    emit LogMessageGenerated(QStringLiteral("[Requirement] %1").arg(saveResult.message));
}

void RequirementWidget::OnLoadClicked()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto loadResult = m_service.LoadDraft(projectRootPath);
    if (loadResult.IsSuccess())
    {
        PopulateForm(loadResult.model);
        ApplyValidationResult(loadResult.validation_result);
        MarkClean();
    }

    SetOperationMessage(loadResult.message, loadResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Requirement] %1").arg(loadResult.message));
}

void RequirementWidget::OnAddKeyPoseClicked()
{
    SaveCurrentKeyPoseEdits();

    RoboSDP::Requirement::Dto::RequirementKeyPoseDto keyPose;
    const int nextIndex = static_cast<int>(m_working_model.workspace_requirements.key_poses.size()) + 1;
    keyPose.pose_id = QStringLiteral("pose_%1").arg(nextIndex, 3, 10, QChar('0'));
    keyPose.name = QStringLiteral("工位%1").arg(nextIndex);
    m_working_model.workspace_requirements.key_poses.push_back(keyPose);

    RefreshKeyPoseList();
    m_current_key_pose_index = static_cast<int>(m_working_model.workspace_requirements.key_poses.size()) - 1;

    {
        const QSignalBlocker blocker(m_key_pose_list);
        m_key_pose_list->setCurrentRow(m_current_key_pose_index);
    }
    LoadCurrentKeyPoseToEditor();
    MarkDirty();
}

void RequirementWidget::OnRemoveKeyPoseClicked()
{
    if (m_working_model.workspace_requirements.key_poses.size() <= 1 ||
        m_current_key_pose_index < 0 ||
        m_current_key_pose_index >= static_cast<int>(m_working_model.workspace_requirements.key_poses.size()))
    {
        return;
    }

    m_working_model.workspace_requirements.key_poses.erase(
        m_working_model.workspace_requirements.key_poses.begin() + m_current_key_pose_index);

    RefreshKeyPoseList();
    m_current_key_pose_index = std::min(
        m_current_key_pose_index,
        static_cast<int>(m_working_model.workspace_requirements.key_poses.size()) - 1);

    {
        const QSignalBlocker blocker(m_key_pose_list);
        m_key_pose_list->setCurrentRow(m_current_key_pose_index);
    }
    LoadCurrentKeyPoseToEditor();
    MarkDirty();
}

void RequirementWidget::OnKeyPoseSelectionChanged(int currentRow)
{
    if (currentRow < 0 || currentRow >= static_cast<int>(m_working_model.workspace_requirements.key_poses.size()))
    {
        return;
    }

    if (m_current_key_pose_index != currentRow)
    {
        SaveCurrentKeyPoseEdits();
        m_current_key_pose_index = currentRow;
    }

    LoadCurrentKeyPoseToEditor();
}

QDoubleSpinBox* RequirementWidget::CreateDoubleSpinBox(
    double minimum,
    double maximum,
    int decimals,
    double step)
{
    auto* spinBox = new QDoubleSpinBox(this);
    spinBox->setRange(minimum, maximum);
    spinBox->setDecimals(decimals);
    spinBox->setSingleStep(step);
    spinBox->setAccelerated(true);
    return spinBox;
}

QSpinBox* RequirementWidget::CreateIntegerSpinBox(int minimum, int maximum, int step)
{
    auto* spinBox = new QSpinBox(this);
    spinBox->setRange(minimum, maximum);
    spinBox->setSingleStep(step);
    spinBox->setAccelerated(true);
    return spinBox;
}

void RequirementWidget::RegisterFieldWidget(const QString& fieldPath, QWidget* widget)
{
    m_field_widgets.insert(fieldPath, widget);
}

void RequirementWidget::SetOperationMessage(const QString& message, bool success)
{
    m_operation_label->setText(message);
    m_operation_label->setStyleSheet(success ? QStringLiteral("color: #1b7f3b;")
                                             : QStringLiteral("color: #b42318;"));
}

} // namespace RoboSDP::Requirement::Ui
