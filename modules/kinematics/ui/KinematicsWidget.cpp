#include "modules/kinematics/ui/KinematicsWidget.h"

#include "core/infrastructure/ProjectManager.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QStringList>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>

namespace RoboSDP::Kinematics::Ui
{

namespace
{

QString FormatJointVector(const std::vector<double>& values)
{
    QStringList parts;
    for (double value : values)
    {
        parts.push_back(QString::number(value, 'f', 3));
    }
    return QStringLiteral("[%1]").arg(parts.join(QStringLiteral(", ")));
}

QString FormatArray3(const std::array<double, 3>& values, int decimals)
{
    return QStringLiteral("(%1, %2, %3)")
        .arg(values[0], 0, 'f', decimals)
        .arg(values[1], 0, 'f', decimals)
        .arg(values[2], 0, 'f', decimals);
}

QString FormatReadyState(bool ready)
{
    return ready ? QStringLiteral("就绪") : QStringLiteral("未就绪");
}

QString FormatUrdfPreviewSummary(const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene)
{
    if (previewScene.IsEmpty())
    {
        return QStringLiteral("尚未导入");
    }

    return QStringLiteral("模型=%1，节点=%2，连杆段=%3")
        .arg(previewScene.model_name.isEmpty() ? QStringLiteral("未命名模型") : previewScene.model_name)
        .arg(previewScene.nodes.size())
        .arg(previewScene.segments.size());
}

QTableWidgetItem* EnsureItem(QTableWidget* table, int row, int column)
{
    QTableWidgetItem* item = table->item(row, column);
    if (item == nullptr)
    {
        item = new QTableWidgetItem();
        table->setItem(row, column, item);
    }
    return item;
}

double ReadTableDouble(QTableWidget* table, int row, int column, double defaultValue = 0.0)
{
    QTableWidgetItem* item = table->item(row, column);
    return item != nullptr ? item->text().toDouble() : defaultValue;
}

QString ReadTableString(QTableWidget* table, int row, int column, const QString& defaultValue = {})
{
    QTableWidgetItem* item = table->item(row, column);
    return item != nullptr ? item->text().trimmed() : defaultValue;
}

void SetReadOnlyItem(QTableWidgetItem* item)
{
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
}

void PopulatePoseEditors(
    const std::array<QDoubleSpinBox*, 6>& editors,
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    if (editors[0] != nullptr) editors[0]->setValue(pose.position_m[0]);
    if (editors[1] != nullptr) editors[1]->setValue(pose.position_m[1]);
    if (editors[2] != nullptr) editors[2]->setValue(pose.position_m[2]);
    if (editors[3] != nullptr) editors[3]->setValue(pose.rpy_deg[0]);
    if (editors[4] != nullptr) editors[4]->setValue(pose.rpy_deg[1]);
    if (editors[5] != nullptr) editors[5]->setValue(pose.rpy_deg[2]);
}

void PopulateTcpEditors(
    const std::array<QDoubleSpinBox*, 6>& editors,
    const RoboSDP::Kinematics::Dto::TcpFrameDto& frame)
{
    if (editors[0] != nullptr) editors[0]->setValue(frame.translation_m[0]);
    if (editors[1] != nullptr) editors[1]->setValue(frame.translation_m[1]);
    if (editors[2] != nullptr) editors[2]->setValue(frame.translation_m[2]);
    if (editors[3] != nullptr) editors[3]->setValue(frame.rpy_deg[0]);
    if (editors[4] != nullptr) editors[4]->setValue(frame.rpy_deg[1]);
    if (editors[5] != nullptr) editors[5]->setValue(frame.rpy_deg[2]);
}

RoboSDP::Kinematics::Dto::CartesianPoseDto ReadPoseEditors(const std::array<QDoubleSpinBox*, 6>& editors)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = {
        editors[0] != nullptr ? editors[0]->value() : 0.0,
        editors[1] != nullptr ? editors[1]->value() : 0.0,
        editors[2] != nullptr ? editors[2]->value() : 0.0};
    pose.rpy_deg = {
        editors[3] != nullptr ? editors[3]->value() : 0.0,
        editors[4] != nullptr ? editors[4]->value() : 0.0,
        editors[5] != nullptr ? editors[5]->value() : 0.0};
    return pose;
}

RoboSDP::Kinematics::Dto::TcpFrameDto ReadTcpEditors(const std::array<QDoubleSpinBox*, 6>& editors)
{
    RoboSDP::Kinematics::Dto::TcpFrameDto frame;
    frame.translation_m = {
        editors[0] != nullptr ? editors[0]->value() : 0.0,
        editors[1] != nullptr ? editors[1]->value() : 0.0,
        editors[2] != nullptr ? editors[2]->value() : 0.0};
    frame.rpy_deg = {
        editors[3] != nullptr ? editors[3]->value() : 0.0,
        editors[4] != nullptr ? editors[4]->value() : 0.0,
        editors[5] != nullptr ? editors[5]->value() : 0.0};
    return frame;
}

void SetEditorsEnabled(const std::array<QDoubleSpinBox*, 6>& editors, bool enabled)
{
    for (QDoubleSpinBox* editor : editors)
    {
        if (editor != nullptr)
        {
            editor->setEnabled(enabled);
        }
    }
}

void PopulateOptionalPoseEditors(
    QCheckBox* enabledCheckBox,
    const std::array<QDoubleSpinBox*, 6>& editors,
    const std::optional<RoboSDP::Kinematics::Dto::CartesianPoseDto>& pose)
{
    const bool enabled = pose.has_value();
    if (enabledCheckBox != nullptr)
    {
        enabledCheckBox->setChecked(enabled);
    }

    if (enabled)
    {
        PopulatePoseEditors(editors, pose.value());
    }

    SetEditorsEnabled(editors, enabled);
}

std::optional<RoboSDP::Kinematics::Dto::CartesianPoseDto> ReadOptionalPoseEditors(
    QCheckBox* enabledCheckBox,
    const std::array<QDoubleSpinBox*, 6>& editors)
{
    if (enabledCheckBox == nullptr || !enabledCheckBox->isChecked())
    {
        return std::nullopt;
    }

    return ReadPoseEditors(editors);
}

} // namespace

KinematicsWidget::KinematicsWidget(QWidget* parent)
    : QWidget(parent)
    , m_topology_storage(m_repository)
    , m_kinematic_storage(m_repository)
    , m_service(m_kinematic_storage, m_topology_storage, &m_logger)
    , m_state(m_service.CreateDefaultState())
{
    BuildUi();
    PopulateForm(m_state.current_model);
    RefreshBackendDiagnostics();
    RenderResults();
    ConnectDirtyTracking();
    MarkClean();
}

QString KinematicsWidget::ModuleName() const
{
    return QStringLiteral("Kinematics");
}

bool KinematicsWidget::HasUnsavedChanges() const
{
    return m_has_unsaved_changes;
}

RoboSDP::Infrastructure::ProjectSaveItemResult KinematicsWidget::SaveCurrentDraft()
{
    m_state.current_model = CollectModelFromForm();
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto saveResult = m_service.SaveDraft(projectRootPath, m_state);
    SetOperationMessage(saveResult.message, saveResult.IsSuccess());
    if (saveResult.IsSuccess())
    {
        MarkClean();
    }
    return {ModuleName(), saveResult.IsSuccess(), saveResult.message};
}

void KinematicsWidget::ConnectDirtyTracking()
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
    for (QTableWidget* table : findChildren<QTableWidget*>())
    {
        connect(table, &QTableWidget::cellChanged, this, [this](int, int) { MarkDirty(); });
    }
}

void KinematicsWidget::MarkDirty()
{
    m_has_unsaved_changes = true;
}

void KinematicsWidget::MarkClean()
{
    m_has_unsaved_changes = false;
}

void KinematicsWidget::TriggerImportUrdf()
{
    // 中文说明：外部顶部功能区只能触发页面既有入口，避免绕过 KinematicsWidget 的 UI 状态同步与日志输出。
    OnImportUrdfClicked();
}

void KinematicsWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    auto* actionLayout = new QHBoxLayout();
    m_import_urdf_button = new QPushButton(QStringLiteral("导入 URDF"), this);
    m_build_from_topology_button = new QPushButton(QStringLiteral("从 Topology 生成"), this);
    m_run_fk_button = new QPushButton(QStringLiteral("执行 FK"), this);
    m_run_ik_button = new QPushButton(QStringLiteral("执行 IK"), this);
    m_sample_workspace_button = new QPushButton(QStringLiteral("采样工作空间"), this);
    m_save_button = new QPushButton(QStringLiteral("保存草稿"), this);
    m_load_button = new QPushButton(QStringLiteral("重新加载"), this);
    m_build_from_topology_button->setObjectName(QStringLiteral("kinematics_build_from_topology_button"));
    m_run_fk_button->setObjectName(QStringLiteral("kinematics_run_fk_button"));
    m_run_ik_button->setObjectName(QStringLiteral("kinematics_run_ik_button"));
    m_sample_workspace_button->setObjectName(QStringLiteral("kinematics_sample_workspace_button"));
    actionLayout->addWidget(m_import_urdf_button);
    actionLayout->addWidget(m_build_from_topology_button);
    actionLayout->addWidget(m_run_fk_button);
    actionLayout->addWidget(m_run_ik_button);
    actionLayout->addWidget(m_sample_workspace_button);
    actionLayout->addWidget(m_save_button);
    actionLayout->addWidget(m_load_button);
    actionLayout->addStretch();

    m_operation_label = new QLabel(QStringLiteral("就绪：请先保存 Topology，再生成 KinematicModel。"), this);
    m_operation_label->setObjectName(QStringLiteral("kinematics_operation_label"));
    m_operation_label->setWordWrap(true);

    auto* tabs = new QTabWidget(this);
    tabs->setDocumentMode(true);
    // 中文说明：Kinematics 页面按业务域拆为页签，底层模型收集、FK/IK 和 URDF 预览链路保持不变。
    tabs->addTab(CreateScrollableTab(CreateModelGroup()), QStringLiteral("模型与坐标系"));
    tabs->addTab(CreateScrollableTab(CreateDhTableGroup()), QStringLiteral("DH/MDH 参数"));
    tabs->addTab(CreateScrollableTab(CreateJointLimitGroup()), QStringLiteral("关节限位"));
    tabs->addTab(CreateScrollableTab(CreateSolverGroup()), QStringLiteral("FK / IK 求解"));
    tabs->addTab(CreateScrollableTab(CreateResultGroup()), QStringLiteral("结果摘要"));

    rootLayout->addLayout(actionLayout);
    rootLayout->addWidget(m_operation_label);
    rootLayout->addWidget(tabs, 1);

    m_preview_pose_update_timer = new QTimer(this);
    m_preview_pose_update_timer->setSingleShot(true);
    // 中文说明：16ms 对齐约 60 FPS 的交互刷新目标；单次定时器会合并 SpinBox 的高频 valueChanged 信号。
    m_preview_pose_update_timer->setInterval(16);
    connect(m_preview_pose_update_timer, &QTimer::timeout, this, [this]() {
        FlushPreviewPoseUpdate();
    });

    connect(m_import_urdf_button, &QPushButton::clicked, this, [this]() { OnImportUrdfClicked(); });
    connect(m_build_from_topology_button, &QPushButton::clicked, this, [this]() { OnBuildFromTopologyClicked(); });
    connect(m_run_fk_button, &QPushButton::clicked, this, [this]() { OnRunFkClicked(); });
    connect(m_run_ik_button, &QPushButton::clicked, this, [this]() { OnRunIkClicked(); });
    connect(m_sample_workspace_button, &QPushButton::clicked, this, [this]() { OnSampleWorkspaceClicked(); });
    connect(m_save_button, &QPushButton::clicked, this, [this]() { OnSaveDraftClicked(); });
    connect(m_load_button, &QPushButton::clicked, this, [this]() { OnLoadClicked(); });
}

QWidget* KinematicsWidget::CreateScrollableTab(QWidget* contentWidget)
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

QGroupBox* KinematicsWidget::CreateModelGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("模型摘要与坐标系"), this);
    auto* layout = new QVBoxLayout(groupBox);

    auto* formLayout = new QFormLayout();
    m_model_name_edit = new QLineEdit(groupBox);
    m_parameter_convention_combo = new QComboBox(groupBox);
    m_parameter_convention_combo->addItem(QStringLiteral("DH"), QStringLiteral("DH"));
    m_parameter_convention_combo->addItem(QStringLiteral("MDH"), QStringLiteral("MDH"));
    m_topology_ref_edit = new QLineEdit(groupBox);
    m_topology_ref_edit->setReadOnly(true);
    m_requirement_ref_edit = new QLineEdit(groupBox);
    m_requirement_ref_edit->setReadOnly(true);
    formLayout->addRow(QStringLiteral("模型名称"), m_model_name_edit);
    formLayout->addRow(QStringLiteral("参数约定"), m_parameter_convention_combo);
    formLayout->addRow(QStringLiteral("Topology 引用"), m_topology_ref_edit);
    formLayout->addRow(QStringLiteral("Requirement 引用"), m_requirement_ref_edit);
    layout->addLayout(formLayout);

    auto* framesLayout = new QHBoxLayout();
    const QStringList poseLabels {
        QStringLiteral("X [m]"),
        QStringLiteral("Y [m]"),
        QStringLiteral("Z [m]"),
        QStringLiteral("RX [deg]"),
        QStringLiteral("RY [deg]"),
        QStringLiteral("RZ [deg]")};

    auto createFrameEditor = [this, &poseLabels, &framesLayout, groupBox](
                                 const QString& title,
                                 std::array<QDoubleSpinBox*, 6>& editors,
                                 const QString& tip,
                                 QCheckBox** enabledCheckBox = nullptr) {
        auto* frameGroup = new QGroupBox(title, groupBox);
        frameGroup->setToolTip(tip);
        auto* frameLayout = new QVBoxLayout(frameGroup);

        if (enabledCheckBox != nullptr)
        {
            *enabledCheckBox = new QCheckBox(QStringLiteral("启用"), frameGroup);
            (*enabledCheckBox)->setChecked(false);
            frameLayout->addWidget(*enabledCheckBox);
        }

        auto* grid = new QGridLayout();

        for (int index = 0; index < 6; ++index)
        {
            const bool isTranslation = index < 3;
            auto* spinBox = CreateDoubleSpinBox(
                isTranslation ? -10.0 : -360.0,
                isTranslation ? 10.0 : 360.0,
                isTranslation ? 4 : 3,
                isTranslation ? 0.01 : 1.0);
            editors[static_cast<std::size_t>(index)] = spinBox;
            grid->addWidget(new QLabel(poseLabels.at(index), frameGroup), index, 0);
            grid->addWidget(spinBox, index, 1);
        }

        frameLayout->addLayout(grid);
        framesLayout->addWidget(frameGroup, 1);
    };

    // 坐标系编辑器覆盖世界基座、法兰、工具、工件和 TCP 参考系。
    createFrameEditor(QStringLiteral("Base Frame"), m_base_frame_spins, QStringLiteral("机器人基坐标系，相对项目世界坐标系定义。"));
    createFrameEditor(QStringLiteral("Flange Frame"), m_flange_frame_spins, QStringLiteral("法兰坐标系，相对末端连杆坐标系定义。"));
    createFrameEditor(
        QStringLiteral("Tool Frame"),
        m_tool_frame_spins,
        QStringLiteral("工具参考系，可选，用于工艺或规划场景定义。"),
        &m_tool_frame_enabled_check);
    createFrameEditor(
        QStringLiteral("Workpiece Frame"),
        m_workpiece_frame_spins,
        QStringLiteral("工件参考系，可选，用于工艺或规划场景定义。"),
        &m_workpiece_frame_enabled_check);
    createFrameEditor(QStringLiteral("TCP Frame"), m_tcp_frame_spins, QStringLiteral("TCP 坐标系，相对法兰坐标系定义。"));
    layout->addLayout(framesLayout);

    if (m_tool_frame_enabled_check != nullptr)
    {
        SetEditorsEnabled(m_tool_frame_spins, false);
        connect(m_tool_frame_enabled_check, &QCheckBox::toggled, this, [this](bool checked) {
            SetEditorsEnabled(m_tool_frame_spins, checked);
        });
    }

    if (m_workpiece_frame_enabled_check != nullptr)
    {
        SetEditorsEnabled(m_workpiece_frame_spins, false);
        connect(m_workpiece_frame_enabled_check, &QCheckBox::toggled, this, [this](bool checked) {
            SetEditorsEnabled(m_workpiece_frame_spins, checked);
        });
    }

    return groupBox;
}

QGroupBox* KinematicsWidget::CreateDhTableGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("DH/MDH 参数表"), this);
    auto* layout = new QVBoxLayout(groupBox);

    m_dh_table = new QTableWidget(groupBox);
    SetupDhTableColumns();
    layout->addWidget(m_dh_table);
    return groupBox;
}

QGroupBox* KinematicsWidget::CreateJointLimitGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("关节限位表"), this);
    auto* layout = new QVBoxLayout(groupBox);

    auto* hintLabel = new QLabel(
        QStringLiteral("最小入口保留 soft/hard limit 与速度、加速度上限，joint_id 由模型自动生成。"),
        groupBox);
    hintLabel->setWordWrap(true);
    layout->addWidget(hintLabel);

    m_joint_limit_table = new QTableWidget(groupBox);
    SetupJointLimitTableColumns();
    layout->addWidget(m_joint_limit_table);
    return groupBox;
}

QGroupBox* KinematicsWidget::CreateSolverGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("求解配置与输入"), this);
    auto* layout = new QVBoxLayout(groupBox);

    auto* solverLayout = new QFormLayout();
    m_solver_type_combo = new QComboBox(groupBox);
    m_solver_type_combo->addItem(QStringLiteral("数值雅可比转置"), QStringLiteral("numeric_jacobian_transpose"));
    m_branch_policy_combo = new QComboBox(groupBox);
    m_branch_policy_combo->addItem(QStringLiteral("最近种子解"), QStringLiteral("nearest_seed"));
    m_max_iterations_spin = new QSpinBox(groupBox);
    m_max_iterations_spin->setRange(1, 5000);
    m_position_tolerance_spin = CreateDoubleSpinBox(0.001, 1000.0, 3, 0.1);
    m_orientation_tolerance_spin = CreateDoubleSpinBox(0.001, 180.0, 3, 0.1);
    m_step_gain_spin = CreateDoubleSpinBox(0.01, 2.0, 3, 0.01);
    solverLayout->addRow(QStringLiteral("IK 求解器"), m_solver_type_combo);
    solverLayout->addRow(QStringLiteral("分支策略"), m_branch_policy_combo);
    solverLayout->addRow(QStringLiteral("最大迭代次数"), m_max_iterations_spin);
    solverLayout->addRow(QStringLiteral("位置容差 [mm]"), m_position_tolerance_spin);
    solverLayout->addRow(QStringLiteral("姿态容差 [deg]"), m_orientation_tolerance_spin);
    solverLayout->addRow(QStringLiteral("步长增益"), m_step_gain_spin);
    layout->addLayout(solverLayout);

    auto* fkGrid = new QGridLayout();
    fkGrid->addWidget(new QLabel(QStringLiteral("FK 关节输入 [deg]"), groupBox), 0, 0, 1, 6);
    for (int index = 0; index < 6; ++index)
    {
        auto* spinBox = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
        m_fk_joint_spins[static_cast<std::size_t>(index)] = spinBox;
        // 中文说明：FK 关节输入可能在拖动时高频变化，这里只排队刷新，真正计算由 QTimer 节流触发。
        connect(spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
            SchedulePreviewPoseUpdate();
        });
        fkGrid->addWidget(new QLabel(QStringLiteral("J%1").arg(index + 1), groupBox), 1, index);
        fkGrid->addWidget(spinBox, 2, index);
    }
    layout->addLayout(fkGrid);

    auto* ikSeedGrid = new QGridLayout();
    ikSeedGrid->addWidget(new QLabel(QStringLiteral("IK 种子关节 [deg]"), groupBox), 0, 0, 1, 6);
    for (int index = 0; index < 6; ++index)
    {
        auto* spinBox = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
        m_ik_seed_joint_spins[static_cast<std::size_t>(index)] = spinBox;
        ikSeedGrid->addWidget(new QLabel(QStringLiteral("J%1").arg(index + 1), groupBox), 1, index);
        ikSeedGrid->addWidget(spinBox, 2, index);
    }
    layout->addLayout(ikSeedGrid);

    auto* ikTargetGrid = new QGridLayout();
    ikTargetGrid->addWidget(new QLabel(QStringLiteral("IK 目标位姿"), groupBox), 0, 0, 1, 6);
    const QStringList poseLabels {
        QStringLiteral("X [m]"),
        QStringLiteral("Y [m]"),
        QStringLiteral("Z [m]"),
        QStringLiteral("RX [deg]"),
        QStringLiteral("RY [deg]"),
        QStringLiteral("RZ [deg]")};
    for (int index = 0; index < 6; ++index)
    {
        auto* spinBox = CreateDoubleSpinBox(-1000.0, 1000.0, index < 3 ? 4 : 3, 0.01);
        m_ik_target_pose_spins[static_cast<std::size_t>(index)] = spinBox;
        ikTargetGrid->addWidget(new QLabel(poseLabels.at(index), groupBox), 1, index);
        ikTargetGrid->addWidget(spinBox, 2, index);
    }
    layout->addLayout(ikTargetGrid);

    auto* workspaceLayout = new QFormLayout();
    m_workspace_sample_count_spin = new QSpinBox(groupBox);
    m_workspace_sample_count_spin->setRange(1, 5000);
    workspaceLayout->addRow(QStringLiteral("采样点数"), m_workspace_sample_count_spin);
    layout->addLayout(workspaceLayout);

    return groupBox;
}

QGroupBox* KinematicsWidget::CreateResultGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("结果摘要"), this);
    auto* layout = new QVBoxLayout(groupBox);
    m_result_summary_edit = new QPlainTextEdit(groupBox);
    m_result_summary_edit->setReadOnly(true);
    layout->addWidget(m_result_summary_edit);
    return groupBox;
}

QDoubleSpinBox* KinematicsWidget::CreateDoubleSpinBox(
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

void KinematicsWidget::SetupDhTableColumns()
{
    m_dh_table->setColumnCount(5);
    m_dh_table->setHorizontalHeaderLabels({
        QStringLiteral("link_id"),
        QStringLiteral("a [m]"),
        QStringLiteral("alpha [deg]"),
        QStringLiteral("d [m]"),
        QStringLiteral("theta_offset [deg]")});
    m_dh_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void KinematicsWidget::SetupJointLimitTableColumns()
{
    m_joint_limit_table->setColumnCount(7);
    m_joint_limit_table->setHorizontalHeaderLabels({
        QStringLiteral("joint_id"),
        QStringLiteral("soft_min"),
        QStringLiteral("soft_max"),
        QStringLiteral("hard_min"),
        QStringLiteral("hard_max"),
        QStringLiteral("max_velocity"),
        QStringLiteral("max_acceleration")});
    m_joint_limit_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

RoboSDP::Kinematics::Dto::KinematicModelDto KinematicsWidget::CollectModelFromForm() const
{
    RoboSDP::Kinematics::Dto::KinematicModelDto model = m_state.current_model;
    model.meta.name = m_model_name_edit->text().trimmed();
    model.parameter_convention = m_parameter_convention_combo->currentData().toString();
    model.meta.topology_ref = m_topology_ref_edit->text().trimmed();
    model.meta.requirement_ref = m_requirement_ref_edit->text().trimmed();
    model.base_frame = ReadPoseEditors(m_base_frame_spins);
    model.flange_frame = ReadPoseEditors(m_flange_frame_spins);
    model.tool_frame = ReadOptionalPoseEditors(m_tool_frame_enabled_check, m_tool_frame_spins);
    model.workpiece_frame = ReadOptionalPoseEditors(m_workpiece_frame_enabled_check, m_workpiece_frame_spins);
    model.tcp_frame = ReadTcpEditors(m_tcp_frame_spins);

    model.links.clear();
    for (int row = 0; row < m_dh_table->rowCount(); ++row)
    {
        model.links.push_back({
            ReadTableString(m_dh_table, row, 0),
            ReadTableDouble(m_dh_table, row, 1),
            ReadTableDouble(m_dh_table, row, 2),
            ReadTableDouble(m_dh_table, row, 3),
            ReadTableDouble(m_dh_table, row, 4)});
    }

    model.joint_limits.clear();
    for (int row = 0; row < m_joint_limit_table->rowCount(); ++row)
    {
        model.joint_limits.push_back({
            ReadTableString(m_joint_limit_table, row, 0),
            {ReadTableDouble(m_joint_limit_table, row, 1), ReadTableDouble(m_joint_limit_table, row, 2)},
            {ReadTableDouble(m_joint_limit_table, row, 3), ReadTableDouble(m_joint_limit_table, row, 4)},
            ReadTableDouble(m_joint_limit_table, row, 5, 180.0),
            ReadTableDouble(m_joint_limit_table, row, 6, 360.0)});
    }

    model.ik_solver_config.solver_type = m_solver_type_combo->currentData().toString();
    model.ik_solver_config.branch_policy = m_branch_policy_combo->currentData().toString();
    model.ik_solver_config.max_iterations = m_max_iterations_spin->value();
    model.ik_solver_config.position_tolerance_mm = m_position_tolerance_spin->value();
    model.ik_solver_config.orientation_tolerance_deg = m_orientation_tolerance_spin->value();
    model.ik_solver_config.step_gain = m_step_gain_spin->value();
    model.joint_count = static_cast<int>(model.links.size());
    return model;
}

void KinematicsWidget::PopulateForm(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    m_model_name_edit->setText(model.meta.name);
    const int conventionIndex = m_parameter_convention_combo->findData(model.parameter_convention);
    m_parameter_convention_combo->setCurrentIndex(conventionIndex >= 0 ? conventionIndex : 0);
    m_topology_ref_edit->setText(model.meta.topology_ref);
    m_requirement_ref_edit->setText(model.meta.requirement_ref);
    PopulatePoseEditors(m_base_frame_spins, model.base_frame);
    PopulatePoseEditors(m_flange_frame_spins, model.flange_frame);
    PopulateOptionalPoseEditors(m_tool_frame_enabled_check, m_tool_frame_spins, model.tool_frame);
    PopulateOptionalPoseEditors(m_workpiece_frame_enabled_check, m_workpiece_frame_spins, model.workpiece_frame);
    PopulateTcpEditors(m_tcp_frame_spins, model.tcp_frame);

    m_dh_table->setRowCount(static_cast<int>(model.links.size()));
    for (int row = 0; row < static_cast<int>(model.links.size()); ++row)
    {
        const auto& link = model.links.at(static_cast<std::size_t>(row));
        auto* idItem = EnsureItem(m_dh_table, row, 0);
        idItem->setText(link.link_id);
        SetReadOnlyItem(idItem);
        EnsureItem(m_dh_table, row, 1)->setText(QString::number(link.a, 'f', 4));
        EnsureItem(m_dh_table, row, 2)->setText(QString::number(link.alpha, 'f', 3));
        EnsureItem(m_dh_table, row, 3)->setText(QString::number(link.d, 'f', 4));
        EnsureItem(m_dh_table, row, 4)->setText(QString::number(link.theta_offset, 'f', 3));
    }

    m_joint_limit_table->setRowCount(static_cast<int>(model.joint_limits.size()));
    for (int row = 0; row < static_cast<int>(model.joint_limits.size()); ++row)
    {
        const auto& limit = model.joint_limits.at(static_cast<std::size_t>(row));
        auto* idItem = EnsureItem(m_joint_limit_table, row, 0);
        idItem->setText(limit.joint_id);
        SetReadOnlyItem(idItem);
        EnsureItem(m_joint_limit_table, row, 1)->setText(QString::number(limit.soft_limit[0], 'f', 3));
        EnsureItem(m_joint_limit_table, row, 2)->setText(QString::number(limit.soft_limit[1], 'f', 3));
        EnsureItem(m_joint_limit_table, row, 3)->setText(QString::number(limit.hard_limit[0], 'f', 3));
        EnsureItem(m_joint_limit_table, row, 4)->setText(QString::number(limit.hard_limit[1], 'f', 3));
        EnsureItem(m_joint_limit_table, row, 5)->setText(QString::number(limit.max_velocity, 'f', 3));
        EnsureItem(m_joint_limit_table, row, 6)->setText(QString::number(limit.max_acceleration, 'f', 3));
    }

    const int solverIndex = m_solver_type_combo->findData(model.ik_solver_config.solver_type);
    m_solver_type_combo->setCurrentIndex(solverIndex >= 0 ? solverIndex : 0);
    const int branchIndex = m_branch_policy_combo->findData(model.ik_solver_config.branch_policy);
    m_branch_policy_combo->setCurrentIndex(branchIndex >= 0 ? branchIndex : 0);
    m_max_iterations_spin->setValue(model.ik_solver_config.max_iterations);
    m_position_tolerance_spin->setValue(model.ik_solver_config.position_tolerance_mm);
    m_orientation_tolerance_spin->setValue(model.ik_solver_config.orientation_tolerance_deg);
    m_step_gain_spin->setValue(model.ik_solver_config.step_gain);
    m_workspace_sample_count_spin->setValue(128);
}

void KinematicsWidget::RefreshBackendDiagnostics()
{
    m_backend_diagnostic = m_service.InspectBackendBuildContext(m_state.current_model);
}

void KinematicsWidget::RenderResults()
{
    QStringList lines;
    lines.push_back(QStringLiteral("模型：%1").arg(m_state.current_model.meta.name));
    lines.push_back(QStringLiteral("参数约定：%1").arg(m_state.current_model.parameter_convention));
    lines.push_back(QStringLiteral("Base Frame 位置[m] = %1").arg(FormatArray3(m_state.current_model.base_frame.position_m, 4)));
    lines.push_back(QStringLiteral("Flange Frame 位置[m] = %1").arg(FormatArray3(m_state.current_model.flange_frame.position_m, 4)));
    lines.push_back(QStringLiteral("Tool Frame：%1").arg(m_state.current_model.tool_frame.has_value()
        ? FormatArray3(m_state.current_model.tool_frame->position_m, 4)
        : QStringLiteral("未启用")));
    lines.push_back(QStringLiteral("Workpiece Frame：%1").arg(m_state.current_model.workpiece_frame.has_value()
        ? FormatArray3(m_state.current_model.workpiece_frame->position_m, 4)
        : QStringLiteral("未启用")));
    lines.push_back(QStringLiteral("TCP Frame 偏移[m] = %1").arg(FormatArray3(m_state.current_model.tcp_frame.translation_m, 4)));
    lines.push_back(QStringLiteral("URDF 预览：%1").arg(FormatUrdfPreviewSummary(m_urdf_preview_scene)));
    if (!m_urdf_preview_scene.IsEmpty())
    {
        lines.push_back(QStringLiteral("URDF 文件：%1").arg(m_urdf_preview_scene.urdf_file_path));
    }

    lines.push_back(QString());
    lines.push_back(QStringLiteral("[模型诊断]"));
    lines.push_back(QStringLiteral("backend_id = %1").arg(m_backend_diagnostic.backend_id));
    lines.push_back(QStringLiteral("语义归一 = %1").arg(FormatReadyState(m_backend_diagnostic.status.normalized_semantics_ready)));
    lines.push_back(QStringLiteral("Frame 语义 = %1").arg(FormatReadyState(m_backend_diagnostic.status.frame_semantics_ready)));
    lines.push_back(QStringLiteral("Joint 顺序 = %1").arg(FormatReadyState(m_backend_diagnostic.status.joint_order_ready)));
    lines.push_back(QStringLiteral("Build-Context = %1").arg(FormatReadyState(m_backend_diagnostic.status.build_context_ready)));
    lines.push_back(QStringLiteral("共享内核 = %1").arg(FormatReadyState(m_backend_diagnostic.status.shared_robot_kernel_ready)));
    if (!m_backend_diagnostic.status.status_message.isEmpty())
    {
        lines.push_back(QStringLiteral("诊断摘要 = %1").arg(m_backend_diagnostic.status.status_message));
    }

    if (m_state.last_fk_result.success)
    {
        const auto& pose = m_state.last_fk_result.tcp_pose;
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[FK] 成功"));
        lines.push_back(QStringLiteral("TCP 位置[m] = %1").arg(FormatArray3(pose.position_m, 4)));
        lines.push_back(QStringLiteral("TCP 姿态[deg] = %1").arg(FormatArray3(pose.rpy_deg, 3)));
        lines.push_back(QStringLiteral("关节输入 = %1").arg(FormatJointVector(m_state.last_fk_result.joint_positions_deg)));
    }
    else
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[FK] %1").arg(m_state.last_fk_result.message.isEmpty()
            ? QStringLiteral("尚未执行")
            : m_state.last_fk_result.message));
    }

    if (m_state.last_ik_result.success)
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[IK] 成功"));
        lines.push_back(QStringLiteral("解 = %1").arg(FormatJointVector(m_state.last_ik_result.joint_positions_deg)));
        lines.push_back(
            QStringLiteral("位置误差 = %1 mm，姿态误差 = %2 deg，迭代次数 = %3")
                .arg(m_state.last_ik_result.position_error_mm, 0, 'f', 3)
                .arg(m_state.last_ik_result.orientation_error_deg, 0, 'f', 3)
                .arg(m_state.last_ik_result.iteration_count));
    }
    else
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[IK] %1").arg(m_state.last_ik_result.message.isEmpty()
            ? QStringLiteral("尚未执行")
            : m_state.last_ik_result.message));
    }

    if (m_state.last_workspace_result.success)
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[Workspace] 成功"));
        lines.push_back(
            QStringLiteral("请求采样 = %1，可达点 = %2，最大半径 = %3 m")
                .arg(m_state.last_workspace_result.requested_sample_count)
                .arg(m_state.last_workspace_result.reachable_sample_count)
                .arg(m_state.last_workspace_result.max_radius_m, 0, 'f', 4));
        lines.push_back(QStringLiteral("包围盒最小点[m] = %1").arg(FormatArray3(m_state.last_workspace_result.min_position_m, 4)));
        lines.push_back(QStringLiteral("包围盒最大点[m] = %1").arg(FormatArray3(m_state.last_workspace_result.max_position_m, 4)));
    }
    else
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[Workspace] %1").arg(m_state.last_workspace_result.message.isEmpty()
            ? QStringLiteral("尚未执行")
            : m_state.last_workspace_result.message));
    }

    m_result_summary_edit->setPlainText(lines.join(QLatin1Char('\n')));
}

void KinematicsWidget::SetOperationMessage(const QString& message, bool success, bool warning)
{
    m_operation_label->setText(message);
    if (warning)
    {
        m_operation_label->setStyleSheet(QStringLiteral("color: #b54708; font-weight: 600;"));
        return;
    }

    m_operation_label->setStyleSheet(success ? QStringLiteral("color: #1b7f3b;")
                                             : QStringLiteral("color: #b42318;"));
}

void KinematicsWidget::EmitTelemetryStatus(
    const QString& engineName,
    const QString& message,
    bool warning)
{
    const QString prefix = warning
        ? QStringLiteral("[警告]")
        : QStringLiteral("[计算完成]");
    emit TelemetryStatusGenerated(
        QStringLiteral("%1 引擎: %2。%3").arg(prefix, engineName, message),
        warning);
}

void KinematicsWidget::SchedulePreviewPoseUpdate()
{
    if (m_preview_pose_update_timer == nullptr ||
        m_urdf_preview_scene.IsEmpty() ||
        m_urdf_preview_model.urdf_source_path.trimmed().isEmpty())
    {
        return;
    }

    // 中文说明：重复 valueChanged 只会重启单次定时器，合并为最多约 60 FPS 的 FK 预览刷新。
    m_preview_pose_update_timer->start();
}

void KinematicsWidget::FlushPreviewPoseUpdate()
{
    if (m_urdf_preview_scene.IsEmpty() ||
        m_urdf_preview_model.urdf_source_path.trimmed().isEmpty())
    {
        return;
    }

    const int expectedJointCount = m_urdf_preview_model.joint_count;
    if (expectedJointCount <= 0)
    {
        return;
    }

    if (expectedJointCount > static_cast<int>(m_fk_joint_spins.size()))
    {
        emit LogMessageGenerated(
            QStringLiteral("[Kinematics][Warning] 当前 URDF 预览关节数量=%1，超过页面 FK 输入数量=%2，已跳过动态预览刷新。")
                .arg(expectedJointCount)
                .arg(static_cast<int>(m_fk_joint_spins.size())));
        return;
    }

    std::vector<double> jointPositionsDeg;
    jointPositionsDeg.reserve(static_cast<std::size_t>(expectedJointCount));
    for (int index = 0; index < expectedJointCount; ++index)
    {
        const auto spinIndex = static_cast<std::size_t>(index);
        jointPositionsDeg.push_back(m_fk_joint_spins[spinIndex] != nullptr
            ? m_fk_joint_spins[spinIndex]->value()
            : 0.0);
    }

    const auto updateResult = m_service.UpdatePreviewPoses(m_urdf_preview_model, jointPositionsDeg);
    if (!updateResult.IsSuccess())
    {
        emit LogMessageGenerated(QStringLiteral("[Kinematics][Warning] %1").arg(updateResult.message));
        return;
    }

    emit PreviewPosesUpdated(updateResult.link_world_poses);
}

void KinematicsWidget::ClearUrdfPreviewContext()
{
    if (m_preview_pose_update_timer != nullptr)
    {
        m_preview_pose_update_timer->stop();
    }

    // 中文说明：从 Topology/JSON 切换模型结构时，旧 URDF 的 Mesh Actor 不能继续留在中央视图里。
    m_urdf_preview_scene = RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto {};
    m_urdf_preview_model = RoboSDP::Kinematics::Dto::KinematicModelDto {};
    emit UrdfPreviewSceneGenerated(m_urdf_preview_scene);
}

std::vector<double> KinematicsWidget::CollectJointInputs(const std::array<QDoubleSpinBox*, 6>& spinBoxes) const
{
    std::vector<double> values;
    values.reserve(spinBoxes.size());
    for (QDoubleSpinBox* spinBox : spinBoxes)
    {
        values.push_back(spinBox != nullptr ? spinBox->value() : 0.0);
    }
    return values;
}

void KinematicsWidget::FillIkTargetFromFkResult()
{
    if (!m_state.last_fk_result.success)
    {
        return;
    }

    const auto& pose = m_state.last_fk_result.tcp_pose;
    m_ik_target_pose_spins[0]->setValue(pose.position_m[0]);
    m_ik_target_pose_spins[1]->setValue(pose.position_m[1]);
    m_ik_target_pose_spins[2]->setValue(pose.position_m[2]);
    m_ik_target_pose_spins[3]->setValue(pose.rpy_deg[0]);
    m_ik_target_pose_spins[4]->setValue(pose.rpy_deg[1]);
    m_ik_target_pose_spins[5]->setValue(pose.rpy_deg[2]);

    for (std::size_t index = 0; index < m_ik_seed_joint_spins.size() &&
                                index < m_state.last_fk_result.joint_positions_deg.size();
         ++index)
    {
        m_ik_seed_joint_spins[index]->setValue(m_state.last_fk_result.joint_positions_deg[index]);
    }
}

void KinematicsWidget::OnImportUrdfClicked()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const QString initialDirectory = projectRootPath.trimmed().isEmpty()
        ? QDir::currentPath()
        : projectRootPath;
    const QString urdfFilePath = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("选择 URDF 文件"),
        initialDirectory,
        QStringLiteral("URDF Files (*.urdf);;XML Files (*.xml);;All Files (*.*)"));

    if (urdfFilePath.isEmpty())
    {
        return;
    }

    const auto importResult = m_service.ImportUrdfPreview(urdfFilePath);
    if (importResult.IsSuccess())
    {
        m_urdf_preview_scene = importResult.preview_scene;
        m_urdf_preview_model = importResult.preview_model;
        RefreshBackendDiagnostics();
        RenderResults();
        emit UrdfPreviewSceneGenerated(m_urdf_preview_scene);
        SchedulePreviewPoseUpdate();
        MarkDirty();
    }

    SetOperationMessage(importResult.message, importResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(importResult.message));
}

void KinematicsWidget::OnBuildFromTopologyClicked()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto buildResult = m_service.BuildFromTopology(projectRootPath);
    if (buildResult.IsSuccess())
    {
        m_state = buildResult.state;
        ClearUrdfPreviewContext();
        PopulateForm(m_state.current_model);
        RefreshBackendDiagnostics();
        RenderResults();
        MarkDirty();
    }

    SetOperationMessage(buildResult.message, buildResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(buildResult.message));
}

void KinematicsWidget::OnRunFkClicked()
{
    m_state.current_model = CollectModelFromForm();
    RefreshBackendDiagnostics();

    RoboSDP::Kinematics::Dto::FkRequestDto request;
    request.joint_positions_deg = CollectJointInputs(m_fk_joint_spins);
    m_state.last_fk_result = m_service.SolveFk(m_state.current_model, request);
    MarkDirty();
    FillIkTargetFromFkResult();
    RenderResults();

    const bool warning = !m_state.last_fk_result.success;
    SetOperationMessage(m_state.last_fk_result.message, m_state.last_fk_result.success, warning);
    EmitTelemetryStatus(
        QStringLiteral("Pinocchio 共享内核"),
        m_state.last_fk_result.message,
        warning);
    emit LogMessageGenerated(QStringLiteral("%1 %2").arg(
        warning ? QStringLiteral("[Kinematics][Warning]") : QStringLiteral("[Kinematics]"),
        m_state.last_fk_result.message));
}

void KinematicsWidget::OnRunIkClicked()
{
    m_state.current_model = CollectModelFromForm();
    RefreshBackendDiagnostics();

    RoboSDP::Kinematics::Dto::IkRequestDto request;
    request.seed_joint_positions_deg = CollectJointInputs(m_ik_seed_joint_spins);
    request.target_pose.position_m = {
        m_ik_target_pose_spins[0]->value(),
        m_ik_target_pose_spins[1]->value(),
        m_ik_target_pose_spins[2]->value()};
    request.target_pose.rpy_deg = {
        m_ik_target_pose_spins[3]->value(),
        m_ik_target_pose_spins[4]->value(),
        m_ik_target_pose_spins[5]->value()};

    m_state.last_ik_result = m_service.SolveIk(m_state.current_model, request);
    MarkDirty();
    RenderResults();

    const bool warning = !m_state.last_ik_result.success;
    SetOperationMessage(m_state.last_ik_result.message, m_state.last_ik_result.success, warning);
    EmitTelemetryStatus(
        QStringLiteral("Pinocchio 数值 IK"),
        m_state.last_ik_result.message,
        warning);
    emit LogMessageGenerated(QStringLiteral("%1 %2").arg(
        warning ? QStringLiteral("[Kinematics][Warning]") : QStringLiteral("[Kinematics]"),
        m_state.last_ik_result.message));
}

void KinematicsWidget::OnSampleWorkspaceClicked()
{
    m_state.current_model = CollectModelFromForm();
    RefreshBackendDiagnostics();

    RoboSDP::Kinematics::Dto::WorkspaceRequestDto request;
    request.sample_count = m_workspace_sample_count_spin->value();
    m_state.last_workspace_result = m_service.SampleWorkspace(m_state.current_model, request);
    MarkDirty();
    RenderResults();

    const bool warning = !m_state.last_workspace_result.success;
    SetOperationMessage(m_state.last_workspace_result.message, m_state.last_workspace_result.success, warning);
    EmitTelemetryStatus(
        QStringLiteral("Pinocchio 共享内核"),
        m_state.last_workspace_result.message,
        warning);
    emit LogMessageGenerated(QStringLiteral("%1 %2").arg(
        warning ? QStringLiteral("[Kinematics][Warning]") : QStringLiteral("[Kinematics]"),
        m_state.last_workspace_result.message));
}

void KinematicsWidget::OnSaveDraftClicked()
{
    const auto saveResult = SaveCurrentDraft();
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(saveResult.message));
}

void KinematicsWidget::OnLoadClicked()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto loadResult = m_service.LoadDraft(projectRootPath);
    if (loadResult.IsSuccess())
    {
        m_state = loadResult.state;
        ClearUrdfPreviewContext();
        PopulateForm(m_state.current_model);
        RefreshBackendDiagnostics();
        RenderResults();
        MarkClean();
    }

    SetOperationMessage(loadResult.message, loadResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(loadResult.message));
}

} // namespace RoboSDP::Kinematics::Ui
