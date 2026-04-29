#include "modules/kinematics/ui/KinematicsWidget.h"

#include "core/infrastructure/ProjectManager.h"

#include <QAbstractItemView>
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
#include <QMessageBox>
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

QString FormatMasterModelType(const QString& masterModelType)
{
    return masterModelType == QStringLiteral("urdf")
        ? QStringLiteral("URDF")
        : QStringLiteral("DH/MDH");
}

QString FormatDerivedModelState(const QString& derivedModelState)
{
    if (derivedModelState == QStringLiteral("stale"))
    {
        return QStringLiteral("待刷新");
    }
    if (derivedModelState == QStringLiteral("not_available"))
    {
        return QStringLiteral("未生成");
    }
    return QStringLiteral("已同步");
}

QString FormatPreviewSourceMode(const QString& previewSourceMode)
{
    if (previewSourceMode == QStringLiteral("urdf_preview"))
    {
        return QStringLiteral("工程模型预览");
    }
    if (previewSourceMode == QStringLiteral("dh_preview"))
    {
        return QStringLiteral("DH/MDH 骨架预览");
    }
    return QStringLiteral("未生成");
}

QString FormatMasterSwitchState(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const QString& previewSourceMode)
{
    if (model.master_model_type == QStringLiteral("urdf") && !model.dh_editable)
    {
        return QStringLiteral("URDF 主模型 / DH 草案只读");
    }
    if (model.master_model_type == QStringLiteral("dh_mdh") && model.dh_editable)
    {
        return previewSourceMode == QStringLiteral("dh_preview")
            ? QStringLiteral("DH/MDH 主模型 / 骨架主链驱动")
            : QStringLiteral("DH/MDH 主模型 / 等待骨架同步");
    }
    return QStringLiteral("状态待确认");
}

QString FormatDhDraftExtractionLevel(const QString& extractionLevel)
{
    if (extractionLevel == QStringLiteral("full"))
    {
        return QStringLiteral("完整提取");
    }
    if (extractionLevel == QStringLiteral("partial"))
    {
        return QStringLiteral("部分提取");
    }
    if (extractionLevel == QStringLiteral("diagnostic_only"))
    {
        return QStringLiteral("仅诊断展示");
    }
    return QStringLiteral("不适用");
}

QString FormatUrdfMasterSourceType(const QString& sourceType)
{
    if (sourceType == QStringLiteral("original_imported"))
    {
        return QStringLiteral("原始导入");
    }
    if (sourceType == QStringLiteral("project_derived"))
    {
        return QStringLiteral("项目派生");
    }
    return QStringLiteral("不适用");
}

QString FormatPreviewSceneSummary(const RoboSDP::Kinematics::Ui::PreviewSceneDto& previewScene)
{
    if (previewScene.IsEmpty())
    {
        return QStringLiteral("尚未生成");
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
    // 找到所有 FK 关节角度输入框 (m_fk_joint_spins)
    for (int i = 0; i < 6; ++i) {
        if (m_fk_joint_spins[i]) {
            connect(m_fk_joint_spins[i], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                this, [this](double) {
                    // 当角度改变时，立即更新 3D 预览
                    UpdateKinematicsPreview(); 
                });
        }
    }
}

void KinematicsWidget::MarkDirty()
{
    if (m_is_populating_form)
    {
        return;
    }
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
    m_promote_dh_draft_button = new QPushButton(QStringLiteral("切换为 DH/MDH 主模型"), this);
    m_switch_to_urdf_master_button = new QPushButton(QStringLiteral("切换为 URDF 主模型"), this);
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
    actionLayout->addWidget(m_promote_dh_draft_button);
    actionLayout->addWidget(m_switch_to_urdf_master_button);
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
    // 中文说明：Kinematics 页面按业务域拆为页签，底层模型收集、FK/IK 和中央预览链路保持不变。
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
    // 中文说明：16ms 对齐约 60 FPS 的交互刷新目标；当前阶段统一用于 URDF 位姿更新与 DH 骨架重建两类预览。
    m_preview_pose_update_timer->setInterval(16);
    connect(m_preview_pose_update_timer, &QTimer::timeout, this, [this]() {
        FlushActivePreviewUpdate();
    });

    connect(m_import_urdf_button, &QPushButton::clicked, this, [this]() { OnImportUrdfClicked(); });
    connect(m_build_from_topology_button, &QPushButton::clicked, this, [this]() { OnBuildFromTopologyClicked(); });
    connect(m_promote_dh_draft_button, &QPushButton::clicked, this, [this]() { OnPromoteDhDraftToMasterClicked(); });
    connect(m_switch_to_urdf_master_button, &QPushButton::clicked, this, [this]() { OnSwitchToUrdfMasterClicked(); });
    connect(m_run_fk_button, &QPushButton::clicked, this, [this]() { OnRunFkClicked(); });
    connect(m_run_ik_button, &QPushButton::clicked, this, [this]() { OnRunIkClicked(); });
    connect(m_sample_workspace_button, &QPushButton::clicked, this, [this]() { OnSampleWorkspaceClicked(); });
    connect(m_save_button, &QPushButton::clicked, this, [this]() { OnSaveDraftClicked(); });
    connect(m_load_button, &QPushButton::clicked, this, [this]() { OnLoadClicked(); });

    // 中文说明：以下连接显式覆盖会影响骨架几何语义的输入项，确保 DH/MDH 表、Base/TCP 和参数约定变化时能刷新中央骨架。
    connect(
        m_parameter_convention_combo,
        static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this,
        [this](int) { ScheduleDhPreviewSceneUpdate(); });
    connect(m_dh_table, &QTableWidget::cellChanged, this, [this](int, int) {
        ScheduleDhPreviewSceneUpdate();
    });

    auto connectPreviewEditors = [this](const std::array<QDoubleSpinBox*, 6>& editors) {
        for (QDoubleSpinBox* editor : editors)
        {
            if (editor == nullptr)
            {
                continue;
            }
            connect(editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
                ScheduleDhPreviewSceneUpdate();
            });
        }
    };
    connectPreviewEditors(m_base_frame_spins);
    connectPreviewEditors(m_flange_frame_spins);
    connectPreviewEditors(m_tcp_frame_spins);
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
    m_master_model_mode_label = new QLabel(groupBox);
    m_derived_model_state_label = new QLabel(groupBox);
    m_preview_source_label = new QLabel(groupBox);
    m_master_switch_state_label = new QLabel(groupBox);
    m_urdf_source_type_label = new QLabel(groupBox);
    m_dh_draft_level_label = new QLabel(groupBox);
    m_dh_draft_status_label = new QLabel(groupBox);
    formLayout->addRow(QStringLiteral("模型名称"), m_model_name_edit);
    formLayout->addRow(QStringLiteral("参数约定"), m_parameter_convention_combo);
    formLayout->addRow(QStringLiteral("Topology 引用"), m_topology_ref_edit);
    formLayout->addRow(QStringLiteral("Requirement 引用"), m_requirement_ref_edit);
    formLayout->addRow(QStringLiteral("草稿主模型"), m_master_model_mode_label);
    formLayout->addRow(QStringLiteral("派生状态"), m_derived_model_state_label);
    formLayout->addRow(QStringLiteral("中央预览来源"), m_preview_source_label);
    formLayout->addRow(QStringLiteral("主模型切换状态"), m_master_switch_state_label);
    formLayout->addRow(QStringLiteral("URDF 来源类型"), m_urdf_source_type_label);
    formLayout->addRow(QStringLiteral("DH 草案级别"), m_dh_draft_level_label);
    formLayout->addRow(QStringLiteral("参数表状态"), m_dh_draft_status_label);
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

    m_dh_readonly_banner_label = new QLabel(groupBox);
    m_dh_readonly_banner_label->setWordWrap(true);
    m_dh_readonly_banner_label->setStyleSheet(QStringLiteral("color: #b54708; background: #fff7ed; border: 1px solid #fdba74; padding: 6px;"));
    m_dh_readonly_banner_label->hide();
    layout->addWidget(m_dh_readonly_banner_label);

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
            ScheduleActivePreviewUpdate();
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
    if (model.master_model_type.trimmed().isEmpty())
    {
        model.master_model_type = QStringLiteral("dh_mdh");
    }
    if (model.derived_model_state.trimmed().isEmpty())
    {
        model.derived_model_state = QStringLiteral("fresh");
    }
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
    if (model.master_model_type == QStringLiteral("urdf"))
    {
        if (model.dh_draft_extraction_level.trimmed().isEmpty())
        {
            model.dh_draft_extraction_level = QStringLiteral("diagnostic_only");
        }
        model.dh_draft_readonly_reason =
            QStringLiteral("当前 DH/MDH 参数表由 URDF 主模型提取，仅用于诊断展示。若需继续参数化设计，请显式切换为 DH/MDH 主模型模式。");
        if (model.conversion_diagnostics.trimmed().isEmpty())
        {
            model.conversion_diagnostics =
                QStringLiteral("当前草稿由 URDF 主模型提取，仅用于 DH/MDH 诊断展示。");
        }
    }
    else
    {
        model.dh_draft_extraction_level.clear();
        model.dh_draft_readonly_reason.clear();
        model.conversion_diagnostics = QStringLiteral("当前草稿以 DH/MDH 参数为主模型。");
    }
    return model;
}

void KinematicsWidget::PopulateForm(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    m_is_populating_form = true;
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
    m_is_populating_form = false;
    RefreshEditingState();
    RefreshModelStatusLabels();
}

void KinematicsWidget::RefreshEditingState()
{
    const bool dhEditable = m_state.current_model.dh_editable;
    const bool isUrdfDraftReadOnly =
        !dhEditable && m_state.current_model.master_model_type == QStringLiteral("urdf");
    const QString urdfMasterCandidatePath = ResolveOriginalImportedUrdfMasterPath().trimmed().isEmpty()
        ? ResolveDerivedUrdfMasterPath()
        : ResolveOriginalImportedUrdfMasterPath();
    const bool canSwitchBackToUrdf =
        m_state.current_model.master_model_type == QStringLiteral("dh_mdh") &&
        !urdfMasterCandidatePath.trimmed().isEmpty() &&
        QFileInfo::exists(urdfMasterCandidatePath);

    if (m_parameter_convention_combo != nullptr)
    {
        m_parameter_convention_combo->setEnabled(dhEditable);
    }

    if (m_dh_table != nullptr)
    {
        m_dh_table->setEditTriggers(
            dhEditable ? QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed | QAbstractItemView::AnyKeyPressed
                       : QAbstractItemView::NoEditTriggers);
    }

    if (m_joint_limit_table != nullptr)
    {
        m_joint_limit_table->setEditTriggers(
            dhEditable ? QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed | QAbstractItemView::AnyKeyPressed
                       : QAbstractItemView::NoEditTriggers);
    }

    auto applyEditorsEnabled = [dhEditable](const std::array<QDoubleSpinBox*, 6>& editors) {
        for (QDoubleSpinBox* editor : editors)
        {
            if (editor != nullptr)
            {
                editor->setEnabled(dhEditable);
            }
        }
    };
    applyEditorsEnabled(m_base_frame_spins);
    applyEditorsEnabled(m_flange_frame_spins);
    applyEditorsEnabled(m_tcp_frame_spins);

    if (m_tool_frame_enabled_check != nullptr)
    {
        m_tool_frame_enabled_check->setEnabled(dhEditable);
        SetEditorsEnabled(m_tool_frame_spins, dhEditable && m_tool_frame_enabled_check->isChecked());
    }
    if (m_workpiece_frame_enabled_check != nullptr)
    {
        m_workpiece_frame_enabled_check->setEnabled(dhEditable);
        SetEditorsEnabled(m_workpiece_frame_spins, dhEditable && m_workpiece_frame_enabled_check->isChecked());
    }

    if (m_promote_dh_draft_button != nullptr)
    {
        const bool canPromote =
            isUrdfDraftReadOnly &&
            !m_state.current_model.links.empty();
        m_promote_dh_draft_button->setEnabled(canPromote);
        m_promote_dh_draft_button->setToolTip(
            canPromote
                ? QStringLiteral("将当前 URDF 提取草案提升为可编辑的 DH/MDH 主模型，并开始由参数表驱动中央骨架与派生 URDF。")
                : QStringLiteral("仅当当前主模型为 URDF，且已提取出可展示的 DH/MDH 草案时可执行此切换。"));
    }

    if (m_switch_to_urdf_master_button != nullptr)
    {
        m_switch_to_urdf_master_button->setEnabled(canSwitchBackToUrdf);
        m_switch_to_urdf_master_button->setToolTip(
            canSwitchBackToUrdf
                ? QStringLiteral("将当前项目回切到 URDF 主模型模式，并重新以工程模型预览驱动中央视图。")
                : QStringLiteral("仅当当前主模型为 DH/MDH，且存在可用的派生 URDF 文件时可执行回切。"));
    }

    if (m_dh_readonly_banner_label != nullptr)
    {
        m_dh_readonly_banner_label->setVisible(isUrdfDraftReadOnly);
        m_dh_readonly_banner_label->setText(
            isUrdfDraftReadOnly
                ? m_state.current_model.dh_draft_readonly_reason
                : QString());
    }
}

void KinematicsWidget::RefreshBackendDiagnostics()
{
    const bool hasPreviewModel =
        !m_preview_scene.IsEmpty() &&
        (!m_preview_model.urdf_source_path.trimmed().isEmpty() ||
          m_preview_source_mode == QStringLiteral("dh_preview"));
    m_backend_diagnostic = m_service.InspectBackendBuildContext(
        hasPreviewModel ? m_preview_model : m_state.current_model);
}

void KinematicsWidget::RenderResults()
{
    QStringList lines;
    lines.push_back(QStringLiteral("模型：%1").arg(m_state.current_model.meta.name));
    lines.push_back(QStringLiteral("草稿主模型：%1").arg(FormatMasterModelType(m_state.current_model.master_model_type)));
    lines.push_back(QStringLiteral("派生状态：%1").arg(FormatDerivedModelState(m_state.current_model.derived_model_state)));
    lines.push_back(QStringLiteral("中央预览来源：%1").arg(FormatPreviewSourceMode(m_preview_source_mode)));
    lines.push_back(QStringLiteral("主模型切换状态：%1").arg(
        FormatMasterSwitchState(m_state.current_model, m_preview_source_mode)));
    lines.push_back(QStringLiteral("URDF 来源类型：%1").arg(
        FormatUrdfMasterSourceType(m_state.current_model.urdf_master_source_type)));
    lines.push_back(QStringLiteral("DH 草案级别：%1").arg(FormatDhDraftExtractionLevel(m_state.current_model.dh_draft_extraction_level)));
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
    lines.push_back(QStringLiteral("骨架预览：%1").arg(FormatPreviewSceneSummary(m_preview_scene)));
    if (!m_preview_scene.IsEmpty())
    {
        lines.push_back(QStringLiteral("预览模型名：%1").arg(m_preview_scene.model_name));
        if (!m_preview_scene.urdf_file_path.trimmed().isEmpty())
        {
            lines.push_back(QStringLiteral("URDF 文件：%1").arg(m_preview_scene.urdf_file_path));
        }
    }
    if (!m_state.current_model.original_imported_urdf_path.trimmed().isEmpty())
    {
        lines.push_back(QStringLiteral("原始导入 URDF：%1").arg(m_state.current_model.original_imported_urdf_path));
    }
    if (!m_state.current_model.conversion_diagnostics.trimmed().isEmpty())
    {
        lines.push_back(QStringLiteral("同步摘要：%1").arg(m_state.current_model.conversion_diagnostics));
    }
    if (!m_state.current_model.dh_draft_readonly_reason.trimmed().isEmpty())
    {
        lines.push_back(QStringLiteral("只读说明：%1").arg(m_state.current_model.dh_draft_readonly_reason));
    }
    if (!m_state.current_model.unified_robot_snapshot.derived_artifact_state_code.trimmed().isEmpty())
    {
        lines.push_back(QStringLiteral("派生产物状态：%1").arg(
            m_state.current_model.unified_robot_snapshot.derived_artifact_state_code));
        lines.push_back(QStringLiteral("派生产物路径：%1").arg(
            m_state.current_model.unified_robot_snapshot.derived_artifact_relative_path.isEmpty()
                ? QStringLiteral("未生成")
                : m_state.current_model.unified_robot_snapshot.derived_artifact_relative_path));
        lines.push_back(QStringLiteral("派生产物新鲜度：%1").arg(
            m_state.current_model.unified_robot_snapshot.derived_artifact_fresh
                ? QStringLiteral("新鲜")
                : (m_state.current_model.unified_robot_snapshot.derived_artifact_exists
                       ? QStringLiteral("待刷新")
                       : QStringLiteral("缺失"))));
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
    RefreshModelStatusLabels();
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

// 【排队总指挥】：当用户在界面上拖动滑块时，第一时间触发这个函数
void KinematicsWidget::ScheduleActivePreviewUpdate()
{
    // 如果当前正在用后台数据自动填充表单（PopulateForm），则直接返回。
    // 这是为了防止 UI 自动赋值时错误触发连环回调，导致死循环。
    if (m_is_populating_form)
    {
        return;
    }

    // 【路由判断】：
    // 如果当前预览的是 DH 骨架 (dh_preview)，或者主模型根本就不是 URDF，
    // 说明此时任何改动都可能涉及连杆长度、结构的改变。
    if (m_preview_source_mode == QStringLiteral("dh_preview") ||
        m_state.current_model.master_model_type != QStringLiteral("urdf"))
    {
        // 走重量级路线：排队请求“重建 3D 场景”
        ScheduleDhPreviewSceneUpdate();
        return;
    }

    // 否则（说明当前是 URDF 主模型，结构锁死，只能动关节），
    // 走轻量级路线：排队请求“更新矩阵位姿”
    SchedulePreviewPoseUpdate();
}
// 【发车总指挥】：当定时器 (16ms) 超时，也就是用户停止拖动时，触发这个函数
void KinematicsWidget::FlushActivePreviewUpdate()
{
    // 【路由判断】：
    // 如果当前是 URDF 预览模式，并且确实存在 URDF 文件路径
    if (m_preview_source_mode == QStringLiteral("urdf_preview") &&
        !m_preview_model.urdf_source_path.trimmed().isEmpty())
    {
        // 走轻量级路线：执行矩阵位姿更新
        FlushPreviewPoseUpdate();
        return;
    }

    // 否则，兜底走重量级路线：执行 3D 场景重建
    FlushDhPreviewSceneUpdate();
}

// 【轻量级 - 排队】：准备更新关节位姿
void KinematicsWidget::SchedulePreviewPoseUpdate()
{
    // 异常拦截：防抖定时器没初始化、当前没有 3D 场景、或者没有 URDF 文件，统统不排队。
    if (m_preview_pose_update_timer == nullptr ||
        m_preview_scene.IsEmpty() ||
        m_preview_model.urdf_source_path.trimmed().isEmpty())
    {
        return;
    }

    // 重置并启动防抖定时器。如果 16ms 内再次调用，定时器会重新从 0 开始算。
    m_preview_pose_update_timer->start();
}
// 【轻量级 - 执行】：真正计算并发送新的 4x4 变换矩阵
void KinematicsWidget::FlushPreviewPoseUpdate()
{
    // 再次进行异常拦截（因为这是延迟执行的，在此期间可能模型被清空了）
    if (m_preview_scene.IsEmpty() ||
        m_preview_model.urdf_source_path.trimmed().isEmpty())
    {
        return;
    }

    // 获取当前模型需要的关节总数
    const int expectedJointCount = m_preview_model.joint_count;
    if (expectedJointCount <= 0)
    {
        return;
    }

    // 安全检查：如果模型需要的关节数，比界面上提供的输入框（m_fk_joint_spins，通常是 6 个）还要多，
    // 说明 UI 承载不了这个复杂的机器人，输出警告并放弃刷新。
    if (expectedJointCount > static_cast<int>(m_fk_joint_spins.size()))
    {
        emit LogMessageGenerated(
            QStringLiteral("[Kinematics][Warning] 当前工程模型预览关节数量=%1，超过页面 FK 输入数量=%2，已跳过动态预览刷新。")
                .arg(expectedJointCount)
                .arg(static_cast<int>(m_fk_joint_spins.size())));
        return;
    }

    // 收集界面上滑块的角度值
    std::vector<double> jointPositionsDeg;
    jointPositionsDeg.reserve(static_cast<std::size_t>(expectedJointCount));
    for (int index = 0; index < expectedJointCount; ++index)
    {
        const auto spinIndex = static_cast<std::size_t>(index);
        // 读取对应的 SpinBox 值，如果指针为空则默认给 0.0
        jointPositionsDeg.push_back(m_fk_joint_spins[spinIndex] != nullptr
            ? m_fk_joint_spins[spinIndex]->value()
            : 0.0);
    }

    // 【核心算力调用】：让 Service (通常包裹了 Pinocchio 等数学引擎) 根据新角度算正解，
    // 得到每个 Link（连杆）在世界坐标系下的绝对位姿 (Pose)。
    const auto updateResult = m_service.UpdatePreviewPoses(m_preview_model, jointPositionsDeg);
    
    // 如果数学引擎计算报错（例如奇异点、超出范围等）
    if (!updateResult.IsSuccess())
    {
        emit LogMessageGenerated(QStringLiteral("[Kinematics][Warning] %1").arg(updateResult.message));
        return;
    }

    // 计算成功！将算好的所有连杆位姿（通常是个 Map<String, Pose>）通过信号广播出去。
    // 主窗口里的 3D 渲染器（VTK/OpenGL）收到后，直接对对应的 Actor 应用新的 Transform，瞬间完成画面更新。
    emit PreviewPosesUpdated(updateResult.link_world_poses);
}

// 【重量级 - 排队】：准备摧毁旧模型并生成新骨架
void KinematicsWidget::ScheduleDhPreviewSceneUpdate()
{
    // 如果正在自动填充表单（避免误触），或者定时器为空，不排队。
    if (m_is_populating_form || m_preview_pose_update_timer == nullptr)
    {
        return;
    }

    // 虽然重建场景很耗性能，但我们依然复用了同一个 16ms 定时器来进行节流防抖。
    // 只要用户还在表格里连续快速输入数字，就不会真正触发极其卡顿的场景重建 (FlushDhPreviewSceneUpdate)。
    m_preview_pose_update_timer->start();
}
void KinematicsWidget::FlushDhPreviewSceneUpdate()
{
    // ==========================================
    // 第一步：从 UI 收集数据，并强制覆写状态机标志位
    // ==========================================
    
    // 1. 读取当前界面上所有输入框和表格的值，打包成 DTO
    m_state.current_model = CollectModelFromForm();
    
    // 2. 强制宣告主权：当前模型的主数据源是 DH/MDH 参数，不再是 URDF
    m_state.current_model.master_model_type = QStringLiteral("dh_mdh");
    // 标记当前派生状态为“新鲜”（表示刚生成，没有过期）
    m_state.current_model.derived_model_state = QStringLiteral("fresh");
    // 锁定 UI 权限：DH 表格可编辑，URDF 视图相关逻辑锁定
    m_state.current_model.dh_editable = true;
    m_state.current_model.urdf_editable = false;

    // ==========================================
    // 第二步：数据清洗与容错 (Sanitization)
    // ==========================================
    
    // 中文说明：从 URDF 提取草案切回 DH/MDH 主模型时，参数约定必须收敛为有效的 DH 或 MDH，
    // 否则后端校验会把当前草稿继续当成 URDF 口径，导致中央骨架预览构建失败。
    // 【解析】：如果之前的草案是从 URDF 强行提取的，它的参数约定（parameter_convention）可能是乱码或空的。
    // 这里做一个安全兜底：如果不是标准的 "DH" 或 "MDH"，就强行重置为 "DH"，防止后端引擎解析崩溃。
    if (m_state.current_model.parameter_convention != QStringLiteral("DH") &&
        m_state.current_model.parameter_convention != QStringLiteral("MDH"))
    {
        m_state.current_model.parameter_convention = QStringLiteral("DH");
    }
    
    // 同步内部的建模模式和诊断日志
    m_state.current_model.modeling_mode = m_state.current_model.parameter_convention;
    m_state.current_model.conversion_diagnostics = QStringLiteral("当前草稿以 DH/MDH 参数为主模型。");
    
    // 刷新后端的诊断状态（检查是否缺少必填项等）
    RefreshBackendDiagnostics();


    // ==========================================
    // 第三步：调用底层物理引擎进行计算 (重量级操作)
    // ==========================================
    
    // 获取当前项目的磁盘根路径
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
        
    // 【核心调用】：把模型参数、当前的关节角度传给 Service。
    // 引擎会根据 DH 参数计算连杆变换，生成对应的 3D 几何体描述（PreviewScene）
    const auto buildResult = m_service.BuildDhPreviewScene(
        m_state.current_model,
        CollectJointInputs(m_fk_joint_spins),
        projectRootPath);
        
        
    // ==========================================
    // 第四步：异常处理 (如果构建失败)
    // ==========================================
    if (!buildResult.IsSuccess())
    {
        // 如果后端引擎报错（比如 DH 参数写得不合法导致矩阵奇异等）
        // 1. 在 UI 顶部显示警告红字
        SetOperationMessage(buildResult.message, false, true);
        // 2. 发送到底部状态栏
        EmitTelemetryStatus(QStringLiteral("DH/MDH 主模型"), buildResult.message, true);
        // 3. 渲染右下角的结果文本框
        RenderResults();
        RefreshModelStatusLabels();
        // 4. 打印日志并中断执行
        emit LogMessageGenerated(QStringLiteral("[Kinematics][Warning] %1").arg(buildResult.message));
        return;
    }


    // ==========================================
    // 第五步：同步引擎快照与 UI 状态 (非常关键)
    // ==========================================
    
    // 中文说明：DH 主模型预览成功后，以 Service 返回的统一模型快照覆盖当前草稿，
    // 这样项目保存、状态展示和后续跨模块主链都能读到同一份 unified_robot_model_ref / joint_order_signature。
    // 【解析】：注意这里！引擎不仅仅是返回了 3D 画面，它还“加工”了你的 current_model。
    // 比如它会自动为你生成全局唯一的关节顺序签名 (joint_order_signature)。
    // 必须用引擎返回的 preview_model 覆盖界面的 current_model，否则保存到硬盘的数据会缺少这些关键签名！
    m_state.current_model = buildResult.preview_model;
    
    // 保存 3D 场景数据和预览数据模式
    m_preview_scene = buildResult.preview_scene;
    m_preview_model = buildResult.preview_model;
    m_preview_source_mode = QStringLiteral("dh_preview");
    
    // 再次刷新面板上的文字状态和诊断信息
    RefreshBackendDiagnostics();
    RenderResults();
    RefreshModelStatusLabels();
    
    // 告诉底部状态栏：Pinocchio 引擎计算完毕
    EmitTelemetryStatus(QStringLiteral("Pinocchio"), buildResult.message, false);
    
    // 【最终动作】：把生成好的 3D 场景数据通过信号发射出去，主窗口的 VTK 会接住它并渲染出机器人模型。
    emit PreviewSceneGenerated(m_preview_scene);
}

void KinematicsWidget::ClearPreviewContext()
{
    if (m_preview_pose_update_timer != nullptr)
    {
        m_preview_pose_update_timer->stop();
    }

    // 中文说明：从 Topology/JSON 切换模型结构时，旧中央预览上下文不能继续留在视图里。
    m_preview_scene = PreviewSceneDto {};
    m_preview_model = RoboSDP::Kinematics::Dto::KinematicModelDto {};
    m_preview_source_mode = QStringLiteral("none");
    emit PreviewSceneGenerated(m_preview_scene);
    RefreshModelStatusLabels();
}

void KinematicsWidget::RefreshModelStatusLabels()
{
    if (m_master_model_mode_label != nullptr)
    {
        m_master_model_mode_label->setText(FormatMasterModelType(m_state.current_model.master_model_type));
    }
    if (m_derived_model_state_label != nullptr)
    {
        m_derived_model_state_label->setText(FormatDerivedModelState(m_state.current_model.derived_model_state));
    }
    if (m_preview_source_label != nullptr)
    {
        m_preview_source_label->setText(FormatPreviewSourceMode(m_preview_source_mode));
    }
    if (m_master_switch_state_label != nullptr)
    {
        m_master_switch_state_label->setText(FormatMasterSwitchState(m_state.current_model, m_preview_source_mode));
    }
    if (m_urdf_source_type_label != nullptr)
    {
        m_urdf_source_type_label->setText(
            FormatUrdfMasterSourceType(m_state.current_model.urdf_master_source_type));
        const QString tooltipText =
            !m_state.current_model.urdf_source_path.trimmed().isEmpty()
                ? m_state.current_model.urdf_source_path
                : (!m_state.current_model.original_imported_urdf_path.trimmed().isEmpty()
                       ? m_state.current_model.original_imported_urdf_path
                       : QString());
        m_urdf_source_type_label->setToolTip(tooltipText);
    }
    if (m_dh_draft_level_label != nullptr)
    {
        m_dh_draft_level_label->setText(FormatDhDraftExtractionLevel(m_state.current_model.dh_draft_extraction_level));
    }
    if (m_dh_draft_status_label != nullptr)
    {
        m_dh_draft_status_label->setText(
            m_state.current_model.dh_editable
                ? QStringLiteral("可编辑")
                : QStringLiteral("只读草案"));
        m_dh_draft_status_label->setToolTip(m_state.current_model.dh_draft_readonly_reason);
    }
}

QString KinematicsWidget::ResolveOriginalImportedUrdfMasterPath() const
{
    if (!m_state.current_model.original_imported_urdf_path.trimmed().isEmpty() &&
        QFileInfo::exists(m_state.current_model.original_imported_urdf_path))
    {
        return m_state.current_model.original_imported_urdf_path;
    }
    return {};
}

QString KinematicsWidget::ResolveDerivedUrdfMasterPath() const
{
    if (!m_state.current_model.urdf_source_path.trimmed().isEmpty() &&
        m_state.current_model.urdf_master_source_type == QStringLiteral("project_derived") &&
        QFileInfo::exists(m_state.current_model.urdf_source_path))
    {
        return m_state.current_model.urdf_source_path;
    }

    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    if (projectRootPath.trimmed().isEmpty())
    {
        return {};
    }

    const QString relativePath =
        m_state.current_model.unified_robot_snapshot.derived_artifact_relative_path.trimmed();
    if (relativePath.isEmpty())
    {
        return {};
    }

    return QDir(projectRootPath).absoluteFilePath(relativePath);
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
        m_state.current_model = importResult.preview_model;
        m_state.current_model.original_imported_urdf_path = urdfFilePath;
        m_state.current_model.urdf_source_path = urdfFilePath;
        m_state.current_model.urdf_master_source_type = QStringLiteral("original_imported");
        m_preview_scene = importResult.preview_scene;
        m_preview_model = importResult.preview_model;
        m_preview_source_mode = QStringLiteral("urdf_preview");
        PopulateForm(m_state.current_model);
        RefreshBackendDiagnostics();
        RenderResults();
        emit PreviewSceneGenerated(m_preview_scene);
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
        ClearPreviewContext();
        
        // 1. 先用新的 D-H 模型填充表格
        PopulateForm(m_state.current_model);
        
        // 2. 【核心修复】：删除旧的 FlushDhPreviewSceneUpdate()，使用新的实时正解渲染器！
        UpdateKinematicsPreview(); 
        
        RefreshBackendDiagnostics();
        RenderResults();
        MarkDirty();
    }

    SetOperationMessage(buildResult.message, buildResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(buildResult.message));
}

void KinematicsWidget::OnPromoteDhDraftToMasterClicked()
{
    if (m_state.current_model.master_model_type != QStringLiteral("urdf") ||
        m_state.current_model.dh_editable)
    {
        return;
    }

    const QString extractionLevel =
        FormatDhDraftExtractionLevel(m_state.current_model.dh_draft_extraction_level);
    const bool hasPromotableDhChain =
        !m_state.current_model.links.empty() &&
        !m_state.current_model.joint_limits.empty() &&
        m_state.current_model.links.size() == m_state.current_model.joint_limits.size();
    const bool diagnosticOnlyDraft =
        m_state.current_model.dh_draft_extraction_level == QStringLiteral("diagnostic_only");

    if (!hasPromotableDhChain || diagnosticOnlyDraft)
    {
        const QString message = diagnosticOnlyDraft
            ? QStringLiteral(
                  "当前 URDF 仅提取到“仅诊断展示”的 DH/MDH 草案，不能直接切换为 DH/MDH 主模型。\n\n"
                  "请继续保持 URDF 主模型进行预览；若需要参数化设计，请改用 Topology 生成 DH 骨架，"
                  "或导入一份可稳定提取 DH 链的 URDF。\n\n"
                  "当前草案级别：%1")
                  .arg(extractionLevel)
            : QStringLiteral(
                  "当前 URDF 提取出的 DH/MDH 草案不完整，无法安全切换为 DH/MDH 主模型。\n\n"
                  "要求：links 与 joint_limits 都非空，且数量一一对应。\n"
                  "当前 links=%1，joint_limits=%2，草案级别=%3")
                  .arg(m_state.current_model.links.size())
                  .arg(m_state.current_model.joint_limits.size())
                  .arg(extractionLevel);

        QMessageBox::warning(
            this,
            QStringLiteral("无法切换为 DH/MDH 主模型"),
            message);
        SetOperationMessage(message, false, true);
        EmitTelemetryStatus(QStringLiteral("DH/MDH 主模型"), message, true);
        emit LogMessageGenerated(QStringLiteral("[Kinematics][Warning] %1").arg(message));
        return;
    }

    const QString warningMessage = QStringLiteral(
        "即将把当前 URDF 提取草案切换为 DH/MDH 主模型。\n\n"
        "切换后将发生以下变化：\n"
        "1. DH/MDH 参数表将变为可编辑；\n"
        "2. 中央三维主视图区将改由 DH/MDH 骨架驱动；\n"
        "3. 软件会重新派生一份最小 URDF 主链产物；\n"
        "4. 原始导入 URDF 不再作为当前草稿的主模型。\n\n"
        "当前草案级别：%1\n"
        "建议先核对 a / alpha / d / theta_offset、Base / Flange / TCP 后再继续。")
        .arg(extractionLevel);

    const int answer = QMessageBox::warning(
        this,
        QStringLiteral("切换为 DH/MDH 主模型"),
        warningMessage,
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (answer != QMessageBox::Yes)
    {
        emit LogMessageGenerated(QStringLiteral("[Kinematics] 已取消切换为 DH/MDH 主模型。"));
        return;
    }

    m_state.current_model = CollectModelFromForm();
    m_state.current_model.master_model_type = QStringLiteral("dh_mdh");
    m_state.current_model.derived_model_state = QStringLiteral("stale");
    m_state.current_model.dh_editable = true;
    m_state.current_model.urdf_editable = false;
    m_state.current_model.modeling_mode = m_state.current_model.parameter_convention;
    m_state.current_model.model_source_mode = QStringLiteral("urdf_draft_promoted");
    if (m_state.current_model.parameter_convention != QStringLiteral("DH") &&
        m_state.current_model.parameter_convention != QStringLiteral("MDH"))
    {
        m_state.current_model.parameter_convention = QStringLiteral("DH");
    }
    m_state.current_model.conversion_diagnostics =
        QStringLiteral("当前草稿已从 URDF 提取草案切换为 DH/MDH 主模型。");
    m_state.current_model.dh_draft_extraction_level.clear();
    m_state.current_model.dh_draft_readonly_reason.clear();
    m_state.current_model.urdf_master_source_type = QStringLiteral("none");
    m_state.current_model.urdf_source_path.clear();
    m_state.current_model.pinocchio_model_ready = false;
    m_state.current_model.unified_robot_model_ref.clear();
    // 中文说明：从 URDF 草案切成 DH 主模型后，关节顺序签名必须按当前参数表重建，不能继续复用导入态的路径型签名。
    m_state.current_model.joint_order_signature.clear();
    m_state.current_model.unified_robot_snapshot = {};

    ClearPreviewContext();
    PopulateForm(m_state.current_model);
    FlushDhPreviewSceneUpdate();
    MarkDirty();

    const QString message = QStringLiteral("已切换为 DH/MDH 主模型模式，中央骨架与派生产物已按参数化主链刷新。");
    SetOperationMessage(message, true);
    EmitTelemetryStatus(QStringLiteral("DH/MDH 主模型"), message, false);
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(message));
}

void KinematicsWidget::OnSwitchToUrdfMasterClicked()
{
    if (m_state.current_model.master_model_type != QStringLiteral("dh_mdh"))
    {
        return;
    }

    const QString originalImportedPath = ResolveOriginalImportedUrdfMasterPath();
    const QString derivedUrdfPath = ResolveDerivedUrdfMasterPath();
    QString urdfFilePath;
    QString selectedSourceType = QStringLiteral("none");

    if (!originalImportedPath.trimmed().isEmpty() &&
        !derivedUrdfPath.trimmed().isEmpty() &&
        QFileInfo(originalImportedPath) != QFileInfo(derivedUrdfPath))
    {
        QMessageBox sourcePicker(this);
        sourcePicker.setIcon(QMessageBox::Question);
        sourcePicker.setWindowTitle(QStringLiteral("选择 URDF 回切来源"));
        sourcePicker.setText(QStringLiteral("检测到两种可用的 URDF 来源，请明确选择要回切到哪一种主模型来源。"));
        sourcePicker.setInformativeText(
            QStringLiteral("原始导入：%1\n\n项目派生：%2")
                .arg(originalImportedPath, derivedUrdfPath));
        auto* originalButton = sourcePicker.addButton(QStringLiteral("使用原始导入 URDF"), QMessageBox::AcceptRole);
        auto* derivedButton = sourcePicker.addButton(QStringLiteral("使用项目派生 URDF"), QMessageBox::ActionRole);
        sourcePicker.addButton(QMessageBox::Cancel);
        sourcePicker.exec();

        if (sourcePicker.clickedButton() == originalButton)
        {
            urdfFilePath = originalImportedPath;
            selectedSourceType = QStringLiteral("original_imported");
        }
        else if (sourcePicker.clickedButton() == derivedButton)
        {
            urdfFilePath = derivedUrdfPath;
            selectedSourceType = QStringLiteral("project_derived");
        }
        else
        {
            emit LogMessageGenerated(QStringLiteral("[Kinematics] 已取消切换为 URDF 主模型。"));
            return;
        }
    }
    else if (!originalImportedPath.trimmed().isEmpty())
    {
        urdfFilePath = originalImportedPath;
        selectedSourceType = QStringLiteral("original_imported");
    }
    else if (!derivedUrdfPath.trimmed().isEmpty())
    {
        urdfFilePath = derivedUrdfPath;
        selectedSourceType = QStringLiteral("project_derived");
    }

    if (urdfFilePath.trimmed().isEmpty() || !QFileInfo::exists(urdfFilePath))
    {
        const QString message = QStringLiteral("当前没有可用的派生 URDF 文件，无法回切到 URDF 主模型模式。");
        SetOperationMessage(message, false, true);
        emit LogMessageGenerated(QStringLiteral("[Kinematics][Warning] %1").arg(message));
        return;
    }

    const QString warningMessage = QStringLiteral(
        "即将把当前草稿回切为 URDF 主模型。\n\n"
        "切换后将发生以下变化：\n"
        "1. 中央三维主视图区将改由 URDF 工程模型预览驱动；\n"
        "2. DH/MDH 参数表将重新回到只读草案模式；\n"
        "3. 之后若继续参数化设计，需要再次显式切回 DH/MDH 主模型。\n\n"
        "本次回切来源：%1\n"
        "本次回切将使用以下 URDF：\n%2")
        .arg(FormatUrdfMasterSourceType(selectedSourceType), urdfFilePath);

    const int answer = QMessageBox::warning(
        this,
        QStringLiteral("切换为 URDF 主模型"),
        warningMessage,
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (answer != QMessageBox::Yes)
    {
        emit LogMessageGenerated(QStringLiteral("[Kinematics] 已取消切换为 URDF 主模型。"));
        return;
    }

    const auto importResult = m_service.ImportUrdfPreview(urdfFilePath);
    if (!importResult.IsSuccess())
    {
        SetOperationMessage(importResult.message, false, true);
        emit LogMessageGenerated(QStringLiteral("[Kinematics][Warning] %1").arg(importResult.message));
        return;
    }

    const auto previousMeta = m_state.current_model.meta;
    const auto previousSnapshot = m_state.current_model.unified_robot_snapshot;
    const QString previousOriginalImportedPath = m_state.current_model.original_imported_urdf_path;

    m_state.current_model = importResult.preview_model;
    m_state.current_model.meta = previousMeta;
    m_state.current_model.meta.source = QStringLiteral("urdf_master");
    m_state.current_model.model_source_mode = selectedSourceType == QStringLiteral("original_imported")
        ? QStringLiteral("urdf_imported")
        : QStringLiteral("derived_urdf_promoted");
    m_state.current_model.urdf_master_source_type = selectedSourceType;
    m_state.current_model.urdf_source_path = urdfFilePath;
    m_state.current_model.original_imported_urdf_path =
        selectedSourceType == QStringLiteral("original_imported")
            ? urdfFilePath
            : previousOriginalImportedPath;
    m_state.current_model.conversion_diagnostics =
        QStringLiteral("当前草稿已回切为 URDF 主模型；DH/MDH 参数表为提取草案，仅用于诊断展示。");
    if (m_state.current_model.dh_draft_extraction_level.trimmed().isEmpty())
    {
        m_state.current_model.dh_draft_extraction_level = QStringLiteral("diagnostic_only");
    }
    if (m_state.current_model.dh_draft_readonly_reason.trimmed().isEmpty())
    {
        m_state.current_model.dh_draft_readonly_reason =
            QStringLiteral("当前 DH/MDH 参数表由 URDF 主模型提取，仅用于诊断展示。若需继续参数化设计，请显式切换为 DH/MDH 主模型模式。");
    }
    m_state.current_model.unified_robot_snapshot = importResult.preview_model.unified_robot_snapshot;
    m_state.current_model.unified_robot_snapshot.source_kinematic_id = m_state.current_model.meta.kinematic_id;
    m_state.current_model.unified_robot_snapshot.model_source_mode = m_state.current_model.model_source_mode;
    m_state.current_model.unified_robot_snapshot.conversion_diagnostics =
        m_state.current_model.conversion_diagnostics;
    if (!previousSnapshot.derived_artifact_relative_path.trimmed().isEmpty())
    {
        m_state.current_model.unified_robot_snapshot.derived_artifact_relative_path =
            previousSnapshot.derived_artifact_relative_path;
    }

    m_preview_scene = importResult.preview_scene;
    m_preview_model = m_state.current_model;
    m_preview_source_mode = QStringLiteral("urdf_preview");
    PopulateForm(m_state.current_model);
    RefreshBackendDiagnostics();
    RenderResults();
    emit PreviewSceneGenerated(m_preview_scene);
    SchedulePreviewPoseUpdate();
    MarkDirty();

    const QString message = QStringLiteral("已切换为 URDF 主模型模式，中央视图改由工程模型预览驱动。");
    SetOperationMessage(message, true);
    EmitTelemetryStatus(QStringLiteral("URDF 主模型"), message, false);
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(message));
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
        ClearPreviewContext();
        PopulateForm(m_state.current_model);
        if (m_state.current_model.master_model_type != QStringLiteral("urdf"))
        {
            FlushDhPreviewSceneUpdate();
        }
        RefreshBackendDiagnostics();
        RenderResults();
        MarkClean();
    }

    SetOperationMessage(loadResult.message, loadResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(loadResult.message));
}

// KinematicsWidget.cpp 末尾添加

void KinematicsWidget::UpdateKinematicsPreview()
{
    // 1. 收集当前的拓扑尺寸（这里建议从 m_state 或通过 service 获取最新的副本）
    // 为了简单，我们先获取当前的运动学模型
    const auto& model = m_state.current_model;
    
    // 2. 收集当前的关节角度
    std::vector<double> currentAngles;
    for (int i = 0; i < 6; ++i) {
        currentAngles.push_back(m_fk_joint_spins[i]->value());
    }

    // 3. 调用 Service 生成带真实位姿的场景
    // 注意：我们需要一个能接收 RobotTopologyModelDto 的转换函数，
    // 或者直接在 KinematicsService 里增加一个接收 KinematicModelDto 的重载
    auto scene = m_service.GenerateSkeletonPreviewFromKinematicModel(model, currentAngles);

    // 4. 发射预览信号
    emit KinematicsPreviewGenerated(scene);
}
} // namespace RoboSDP::Kinematics::Ui
