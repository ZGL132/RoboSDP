#include "modules/requirement/ui/RequirementWidget.h"

#include "core/infrastructure/ProjectManager.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
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

#include <algorithm>
#include <cmath>

namespace RoboSDP::Requirement::Ui
{

RequirementWidget::RequirementWidget(RoboSDP::Logging::ILogger* logger, QWidget* parent)
    : QWidget(parent)
    , m_storage(m_repository)
    , m_logger(logger)
    , m_service(m_storage, m_validator, m_logger)
{
    BuildUi();
    PopulateForm(m_service.CreateDefaultModel());
    ConnectDirtyTracking();
    connect(
        &RoboSDP::Infrastructure::ProjectManager::instance(),
        &RoboSDP::Infrastructure::ProjectManager::projectPathChanged,
        this,
        [this](const QString&) { SyncProjectNameFromProjectContext(); });
    SyncProjectNameFromProjectContext();
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

void RequirementWidget::RefreshPreview()
{
    EmitRequirementPreview();
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
        connect(editor, &QLineEdit::textEdited, this, [this]() {
            MarkDirty();
            EmitRequirementPreview();
        });
    }
    for (QComboBox* editor : findChildren<QComboBox*>())
    {
        connect(editor, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [this](int) {
            MarkDirty();
            EmitRequirementPreview();
        });
    }
    for (QDoubleSpinBox* editor : findChildren<QDoubleSpinBox*>())
    {
        connect(editor, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [this](double) {
            MarkDirty();
            EmitRequirementPreview();
        });
    }
    for (QSpinBox* editor : findChildren<QSpinBox*>())
    {
        connect(editor, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, [this](int) {
            MarkDirty();
            EmitRequirementPreview();
        });
    }
    for (QCheckBox* editor : findChildren<QCheckBox*>())
    {
        if (editor == m_show_workspace_preview_check || editor == m_show_key_pose_preview_check)
        {
            continue;
        }
        connect(editor, &QCheckBox::toggled, this, [this](bool) {
            MarkDirty();
            EmitRequirementPreview();
        });
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

void RequirementWidget::EmitRequirementPreview()
{
    if (m_is_populating_form)
    {
        return;
    }

    EmitWorkspacePreview();
    EmitKeyPosePreview();
}

void RequirementWidget::EmitWorkspacePreview()
{
    if (m_show_workspace_preview_check != nullptr && !m_show_workspace_preview_check->isChecked())
    {
        emit WorkspacePreviewUpdated({});
        return;
    }

    emit WorkspacePreviewUpdated(BuildWorkspacePreviewPoints());
}

void RequirementWidget::EmitKeyPosePreview()
{
    SaveCurrentKeyPoseEdits();
    NormalizeWorkingKeyPoseIdsAndNames();
    RefreshKeyPoseList();
    if (m_key_pose_list != nullptr && m_current_key_pose_index >= 0)
    {
        const QSignalBlocker blocker(m_key_pose_list);
        m_key_pose_list->setCurrentRow(m_current_key_pose_index);
    }
    if (m_show_key_pose_preview_check != nullptr && !m_show_key_pose_preview_check->isChecked())
    {
        emit KeyPosePreviewUpdated({}, -1);
        return;
    }

    emit KeyPosePreviewUpdated(m_working_model.workspace_requirements.key_poses, m_current_key_pose_index);
}

void RequirementWidget::RefreshTemplateOptions()
{
    if (m_template_combo == nullptr)
    {
        return;
    }

    const QSignalBlocker blocker(m_template_combo);
    const QString previousTemplateId = !m_working_model.project_meta.selected_template_id.trimmed().isEmpty()
        ? m_working_model.project_meta.selected_template_id.trimmed()
        : m_template_combo->currentData().toString().trimmed();

    const double ratedPayload = m_rated_payload_spin != nullptr ? m_rated_payload_spin->value() : 0.0;
    const double maxPayload = m_max_payload_spin != nullptr ? m_max_payload_spin->value() : 0.0;
    const double maxRadius = m_max_radius_spin != nullptr ? m_max_radius_spin->value() : 0.0;
    const double repeatability = m_repeatability_spin != nullptr ? m_repeatability_spin->value() : 0.0;
    const auto summaries = m_template_catalog.FilterByRequirement(ratedPayload, maxPayload, maxRadius, repeatability);

    m_template_combo->clear();
    m_template_combo->addItem(QStringLiteral("自动推荐（按需求筛选）"), QString());
    for (const auto& summary : summaries)
    {
        const QString itemText = QStringLiteral("%1 | %2kg | %3m | ±%4mm")
                                     .arg(summary.display_name)
                                     .arg(summary.rated_payload_kg, 0, 'f', 1)
                                     .arg(summary.reach_m, 0, 'f', 3)
                                     .arg(summary.repeatability_mm, 0, 'f', 3);
        m_template_combo->addItem(itemText, summary.template_id);
        const int index = m_template_combo->count() - 1;
        m_template_combo->setItemData(index, summary.display_name, Qt::UserRole + 1);
        m_template_combo->setItemData(index, summary.brand, Qt::UserRole + 2);
        m_template_combo->setItemData(index, summary.rated_payload_kg, Qt::UserRole + 3);
        m_template_combo->setItemData(index, summary.max_payload_kg, Qt::UserRole + 4);
        m_template_combo->setItemData(index, summary.reach_m, Qt::UserRole + 5);
        m_template_combo->setItemData(index, summary.repeatability_mm, Qt::UserRole + 6);
        m_template_combo->setItemData(index, summary.description, Qt::ToolTipRole);
    }

    const int previousIndex = m_template_combo->findData(previousTemplateId);
    if (previousIndex >= 0)
    {
        m_template_combo->setCurrentIndex(previousIndex);
    }
    else if (m_template_combo->count() > 1)
    {
        m_template_combo->setCurrentIndex(1);
    }
    else
    {
        m_template_combo->setCurrentIndex(0);
    }

    ApplySelectedTemplateToForm(false);
}

void RequirementWidget::ApplySelectedTemplateToForm(bool refreshParameters)
{
    if (m_template_combo == nullptr)
    {
        return;
    }

    const int index = m_template_combo->currentIndex();
    const QString templateId = m_template_combo->currentData().toString().trimmed();
    const QString templateName = m_template_combo->itemData(index, Qt::UserRole + 1).toString();
    const QString brand = m_template_combo->itemData(index, Qt::UserRole + 2).toString();
    const double ratedPayload = m_template_combo->itemData(index, Qt::UserRole + 3).toDouble();
    const double maxPayload = m_template_combo->itemData(index, Qt::UserRole + 4).toDouble();
    const double reach = m_template_combo->itemData(index, Qt::UserRole + 5).toDouble();
    const double repeatability = m_template_combo->itemData(index, Qt::UserRole + 6).toDouble();

    m_working_model.project_meta.selected_template_id = templateId;
    m_working_model.project_meta.selected_template_name = templateName;
    m_working_model.project_meta.selected_template_brand = brand;

    if (m_template_summary_label != nullptr)
    {
        if (templateId.isEmpty())
        {
            m_template_summary_label->setText(QStringLiteral("自动推荐会在构型生成时按当前 Requirement 选择评分最高的模板。"));
        }
        else
        {
            m_template_summary_label->setText(
                QStringLiteral("%1：额定负载 %2 kg，最大负载 %3 kg，臂展 %4 m，重复定位精度 ±%5 mm。")
                    .arg(templateName)
                    .arg(ratedPayload, 0, 'f', 1)
                    .arg(maxPayload, 0, 'f', 1)
                    .arg(reach, 0, 'f', 3)
                    .arg(repeatability, 0, 'f', 3));
        }
    }

    if (!refreshParameters || templateId.isEmpty())
    {
        return;
    }

    const QString projectName = m_project_name_edit != nullptr ? m_project_name_edit->text() : QString();
    const QString scenarioType =
        m_scenario_type_combo != nullptr ? m_scenario_type_combo->currentData().toString() : QString();
    auto templateModel = m_service.CreateTemplateModelByPayload(ratedPayload > 0.0 ? ratedPayload : m_rated_payload_spin->value());
    templateModel.project_meta.project_name = projectName;
    templateModel.project_meta.scenario_type = scenarioType;
    templateModel.project_meta.description =
        QStringLiteral("参考 %1 级别 6R 工业机器人任务需求默认值").arg(templateName);
    templateModel.project_meta.selected_template_id = templateId;
    templateModel.project_meta.selected_template_name = templateName;
    templateModel.project_meta.selected_template_brand = brand;
    if (ratedPayload > 0.0)
    {
        templateModel.load_requirements.rated_payload = ratedPayload;
    }
    if (maxPayload > 0.0)
    {
        templateModel.load_requirements.max_payload = maxPayload;
    }
    if (reach > 0.0)
    {
        templateModel.workspace_requirements.max_radius = reach;
        templateModel.workspace_requirements.min_radius = std::max(0.12, reach * 0.16);
        templateModel.workspace_requirements.max_height = reach * 1.05;
        templateModel.workspace_requirements.min_height = -reach * 0.32;
        if (!templateModel.workspace_requirements.key_poses.empty())
        {
            auto& keyPose = templateModel.workspace_requirements.key_poses.front();
            keyPose.pose[0] = reach * 0.58;
            keyPose.pose[2] = reach * 0.40;
        }
    }
    if (repeatability > 0.0)
    {
        templateModel.accuracy_requirements.repeatability = repeatability;
        templateModel.accuracy_requirements.absolute_accuracy = std::max(0.2, repeatability * 10.0);
        templateModel.accuracy_requirements.tracking_accuracy = std::max(0.15, repeatability * 6.0);
        templateModel.accuracy_requirements.tcp_position_tol = std::max(0.2, repeatability * 10.0);
        if (!templateModel.workspace_requirements.key_poses.empty())
        {
            templateModel.workspace_requirements.key_poses.front().position_tol = std::max(0.1, repeatability * 10.0);
        }
    }

    PopulateForm(templateModel);
}

std::vector<std::array<double, 3>> RequirementWidget::BuildWorkspacePreviewPoints() const
{
    std::vector<std::array<double, 3>> points;

    const double maxRadius = std::max(0.0, m_max_radius_spin != nullptr ? m_max_radius_spin->value() : 0.0);
    const double minRadius = std::clamp(
        m_min_radius_spin != nullptr ? m_min_radius_spin->value() : 0.0,
        0.0,
        maxRadius);
    const double minHeight = m_min_height_spin != nullptr ? m_min_height_spin->value() : 0.0;
    const double maxHeight = m_max_height_spin != nullptr ? m_max_height_spin->value() : 0.0;
    if (maxRadius <= 0.0 || maxHeight <= minHeight)
    {
        return points;
    }

    constexpr int kSegments = 96;
    constexpr double kPi = 3.14159265358979323846;
    points.reserve(static_cast<std::size_t>(kSegments * 6));

    auto appendRing = [&points](double radius, double height) {
        if (radius <= 0.0)
        {
            return;
        }
        for (int i = 0; i < kSegments; ++i)
        {
            const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSegments);
            points.push_back({radius * std::cos(angle), radius * std::sin(angle), height});
        }
    };

    appendRing(maxRadius, minHeight);
    appendRing(maxRadius, maxHeight);
    appendRing(minRadius, minHeight);
    appendRing(minRadius, maxHeight);

    for (int i = 0; i < kSegments; i += 8)
    {
        const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSegments);
        const double cosValue = std::cos(angle);
        const double sinValue = std::sin(angle);
        for (int h = 0; h <= 6; ++h)
        {
            const double t = static_cast<double>(h) / 6.0;
            const double height = minHeight + (maxHeight - minHeight) * t;
            points.push_back({maxRadius * cosValue, maxRadius * sinValue, height});
            if (minRadius > 0.0)
            {
                points.push_back({minRadius * cosValue, minRadius * sinValue, height});
            }
        }
    }

    return points;
}

void RequirementWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    m_operation_label = new QLabel(QStringLiteral("就绪：请录入任务需求基础字段。"), this);
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
    m_project_name_edit->setReadOnly(true);
    m_project_name_edit->setPlaceholderText(QStringLiteral("由 project.json 决定"));
    m_project_name_edit->setToolTip(QStringLiteral("项目名称属于项目级元数据，Requirement 页面仅只读引用。"));
    m_scenario_type_combo = new QComboBox(groupBox);
    m_description_edit = new QLineEdit(groupBox);
    m_template_combo = new QComboBox(groupBox);
    m_template_combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    m_template_combo->setMinimumContentsLength(28);
    m_template_summary_label = new QLabel(groupBox);
    m_template_summary_label->setWordWrap(true);
    m_template_summary_label->setText(QStringLiteral("请先输入负载和工作空间参数，系统会筛选相近的经典机器人模板。"));

    m_scenario_type_combo->addItem(QStringLiteral("搬运"), QStringLiteral("handling"));
    m_scenario_type_combo->addItem(QStringLiteral("焊接"), QStringLiteral("welding"));
    m_scenario_type_combo->addItem(QStringLiteral("装配"), QStringLiteral("assembly"));
    m_scenario_type_combo->addItem(QStringLiteral("打磨"), QStringLiteral("grinding"));
    m_scenario_type_combo->addItem(QStringLiteral("喷涂"), QStringLiteral("spraying"));
    m_scenario_type_combo->addItem(QStringLiteral("自定义"), QStringLiteral("custom"));

    layout->addRow(QStringLiteral("项目名称"), m_project_name_edit);
    layout->addRow(QStringLiteral("场景类型"), m_scenario_type_combo);
    layout->addRow(QStringLiteral("参考模板"), m_template_combo);
    layout->addRow(QStringLiteral("模板参数"), m_template_summary_label);
    layout->addRow(QStringLiteral("描述"), m_description_edit);

    RegisterFieldWidget(QStringLiteral("project_meta.project_name"), m_project_name_edit);
    RegisterFieldWidget(QStringLiteral("project_meta.scenario_type"), m_scenario_type_combo);

    connect(
        m_template_combo,
        static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this,
        [this](int index) { OnTemplateSelectionChanged(index); });

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

    connect(
        m_rated_payload_spin,
        static_cast<void (QDoubleSpinBox::*)()>(&QDoubleSpinBox::editingFinished),
        this,
        [this]() { OnRatedPayloadChanged(m_rated_payload_spin->value()); });

    return groupBox;
}

QGroupBox* RequirementWidget::CreateWorkspaceGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("工作空间与关键工位"), this);
    auto* layout = new QFormLayout(groupBox);
    auto* workspaceGroup = new QGroupBox(QStringLiteral("工作空间约束"), groupBox);
    auto* workspaceLayout = new QFormLayout(workspaceGroup);
    auto* keyPoseGroup = new QGroupBox(QStringLiteral("关键工位"), groupBox);
    auto* keyPoseLayout = new QFormLayout(keyPoseGroup);
    layout->addRow(workspaceGroup);
    layout->addRow(keyPoseGroup);

    m_max_radius_spin = CreateDoubleSpinBox(0.0, 1.0e6, 4, 0.01);
    m_min_radius_spin = CreateDoubleSpinBox(0.0, 1.0e6, 4, 0.01);
    m_max_height_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_min_height_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_base_mount_type_combo = new QComboBox(workspaceGroup);
    m_hollow_wrist_requirement_combo = new QComboBox(workspaceGroup);
    m_reserved_channel_diameter_spin = CreateDoubleSpinBox(0.0, 1.0e6, 2, 1.0);
    m_show_workspace_preview_check = new QCheckBox(QStringLiteral("显示工作空间"), workspaceGroup);
    m_show_workspace_preview_check->setChecked(true);
    m_show_workspace_preview_check->setToolTip(QStringLiteral("控制三维视图中的工作空间边界显示，不影响保存内容。"));
    m_show_key_pose_preview_check = new QCheckBox(QStringLiteral("显示关键工位"), keyPoseGroup);
    m_show_key_pose_preview_check->setChecked(true);
    m_show_key_pose_preview_check->setToolTip(QStringLiteral("控制三维视图中的关键工位标记显示，不影响保存内容。"));
    m_key_pose_list = new QListWidget(keyPoseGroup);
    m_key_pose_list->setSelectionMode(QAbstractItemView::SingleSelection);
    m_key_pose_list->setMinimumHeight(120);
    m_add_key_pose_button = new QPushButton(QStringLiteral("新增工位"), keyPoseGroup);
    m_remove_key_pose_button = new QPushButton(QStringLiteral("删除工位"), keyPoseGroup);

    /**
     * @brief 显式录入 Topology 直接相关的 base_constraints 字段。
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

    auto* keyPoseToolbarWidget = new QWidget(keyPoseGroup);
    auto* keyPoseToolbarLayout = new QHBoxLayout(keyPoseToolbarWidget);
    keyPoseToolbarLayout->setContentsMargins(0, 0, 0, 0);
    keyPoseToolbarLayout->addWidget(m_add_key_pose_button);
    keyPoseToolbarLayout->addWidget(m_remove_key_pose_button);
    keyPoseToolbarLayout->addStretch();

    m_key_pose_id_edit = new QLineEdit(keyPoseGroup);
    m_key_pose_id_edit->setVisible(false);
    m_key_pose_name_edit = new QLineEdit(keyPoseGroup);
    m_key_pose_name_edit->setReadOnly(true);
    m_key_pose_name_edit->setToolTip(QStringLiteral("名称由工位顺序自动生成，新增或删除后自动重排。"));
    m_key_pose_x_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_key_pose_y_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_key_pose_z_spin = CreateDoubleSpinBox(-1.0e6, 1.0e6, 4, 0.01);
    m_key_pose_rx_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_key_pose_ry_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_key_pose_rz_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_key_pose_position_tol_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_key_pose_orientation_tol_spin = CreateDoubleSpinBox(0.0, 1.0e6, 3, 0.1);
    m_key_pose_required_direction_check = new QCheckBox(QStringLiteral("启用工具方向要求"), keyPoseGroup);
    m_key_pose_required_direction_check->setVisible(false);
    m_key_pose_direction_x_spin = CreateDoubleSpinBox(-1.0, 1.0, 4, 0.1);
    m_key_pose_direction_y_spin = CreateDoubleSpinBox(-1.0, 1.0, 4, 0.1);
    m_key_pose_direction_z_spin = CreateDoubleSpinBox(-1.0, 1.0, 4, 0.1);
    m_key_pose_direction_x_spin->setVisible(false);
    m_key_pose_direction_y_spin->setVisible(false);
    m_key_pose_direction_z_spin->setVisible(false);

    workspaceLayout->addRow(QStringLiteral("最大半径 [m]"), m_max_radius_spin);
    workspaceLayout->addRow(QStringLiteral("最小半径 [m]"), m_min_radius_spin);
    workspaceLayout->addRow(QStringLiteral("最大高度 [m]"), m_max_height_spin);
    workspaceLayout->addRow(QStringLiteral("最小高度 [m]"), m_min_height_spin);
    workspaceLayout->addRow(QStringLiteral("基座安装偏好"), m_base_mount_type_combo);
    workspaceLayout->addRow(QStringLiteral("中空腕需求"), m_hollow_wrist_requirement_combo);
    workspaceLayout->addRow(QStringLiteral("预留通道直径 [mm]"), m_reserved_channel_diameter_spin);
    workspaceLayout->addRow(QStringLiteral("三维显示"), m_show_workspace_preview_check);
    keyPoseLayout->addRow(QStringLiteral("三维显示"), m_show_key_pose_preview_check);
    keyPoseLayout->addRow(QStringLiteral("关键工位名称"), m_key_pose_name_edit);
    keyPoseLayout->addRow(QStringLiteral("工位 X [m]"), m_key_pose_x_spin);
    keyPoseLayout->addRow(QStringLiteral("工位 Y [m]"), m_key_pose_y_spin);
    keyPoseLayout->addRow(QStringLiteral("工位 Z [m]"), m_key_pose_z_spin);
    keyPoseLayout->addRow(QStringLiteral("工位 RX [deg]"), m_key_pose_rx_spin);
    keyPoseLayout->addRow(QStringLiteral("工位 RY [deg]"), m_key_pose_ry_spin);
    keyPoseLayout->addRow(QStringLiteral("工位 RZ [deg]"), m_key_pose_rz_spin);
    keyPoseLayout->addRow(QStringLiteral("位置容限 [mm]"), m_key_pose_position_tol_spin);
    keyPoseLayout->addRow(QStringLiteral("姿态容限 [deg]"), m_key_pose_orientation_tol_spin);

    keyPoseLayout->addRow(QStringLiteral("关键工位操作"), keyPoseToolbarWidget);
    keyPoseLayout->addRow(QStringLiteral("关键工位列表"), m_key_pose_list);

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
    connect(
        m_show_workspace_preview_check,
        &QCheckBox::toggled,
        this,
        [this](bool) { OnPreviewVisibilityChanged(); });
    connect(
        m_show_key_pose_preview_check,
        &QCheckBox::toggled,
        this,
        [this](bool) { OnPreviewVisibilityChanged(); });
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

    SyncProjectNameFromProjectContext();
    model.project_meta.project_name = m_project_name_edit->text().trimmed();
    model.project_meta.scenario_type = m_scenario_type_combo->currentData().toString();
    model.project_meta.description = m_description_edit->text().trimmed();
    if (m_template_combo != nullptr)
    {
        const int templateIndex = m_template_combo->currentIndex();
        model.project_meta.selected_template_id = m_template_combo->currentData().toString().trimmed();
        model.project_meta.selected_template_name =
            m_template_combo->itemData(templateIndex, Qt::UserRole + 1).toString();
        model.project_meta.selected_template_brand =
            m_template_combo->itemData(templateIndex, Qt::UserRole + 2).toString();
    }

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
    NormalizeWorkingKeyPoseIdsAndNames();
    model.workspace_requirements.key_poses = m_working_model.workspace_requirements.key_poses;

    const int currentIndex = std::clamp(
        m_current_key_pose_index,
        0,
        static_cast<int>(model.workspace_requirements.key_poses.size()) - 1);
    model.workspace_requirements.key_poses[static_cast<std::size_t>(currentIndex)] =
        CollectKeyPoseFromEditor();
    m_working_model.workspace_requirements.key_poses = model.workspace_requirements.key_poses;
    NormalizeWorkingKeyPoseIdsAndNames();
    model.workspace_requirements.key_poses = m_working_model.workspace_requirements.key_poses;

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
    m_is_populating_form = true;
    m_working_model = model;
    if (m_working_model.workspace_requirements.key_poses.empty())
    {
        m_working_model.workspace_requirements.key_poses.push_back(
            RoboSDP::Requirement::Dto::RequirementKeyPoseDto {});
    }
    NormalizeWorkingKeyPoseIdsAndNames();

    SyncProjectNameFromProjectContext();

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
    RefreshTemplateOptions();

    RefreshKeyPoseList();
    m_current_key_pose_index = 0;
    {
        const QSignalBlocker blocker(m_key_pose_list);
        m_key_pose_list->setCurrentRow(m_current_key_pose_index);
    }
    LoadCurrentKeyPoseToEditor();
    m_is_populating_form = false;
    EmitRequirementPreview();
}

void RequirementWidget::OnRatedPayloadChanged(double ratedPayloadKg)
{
    if (m_is_populating_form)
    {
        return;
    }

    const QString projectName = m_project_name_edit != nullptr ? m_project_name_edit->text() : QString();
    const QString scenarioType =
        m_scenario_type_combo != nullptr ? m_scenario_type_combo->currentData().toString() : QString();
    auto templateModel = m_service.CreateTemplateModelByPayload(ratedPayloadKg);
    templateModel.project_meta.project_name = projectName;
    if (!scenarioType.trimmed().isEmpty())
    {
        templateModel.project_meta.scenario_type = scenarioType;
    }
    templateModel.project_meta.selected_template_id = QString();
    templateModel.project_meta.selected_template_name = QString();
    templateModel.project_meta.selected_template_brand = QString();

    PopulateForm(templateModel);
    MarkDirty();
    SetOperationMessage(
        QStringLiteral("已按额定负载匹配 KUKA 参考型号，并刷新任务需求默认参数。"),
        true);
}

void RequirementWidget::OnTemplateSelectionChanged(int index)
{
    if (m_is_populating_form || index < 0)
    {
        return;
    }

    ApplySelectedTemplateToForm(true);
    MarkDirty();
    EmitRequirementPreview();
    SetOperationMessage(QStringLiteral("已按所选参考模板刷新任务需求默认参数。"), true);
}

void RequirementWidget::SyncProjectNameFromProjectContext()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();

    QString projectName = ResolveProjectNameFromProjectFile(projectRootPath);
    if (projectName.trimmed().isEmpty() && !projectRootPath.trimmed().isEmpty())
    {
        projectName = QFileInfo(projectRootPath).fileName();
    }

    const QSignalBlocker blocker(m_project_name_edit);
    m_project_name_edit->setText(projectName.trimmed());
    m_working_model.project_meta.project_name = projectName.trimmed();
}

QString RequirementWidget::ResolveProjectNameFromProjectFile(const QString& projectRootPath) const
{
    if (projectRootPath.trimmed().isEmpty())
    {
        return {};
    }

    QFile projectFile(QDir(projectRootPath).filePath(QStringLiteral("project.json")));
    if (!projectFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return {};
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(projectFile.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        return {};
    }

    return document.object().value(QStringLiteral("project_name")).toString().trimmed();
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

void RequirementWidget::NormalizeWorkingKeyPoseIdsAndNames()
{
    for (std::size_t index = 0; index < m_working_model.workspace_requirements.key_poses.size(); ++index)
    {
        auto& keyPose = m_working_model.workspace_requirements.key_poses[index];
        const int displayIndex = static_cast<int>(index) + 1;
        keyPose.pose_id = QStringLiteral("pose_%1").arg(displayIndex, 3, 10, QChar('0'));
        keyPose.name = QStringLiteral("工位%1").arg(displayIndex);
    }
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
    NormalizeWorkingKeyPoseIdsAndNames();

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
    if (m_current_key_pose_index >= 0 &&
        m_current_key_pose_index < static_cast<int>(m_working_model.workspace_requirements.key_poses.size()))
    {
        keyPose = m_working_model.workspace_requirements.key_poses[static_cast<std::size_t>(m_current_key_pose_index)];
    }
    m_working_model.workspace_requirements.key_poses.push_back(keyPose);
    NormalizeWorkingKeyPoseIdsAndNames();

    RefreshKeyPoseList();
    m_current_key_pose_index = static_cast<int>(m_working_model.workspace_requirements.key_poses.size()) - 1;

    {
        const QSignalBlocker blocker(m_key_pose_list);
        m_key_pose_list->setCurrentRow(m_current_key_pose_index);
    }
    LoadCurrentKeyPoseToEditor();
    MarkDirty();
    EmitRequirementPreview();
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
    NormalizeWorkingKeyPoseIdsAndNames();

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
    EmitRequirementPreview();
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
    EmitKeyPosePreview();
}

void RequirementWidget::OnPreviewVisibilityChanged()
{
    EmitRequirementPreview();
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
