#include "modules/topology/ui/TopologyWidget.h"

#include "core/infrastructure/ProjectManager.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QStringList>
#include <QVBoxLayout>

namespace RoboSDP::Topology::Ui
{

namespace
{

/**
 * @brief 将逗号分隔的中空关节字符串整理为数组。
 * 保持 UI 只负责轻量字符串编辑，DTO 中仍保存结构化关节 ID 集合。
 */
std::vector<QString> ParseJointIdList(const QString& text)
{
    std::vector<QString> values;
    const QStringList tokens =
        text.split(QRegularExpression(QStringLiteral("[,，;；\\s]+")), Qt::SkipEmptyParts);
    values.reserve(static_cast<std::size_t>(tokens.size()));
    for (const QString& token : tokens)
    {
        const QString normalized = token.trimmed();
        if (!normalized.isEmpty())
        {
            values.push_back(normalized);
        }
    }

    return values;
}

/// 将关节 ID 集合回填到单行编辑器，方便最小骨架阶段快速编辑。
QString JoinJointIdList(const std::vector<QString>& values)
{
    QStringList tokens;
    tokens.reserve(static_cast<int>(values.size()));
    for (const QString& value : values)
    {
        const QString normalized = value.trimmed();
        if (!normalized.isEmpty())
        {
            tokens.push_back(normalized);
        }
    }

    return tokens.join(QStringLiteral(", "));
}

/**
 * @brief 解析轴线关系文本。
 * 每行格式固定为 `joint_a,joint_b,relation_type`，便于最小增量阶段快速录入。
 */
std::vector<RoboSDP::Topology::Dto::TopologyAxisRelationDto> ParseAxisRelationsText(const QString& text)
{
    std::vector<RoboSDP::Topology::Dto::TopologyAxisRelationDto> values;
    const QStringList lines = text.split(QRegularExpression(QStringLiteral("[\\r\\n]+")), Qt::SkipEmptyParts);
    values.reserve(static_cast<std::size_t>(lines.size()));
    for (const QString& line : lines)
    {
        const QStringList columns =
            line.split(QRegularExpression(QStringLiteral("[,，;；\\s]+")), Qt::SkipEmptyParts);
        if (columns.isEmpty())
        {
            continue;
        }

        RoboSDP::Topology::Dto::TopologyAxisRelationDto relation;
        if (columns.size() > 0)
        {
            relation.joint_pair[0] = columns.at(0).trimmed();
        }
        if (columns.size() > 1)
        {
            relation.joint_pair[1] = columns.at(1).trimmed();
        }
        if (columns.size() > 2)
        {
            relation.relation_type = columns.at(2).trimmed();
        }
        values.push_back(relation);
    }

    return values;
}

/// 将轴线关系列表回填为多行文本，方便最小骨架阶段编辑与审查。
QString BuildAxisRelationsText(const std::vector<RoboSDP::Topology::Dto::TopologyAxisRelationDto>& values)
{
    QStringList lines;
    lines.reserve(static_cast<int>(values.size()));
    for (const auto& relation : values)
    {
        lines.push_back(
            QStringLiteral("%1, %2, %3")
                .arg(relation.joint_pair[0], relation.joint_pair[1], relation.relation_type));
    }

    return lines.join(QLatin1Char('\n'));
}

} // namespace

TopologyWidget::TopologyWidget(QWidget* parent)
    : QWidget(parent)
    , m_requirement_storage(m_repository)
    , m_topology_storage(m_repository)
    , m_service(m_topology_storage, m_validator, m_template_loader, m_requirement_storage, &m_logger)
    , m_state(m_service.CreateDefaultState())
{
    BuildUi();
    auto& projectManager = RoboSDP::Infrastructure::ProjectManager::instance();
    connect(
        &projectManager,
        &RoboSDP::Infrastructure::ProjectManager::projectPathChanged,
        this,
        [this](const QString& newPath) {
            // 中文说明：项目目录只展示全局 ProjectManager 的当前路径，不作为模块私有状态。
            m_project_root_edit->setText(QDir::toNativeSeparators(newPath));
        });
    if (!projectManager.getCurrentProjectPath().isEmpty())
    {
        m_project_root_edit->setText(QDir::toNativeSeparators(projectManager.getCurrentProjectPath()));
    }
    PopulateForm(m_state.current_model);
    RefreshTemplateOptions();
    RenderCandidates();
    RenderRecommendation();
}

QString TopologyWidget::ModuleName() const
{
    return QStringLiteral("Topology");
}

RoboSDP::Infrastructure::ProjectSaveItemResult TopologyWidget::SaveCurrentDraft()
{
    m_state.current_model = CollectModelFromForm();
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto saveResult = m_service.SaveDraft(projectRootPath, m_state);
    ApplyValidationResult(saveResult.validation_result);
    SetOperationMessage(saveResult.message, saveResult.IsSuccess());
    return {ModuleName(), saveResult.IsSuccess(), saveResult.message};
}

void TopologyWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    auto* projectPathLayout = new QHBoxLayout();
    auto* projectPathLabel = new QLabel(QStringLiteral("项目目录"), this);
    m_project_root_edit = new QLineEdit(this);
    m_project_root_edit->setReadOnly(true);
    m_project_root_edit->setPlaceholderText(QStringLiteral("请先通过顶部功能区新建或打开项目"));
    m_browse_button = new QPushButton(QStringLiteral("选择目录"), this);
    m_browse_button->setVisible(false);
    m_browse_button->setToolTip(QStringLiteral("项目目录已改由顶部功能区统一管理。"));
    projectPathLayout->addWidget(projectPathLabel);
    projectPathLayout->addWidget(m_project_root_edit, 1);
    projectPathLayout->addWidget(m_browse_button);

    auto* templateLayout = new QHBoxLayout();
    auto* templateLabel = new QLabel(QStringLiteral("构型模板"), this);
    m_template_combo = new QComboBox(this);
    m_refresh_template_button = new QPushButton(QStringLiteral("刷新模板"), this);
    templateLayout->addWidget(templateLabel);
    templateLayout->addWidget(m_template_combo, 1);
    templateLayout->addWidget(m_refresh_template_button);

    auto* actionLayout = new QHBoxLayout();
    m_generate_button = new QPushButton(QStringLiteral("生成候选"), this);
    m_validate_button = new QPushButton(QStringLiteral("校验"), this);
    m_save_button = new QPushButton(QStringLiteral("保存草稿"), this);
    m_load_button = new QPushButton(QStringLiteral("重新加载"), this);
    actionLayout->addWidget(m_generate_button);
    actionLayout->addWidget(m_validate_button);
    actionLayout->addWidget(m_save_button);
    actionLayout->addWidget(m_load_button);
    actionLayout->addStretch();

    m_operation_label = new QLabel(
        QStringLiteral("就绪：请先保存 Requirement，再生成或编辑 Topology 草稿。"),
        this);
    m_operation_label->setWordWrap(true);

    auto* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);

    auto* scrollContent = new QWidget(scrollArea);
    auto* contentLayout = new QVBoxLayout(scrollContent);
    contentLayout->setContentsMargins(4, 4, 4, 4);
    contentLayout->setSpacing(8);
    contentLayout->addWidget(CreateTopologyGroup());
    contentLayout->addWidget(CreateCandidateGroup());
    contentLayout->addWidget(CreateValidationGroup());
    contentLayout->addStretch();
    scrollArea->setWidget(scrollContent);

    rootLayout->addLayout(projectPathLayout);
    rootLayout->addLayout(templateLayout);
    rootLayout->addLayout(actionLayout);
    rootLayout->addWidget(m_operation_label);
    rootLayout->addWidget(scrollArea, 1);

    connect(
        m_refresh_template_button,
        &QPushButton::clicked,
        this,
        [this]() { OnRefreshTemplatesClicked(); });
    connect(m_generate_button, &QPushButton::clicked, this, [this]() { OnGenerateClicked(); });
    connect(m_validate_button, &QPushButton::clicked, this, [this]() { OnValidateClicked(); });
    connect(m_save_button, &QPushButton::clicked, this, [this]() { OnSaveDraftClicked(); });
    connect(m_load_button, &QPushButton::clicked, this, [this]() { OnLoadClicked(); });
}

void TopologyWidget::RefreshTemplateOptions()
{
    const QString previousTemplateId = m_template_combo->currentData().toString();
    m_template_combo->clear();
    m_template_combo->addItem(QStringLiteral("全部模板（推荐）"), QStringLiteral("__all__"));

    const auto templates = m_service.ListTemplates();
    for (const auto& templateSummary : templates)
    {
        m_template_combo->addItem(templateSummary.display_name, templateSummary.template_id);
        const int index = m_template_combo->count() - 1;
        m_template_combo->setItemData(index, templateSummary.description, Qt::ToolTipRole);
    }

    const int previousIndex = m_template_combo->findData(previousTemplateId);
    if (previousIndex >= 0)
    {
        m_template_combo->setCurrentIndex(previousIndex);
    }
}

QGroupBox* TopologyWidget::CreateTopologyGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("构型骨架"), this);
    auto* layout = new QFormLayout(groupBox);

    m_topology_name_edit = new QLineEdit(groupBox);
    m_base_mount_combo = new QComboBox(groupBox);
    m_base_mount_combo->addItem(QStringLiteral("落地"), QStringLiteral("floor"));
    m_base_mount_combo->addItem(QStringLiteral("壁挂"), QStringLiteral("wall"));
    m_base_mount_combo->addItem(QStringLiteral("顶装"), QStringLiteral("ceiling"));
    m_base_mount_combo->addItem(QStringLiteral("立柱"), QStringLiteral("pedestal"));
    m_base_height_spin = CreateDoubleSpinBox(0.0, 1.0e6, 4, 0.01);
    m_base_orientation_x_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_base_orientation_y_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_base_orientation_z_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_j1_range_min_spin = CreateDoubleSpinBox(-720.0, 720.0, 3, 1.0);
    m_j1_range_max_spin = CreateDoubleSpinBox(-720.0, 720.0, 3, 1.0);
    m_shoulder_type_edit = new QLineEdit(groupBox);
    m_elbow_type_edit = new QLineEdit(groupBox);
    m_wrist_type_edit = new QLineEdit(groupBox);
    m_wrist_intersection_check = new QCheckBox(QStringLiteral("启用腕部交点"), groupBox);
    m_wrist_offset_check = new QCheckBox(QStringLiteral("启用腕部偏置"), groupBox);
    m_internal_routing_check = new QCheckBox(QStringLiteral("需要内部走线"), groupBox);
    m_hollow_wrist_check = new QCheckBox(QStringLiteral("需要中空腕"), groupBox);
    m_reserved_channel_spin = CreateDoubleSpinBox(0.0, 1.0e6, 2, 1.0);
    m_seventh_axis_reserved_check = new QCheckBox(QStringLiteral("预留第七轴"), groupBox);
    m_hollow_joint_ids_edit = new QLineEdit(groupBox);
    m_axis_relations_edit = new QPlainTextEdit(groupBox);
    m_axis_relations_edit->setPlaceholderText(
        QStringLiteral("joint_2, joint_3, parallel\njoint_4, joint_5, perpendicular"));
    m_axis_relations_edit->setMinimumHeight(84);

    /**
     * @brief 最小骨架阶段使用固定 6 轴角色编辑器承接 joint_roles。
     * 先保证关节功能分配可编辑、可保存、可回读，不提前引入复杂表格。
     */
    for (int index = 0; index < static_cast<int>(m_joint_role_edits.size()); ++index)
    {
        m_joint_role_edits[static_cast<std::size_t>(index)] = new QLineEdit(groupBox);
    }

    layout->addRow(QStringLiteral("构型名称"), m_topology_name_edit);
    layout->addRow(QStringLiteral("基座安装方式"), m_base_mount_combo);
    layout->addRow(QStringLiteral("基座高度 [m]"), m_base_height_spin);
    layout->addRow(QStringLiteral("基座 RX [deg]"), m_base_orientation_x_spin);
    layout->addRow(QStringLiteral("基座 RY [deg]"), m_base_orientation_y_spin);
    layout->addRow(QStringLiteral("基座 RZ [deg]"), m_base_orientation_z_spin);
    layout->addRow(QStringLiteral("J1 行程最小 [deg]"), m_j1_range_min_spin);
    layout->addRow(QStringLiteral("J1 行程最大 [deg]"), m_j1_range_max_spin);
    layout->addRow(QStringLiteral("肩部形式"), m_shoulder_type_edit);
    layout->addRow(QStringLiteral("肘部形式"), m_elbow_type_edit);
    layout->addRow(QStringLiteral("腕部形式"), m_wrist_type_edit);
    layout->addRow(QStringLiteral("腕部交点"), m_wrist_intersection_check);
    layout->addRow(QStringLiteral("腕部偏置"), m_wrist_offset_check);
    layout->addRow(QStringLiteral("内部走线"), m_internal_routing_check);
    layout->addRow(QStringLiteral("中空腕"), m_hollow_wrist_check);
    layout->addRow(QStringLiteral("预留通道直径 [mm]"), m_reserved_channel_spin);
    layout->addRow(QStringLiteral("预留第七轴"), m_seventh_axis_reserved_check);
    layout->addRow(QStringLiteral("中空关节 ID"), m_hollow_joint_ids_edit);
    layout->addRow(QStringLiteral("轴线关系"), m_axis_relations_edit);
    layout->addRow(QStringLiteral("J1 功能角色"), m_joint_role_edits[0]);
    layout->addRow(QStringLiteral("J2 功能角色"), m_joint_role_edits[1]);
    layout->addRow(QStringLiteral("J3 功能角色"), m_joint_role_edits[2]);
    layout->addRow(QStringLiteral("J4 功能角色"), m_joint_role_edits[3]);
    layout->addRow(QStringLiteral("J5 功能角色"), m_joint_role_edits[4]);
    layout->addRow(QStringLiteral("J6 功能角色"), m_joint_role_edits[5]);

    RegisterFieldWidget(QStringLiteral("meta.name"), m_topology_name_edit);
    RegisterFieldWidget(QStringLiteral("robot_definition.base_mount_type"), m_base_mount_combo);
    RegisterFieldWidget(QStringLiteral("robot_definition.base_height_m"), m_base_height_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.base_orientation[0]"), m_base_orientation_x_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.base_orientation[1]"), m_base_orientation_y_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.base_orientation[2]"), m_base_orientation_z_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.j1_rotation_range_deg[0]"), m_j1_range_min_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.j1_rotation_range_deg[1]"), m_j1_range_max_spin);
    RegisterFieldWidget(QStringLiteral("layout.shoulder_type"), m_shoulder_type_edit);
    RegisterFieldWidget(QStringLiteral("layout.elbow_type"), m_elbow_type_edit);
    RegisterFieldWidget(QStringLiteral("layout.wrist_type"), m_wrist_type_edit);
    RegisterFieldWidget(QStringLiteral("layout.wrist_intersection"), m_wrist_intersection_check);
    RegisterFieldWidget(QStringLiteral("layout.wrist_offset"), m_wrist_offset_check);
    RegisterFieldWidget(QStringLiteral("layout.internal_routing_required"), m_internal_routing_check);
    RegisterFieldWidget(QStringLiteral("layout.hollow_wrist_required"), m_hollow_wrist_check);
    RegisterFieldWidget(QStringLiteral("layout.reserved_channel_diameter_mm"), m_reserved_channel_spin);
    RegisterFieldWidget(QStringLiteral("layout.seventh_axis_reserved"), m_seventh_axis_reserved_check);
    RegisterFieldWidget(QStringLiteral("layout.hollow_joint_ids"), m_hollow_joint_ids_edit);
    RegisterFieldWidget(QStringLiteral("axis_relations"), m_axis_relations_edit);
    for (int index = 0; index < static_cast<int>(m_joint_role_edits.size()); ++index)
    {
        RegisterFieldWidget(
            QStringLiteral("joints[%1].role").arg(index),
            m_joint_role_edits[static_cast<std::size_t>(index)]);
    }

    return groupBox;
}

QGroupBox* TopologyWidget::CreateCandidateGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("候选与推荐"), this);
    auto* layout = new QVBoxLayout(groupBox);

    m_candidate_summary_label = new QLabel(QStringLiteral("尚未生成候选构型。"), groupBox);
    m_candidate_summary_label->setWordWrap(true);
    m_candidate_list = new QListWidget(groupBox);
    m_candidate_list->setSelectionMode(QAbstractItemView::NoSelection);

    m_recommendation_summary_label = new QLabel(QStringLiteral("尚未生成推荐结果。"), groupBox);
    m_recommendation_summary_label->setWordWrap(true);
    m_recommendation_reason_list = new QListWidget(groupBox);
    m_recommendation_reason_list->setSelectionMode(QAbstractItemView::NoSelection);

    layout->addWidget(m_candidate_summary_label);
    layout->addWidget(m_candidate_list);
    layout->addWidget(m_recommendation_summary_label);
    layout->addWidget(m_recommendation_reason_list);

    return groupBox;
}

QGroupBox* TopologyWidget::CreateValidationGroup()
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

QDoubleSpinBox* TopologyWidget::CreateDoubleSpinBox(
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

void TopologyWidget::RegisterFieldWidget(const QString& fieldPath, QWidget* widget)
{
    m_field_widgets.insert(fieldPath, widget);
}

RoboSDP::Topology::Dto::RobotTopologyModelDto TopologyWidget::CollectModelFromForm() const
{
    RoboSDP::Topology::Dto::RobotTopologyModelDto model = m_state.current_model;

    model.meta.name = m_topology_name_edit->text().trimmed();
    model.robot_definition.base_mount_type = m_base_mount_combo->currentData().toString();
    model.robot_definition.base_height_m = m_base_height_spin->value();
    model.robot_definition.base_orientation = {
        m_base_orientation_x_spin->value(),
        m_base_orientation_y_spin->value(),
        m_base_orientation_z_spin->value()};
    model.robot_definition.j1_rotation_range_deg = {
        m_j1_range_min_spin->value(),
        m_j1_range_max_spin->value()};

    model.layout.shoulder_type = m_shoulder_type_edit->text().trimmed();
    model.layout.elbow_type = m_elbow_type_edit->text().trimmed();
    model.layout.wrist_type = m_wrist_type_edit->text().trimmed();
    model.layout.wrist_intersection = m_wrist_intersection_check->isChecked();
    model.layout.wrist_offset = m_wrist_offset_check->isChecked();
    model.layout.internal_routing_required = m_internal_routing_check->isChecked();
    model.layout.hollow_wrist_required = m_hollow_wrist_check->isChecked();
    model.layout.reserved_channel_diameter_mm = m_reserved_channel_spin->value();
    model.layout.seventh_axis_reserved = m_seventh_axis_reserved_check->isChecked();
    model.layout.hollow_joint_ids = ParseJointIdList(m_hollow_joint_ids_edit->text());
    model.axis_relations = ParseAxisRelationsText(m_axis_relations_edit->toPlainText());

    for (int index = 0; index < static_cast<int>(m_joint_role_edits.size()) &&
                        index < static_cast<int>(model.joints.size());
         ++index)
    {
        model.joints[static_cast<std::size_t>(index)].role =
            m_joint_role_edits[static_cast<std::size_t>(index)]->text().trimmed();
    }

    return model;
}

void TopologyWidget::PopulateForm(const RoboSDP::Topology::Dto::RobotTopologyModelDto& model)
{
    m_topology_name_edit->setText(model.meta.name);

    const int baseMountIndex = m_base_mount_combo->findData(model.robot_definition.base_mount_type);
    m_base_mount_combo->setCurrentIndex(baseMountIndex >= 0 ? baseMountIndex : 0);

    m_base_height_spin->setValue(model.robot_definition.base_height_m);
    m_base_orientation_x_spin->setValue(model.robot_definition.base_orientation[0]);
    m_base_orientation_y_spin->setValue(model.robot_definition.base_orientation[1]);
    m_base_orientation_z_spin->setValue(model.robot_definition.base_orientation[2]);
    m_j1_range_min_spin->setValue(model.robot_definition.j1_rotation_range_deg[0]);
    m_j1_range_max_spin->setValue(model.robot_definition.j1_rotation_range_deg[1]);

    m_shoulder_type_edit->setText(model.layout.shoulder_type);
    m_elbow_type_edit->setText(model.layout.elbow_type);
    m_wrist_type_edit->setText(model.layout.wrist_type);
    m_wrist_intersection_check->setChecked(model.layout.wrist_intersection);
    m_wrist_offset_check->setChecked(model.layout.wrist_offset);
    m_internal_routing_check->setChecked(model.layout.internal_routing_required);
    m_hollow_wrist_check->setChecked(model.layout.hollow_wrist_required);
    m_reserved_channel_spin->setValue(model.layout.reserved_channel_diameter_mm);
    m_seventh_axis_reserved_check->setChecked(model.layout.seventh_axis_reserved);
    m_hollow_joint_ids_edit->setText(JoinJointIdList(model.layout.hollow_joint_ids));
    m_axis_relations_edit->setPlainText(BuildAxisRelationsText(model.axis_relations));

    for (int index = 0; index < static_cast<int>(m_joint_role_edits.size()); ++index)
    {
        const QString role = index < static_cast<int>(model.joints.size())
            ? model.joints[static_cast<std::size_t>(index)].role
            : QString();
        m_joint_role_edits[static_cast<std::size_t>(index)]->setText(role);
    }
}

void TopologyWidget::RenderCandidates()
{
    m_candidate_list->clear();

    if (m_state.candidates.empty())
    {
        m_candidate_summary_label->setText(QStringLiteral("尚未生成候选构型。"));
        return;
    }

    m_candidate_summary_label->setText(
        QStringLiteral("已生成 %1 个模板化 6R 串联候选构型。").arg(m_state.candidates.size()));

    for (const auto& candidate : m_state.candidates)
    {
        QString itemText = QStringLiteral("%1 | 模板=%2 | 评分=%3")
                               .arg(candidate.title, candidate.template_id)
                               .arg(candidate.score, 0, 'f', 1);

        if (candidate.candidate_id == m_state.recommendation.recommended_candidate_id)
        {
            itemText += QStringLiteral(" | 推荐");
        }

        auto* item = new QListWidgetItem(itemText, m_candidate_list);
        QString toolTip;
        for (const QString& reason : candidate.recommendation_reason)
        {
            if (!toolTip.isEmpty())
            {
                toolTip += QLatin1Char('\n');
            }

            toolTip += QStringLiteral("- %1").arg(reason);
        }
        item->setToolTip(toolTip);
    }
}

void TopologyWidget::RenderRecommendation()
{
    m_recommendation_reason_list->clear();

    if (m_state.recommendation.recommended_candidate_id.trimmed().isEmpty())
    {
        m_recommendation_summary_label->setText(QStringLiteral("尚未生成推荐结果。"));
        return;
    }

    m_recommendation_summary_label->setText(
        QStringLiteral("推荐模板：%1，综合评分 %2。")
            .arg(m_state.recommendation.recommended_template_id)
            .arg(m_state.recommendation.combined_score, 0, 'f', 1));

    for (const QString& reason : m_state.recommendation.recommendation_reason)
    {
        new QListWidgetItem(reason, m_recommendation_reason_list);
    }
}

void TopologyWidget::ClearValidationState()
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

void TopologyWidget::ApplyValidationResult(
    const RoboSDP::Topology::Validation::TopologyValidationResult& result)
{
    ClearValidationState();

    if (result.IsValid())
    {
        m_validation_summary_label->setStyleSheet(QStringLiteral("color: #1b7f3b;"));
        m_validation_summary_label->setText(
            QStringLiteral("Topology 基础校验通过，可继续保存并流向 Kinematics。"));
        return;
    }

    m_validation_summary_label->setStyleSheet(QStringLiteral("color: #b42318;"));
    m_validation_summary_label->setText(
        QStringLiteral("Topology 基础校验发现 %1 条问题，其中错误 %2 条，警告 %3 条。")
            .arg(result.issues.size())
            .arg(result.ErrorCount())
            .arg(result.WarningCount()));

    for (const auto& issue : result.issues)
    {
        auto* item = new QListWidgetItem(
            QStringLiteral("[%1] %2 - %3")
                .arg(RoboSDP::Topology::Validation::ToString(issue.severity), issue.field, issue.message_zh),
            m_validation_issue_list);
        item->setToolTip(issue.code);

        QWidget* fieldWidget = m_field_widgets.value(issue.field, nullptr);
        if (fieldWidget == nullptr && issue.field.startsWith(QStringLiteral("layout.hollow_joint_ids[")))
        {
            fieldWidget = m_hollow_joint_ids_edit;
        }
        if (fieldWidget == nullptr && issue.field.startsWith(QStringLiteral("axis_relations[")))
        {
            fieldWidget = m_axis_relations_edit;
        }
        if (fieldWidget != nullptr)
        {
            fieldWidget->setStyleSheet(
                issue.severity == RoboSDP::Topology::Validation::ValidationSeverity::Warning
                    ? QStringLiteral("border: 1px solid #d97706;")
                    : QStringLiteral("border: 1px solid #dc2626;"));
            fieldWidget->setToolTip(issue.message_zh);
        }
    }
}

void TopologyWidget::SetOperationMessage(const QString& message, bool success)
{
    m_operation_label->setText(message);
    m_operation_label->setStyleSheet(success ? QStringLiteral("color: #1b7f3b;")
                                             : QStringLiteral("color: #b42318;"));
}

void TopologyWidget::OnBrowseProjectRootClicked()
{
    const QString selectedDirectory = QFileDialog::getExistingDirectory(
        this,
        QStringLiteral("选择 Topology 项目目录"),
        m_project_root_edit->text().trimmed());

    if (!selectedDirectory.isEmpty())
    {
        m_project_root_edit->setText(QDir::toNativeSeparators(selectedDirectory));
    }
}

void TopologyWidget::OnRefreshTemplatesClicked()
{
    RefreshTemplateOptions();
    SetOperationMessage(QStringLiteral("构型模板列表已刷新。"), true);
    emit LogMessageGenerated(QStringLiteral("[Topology] 构型模板列表已刷新。"));
}

void TopologyWidget::OnGenerateClicked()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto generateResult = m_service.GenerateCandidatesFromRequirement(
        projectRootPath,
        m_template_combo->currentData().toString());

    if (generateResult.IsSuccess())
    {
        m_state = generateResult.state;
        PopulateForm(m_state.current_model);
        RenderCandidates();
        RenderRecommendation();
        ApplyValidationResult(generateResult.validation_result);
    }

    SetOperationMessage(generateResult.message, generateResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Topology] %1").arg(generateResult.message));
}

void TopologyWidget::OnValidateClicked()
{
    m_state.current_model = CollectModelFromForm();
    const auto validationResult = m_service.Validate(m_state.current_model);
    ApplyValidationResult(validationResult);

    SetOperationMessage(
        validationResult.IsValid()
            ? QStringLiteral("Topology 基础校验通过。")
            : QStringLiteral("Topology 基础校验未通过，请根据问题列表修正字段。"),
        validationResult.IsValid());

    emit LogMessageGenerated(
        validationResult.IsValid()
            ? QStringLiteral("[Topology] 基础校验通过。")
            : QStringLiteral("[Topology] 基础校验发现 %1 条问题。").arg(validationResult.issues.size()));
}

void TopologyWidget::OnSaveDraftClicked()
{
    const auto saveResult = SaveCurrentDraft();
    emit LogMessageGenerated(QStringLiteral("[Topology] %1").arg(saveResult.message));
}

void TopologyWidget::OnLoadClicked()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto loadResult = m_service.LoadDraft(projectRootPath);
    if (loadResult.IsSuccess())
    {
        m_state = loadResult.state;
        PopulateForm(m_state.current_model);
        RenderCandidates();
        RenderRecommendation();
        ApplyValidationResult(loadResult.validation_result);
    }

    SetOperationMessage(loadResult.message, loadResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Topology] %1").arg(loadResult.message));
}

} // namespace RoboSDP::Topology::Ui
