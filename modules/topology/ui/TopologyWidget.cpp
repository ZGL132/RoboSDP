#include "modules/topology/ui/TopologyWidget.h"

#include "core/infrastructure/ProjectManager.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
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
#include <QTabWidget>
#include <QVBoxLayout>

// 引入 KinematicsService 以使用其数学引擎将 Topology 转为可视化的预览场景
#include "modules/kinematics/service/KinematicsService.h"

namespace RoboSDP::Topology::Ui
{

namespace
{

/**
 * @brief 将用户输入的逗号分隔的中空关节字符串（例如 "joint_1, joint_2"）整理并拆分为字符串数组。
 * 这样做可以让 UI 保持轻量的单行文本输入，但在底层 DTO 中存储严谨的集合结构。
 */
std::vector<QString> ParseJointIdList(const QString& text)
{
    std::vector<QString> values;
    // 使用正则支持逗号、分号和空格分隔符
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

/**
 * @brief 将关节 ID 集合重新拼接为单行字符串，方便在 UI 输入框中显示和二次编辑。
 */
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
 * @brief 解析多行轴线关系文本。每行按逗号拆分成 `joint_a`, `joint_b`, `relation_type`。
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

/**
 * @brief 将轴线关系对象集合还原为多行文本，供 UI QPlainTextEdit 展示。
 */
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

// ==================== 生命周期与初始化 ====================

TopologyWidget::TopologyWidget(QWidget* parent)
    : QWidget(parent)
    , m_requirement_storage(m_repository)
    , m_topology_storage(m_repository)
    , m_service(m_topology_storage, m_validator, m_template_loader, m_requirement_storage, &m_logger)
    , m_state(m_service.CreateDefaultState())
{
    // 1. 搭建 UI 组件树
    BuildUi();
    // 2. 将默认状态/加载的状态填充至表单
    PopulateForm(m_state.current_model);
    // 3. 加载可用模板下拉框
    RefreshTemplateOptions();
    // 4. 渲染生成的候选和推荐面板
    RenderCandidates();
    RenderRecommendation();
    // 5. 挂载数据变动监听
    ConnectDirtyTracking();
    // 6. 初始状态标记为干净
    MarkClean();
}

QString TopologyWidget::ModuleName() const
{
    return QStringLiteral("Topology");
}

bool TopologyWidget::HasUnsavedChanges() const
{
    return m_has_unsaved_changes;
}

RoboSDP::Infrastructure::ProjectSaveItemResult TopologyWidget::SaveCurrentDraft()
{
    // 从表单收集最新数据并保存到磁盘
    m_state.current_model = CollectModelFromForm();
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto saveResult = m_service.SaveDraft(projectRootPath, m_state);
    
    // 保存时附带校验逻辑，如果数据不合法，表单会亮红框提示
    ApplyValidationResult(saveResult.validation_result);
    SetOperationMessage(saveResult.message, saveResult.IsSuccess());
    
    if (saveResult.IsSuccess())
    {
        MarkClean();
    }
    return {ModuleName(), saveResult.IsSuccess(), saveResult.message};
}

// ==================== 脏检查与实时事件追踪 ====================

void TopologyWidget::ConnectDirtyTracking()
{
    // 遍历所有特定类型的输入控件，一旦内容改变，就将标记设为 Dirty
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
            // 【核心特性】：由于 SpinBox 主要控制机器人的物理尺寸（DH参数），
            // 当尺寸发生任何滑动或改变时，立即触发 UpdateLivePreview() 更新 3D 骨架模型。
            UpdateLivePreview();
        });
    }
    for (QCheckBox* editor : findChildren<QCheckBox*>())
    {
        connect(editor, &QCheckBox::toggled, this, [this](bool) { MarkDirty(); });
    }
    for (QPlainTextEdit* editor : findChildren<QPlainTextEdit*>())
    {
        if (!editor->isReadOnly())
        {
            connect(editor, &QPlainTextEdit::textChanged, this, [this]() { MarkDirty(); });
        }
    }
}

void TopologyWidget::MarkDirty()
{
    m_has_unsaved_changes = true;
}

void TopologyWidget::MarkClean()
{
    m_has_unsaved_changes = false;
}

// ==================== UI 树构建逻辑 ====================

void TopologyWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    // 1. 顶部：模板选择区
    // 操作按钮已迁移至 Ribbon 构型工具页签，此处仅保留模板下拉框
    auto* templateLayout = new QHBoxLayout();
    auto* templateLabel = new QLabel(QStringLiteral("构型模板"), this);
    m_template_combo = new QComboBox(this);
    templateLayout->addWidget(templateLabel);
    templateLayout->addWidget(m_template_combo, 1);

    // 状态提示条
    m_operation_label = new QLabel(
        QStringLiteral("就绪：请先保存 Requirement，再生成或编辑 Topology 草稿。"),
        this);
    m_operation_label->setWordWrap(true);

    // 2. 中部：多页签面板区
    auto* tabs = new QTabWidget(this);
    tabs->setDocumentMode(true);
    // 按业务域拆分页面视图，将长表单放进 ScrollArea 中
    tabs->addTab(CreateScrollableTab(CreateTopologyGroup()), QStringLiteral("构型骨架"));
    tabs->addTab(CreateScrollableTab(CreateCandidateGroup()), QStringLiteral("候选方案"));
    tabs->addTab(CreateScrollableTab(CreateValidationGroup()), QStringLiteral("校验结果"));

    // 将所有布局添加到主根布局
    rootLayout->addLayout(templateLayout);
    rootLayout->addWidget(m_operation_label);
    rootLayout->addWidget(tabs, 1);
}

QWidget* TopologyWidget::CreateScrollableTab(QWidget* contentWidget)
{
    // 将普通 Widget 包装为带滚动条的视图，防止输入项过多时撑出屏幕外部
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

void TopologyWidget::RefreshTemplateOptions()
{
    // 刷新模板下拉框的内容
    const QString previousTemplateId = m_template_combo->currentData().toString();
    m_template_combo->clear();
    m_template_combo->addItem(QStringLiteral("全部模板（推荐）"), QStringLiteral("__all__"));

    const auto templates = m_service.ListTemplates();
    for (const auto& templateSummary : templates)
    {
        m_template_combo->addItem(templateSummary.display_name, templateSummary.template_id);
        const int index = m_template_combo->count() - 1;
        // 将模板描述设为 tooltip 方便提示
        m_template_combo->setItemData(index, templateSummary.description, Qt::ToolTipRole);
    }

    // 尝试恢复之前选中的模板
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
    
    // --- DH 运动学关键尺寸 (负责决定机器人在 3D 视图中的形状) ---
    m_base_height_spin = CreateDoubleSpinBox(0.0, 10.0, 4, 0.001);
    m_shoulder_offset_spin = CreateDoubleSpinBox(0.0, 10.0, 4, 0.001);
    m_upper_arm_length_spin = CreateDoubleSpinBox(0.001, 10.0, 4, 0.001);
    m_forearm_length_spin = CreateDoubleSpinBox(0.001, 10.0, 4, 0.001);
    m_wrist_offset_spin = CreateDoubleSpinBox(0.0, 10.0, 4, 0.001);

    // --- 机械与安装配置 ---
    m_base_mount_combo = new QComboBox(groupBox);
    m_base_mount_combo->addItem(QStringLiteral("落地"), QStringLiteral("floor"));
    m_base_mount_combo->addItem(QStringLiteral("壁挂"), QStringLiteral("wall"));
    m_base_mount_combo->addItem(QStringLiteral("顶装"), QStringLiteral("ceiling"));
    m_base_mount_combo->addItem(QStringLiteral("立柱"), QStringLiteral("pedestal"));
    m_base_orientation_x_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_base_orientation_y_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_base_orientation_z_spin = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
    m_j1_range_min_spin = CreateDoubleSpinBox(-720.0, 720.0, 3, 1.0);
    m_j1_range_max_spin = CreateDoubleSpinBox(-720.0, 720.0, 3, 1.0);
    
    // --- 走线与扩展配置 ---
    m_internal_routing_check = new QCheckBox(QStringLiteral("需要内部走线"), groupBox);
    m_hollow_wrist_check = new QCheckBox(QStringLiteral("需要中空腕"), groupBox);
    m_reserved_channel_spin = CreateDoubleSpinBox(0.0, 1.0e6, 2, 1.0);
    m_seventh_axis_reserved_check = new QCheckBox(QStringLiteral("预留第七轴"), groupBox);
    m_hollow_joint_ids_edit = new QLineEdit(groupBox);
    m_hollow_joint_ids_edit->setPlaceholderText(QStringLiteral("例如: joint_4, joint_5, joint_6"));

    // 将上面创建的控件添加到表单布局中，并使用 QLabel 作为分隔区域的标题
    layout->addRow(QStringLiteral("构型名称"), m_topology_name_edit);
    
    auto* labelDH = new QLabel(QStringLiteral("--- 运动学关键尺寸 (DH) ---"), groupBox);
    labelDH->setStyleSheet(QStringLiteral("color: #0284c7; font-weight: bold; margin-top: 10px;"));
    layout->addRow(labelDH);
    layout->addRow(QStringLiteral("基座高度 (d1) [m]"), m_base_height_spin);
    layout->addRow(QStringLiteral("肩部偏置 (a1) [m]"), m_shoulder_offset_spin);
    layout->addRow(QStringLiteral("大臂长度 (a2) [m]"), m_upper_arm_length_spin);
    layout->addRow(QStringLiteral("小臂长度 (d4) [m]"), m_forearm_length_spin);
    layout->addRow(QStringLiteral("腕法兰偏置 (d6) [m]"), m_wrist_offset_spin);

    auto* labelMech = new QLabel(QStringLiteral("--- 附加机械与安装配置 ---"), groupBox);
    labelMech->setStyleSheet(QStringLiteral("color: #4b5563; font-weight: bold; margin-top: 10px;"));
    layout->addRow(labelMech);
    layout->addRow(QStringLiteral("基座安装方式"), m_base_mount_combo);
    layout->addRow(QStringLiteral("基座 RX [deg]"), m_base_orientation_x_spin);
    layout->addRow(QStringLiteral("基座 RY [deg]"), m_base_orientation_y_spin);
    layout->addRow(QStringLiteral("基座 RZ [deg]"), m_base_orientation_z_spin);
    layout->addRow(QStringLiteral("J1 行程最小 [deg]"), m_j1_range_min_spin);
    layout->addRow(QStringLiteral("J1 行程最大 [deg]"), m_j1_range_max_spin);
    
    auto* labelWiring = new QLabel(QStringLiteral("--- 走线预留与扩展 ---"), groupBox);
    labelWiring->setStyleSheet(QStringLiteral("color: #4b5563; font-weight: bold; margin-top: 10px;"));
    layout->addRow(labelWiring);
    layout->addRow(QStringLiteral("内部走线"), m_internal_routing_check);
    layout->addRow(QStringLiteral("中空腕"), m_hollow_wrist_check);
    layout->addRow(QStringLiteral("预留通道直径 [mm]"), m_reserved_channel_spin);
    layout->addRow(QStringLiteral("中空关节 ID"), m_hollow_joint_ids_edit);
    layout->addRow(QStringLiteral("预留第七轴"), m_seventh_axis_reserved_check);

    // 注册字段路径（JSON路径）与 UI 控件的映射关系。
    // 这一步非常重要：当后台校验（Validate）发现某个参数（如 d1 越界）出错时，
    // UI 可以通过这个映射表找到对应的控件，将其边框变红。
    RegisterFieldWidget(QStringLiteral("meta.name"), m_topology_name_edit);
    RegisterFieldWidget(QStringLiteral("robot_definition.base_height_m"), m_base_height_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.shoulder_offset_m"), m_shoulder_offset_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.upper_arm_length_m"), m_upper_arm_length_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.forearm_length_m"), m_forearm_length_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.wrist_offset_m"), m_wrist_offset_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.base_mount_type"), m_base_mount_combo);
    RegisterFieldWidget(QStringLiteral("robot_definition.j1_rotation_range_deg[0]"), m_j1_range_min_spin);
    RegisterFieldWidget(QStringLiteral("robot_definition.j1_rotation_range_deg[1]"), m_j1_range_max_spin);
    RegisterFieldWidget(QStringLiteral("layout.reserved_channel_diameter_mm"), m_reserved_channel_spin);
    RegisterFieldWidget(QStringLiteral("layout.hollow_joint_ids"), m_hollow_joint_ids_edit);

    return groupBox;
}

QGroupBox* TopologyWidget::CreateCandidateGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("候选与推荐"), this);
    auto* layout = new QVBoxLayout(groupBox);

    // 候选配置的展示面板
    m_candidate_summary_label = new QLabel(QStringLiteral("尚未生成候选构型。"), groupBox);
    m_candidate_summary_label->setWordWrap(true);
    m_candidate_list = new QListWidget(groupBox);
    m_candidate_list->setSelectionMode(QAbstractItemView::NoSelection);

    // 推荐理由的展示面板
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
    // 用于展示表单填写是否合规的错误信息面板
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
    // 工具方法：快速实例化带有统一行为的浮点数输入框
    auto* spinBox = new QDoubleSpinBox(this);
    spinBox->setRange(minimum, maximum);
    spinBox->setDecimals(decimals);
    spinBox->setSingleStep(step);
    spinBox->setAccelerated(true); // 支持长按键盘箭头加速增减
    return spinBox;
}

void TopologyWidget::RegisterFieldWidget(const QString& fieldPath, QWidget* widget)
{
    m_field_widgets.insert(fieldPath, widget);
}

// ==================== 数据流转逻辑 (DTO <-> UI) ====================

RoboSDP::Topology::Dto::RobotTopologyModelDto TopologyWidget::CollectModelFromForm() const
{
    // 将界面上控件里的值抽取出来，赋给内存中的模型 DTO
    RoboSDP::Topology::Dto::RobotTopologyModelDto model = m_state.current_model;

    model.meta.name = m_topology_name_edit->text().trimmed();
    
    model.robot_definition.base_mount_type = m_base_mount_combo->currentData().toString();
    model.robot_definition.base_height_m = m_base_height_spin->value();
    model.robot_definition.shoulder_offset_m = m_shoulder_offset_spin->value();
    model.robot_definition.upper_arm_length_m = m_upper_arm_length_spin->value();
    model.robot_definition.forearm_length_m = m_forearm_length_spin->value();
    model.robot_definition.wrist_offset_m = m_wrist_offset_spin->value();

    model.robot_definition.base_orientation = {
        m_base_orientation_x_spin->value(),
        m_base_orientation_y_spin->value(),
        m_base_orientation_z_spin->value()};
    model.robot_definition.j1_rotation_range_deg = {
        m_j1_range_min_spin->value(),
        m_j1_range_max_spin->value()};

    model.layout.internal_routing_required = m_internal_routing_check->isChecked();
    model.layout.hollow_wrist_required = m_hollow_wrist_check->isChecked();
    model.layout.reserved_channel_diameter_mm = m_reserved_channel_spin->value();
    model.layout.seventh_axis_reserved = m_seventh_axis_reserved_check->isChecked();
    model.layout.hollow_joint_ids = ParseJointIdList(m_hollow_joint_ids_edit->text());

    return model;
}

void TopologyWidget::PopulateForm(const RoboSDP::Topology::Dto::RobotTopologyModelDto& model)
{
    // 将内存模型 DTO 的值，反向填充回 UI 的控件中（主要用于加载草稿或生成候选后）
    m_topology_name_edit->setText(model.meta.name);

    const int baseMountIndex = m_base_mount_combo->findData(model.robot_definition.base_mount_type);
    m_base_mount_combo->setCurrentIndex(baseMountIndex >= 0 ? baseMountIndex : 0);

    m_base_height_spin->setValue(model.robot_definition.base_height_m);
    m_shoulder_offset_spin->setValue(model.robot_definition.shoulder_offset_m);
    m_upper_arm_length_spin->setValue(model.robot_definition.upper_arm_length_m);
    m_forearm_length_spin->setValue(model.robot_definition.forearm_length_m);
    m_wrist_offset_spin->setValue(model.robot_definition.wrist_offset_m);

    m_base_orientation_x_spin->setValue(model.robot_definition.base_orientation[0]);
    m_base_orientation_y_spin->setValue(model.robot_definition.base_orientation[1]);
    m_base_orientation_z_spin->setValue(model.robot_definition.base_orientation[2]);
    m_j1_range_min_spin->setValue(model.robot_definition.j1_rotation_range_deg[0]);
    m_j1_range_max_spin->setValue(model.robot_definition.j1_rotation_range_deg[1]);

    m_internal_routing_check->setChecked(model.layout.internal_routing_required);
    m_hollow_wrist_check->setChecked(model.layout.hollow_wrist_required);
    m_reserved_channel_spin->setValue(model.layout.reserved_channel_diameter_mm);
    m_seventh_axis_reserved_check->setChecked(model.layout.seventh_axis_reserved);
    m_hollow_joint_ids_edit->setText(JoinJointIdList(model.layout.hollow_joint_ids));
    
    // --- 新增：表单从数据层填充完毕后，立刻发送一次初始的三维预览请求 ---
    UpdateLivePreview();
}

// ==================== UI 渲染与更新 ====================

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

    // 遍历所有生成的构型候选方案，在列表中展示标题、评分等信息
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
        
        // 将推荐理由拼接到 Tooltip 中，鼠标悬浮时可查看详情
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

    // 逐条写入评分和推荐理由
    for (const QString& reason : m_state.recommendation.recommendation_reason)
    {
        new QListWidgetItem(reason, m_recommendation_reason_list);
    }
}

void TopologyWidget::ClearValidationState()
{
    // 清空所有控件的红色错误边框和错误提示
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

    // 校验失败处理：显示错误汇总
    m_validation_summary_label->setStyleSheet(QStringLiteral("color: #b42318;"));
    m_validation_summary_label->setText(
        QStringLiteral("Topology 基础校验发现 %1 条问题，其中错误 %2 条，警告 %3 条。")
            .arg(result.issues.size())
            .arg(result.ErrorCount())
            .arg(result.WarningCount()));

    // 遍历每条问题记录
    for (const auto& issue : result.issues)
    {
        auto* item = new QListWidgetItem(
            QStringLiteral("[%1] %2 - %3")
                .arg(RoboSDP::Topology::Validation::ToString(issue.severity), issue.field, issue.message_zh),
            m_validation_issue_list);
        item->setToolTip(issue.code);

        // 查找映射表，给对应的出错控件画上红框/黄框
        QWidget* fieldWidget = m_field_widgets.value(issue.field, nullptr);
        
        // 特殊处理数组字段的报错兜底
        if (fieldWidget == nullptr && issue.field.startsWith(QStringLiteral("layout.hollow_joint_ids[")))
        {
            fieldWidget = m_hollow_joint_ids_edit;
        }

        if (fieldWidget != nullptr)
        {
            fieldWidget->setStyleSheet(
                issue.severity == RoboSDP::Topology::Validation::ValidationSeverity::Warning
                    ? QStringLiteral("border: 1px solid #d97706;")  // 警告用橙色
                    : QStringLiteral("border: 1px solid #dc2626;")); // 错误用红色
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

// ==================== 事件响应槽函数 (Slots) ====================

void TopologyWidget::OnRefreshTemplatesClicked()
{
    RefreshTemplateOptions();
    SetOperationMessage(QStringLiteral("构型模板列表已刷新。"), true);
    emit LogMessageGenerated(QStringLiteral("[Topology] 构型模板列表已刷新。"));
    emit StatusChanged();
}

void TopologyWidget::OnGenerateClicked()
{
    // 调用后台服务，根据上游的 Requirement 生成适合的 Topology 候选方案
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto generateResult = m_service.GenerateCandidatesFromRequirement(
        projectRootPath,
        m_template_combo->currentData().toString());

    if (generateResult.IsSuccess())
    {
        m_state = generateResult.state;
        PopulateForm(m_state.current_model); // 方案生成后自动填入表单
        RenderCandidates();
        RenderRecommendation();
        ApplyValidationResult(generateResult.validation_result);
        MarkDirty();
    }

    SetOperationMessage(generateResult.message, generateResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Topology] %1").arg(generateResult.message));
    emit StatusChanged();
}

void TopologyWidget::OnValidateClicked()
{
    // 用户手动触发校验
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
    emit StatusChanged();
}

void TopologyWidget::OnSaveDraftClicked()
{
    const auto saveResult = SaveCurrentDraft();
    emit LogMessageGenerated(QStringLiteral("[Topology] %1").arg(saveResult.message));
    emit StatusChanged();
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
        MarkClean();
    }

    SetOperationMessage(loadResult.message, loadResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Topology] %1").arg(loadResult.message));
    emit StatusChanged();
}


// ── Ribbon 按钮状态查询实现 ──

bool TopologyWidget::CanGenerate() const
{
    // 构型生成需要已选中有效模板
    return m_template_combo != nullptr && m_template_combo->currentIndex() >= 0;
}

bool TopologyWidget::CanValidate() const
{
    // 校验构型需要至少已填写构型名称
    return m_topology_name_edit != nullptr && !m_topology_name_edit->text().trimmed().isEmpty();
}


// ==================== 实时 3D 预览更新 ====================
// 在文件末尾实现 UpdateLivePreview() 方法
void TopologyWidget::UpdateLivePreview()
{
    // 1. 直接从当前 UI 表单收集最新填写的物理尺寸和配置，生成临时的 Topology 数据结构
    auto currentTopologyModel = CollectModelFromForm();

    // 2. 调用 KinematicsService (运动学服务) 的静态/独立纯数学接口。
    // 该接口会将 Topology 中的简单距离参数（如肩部偏置、大小臂长度）转换为
    // 标准的可供 3D 渲染的连杆和节点描述 (UrdfPreviewSceneDto)。
    auto scene = RoboSDP::Kinematics::Service::KinematicsService::GenerateSkeletonPreview(currentTopologyModel);

    // 3. 将生成的 3D 骨架场景通过 Qt 信号广播出去。
    // MainWindow 监听到此信号后，会指派中央的三维视图区（VTK 引擎）重新绘制这些几何体，
    // 从而实现“在左侧调节参数，右侧立刻看到机器人长短变化”的联动交互。
    emit TopologyPreviewGenerated(scene);
}

} // namespace RoboSDP::Topology::Ui