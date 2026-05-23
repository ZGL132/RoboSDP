#include "modules/kinematics/ui/KinematicsWidget.h"
#include "apps/desktop-qt/widgets/common/CollapsibleSectionWidget.h"

#include "core/infrastructure/ProjectManager.h"
#include "modules/kinematics/domain/KinematicModelStatePolicy.h"

#include <QAbstractItemView>
#include <QBrush>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
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
#include <QSlider>
#include <QSizePolicy>
#include <QStringList>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QToolBox>
#include <QVBoxLayout>
#include <QDateTime>
#include <cmath>

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
    if (RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::IsImportedUrdfReference(model))
    {
        if (model.urdf_master_source_type == QStringLiteral("project_derived"))
        {
            return QStringLiteral("DH 派生最小 URDF 预览 / 只读不可复制");
        }
        return QStringLiteral("工程 URDF 参考 / DH 诊断草案只读");
    }
    if (RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::IsDhParametric(model))
    {
        return previewSourceMode == QStringLiteral("dh_preview")
            ? QStringLiteral("DH/MDH 参数化设计 / 骨架驱动")
            : QStringLiteral("DH/MDH 参数化设计 / 等待骨架同步");
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

bool IsFiniteArray3(const std::array<double, 3>& values)
{
    return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

bool IsFinitePose(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    return IsFiniteArray3(pose.position_m) && IsFiniteArray3(pose.rpy_deg);
}

bool IsFiniteTcpFrame(const RoboSDP::Kinematics::Dto::TcpFrameDto& frame)
{
    return IsFiniteArray3(frame.translation_m) && IsFiniteArray3(frame.rpy_deg);
}

QFrame* CreateKinematicsCard(QWidget* parent, const QString& objectName = QString())
{
    auto* card = new QFrame(parent);
    if (!objectName.isEmpty())
    {
        card->setObjectName(objectName);
    }
    card->setFrameShape(QFrame::NoFrame);
    return card;
}

QLabel* CreateStatusBadge(QWidget* parent, const QString& text)
{
    auto* label = new QLabel(text, parent);
    label->setObjectName(QStringLiteral("kinematicsBadge"));
    label->setAlignment(Qt::AlignCenter);
    label->setMinimumHeight(22);
    label->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
    return label;
}

void ApplyBadgeStyle(QLabel* label, const QString& text, const QString& tone)
{
    if (label == nullptr)
    {
        return;
    }

    QString background = QStringLiteral("#eef2ff");
    QString color = QStringLiteral("#3730a3");
    QString border = QStringLiteral("#c7d2fe");
    if (tone == QStringLiteral("success"))
    {
        background = QStringLiteral("#ecfdf3");
        color = QStringLiteral("#027a48");
        border = QStringLiteral("#abefc6");
    }
    else if (tone == QStringLiteral("warning"))
    {
        background = QStringLiteral("#fff7ed");
        color = QStringLiteral("#b54708");
        border = QStringLiteral("#fed7aa");
    }
    else if (tone == QStringLiteral("error"))
    {
        background = QStringLiteral("#fef3f2");
        color = QStringLiteral("#b42318");
        border = QStringLiteral("#fecdca");
    }
    else if (tone == QStringLiteral("neutral"))
    {
        background = QStringLiteral("#f8fafc");
        color = QStringLiteral("#475467");
        border = QStringLiteral("#d0d5dd");
    }

    label->setText(text);
    label->setStyleSheet(QStringLiteral(
        "QLabel#kinematicsBadge{"
        "color:%2;padding:0 4px;font-weight:600;"
        "}").arg(background, color, border));
}

void StyleKinematicsTable(QTableWidget* table)
{
    if (table == nullptr)
    {
        return;
    }

    table->setAlternatingRowColors(true);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->verticalHeader()->setDefaultSectionSize(28);
    table->verticalHeader()->setMinimumSectionSize(26);
    table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    table->horizontalHeader()->setHighlightSections(false);
    table->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);
    table->setShowGrid(true);
    table->setStyleSheet(QString());
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

bool TryReadTableDouble(const QTableWidget* table, int row, int column, double& value)
{
    if (table == nullptr)
    {
        value = 0.0;
        return false;
    }

    const QTableWidgetItem* item = table->item(row, column);
    if (item == nullptr)
    {
        value = 0.0;
        return false;
    }

    bool ok = false;
    value = item->text().trimmed().toDouble(&ok);
    return ok;
}

QString ReadTableString(QTableWidget* table, int row, int column, const QString& defaultValue = {})
{
    QTableWidgetItem* item = table->item(row, column);
    return item != nullptr ? item->text().trimmed() : defaultValue;
}

void SetReadOnlyItem(QTableWidgetItem* item)
{
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    item->setBackground(QColor(QStringLiteral("#f8fafc")));
    item->setForeground(QColor(QStringLiteral("#475467")));
}

void StyleTableItem(QTableWidgetItem* item, bool numeric)
{
    if (item == nullptr)
    {
        return;
    }
    item->setTextAlignment(numeric ? Qt::AlignRight | Qt::AlignVCenter
                                   : Qt::AlignLeft | Qt::AlignVCenter);
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

KinematicsWidget::KinematicsWidget(RoboSDP::Logging::ILogger* logger, QWidget* parent)
    : QWidget(parent)
    , m_topology_storage(m_repository)
    , m_kinematic_storage(m_repository)
    , m_logger(logger)
    , m_service(m_kinematic_storage, m_topology_storage, m_logger)
    , m_state(m_service.CreateDefaultState())
{
    BuildUi();
    PopulateForm(m_state.current_model);
    RefreshBackendDiagnostics();
    RenderResults();
    ConnectDirtyTracking();
    MarkClean();
}

void KinematicsWidget::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);
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
/**
 * @brief 绑定所有 UI 控件的数据变化事件，用于追踪未保存状态（Dirty Tracking）和触发实时渲染。
 * @details 
 * 采用 Qt 的 findChildren 反射机制，批量遍历不同类型的控件并挂载信号槽。
 * 这样无论以后在 UI 上增加多少个普通输入框，都不需要在这里手动补充连接代码。
 */
void KinematicsWidget::ConnectDirtyTracking()
{
    // 1. 遍历界面上所有的单行文本框 (例如：模型名称、拓扑引用等)
    for (QLineEdit* editor : findChildren<QLineEdit*>())
    {
        // textEdited 信号：只有当用户真正在键盘上打字修改时才触发（代码 setValue 不会触发）
        connect(editor, &QLineEdit::textEdited, this, [this]() { MarkDirty(); });
    }
    
    // 2. 遍历所有的下拉选项框 (例如：参数约定 DH/MDH、求解器类型等)
    for (QComboBox* editor : findChildren<QComboBox*>())
    {
        // 强转信号指针，解决 QComboBox::currentIndexChanged 函数重载的歧义问题
        connect(editor, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [this](int) {
            MarkDirty(); // 只要选项改变，标记为“未保存”
        });
    }
    
    // 3. 遍历所有的双精度浮点数输入框 (涵盖了大量的尺寸、位置、容差参数)
    for (QDoubleSpinBox* editor : findChildren<QDoubleSpinBox*>())
    {
        connect(editor, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [this](double) {
            MarkDirty();
        });
    }
    
    // 4. 遍历所有的整数输入框 (例如：最大迭代次数、采样点数)
    for (QSpinBox* editor : findChildren<QSpinBox*>())
    {
        connect(editor, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, [this](int) {
            MarkDirty();
        });
    }
    
    // 5. 遍历所有的复选框 (例如：是否启用 Tool Frame、Workpiece Frame 等)
    for (QCheckBox* editor : findChildren<QCheckBox*>())
    {
        connect(editor, &QCheckBox::toggled, this, [this](bool) { MarkDirty(); });
    }
    
    // 6. 遍历所有的表格组件 (核心的 DH 参数表、关节限位表)
    for (QTableWidget* table : findChildren<QTableWidget*>())
    {
        // cellChanged 信号：当表格中任意一个单元格的内容发生修改时触发
        connect(table, &QTableWidget::cellChanged, this, [this](int, int) { MarkDirty(); });
    }
}

void KinematicsWidget::MarkDirty()
{
    if (m_is_populating_form)
    {
        return;
    }
    m_has_unsaved_changes = true;
    RefreshValidationState();
    RefreshHeaderBadges();
}

void KinematicsWidget::MarkClean()
{
    m_has_unsaved_changes = false;
    RefreshHeaderBadges();
}

void KinematicsWidget::TriggerImportUrdf()
{
    OnImportUrdfClicked();
}

void KinematicsWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(10, 10, 10, 10);
    rootLayout->setSpacing(10);

    setObjectName(QStringLiteral("kinematicsWidget"));
    setStyleSheet(QString());

    auto* headerCard = CreateKinematicsCard(this, QStringLiteral("kinematicsHeaderCard"));
    auto* headerLayout = new QVBoxLayout(headerCard);
    headerLayout->setContentsMargins(12, 10, 12, 10);
    headerLayout->setSpacing(8);

    auto* titleRow = new QHBoxLayout();
    titleRow->setSpacing(8);
    auto* titleLabel = new QLabel(QStringLiteral("运动学建模"), headerCard);
    auto* subtitleLabel = new QLabel(QStringLiteral("DH/MDH 参数、FK/IK 验证、工作空间与模型诊断"), headerCard);
    titleRow->addWidget(titleLabel);
    titleRow->addWidget(subtitleLabel);
    titleRow->addStretch(1);
    headerLayout->addLayout(titleRow);

    auto* badgeRow = new QHBoxLayout();
    badgeRow->setSpacing(6);
    m_validation_badge_label = CreateStatusBadge(headerCard, QStringLiteral("等待校验"));
    m_dirty_badge_label = CreateStatusBadge(headerCard, QStringLiteral("已保存"));
    m_sync_badge_label = CreateStatusBadge(headerCard, QStringLiteral("未生成"));
    m_source_badge_label = CreateStatusBadge(headerCard, QStringLiteral("未加载"));
    badgeRow->addWidget(m_validation_badge_label);
    badgeRow->addWidget(m_dirty_badge_label);
    badgeRow->addWidget(m_sync_badge_label);
    badgeRow->addWidget(m_source_badge_label);
    badgeRow->addStretch(1);
    headerLayout->addLayout(badgeRow);

    m_operation_label = new QLabel(QStringLiteral("就绪：请先保存 Topology，再生成 KinematicModel。"), headerCard);
    m_operation_label->setWordWrap(true);
    m_operation_label->hide();

    m_validation_label = new QLabel(QStringLiteral("模型校验：等待载入运动学草稿。"), headerCard);
    m_validation_label->setWordWrap(true);
    m_validation_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    headerLayout->addWidget(m_validation_label);
    headerCard->hide();

    auto* tabs = new QTabWidget(this);
    tabs->setDocumentMode(true);
    tabs->addTab(CreateScrollableTab(CreateDesignInputPage()), QStringLiteral("模型参数"));
    tabs->addTab(CreateScrollableTab(CreateVerificationPage()), QStringLiteral("FK / IK"));
    tabs->addTab(CreateScrollableTab(CreateAdvancedAnalysisPage()), QStringLiteral("工作空间"));
    tabs->addTab(CreateScrollableTab(CreateDiagnosticsPage()), QStringLiteral("诊断"));

    rootLayout->addWidget(tabs, 1);

    // A/B 通道：将涉及"结构变动"的输入项绑定至重量级通道 SyncStructureAndPreview
    connect(
        m_parameter_convention_combo,
        static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this,
        [this](int) { SyncStructureAndPreview(); });

    connect(m_dh_table, &QTableWidget::cellChanged, this, [this](int, int) {
        SyncStructureAndPreview();
    });

    auto connectPreviewEditors = [this](const std::array<QDoubleSpinBox*, 6>& editors) {
        for (QDoubleSpinBox* editor : editors)
        {
            if (editor == nullptr) continue;
            connect(editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
                SyncStructureAndPreview();
            });
        }
    };
    connectPreviewEditors(m_base_frame_spins);
    connectPreviewEditors(m_flange_frame_spins);
    connectPreviewEditors(m_tcp_frame_spins);
}

QWidget* KinematicsWidget::CreateDesignInputPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);
    layout->addWidget(CreateModelGroup());

    auto* tableTabs = new QTabWidget(page);
    tableTabs->setDocumentMode(true);
    tableTabs->addTab(CreateDhTableGroup(), QStringLiteral("DH 参数"));
    tableTabs->addTab(CreateJointLimitGroup(), QStringLiteral("关节限位"));
    layout->addWidget(tableTabs);

    layout->addStretch();
    return page;
}

QWidget* KinematicsWidget::CreateVerificationPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);
    layout->addWidget(CreateSolverGroup());
    layout->addStretch();
    return page;
}

QWidget* KinematicsWidget::CreateDiagnosticsPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);
    layout->addWidget(CreateResultGroup());
    layout->addStretch();
    return page;
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
        if (auto* rootGroupBox = qobject_cast<QGroupBox*>(contentWidget))
        {
            rootGroupBox->setTitle(QString());
            rootGroupBox->setFlat(true);
            rootGroupBox->setObjectName(QStringLiteral("tabRootGroupBox"));
            rootGroupBox->setStyleSheet(QStringLiteral(
                "QGroupBox#tabRootGroupBox{border:none;margin-top:0;background:transparent;}"
                "QGroupBox#tabRootGroupBox::title{height:0px;padding:0px;}"));
        }
        contentLayout->addWidget(contentWidget);
    }
    contentLayout->addStretch();
    scrollArea->setWidget(scrollContent);

    tabLayout->addWidget(scrollArea, 1);
    return tabPage;
}

QWidget* KinematicsWidget::CreateModelGroup()
{
    auto* pageWidget = CreateKinematicsCard(this, QStringLiteral("kinematicsModelCard"));
    auto* pageLayout = new QVBoxLayout(pageWidget);
    pageLayout->setContentsMargins(12, 12, 12, 12);
    pageLayout->setSpacing(10);

    // 1. Basic properties
    auto* basicPropsWidget = new QWidget(pageWidget);
    auto* formLayout = new QFormLayout(basicPropsWidget);
    formLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    formLayout->setContentsMargins(8, 8, 8, 8);
    formLayout->setHorizontalSpacing(10);
    formLayout->setVerticalSpacing(8);
    m_model_name_edit = new QLineEdit(basicPropsWidget);
    m_parameter_convention_combo = new QComboBox(basicPropsWidget);
    m_parameter_convention_combo->addItem(QStringLiteral("DH"), QStringLiteral("DH"));
    m_parameter_convention_combo->addItem(QStringLiteral("MDH"), QStringLiteral("MDH"));
    formLayout->addRow(QStringLiteral("模型名称"), m_model_name_edit);
    formLayout->addRow(QStringLiteral("参数约定"), m_parameter_convention_combo);

    auto* basicSection = new QGroupBox(QStringLiteral("基础模型属性"), pageWidget);
    auto* bsLayout = new QVBoxLayout(basicSection); bsLayout->addWidget(basicPropsWidget);
    
    pageLayout->addWidget(basicSection);

    // 2. Source / synchronization summary
    auto* debugWidget = new QWidget(pageWidget);
    auto* debugForm = new QFormLayout(debugWidget);
    debugForm->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    debugForm->setContentsMargins(8, 8, 8, 8);
    debugForm->setHorizontalSpacing(10);
    debugForm->setVerticalSpacing(7);

    m_topology_ref_edit = new QLineEdit(debugWidget);
    m_topology_ref_edit->setReadOnly(true);
    m_requirement_ref_edit = new QLineEdit(debugWidget);
    m_requirement_ref_edit->setReadOnly(true);
    m_master_model_mode_label = new QLabel(debugWidget);
    m_derived_model_state_label = new QLabel(debugWidget);
    m_preview_source_label = new QLabel(debugWidget);
    m_master_switch_state_label = new QLabel(debugWidget);
    m_urdf_source_type_label = new QLabel(debugWidget);
    m_dh_draft_level_label = new QLabel(debugWidget);
    m_dh_draft_status_label = new QLabel(debugWidget);

    const QList<QLabel*> sourceLabels = {
        m_master_model_mode_label,
        m_derived_model_state_label,
        m_preview_source_label,
        m_master_switch_state_label,
        m_urdf_source_type_label,
        m_dh_draft_level_label,
        m_dh_draft_status_label,
    };
    for (QLabel* label : sourceLabels)
    {
        if (label != nullptr)
        {
            label->setTextInteractionFlags(Qt::TextSelectableByMouse);
            label->setStyleSheet(QStringLiteral("color:#334155;"));
        }
    }

    debugForm->addRow(QStringLiteral("设计来源"), m_master_model_mode_label);
    debugForm->addRow(QStringLiteral("Topology 引用"), m_topology_ref_edit);
    debugForm->addRow(QStringLiteral("Requirement 引用"), m_requirement_ref_edit);
    debugForm->addRow(QStringLiteral("派生状态"), m_derived_model_state_label);
    debugForm->addRow(QStringLiteral("预览数据源"), m_preview_source_label);
    debugForm->addRow(QStringLiteral("模型来源说明"), m_master_switch_state_label);
    debugForm->addRow(QStringLiteral("URDF 来源类型"), m_urdf_source_type_label);
    debugForm->addRow(QStringLiteral("DH 草案级别"), m_dh_draft_level_label);
    debugForm->addRow(QStringLiteral("DH 草案状态"), m_dh_draft_status_label);

    auto* sourceSection = new QGroupBox(QStringLiteral("模型来源与同步状态"), pageWidget);
    auto* sourceLayout = new QVBoxLayout(sourceSection);
    sourceLayout->addWidget(debugWidget);
    pageLayout->addWidget(sourceSection);

    // 3. Physical frames
    auto* framesWidget = new QWidget(pageWidget);
    auto* framesLayout = new QGridLayout(framesWidget);
    framesLayout->setSpacing(8);

    const QStringList poseLabels {
        QStringLiteral("X [m]"), QStringLiteral("Y [m]"), QStringLiteral("Z [m]"),
        QStringLiteral("RX [deg]"), QStringLiteral("RY [deg]"), QStringLiteral("RZ [deg]")};

    auto createFrameEditor = [this, &poseLabels](
                                 const QString& title,
                                 std::array<QDoubleSpinBox*, 6>& editors,
                                 const QString& tip,
                                 QCheckBox** enabledCheckBox = nullptr) -> QGroupBox* {
        auto* frameGroup = new QGroupBox(title);
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
        return frameGroup;
    };

    auto* baseGroup = createFrameEditor(QStringLiteral("Base Frame (基座)"), m_base_frame_spins, QStringLiteral("机器人的全局基座坐标系"));
    auto* flangeGroup = createFrameEditor(QStringLiteral("Flange Frame (法兰)"), m_flange_frame_spins, QStringLiteral("机器人末端法兰盘坐标系"));
    auto* toolGroup = createFrameEditor(QStringLiteral("Tool Frame (工具)"), m_tool_frame_spins, QStringLiteral("安装在法兰上的工具坐标系"), &m_tool_frame_enabled_check);
    auto* workpieceGroup = createFrameEditor(QStringLiteral("Workpiece Frame (工件)"), m_workpiece_frame_spins, QStringLiteral("环境中的工件参考坐标系"), &m_workpiece_frame_enabled_check);
    auto* tcpGroup = createFrameEditor(QStringLiteral("TCP Frame (工具中心点)"), m_tcp_frame_spins, QStringLiteral("控制的最终工具中心点坐标系"));

    framesLayout->addWidget(baseGroup, 0, 0);
    framesLayout->addWidget(flangeGroup, 0, 1);
    framesLayout->addWidget(workpieceGroup, 1, 0);
    framesLayout->addWidget(toolGroup, 1, 1);
    framesLayout->addWidget(tcpGroup, 2, 1);

    auto* framesSection = new CollapsibleSectionWidget(QStringLiteral("物理坐标系设置"), pageWidget);
    framesSection->SetContent(framesWidget);
    framesSection->SetCollapsed(false);

    pageLayout->addWidget(framesSection);

    pageLayout->addStretch();

    if (m_tool_frame_enabled_check != nullptr)
    {
        connect(m_tool_frame_enabled_check, &QCheckBox::toggled, this, [this](bool checked) {
            for(auto* spin : m_tool_frame_spins) {
                if(spin) spin->setEnabled(checked);
            }
        });
    }

    if (m_workpiece_frame_enabled_check != nullptr)
    {
        connect(m_workpiece_frame_enabled_check, &QCheckBox::toggled, this, [this](bool checked) {
            for(auto* spin : m_workpiece_frame_spins) {
                if(spin) spin->setEnabled(checked);
            }
        });
    }

    return pageWidget;
}

QGroupBox* KinematicsWidget::CreateDhTableGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("DH/MDH 参数表"), this);
    auto* layout = new QVBoxLayout(groupBox);
    layout->setContentsMargins(10, 14, 10, 10);
    layout->setSpacing(8);

    m_dh_readonly_banner_label = new QLabel(groupBox);
    m_dh_readonly_banner_label->setWordWrap(true);
    m_dh_readonly_banner_label->hide();
    layout->addWidget(m_dh_readonly_banner_label);

    auto* hintLabel = new QLabel(
        QStringLiteral("长度单位统一为 m，角度单位统一为 deg；修改 DH 参数会实时重建中央骨架预览。"),
        groupBox);
    hintLabel->setWordWrap(true);
    layout->addWidget(hintLabel);

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
    auto* groupBox = new QGroupBox(QStringLiteral("位姿验证与关节示教"), this);
    auto* layout = new QVBoxLayout(groupBox);

    auto* interactivePage = CreateInteractivePage();
    layout->addWidget(interactivePage);

    return groupBox;
}

QWidget* KinematicsWidget::CreateSolverConfigPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);

    auto* solverLayout = new QFormLayout();
    solverLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_solver_type_combo = new QComboBox(page);
    m_solver_type_combo->addItem(QStringLiteral("数值雅可比转置"), QStringLiteral("numeric_jacobian_transpose"));
    m_solver_type_combo->addItem(QStringLiteral("闭式解析求解器"), QStringLiteral("analytical_closed_form"));
    m_branch_policy_combo = new QComboBox(page);
    m_branch_policy_combo->addItem(QStringLiteral("最近种子解"), QStringLiteral("nearest_seed"));
    m_max_iterations_spin = new QSpinBox(page);
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
    layout->addStretch();
    return page;
}

QWidget* KinematicsWidget::CreateInteractivePage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    auto* fkGroup = new QGroupBox(QStringLiteral("正运动学与关节示教 (FK)"), page);
    auto* fkLayout = new QVBoxLayout(fkGroup);

    auto* fkHeaderLayout = new QHBoxLayout();
    auto* fkHintLabel = new QLabel(QStringLiteral("逐关节拖动滑块或输入角度，中央骨架会实时刷新。"), fkGroup);
    fkHintLabel->setWordWrap(true);
    fkHeaderLayout->addWidget(fkHintLabel, 1);
    m_scroll_step_spin = CreateDoubleSpinBox(0.1, 10.0, 1, 0.1);
    m_scroll_step_spin->setValue(1.0);
    m_scroll_step_spin->setToolTip(QStringLiteral(
        "设置鼠标滚轮每格对应的关节角度增量（度/格）。\n"
        "同时同步影响 FK 关节输入框的上下箭头步进值。"));
    auto* sensitivityLabel = new QLabel(QStringLiteral("滚轮灵敏度 [°/格]"), fkGroup);
    fkHeaderLayout->addWidget(sensitivityLabel);
    fkHeaderLayout->addWidget(m_scroll_step_spin);
    fkLayout->addLayout(fkHeaderLayout);

    // 灵敏度值变更 → 同步 FK 输入框步进 + 通知 3D 视图
    connect(m_scroll_step_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this, [this](double newStep) {
            ApplyStepToAllSpinBoxes(newStep);
            emit signalScrollStepChanged(newStep);
        });

    // FK 关节输入网格
    m_fk_grid = new QGridLayout();
    m_fk_grid->setColumnStretch(2, 1);
    m_fk_grid->setHorizontalSpacing(8);
    m_fk_grid->setVerticalSpacing(6);
    m_fk_grid->addWidget(new QLabel(QStringLiteral("关节"), fkGroup), 0, 0);
    m_fk_grid->addWidget(new QLabel(QStringLiteral("角度 [deg]"), fkGroup), 0, 1);
    m_fk_grid->addWidget(new QLabel(QStringLiteral("滑块"), fkGroup), 0, 2);
    m_fk_grid->addWidget(new QLabel(QStringLiteral("裕量"), fkGroup), 0, 3);
    fkLayout->addLayout(m_fk_grid);

    // ── FK 实时 TCP 位姿读出区 ──────────────────────────
    {
        auto* readoutFrame = new QFrame(fkGroup);
        readoutFrame->setObjectName(QStringLiteral("fkReadoutFrame"));
        readoutFrame->setStyleSheet(QStringLiteral(
            "QFrame#fkReadoutFrame{"
            "background:#f0f4ff;border:1px solid #c7d2fe;"
            "border-radius:3px;padding:4px;}"));
        auto* roLayout = new QVBoxLayout(readoutFrame);
        roLayout->setContentsMargins(8, 4, 8, 4);
        roLayout->setSpacing(1);

        auto* roTitle = new QLabel(QStringLiteral("FK 计算结果"), readoutFrame);
        roTitle->setStyleSheet(QStringLiteral(
            "font-weight:bold;color:#3730a3;font-size:11px;"));
        roLayout->addWidget(roTitle);

        m_fk_tcp_pos_label = new QLabel(
            QStringLiteral("P: (  ---- ,  ---- ,  ---- ) m"), readoutFrame);
        m_fk_tcp_pos_label->setStyleSheet(QStringLiteral(
            "font-family:monospace;font-size:11px;color:#1e293b;"));
        roLayout->addWidget(m_fk_tcp_pos_label);

        m_fk_tcp_rpy_label = new QLabel(
            QStringLiteral("R: (  ---- ,  ---- ,  ---- ) deg"), readoutFrame);
        m_fk_tcp_rpy_label->setStyleSheet(QStringLiteral(
            "font-family:monospace;font-size:11px;color:#1e293b;"));
        roLayout->addWidget(m_fk_tcp_rpy_label);

        fkLayout->addWidget(readoutFrame);
    }

    layout->addWidget(fkGroup);

    auto* ikGroup = new QGroupBox(QStringLiteral("逆运动学与目标位姿 (IK)"), page);
    auto* ikLayout = new QVBoxLayout(ikGroup);
    ikLayout->setSpacing(8);

    auto* ikTargetGrid = new QGridLayout();
    ikTargetGrid->setColumnStretch(1, 1);
    ikTargetGrid->setColumnStretch(3, 1);
    const QStringList poseLabels {
        QStringLiteral("X [m]"),
        QStringLiteral("Y [m]"),
        QStringLiteral("Z [m]"),
        QStringLiteral("RX [deg]"),
        QStringLiteral("RY [deg]"),
        QStringLiteral("RZ [deg]")};
    for (int index = 0; index < 6; ++index)
    {
        const bool isTranslation = index < 3;
        auto* spinBox = CreateDoubleSpinBox(
            isTranslation ? -10.0 : -360.0,
            isTranslation ? 10.0 : 360.0,
            isTranslation ? 4 : 3,
            isTranslation ? 0.001 : 1.0);
        spinBox->setToolTip(isTranslation
            ? QStringLiteral("位置单位为 m。常规工业机械臂建议保持在 ±10 m 以内。")
            : QStringLiteral("姿态单位为 deg，使用 RPY 欧拉角。"));
        m_ik_target_pose_spins[static_cast<std::size_t>(index)] = spinBox;
        connect(
            spinBox,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this](double) {
                if (m_is_populating_form)
                {
                    return;
                }
                RoboSDP::Kinematics::Dto::CartesianPoseDto targetPose;
                targetPose.position_m = {
                    m_ik_target_pose_spins[0]->value(),
                    m_ik_target_pose_spins[1]->value(),
                    m_ik_target_pose_spins[2]->value()};
                targetPose.rpy_deg = {
                    m_ik_target_pose_spins[3]->value(),
                    m_ik_target_pose_spins[4]->value(),
                    m_ik_target_pose_spins[5]->value()};
                emit IkPoseComparisonUpdated(
                    targetPose,
                    RoboSDP::Kinematics::Dto::CartesianPoseDto {},
                    0.0,
                    0.0,
                    false,
                    false);
            });
        const int row = index % 3;
        const int labelColumn = index < 3 ? 0 : 2;
        const int editorColumn = index < 3 ? 1 : 3;
        ikTargetGrid->addWidget(new QLabel(poseLabels.at(index), ikGroup), row, labelColumn);
        ikTargetGrid->addWidget(spinBox, row, editorColumn);
    }
    ikLayout->addLayout(ikTargetGrid);

    auto* ikActionRow = new QHBoxLayout();
    auto* fillFromFkButton = new QPushButton(QStringLiteral("填入当前 FK 结果"), ikGroup);
    fillFromFkButton->setToolTip(QStringLiteral("将最近一次 FK 结果填入 IK 目标位姿，并把当前关节角作为 IK 种子。"));
    connect(fillFromFkButton, &QPushButton::clicked, this, &KinematicsWidget::FillIkTargetFromFkResult);
    auto* runIkButton = new QPushButton(QStringLiteral("执行 IK 求解"), ikGroup);
    runIkButton->setToolTip(QStringLiteral("求解目标位姿对应的关节角。"));
    connect(runIkButton, &QPushButton::clicked, this, &KinematicsWidget::OnRunIkClicked);
    ikActionRow->addWidget(fillFromFkButton);
    ikActionRow->addStretch();
    ikActionRow->addWidget(runIkButton);
    ikLayout->addLayout(ikActionRow);

    auto* advancedContent = new QWidget(ikGroup);
    auto* advancedLayout = new QVBoxLayout(advancedContent);
    advancedLayout->setContentsMargins(8, 6, 8, 6);
    advancedLayout->setSpacing(8);

    m_ik_seed_grid = new QGridLayout();
    m_ik_seed_grid->setHorizontalSpacing(6);
    m_ik_seed_grid->setVerticalSpacing(4);
    m_ik_seed_grid->addWidget(new QLabel(QStringLiteral("IK 种子关节 [deg]"), advancedContent), 0, 0, 1, 6);
    advancedLayout->addLayout(m_ik_seed_grid);
    advancedLayout->addWidget(CreateSolverConfigPage());

    auto* advancedIkSection = new CollapsibleSectionWidget(QStringLiteral("高级求解参数 (IK Seed & Solver)"), ikGroup);
    advancedIkSection->SetContent(advancedContent);
    advancedIkSection->SetCollapsed(true);
    ikLayout->addWidget(advancedIkSection);

    AdjustJointInputCount(6);

    // ── IK 多解浏览器（默认折叠） ──────────────────────────────
    auto* solutionContent = new QWidget(ikGroup);
    auto* solutionLayout = new QVBoxLayout(solutionContent);
    solutionLayout->setContentsMargins(4, 4, 4, 4);
    m_ik_solution_combo = new QComboBox(solutionContent);
    m_ik_solution_combo->setEnabled(false);
    m_ik_solution_combo->setToolTip(QStringLiteral("选择闭式解析 IK 的不同空间构型解"));
    auto* comboRow = new QHBoxLayout();
    comboRow->addWidget(m_ik_solution_combo, 1);
    auto* applySolutionBtn = new QPushButton(QStringLiteral("应用至 FK"), solutionContent);
    applySolutionBtn->setToolTip(QStringLiteral("将当前选中解的角度填入 FK 滑块"));
    applySolutionBtn->setEnabled(false);
    connect(applySolutionBtn, &QPushButton::clicked, this, [this]() {
        if (m_ik_solution_combo->currentIndex() < 0) return;
        const auto& allSol = m_state.last_ik_result.all_solutions_deg;
        const int idx = m_ik_solution_combo->currentIndex();
        if (idx < static_cast<int>(allSol.size()))
        {
            for (std::size_t i = 0; i < m_fk_joint_spins.size() && i < allSol[idx].size(); ++i)
                m_fk_joint_spins[i]->setValue(allSol[idx][i]);
        }
    });
    connect(m_ik_solution_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [applySolutionBtn](int) { applySolutionBtn->setEnabled(true); });
    comboRow->addWidget(applySolutionBtn);
    solutionLayout->addLayout(comboRow);

    auto* solutionGrid = new QGridLayout();
    solutionGrid->setColumnStretch(1, 1);
    solutionGrid->setColumnStretch(3, 1);
    for (int i = 0; i < 6; ++i)
    {
        auto* spin = CreateDoubleSpinBox(-9999.0, 9999.0, 3, 1.0);
        spin->setReadOnly(true);
        spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
        spin->setStyleSheet(QStringLiteral("background: #f0f0f0; color: #333;"));
        m_ik_solution_spins.push_back(spin);
        const int row = i / 2;
        const int labelColumn = (i % 2) * 2;
        const int editorColumn = labelColumn + 1;
        solutionGrid->addWidget(new QLabel(QStringLiteral("J%1").arg(i + 1), solutionContent), row, labelColumn);
        solutionGrid->addWidget(spin, row, editorColumn);
    }
    solutionLayout->addLayout(solutionGrid);

    connect(m_ik_solution_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this](int index) {
            if (index < 0) return;
            const auto& allSol = m_state.last_ik_result.all_solutions_deg;
            if (index < static_cast<int>(allSol.size()))
            {
                for (std::size_t j = 0; j < m_ik_solution_spins.size() && j < allSol[index].size(); ++j)
                    m_ik_solution_spins[j]->setValue(allSol[index][j]);
            }
        });

    auto* solutionSection = new CollapsibleSectionWidget(
        QStringLiteral("IK 多解浏览器"), ikGroup);
    solutionSection->SetContent(solutionContent);
    solutionSection->SetCollapsed(true);
    ikLayout->addWidget(solutionSection);
    layout->addWidget(ikGroup);

    layout->addStretch();
    return page;
}

QWidget* KinematicsWidget::CreateAdvancedAnalysisPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    // ── 工作空间采样与奇异区分析 ──────────────────────────────
    auto* workspaceGroup = new QGroupBox(QStringLiteral("工作空间采样与奇异区分析"), page);
    auto* workspaceRootLayout = new QVBoxLayout(workspaceGroup);

    auto* workspaceLayout = new QFormLayout();
    workspaceLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_workspace_sample_count_spin = new QSpinBox(workspaceGroup);
    m_workspace_sample_count_spin->setRange(1, 100000);
    m_workspace_sample_count_spin->setValue(128);
    workspaceLayout->addRow(QStringLiteral("采样点数"), m_workspace_sample_count_spin);
    m_singularity_threshold_spin = new QDoubleSpinBox(workspaceGroup);
    m_singularity_threshold_spin->setRange(1.0, 100000.0);
    m_singularity_threshold_spin->setDecimals(0);
    m_singularity_threshold_spin->setValue(1000.0);
    m_singularity_threshold_spin->setToolTip(QStringLiteral("条件数 κ 阈值：κ > 此值标记为奇异（越小越敏感）"));
    workspaceLayout->addRow(QStringLiteral("奇异阈值 κ"), m_singularity_threshold_spin);
    workspaceRootLayout->addLayout(workspaceLayout);

    auto* workspaceButtonRow = new QHBoxLayout();
    auto* workspaceBtn = new QPushButton(QStringLiteral("采样工作空间"), workspaceGroup);
    workspaceBtn->setToolTip(QStringLiteral("按当前关节限位采样 TCP 可达点云。"));
    connect(workspaceBtn, &QPushButton::clicked, this, &KinematicsWidget::OnSampleWorkspaceClicked);
    auto* singularityBtn = new QPushButton(QStringLiteral("奇异区分析"), workspaceGroup);
    singularityBtn->setToolTip(QStringLiteral("对工作空间进行带 Jacobian 条件数分析的采样，识别奇异区域。"));
    connect(singularityBtn, &QPushButton::clicked, this, &KinematicsWidget::OnSingularityAnalysisClicked);
    workspaceButtonRow->addWidget(workspaceBtn);
    workspaceButtonRow->addWidget(singularityBtn);
    workspaceRootLayout->addLayout(workspaceButtonRow);

    m_singularity_result_label = new QLabel(QStringLiteral("尚未分析"), workspaceGroup);
    m_singularity_result_label->setWordWrap(true);
    m_singularity_result_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_singularity_result_label->setStyleSheet(QStringLiteral("color:#475467; background:#f8fafc; padding:6px; border-radius:4px;"));
    workspaceRootLayout->addWidget(m_singularity_result_label);

    layout->addWidget(workspaceGroup);

    // ── 关键工位可达性检测 ─────────────────────────────────────
    auto* reachGroup = new QGroupBox(QStringLiteral("关键工位可达性检测"), page);
    auto* reachLayout = new QVBoxLayout(reachGroup);
    auto* reachTargetGrid = new QGridLayout();
    reachTargetGrid->setColumnStretch(1, 1);
    reachTargetGrid->setColumnStretch(3, 1);
    const QStringList reachLabels {
        QStringLiteral("X [m]"), QStringLiteral("Y [m]"), QStringLiteral("Z [m]"),
        QStringLiteral("RX [deg]"), QStringLiteral("RY [deg]"), QStringLiteral("RZ [deg]")};
    for (int idx = 0; idx < 6; ++idx)
    {
        const bool isTranslation = idx < 3;
        auto* spin = CreateDoubleSpinBox(
            isTranslation ? -10.0 : -360.0,
            isTranslation ? 10.0 : 360.0,
            isTranslation ? 4 : 3,
            isTranslation ? 0.001 : 1.0);
        spin->setToolTip(isTranslation
            ? QStringLiteral("位置单位为 m。常规工业机械臂建议保持在 ±10 m 以内。")
            : QStringLiteral("姿态单位为 deg，使用 RPY 欧拉角。"));
        m_reach_target_spins[idx] = spin;
        const int row = idx % 3;
        const int labelColumn = idx < 3 ? 0 : 2;
        const int editorColumn = idx < 3 ? 1 : 3;
        reachTargetGrid->addWidget(new QLabel(reachLabels.at(idx), reachGroup), row, labelColumn);
        reachTargetGrid->addWidget(spin, row, editorColumn);
    }
    reachLayout->addLayout(reachTargetGrid);

    auto* reachBtnRow = new QHBoxLayout();
    auto* reachCheckBtn = new QPushButton(QStringLiteral("检测可达性"), reachGroup);
    connect(reachCheckBtn, &QPushButton::clicked, this, &KinematicsWidget::OnCheckReachabilityClicked);
    reachBtnRow->addWidget(reachCheckBtn);
    reachBtnRow->addStretch();
    reachLayout->addLayout(reachBtnRow);

    // 种子数量设置
    auto* reachSeedLayout = new QFormLayout();
    m_reach_seed_count_spin = new QSpinBox(reachGroup);
    m_reach_seed_count_spin->setRange(1, 100);
    m_reach_seed_count_spin->setValue(20);
    reachSeedLayout->addRow(QStringLiteral("种子数"), m_reach_seed_count_spin);
    reachLayout->addLayout(reachSeedLayout);

    m_reach_result_label = new QLabel(QStringLiteral("尚未检测"), reachGroup);
    m_reach_result_label->setWordWrap(true);
    m_reach_result_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_reach_result_label->setStyleSheet(QStringLiteral("color:#475467; background:#f8fafc; padding:6px; border-radius:4px;"));
    reachLayout->addWidget(m_reach_result_label);

    layout->addWidget(reachGroup);

    // ── 姿态可达性分析 ──────────────────────────────────────
    auto* orientGroup = new QGroupBox(QStringLiteral("姿态可达性分析"), page);
    auto* orientLayout = new QVBoxLayout(orientGroup);
    auto* orientPosGrid = new QGridLayout();
    orientPosGrid->setColumnStretch(1, 1);
    orientPosGrid->setColumnStretch(3, 1);
    orientPosGrid->setColumnStretch(5, 1);
    const QStringList posLabels {QStringLiteral("X [m]"), QStringLiteral("Y [m]"), QStringLiteral("Z [m]")};
    for (int idx = 0; idx < 3; ++idx)
    {
        auto* spin = CreateDoubleSpinBox(-10.0, 10.0, 4, 0.001);
        spin->setToolTip(QStringLiteral("位置单位为 m。常规工业机械臂建议保持在 ±10 m 以内。"));
        m_orient_pos_spins[idx] = spin;
        orientPosGrid->addWidget(new QLabel(posLabels.at(idx), orientGroup), 0, idx * 2);
        orientPosGrid->addWidget(spin, 0, idx * 2 + 1);
    }
    orientLayout->addLayout(orientPosGrid);

    auto* orientParamLayout = new QFormLayout();
    m_orient_steps_spin = new QSpinBox(orientGroup);
    m_orient_steps_spin->setRange(3, 15);
    m_orient_steps_spin->setValue(7);
    orientParamLayout->addRow(QStringLiteral("每轴步数"), m_orient_steps_spin);
    m_orient_range_spin = new QDoubleSpinBox(orientGroup);
    m_orient_range_spin->setRange(10.0, 360.0);
    m_orient_range_spin->setValue(180.0);
    m_orient_range_spin->setSingleStep(10.0);
    orientParamLayout->addRow(QStringLiteral("搜索范围 [deg]"), m_orient_range_spin);
    orientLayout->addLayout(orientParamLayout);

    auto* orientBtnRow = new QHBoxLayout();
    auto* orientCheckBtn = new QPushButton(QStringLiteral("分析姿态可达性"), orientGroup);
    connect(orientCheckBtn, &QPushButton::clicked, this, &KinematicsWidget::OnOrientationReachabilityClicked);
    orientBtnRow->addWidget(orientCheckBtn);
    orientBtnRow->addStretch();
    orientLayout->addLayout(orientBtnRow);

    m_orient_result_label = new QLabel(QStringLiteral("尚未分析"), orientGroup);
    m_orient_result_label->setWordWrap(true);
    m_orient_result_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_orient_result_label->setStyleSheet(QStringLiteral("color:#475467; background:#f8fafc; padding:6px; border-radius:4px;"));
    orientLayout->addWidget(m_orient_result_label);

    layout->addWidget(orientGroup);

    layout->addStretch();
    return page;
}

QGroupBox* KinematicsWidget::CreateResultGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("结果摘要"), this);
    auto* layout = new QVBoxLayout(groupBox);

    // FK 结果 → IK 目标/种子一键填入按钮
    auto* btnLayout = new QHBoxLayout();
    auto* fkToIkBtn = new QPushButton(QStringLiteral("填入 IK 目标/种子"), groupBox);
    fkToIkBtn->setToolTip(QStringLiteral("将 FK 求解得到的 TCP 位姿填入 IK 目标，并将当前关节角填入 IK 种子。"));
    connect(fkToIkBtn, &QPushButton::clicked, this, &KinematicsWidget::FillIkTargetFromFkResult);
    btnLayout->addWidget(fkToIkBtn);
    btnLayout->addStretch();
    layout->addLayout(btnLayout);

    // 结构化展示区域：滚动容器内含分类 QGroupBox
    auto* scrollArea = new QScrollArea(groupBox);
    scrollArea->setWidgetResizable(true);
    auto* scrollContent = new QWidget(scrollArea);
    auto* scrollLayout = new QVBoxLayout(scrollContent);
    scrollLayout->setContentsMargins(0, 0, 0, 0);

    auto createSection = [&](const QString& title, QLabel*& labelRef) -> QGroupBox* {
        auto* section = new QGroupBox(title, scrollContent);
        auto* sectionLayout = new QVBoxLayout(section);
        labelRef = new QLabel(QStringLiteral("尚未执行"), section);
        labelRef->setWordWrap(true);
        labelRef->setTextInteractionFlags(Qt::TextSelectableByMouse);
        sectionLayout->addWidget(labelRef);
        return section;
    };

    scrollLayout->addWidget(createSection(QStringLiteral("模型概要"), m_result_model_label));
    scrollLayout->addWidget(createSection(QStringLiteral("诊断信息"), m_result_diag_label));
    scrollLayout->addWidget(createSection(QStringLiteral("模型校验明细"), m_result_validation_label));
    // FK 结果区（含可操作度子标签）
    {
        auto* fkSection = new QGroupBox(QStringLiteral("正运动学 FK"), scrollContent);
        auto* fkSectionLayout = new QVBoxLayout(fkSection);
        m_result_fk_label = new QLabel(QStringLiteral("尚未执行"), fkSection);
        m_result_fk_label->setWordWrap(true);
        m_result_fk_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        fkSectionLayout->addWidget(m_result_fk_label);
        m_result_fk_manipulability_label = new QLabel(QStringLiteral(""), fkSection);
        m_result_fk_manipulability_label->setStyleSheet(QStringLiteral("font-size: 11px; color: #555;"));
        m_result_fk_manipulability_label->setVisible(false);
        fkSectionLayout->addWidget(m_result_fk_manipulability_label);
        scrollLayout->addWidget(fkSection);
    }
    scrollLayout->addWidget(createSection(QStringLiteral("逆运动学 IK"), m_result_ik_label));
    scrollLayout->addWidget(createSection(QStringLiteral("工作空间"), m_result_ws_label));
    scrollLayout->addStretch();

    scrollArea->setWidget(scrollContent);
    layout->addWidget(scrollArea, 1);

    // 底部保留纯文本详情（折叠为精简模式，默认隐藏）
    m_result_summary_edit = new QPlainTextEdit(groupBox);
    m_result_summary_edit->setReadOnly(true);
    m_result_summary_edit->setMaximumHeight(120);
    m_result_summary_edit->hide();
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
        QStringLiteral("Link"),
        QStringLiteral("a [m]"),
        QStringLiteral("alpha [deg]"),
        QStringLiteral("d [m]"),
        QStringLiteral("theta offset [deg]")});
    m_dh_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    m_dh_table->horizontalHeader()->setStretchLastSection(true);
    m_dh_table->setColumnWidth(0, 82);
    m_dh_table->setColumnWidth(1, 74);
    m_dh_table->setColumnWidth(2, 98);
    m_dh_table->setColumnWidth(3, 74);
    StyleKinematicsTable(m_dh_table);
}

void KinematicsWidget::SetupJointLimitTableColumns()
{
    m_joint_limit_table->setColumnCount(7);
    m_joint_limit_table->setHorizontalHeaderLabels({
        QStringLiteral("Joint"),
        QStringLiteral("soft min [deg]"),
        QStringLiteral("soft max [deg]"),
        QStringLiteral("hard min [deg]"),
        QStringLiteral("hard max [deg]"),
        QStringLiteral("velocity [deg/s]"),
        QStringLiteral("accel [deg/s²]")});
    m_joint_limit_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    m_joint_limit_table->horizontalHeader()->setStretchLastSection(true);
    m_joint_limit_table->setColumnWidth(0, 78);
    m_joint_limit_table->setColumnWidth(1, 104);
    m_joint_limit_table->setColumnWidth(2, 104);
    m_joint_limit_table->setColumnWidth(3, 104);
    m_joint_limit_table->setColumnWidth(4, 104);
    m_joint_limit_table->setColumnWidth(5, 118);
    StyleKinematicsTable(m_joint_limit_table);
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

    if (RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::IsImportedUrdfReference(model) ||
        model.master_model_type == QStringLiteral("urdf"))
    {
        RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::ApplyImportedUrdfReferenceState(
            model,
            model.urdf_source_path,
            model.urdf_master_source_type);
    }
    else
    {
        RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::ApplyDhParametricState(
            model,
            model.model_source_mode);
    }
    return model;
}

void KinematicsWidget::PopulateForm(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    m_is_populating_form = true;
    // ✅ 新增这一行：根据当前模型的真实轴数动态控制界面滑块！
    AdjustJointInputCount(model.joint_count);
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
        StyleTableItem(idItem, false);
        auto* aItem = EnsureItem(m_dh_table, row, 1);
        auto* alphaItem = EnsureItem(m_dh_table, row, 2);
        auto* dItem = EnsureItem(m_dh_table, row, 3);
        auto* thetaItem = EnsureItem(m_dh_table, row, 4);
        aItem->setText(QString::number(link.a, 'f', 4));
        alphaItem->setText(QString::number(link.alpha, 'f', 3));
        dItem->setText(QString::number(link.d, 'f', 4));
        thetaItem->setText(QString::number(link.theta_offset, 'f', 3));
        StyleTableItem(aItem, true);
        StyleTableItem(alphaItem, true);
        StyleTableItem(dItem, true);
        StyleTableItem(thetaItem, true);
    }

    m_joint_limit_table->setRowCount(static_cast<int>(model.joint_limits.size()));
    for (int row = 0; row < static_cast<int>(model.joint_limits.size()); ++row)
    {
        const auto& limit = model.joint_limits.at(static_cast<std::size_t>(row));
        auto* idItem = EnsureItem(m_joint_limit_table, row, 0);
        idItem->setText(limit.joint_id);
        SetReadOnlyItem(idItem);
        StyleTableItem(idItem, false);
        for (int column = 1; column <= 6; ++column)
        {
            QTableWidgetItem* item = EnsureItem(m_joint_limit_table, row, column);
            StyleTableItem(item, true);
        }
        m_joint_limit_table->item(row, 1)->setText(QString::number(limit.soft_limit[0], 'f', 3));
        m_joint_limit_table->item(row, 2)->setText(QString::number(limit.soft_limit[1], 'f', 3));
        m_joint_limit_table->item(row, 3)->setText(QString::number(limit.hard_limit[0], 'f', 3));
        m_joint_limit_table->item(row, 4)->setText(QString::number(limit.hard_limit[1], 'f', 3));
        m_joint_limit_table->item(row, 5)->setText(QString::number(limit.max_velocity, 'f', 3));
        m_joint_limit_table->item(row, 6)->setText(QString::number(limit.max_acceleration, 'f', 3));
    }

    const int solverIndex = m_solver_type_combo->findData(model.ik_solver_config.solver_type);
    m_solver_type_combo->setCurrentIndex(solverIndex >= 0 ? solverIndex : 0);
    const int branchIndex = m_branch_policy_combo->findData(model.ik_solver_config.branch_policy);
    m_branch_policy_combo->setCurrentIndex(branchIndex >= 0 ? branchIndex : 0);
    m_max_iterations_spin->setValue(model.ik_solver_config.max_iterations);
    m_position_tolerance_spin->setValue(model.ik_solver_config.position_tolerance_mm);
    m_orientation_tolerance_spin->setValue(model.ik_solver_config.orientation_tolerance_deg);
    m_step_gain_spin->setValue(model.ik_solver_config.step_gain);
    
    m_is_populating_form = false;
    RefreshEditingState();
    UpdateFkJointLimitLabels();
    RefreshModelStatusLabels();
    RefreshValidationState();
}

void KinematicsWidget::RefreshEditingState()
{
    const bool dhEditable = m_state.current_model.dh_editable;
    const bool isUrdfDraftReadOnly =
        !dhEditable && m_state.current_model.master_model_type == QStringLiteral("urdf");
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


    if (m_dh_readonly_banner_label != nullptr)
    {
        m_dh_readonly_banner_label->setVisible(isUrdfDraftReadOnly);
        m_dh_readonly_banner_label->setText(
            isUrdfDraftReadOnly
                ? m_state.current_model.dh_draft_readonly_reason
                : QString());
    }
}

QStringList KinematicsWidget::BuildValidationIssues(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const
{
    QStringList issues;

    if (model.meta.name.trimmed().isEmpty())
    {
        issues.push_back(QStringLiteral("模型名称为空"));
    }

    if (model.parameter_convention != QStringLiteral("DH") &&
        model.parameter_convention != QStringLiteral("MDH"))
    {
        issues.push_back(QStringLiteral("参数约定必须是 DH 或 MDH"));
    }

    if (model.links.empty())
    {
        issues.push_back(QStringLiteral("DH/MDH 连杆参数为空"));
    }

    if (model.joint_count != static_cast<int>(model.links.size()))
    {
        issues.push_back(QStringLiteral("关节数量与 DH/MDH 行数不一致"));
    }

    if (model.joint_limits.size() != model.links.size())
    {
        issues.push_back(QStringLiteral("关节限位行数与 DH/MDH 行数不一致"));
    }

    for (std::size_t index = 0; index < model.links.size(); ++index)
    {
        const auto& link = model.links[index];
        const QString rowLabel = QStringLiteral("DH 第 %1 行").arg(index + 1);
        if (link.link_id.trimmed().isEmpty())
        {
            issues.push_back(QStringLiteral("%1 link_id 为空").arg(rowLabel));
        }
        if (!std::isfinite(link.a) ||
            !std::isfinite(link.alpha) ||
            !std::isfinite(link.d) ||
            !std::isfinite(link.theta_offset))
        {
            issues.push_back(QStringLiteral("%1 存在非有限数值").arg(rowLabel));
        }
        if (std::abs(link.a) > 20.0 || std::abs(link.d) > 20.0)
        {
            issues.push_back(QStringLiteral("%1 长度超过 20 m，请确认单位是否误填为 mm").arg(rowLabel));
        }
    }

    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        const auto& limit = model.joint_limits[index];
        const QString rowLabel = QStringLiteral("关节限位第 %1 行").arg(index + 1);
        if (limit.joint_id.trimmed().isEmpty())
        {
            issues.push_back(QStringLiteral("%1 joint_id 为空").arg(rowLabel));
        }
        if (!std::isfinite(limit.soft_limit[0]) ||
            !std::isfinite(limit.soft_limit[1]) ||
            !std::isfinite(limit.hard_limit[0]) ||
            !std::isfinite(limit.hard_limit[1]) ||
            !std::isfinite(limit.max_velocity) ||
            !std::isfinite(limit.max_acceleration))
        {
            issues.push_back(QStringLiteral("%1 存在非有限数值").arg(rowLabel));
        }
        if (limit.soft_limit[0] > limit.soft_limit[1])
        {
            issues.push_back(QStringLiteral("%1 soft_min 大于 soft_max").arg(rowLabel));
        }
        if (limit.hard_limit[0] > limit.hard_limit[1])
        {
            issues.push_back(QStringLiteral("%1 hard_min 大于 hard_max").arg(rowLabel));
        }
        if (limit.soft_limit[0] < limit.hard_limit[0] ||
            limit.soft_limit[1] > limit.hard_limit[1])
        {
            issues.push_back(QStringLiteral("%1 软限位超出硬限位范围").arg(rowLabel));
        }
        if (limit.max_velocity <= 0.0 || limit.max_acceleration <= 0.0)
        {
            issues.push_back(QStringLiteral("%1 速度/加速度上限必须大于 0").arg(rowLabel));
        }
    }

    if (!IsFinitePose(model.base_frame))
    {
        issues.push_back(QStringLiteral("基坐标系存在非有限数值"));
    }
    if (!IsFinitePose(model.flange_frame))
    {
        issues.push_back(QStringLiteral("法兰坐标系存在非有限数值"));
    }
    if (!IsFiniteTcpFrame(model.tcp_frame))
    {
        issues.push_back(QStringLiteral("TCP 坐标系存在非有限数值"));
    }
    if (model.tool_frame.has_value() && !IsFinitePose(model.tool_frame.value()))
    {
        issues.push_back(QStringLiteral("Tool Frame 存在非有限数值"));
    }
    if (model.workpiece_frame.has_value() && !IsFinitePose(model.workpiece_frame.value()))
    {
        issues.push_back(QStringLiteral("Workpiece Frame 存在非有限数值"));
    }

    if (model.master_model_type == QStringLiteral("urdf") && model.dh_editable)
    {
        issues.push_back(QStringLiteral("工程 URDF 参考视图下 DH 表不应处于可编辑状态"));
    }
    if (model.master_model_type == QStringLiteral("dh_mdh") && !model.dh_editable)
    {
        issues.push_back(QStringLiteral("DH/MDH 参数化设计模型下 DH 表应处于可编辑状态"));
    }

    if (m_dh_table != nullptr)
    {
        for (int row = 0; row < m_dh_table->rowCount(); ++row)
        {
            for (int column = 1; column <= 4; ++column)
            {
                double value = 0.0;
                if (!TryReadTableDouble(m_dh_table, row, column, value))
                {
                    issues.push_back(QStringLiteral("DH 第 %1 行第 %2 列不是有效数值")
                        .arg(row + 1)
                        .arg(column + 1));
                }
            }
        }
    }

    if (m_joint_limit_table != nullptr)
    {
        for (int row = 0; row < m_joint_limit_table->rowCount(); ++row)
        {
            for (int column = 1; column <= 6; ++column)
            {
                double value = 0.0;
                if (!TryReadTableDouble(m_joint_limit_table, row, column, value))
                {
                    issues.push_back(QStringLiteral("关节限位第 %1 行第 %2 列不是有效数值")
                        .arg(row + 1)
                        .arg(column + 1));
                }
            }
        }
    }

    return issues;
}

void KinematicsWidget::RefreshValidationHighlights(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    const bool dhSignalsWereBlocked =
        m_dh_table != nullptr ? m_dh_table->blockSignals(true) : false;
    const bool limitSignalsWereBlocked =
        m_joint_limit_table != nullptr ? m_joint_limit_table->blockSignals(true) : false;

    auto clearTable = [](QTableWidget* table) {
        if (table == nullptr)
        {
            return;
        }
        for (int row = 0; row < table->rowCount(); ++row)
        {
            for (int column = 0; column < table->columnCount(); ++column)
            {
                QTableWidgetItem* item = table->item(row, column);
                if (item == nullptr)
                {
                    continue;
                }
                if (column == 0)
                {
                    item->setBackground(QColor(QStringLiteral("#f8fafc")));
                    item->setForeground(QColor(QStringLiteral("#475467")));
                }
                else
                {
                    item->setBackground(QBrush());
                    item->setForeground(QBrush());
                }
                item->setToolTip(QString());
            }
        }
    };

    auto markCell = [](QTableWidget* table, int row, int column, const QString& message) {
        if (table == nullptr || row < 0 || column < 0 ||
            row >= table->rowCount() || column >= table->columnCount())
        {
            return;
        }
        QTableWidgetItem* item = table->item(row, column);
        if (item == nullptr)
        {
            return;
        }
        item->setBackground(QColor(QStringLiteral("#fff1f3")));
        item->setForeground(QColor(QStringLiteral("#912018")));
        item->setToolTip(message);
    };

    clearTable(m_dh_table);
    clearTable(m_joint_limit_table);

    if (m_dh_table != nullptr)
    {
        for (int row = 0; row < m_dh_table->rowCount(); ++row)
        {
            if (ReadTableString(m_dh_table, row, 0).isEmpty())
            {
                markCell(m_dh_table, row, 0, QStringLiteral("link_id 不能为空。"));
            }

            for (int column = 1; column <= 4; ++column)
            {
                double value = 0.0;
                if (!TryReadTableDouble(m_dh_table, row, column, value))
                {
                    markCell(m_dh_table, row, column, QStringLiteral("请输入有效数值。"));
                    continue;
                }

                if ((column == 1 || column == 3) && std::abs(value) > 20.0)
                {
                    markCell(
                        m_dh_table,
                        row,
                        column,
                        QStringLiteral("长度超过 20 m，请确认当前单位是 m，而不是 mm。"));
                }
            }
        }
    }

    if (m_joint_limit_table != nullptr)
    {
        for (int row = 0; row < m_joint_limit_table->rowCount(); ++row)
        {
            if (ReadTableString(m_joint_limit_table, row, 0).isEmpty())
            {
                markCell(m_joint_limit_table, row, 0, QStringLiteral("joint_id 不能为空。"));
            }

            double values[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            bool valid[6] = {true, true, true, true, true, true};
            for (int column = 1; column <= 6; ++column)
            {
                valid[column - 1] = TryReadTableDouble(m_joint_limit_table, row, column, values[column - 1]);
                if (!valid[column - 1])
                {
                    markCell(m_joint_limit_table, row, column, QStringLiteral("请输入有效数值。"));
                }
            }

            if (valid[0] && valid[1] && values[0] > values[1])
            {
                markCell(m_joint_limit_table, row, 1, QStringLiteral("soft_min 不能大于 soft_max。"));
                markCell(m_joint_limit_table, row, 2, QStringLiteral("soft_max 不能小于 soft_min。"));
            }
            if (valid[2] && valid[3] && values[2] > values[3])
            {
                markCell(m_joint_limit_table, row, 3, QStringLiteral("hard_min 不能大于 hard_max。"));
                markCell(m_joint_limit_table, row, 4, QStringLiteral("hard_max 不能小于 hard_min。"));
            }
            if (valid[0] && valid[2] && values[0] < values[2])
            {
                markCell(m_joint_limit_table, row, 1, QStringLiteral("软限位下界不能小于硬限位下界。"));
            }
            if (valid[1] && valid[3] && values[1] > values[3])
            {
                markCell(m_joint_limit_table, row, 2, QStringLiteral("软限位上界不能大于硬限位上界。"));
            }
            if (valid[4] && values[4] <= 0.0)
            {
                markCell(m_joint_limit_table, row, 5, QStringLiteral("最大速度必须大于 0。"));
            }
            if (valid[5] && values[5] <= 0.0)
            {
                markCell(m_joint_limit_table, row, 6, QStringLiteral("最大加速度必须大于 0。"));
            }
        }
    }

    if (m_dh_table != nullptr)
    {
        m_dh_table->blockSignals(dhSignalsWereBlocked);
    }
    if (m_joint_limit_table != nullptr)
    {
        m_joint_limit_table->blockSignals(limitSignalsWereBlocked);
    }

    Q_UNUSED(model);
}

void KinematicsWidget::RefreshValidationState()
{
    if (m_validation_label == nullptr || m_is_populating_form)
    {
        return;
    }

    const auto model = CollectModelFromForm();
    const QStringList issues = BuildValidationIssues(model);
    RefreshValidationHighlights(model);
    if (issues.isEmpty())
    {
        m_validation_label->setText(QStringLiteral("模型校验：通过。DH/MDH、关节限位、坐标系与主模型状态当前一致。"));
        m_validation_label->setStyleSheet(QStringLiteral(
            "padding:7px 9px;border-radius:6px;background:#ecfdf3;color:#027a48;border:1px solid #abefc6;font-weight:600;"));
        if (m_result_validation_label != nullptr)
        {
            m_result_validation_label->setText(QStringLiteral("状态：通过\n当前模型可用于 FK / IK / 工作空间分析。"));
            m_result_validation_label->setStyleSheet(QStringLiteral("color:#027a48;"));
        }
        RefreshHeaderBadges();
        return;
    }

    const QStringList previewIssues = issues.mid(0, 3);
    const QString suffix = issues.size() > previewIssues.size()
        ? QStringLiteral(" 等 %1 项").arg(issues.size())
        : QString();
    m_validation_label->setText(
        QStringLiteral("模型校验：发现 %1 项待确认：%2%3")
            .arg(issues.size())
            .arg(previewIssues.join(QStringLiteral("；")))
            .arg(suffix));
    m_validation_label->setStyleSheet(QStringLiteral(
        "padding:7px 9px;border-radius:6px;background:#fff7ed;color:#b54708;border:1px solid #fed7aa;font-weight:600;"));
    if (m_result_validation_label != nullptr)
    {
        QStringList lines;
        lines.push_back(QStringLiteral("状态：发现 %1 项待确认").arg(issues.size()));
        for (int index = 0; index < issues.size(); ++index)
        {
            lines.push_back(QStringLiteral("%1. %2").arg(index + 1).arg(issues.at(index)));
        }
        m_result_validation_label->setText(lines.join(QLatin1Char('\n')));
        m_result_validation_label->setStyleSheet(QStringLiteral("color:#b54708;"));
    }
    RefreshHeaderBadges();
}

void KinematicsWidget::RefreshHeaderBadges()
{
    if (m_validation_badge_label == nullptr)
    {
        return;
    }

    const auto model = CollectModelFromForm();
    const QStringList issues = BuildValidationIssues(model);
    if (issues.isEmpty())
    {
        ApplyBadgeStyle(m_validation_badge_label, QStringLiteral("校验通过"), QStringLiteral("success"));
    }
    else
    {
        ApplyBadgeStyle(
            m_validation_badge_label,
            QStringLiteral("待确认 %1").arg(issues.size()),
            QStringLiteral("warning"));
    }

    ApplyBadgeStyle(
        m_dirty_badge_label,
        m_has_unsaved_changes ? QStringLiteral("未保存") : QStringLiteral("已保存"),
        m_has_unsaved_changes ? QStringLiteral("warning") : QStringLiteral("success"));

    const QString derivedState = FormatDerivedModelState(m_state.current_model.derived_model_state);
    ApplyBadgeStyle(
        m_sync_badge_label,
        derivedState,
        derivedState == QStringLiteral("已同步") ? QStringLiteral("success") : QStringLiteral("warning"));

    const QString sourceMode =
        RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::IsImportedUrdfReference(m_state.current_model)
            ? QStringLiteral("URDF 参考")
            : (RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::IsDhParametric(m_state.current_model)
                   ? QStringLiteral("DH 主模型")
                   : QStringLiteral("未加载"));
    ApplyBadgeStyle(
        m_source_badge_label,
        sourceMode,
        sourceMode == QStringLiteral("未加载") ? QStringLiteral("neutral") : QStringLiteral("info"));
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
    // ── 模型概要 ──────────────────────────────────────────────
    {
        QStringList lines;
        lines.push_back(QStringLiteral("模型：%1").arg(m_state.current_model.meta.name));
        lines.push_back(QStringLiteral("模型语义：%1").arg(
            RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::BuildUserFacingStateText(
                m_state.current_model)));
        lines.push_back(QStringLiteral("派生状态：%1").arg(FormatDerivedModelState(m_state.current_model.derived_model_state)));
        lines.push_back(QStringLiteral("中央预览来源：%1").arg(FormatPreviewSourceMode(m_preview_source_mode)));
        lines.push_back(QStringLiteral("参数约定：%1").arg(m_state.current_model.parameter_convention));
        lines.push_back(QStringLiteral("骨架预览：%1").arg(FormatPreviewSceneSummary(m_preview_scene)));
        if (!m_preview_scene.IsEmpty())
        {
            lines.push_back(QStringLiteral("预览模型名：%1").arg(m_preview_scene.model_name));
            if (!m_preview_scene.urdf_file_path.trimmed().isEmpty())
            {
                lines.push_back(QStringLiteral("URDF 文件：%1").arg(m_preview_scene.urdf_file_path));
            }
        }
        if (!m_state.current_model.conversion_diagnostics.trimmed().isEmpty())
        {
            lines.push_back(QStringLiteral("同步摘要：%1").arg(m_state.current_model.conversion_diagnostics));
        }
        if (!m_state.current_model.original_imported_urdf_path.trimmed().isEmpty())
        {
            lines.push_back(QStringLiteral("原始导入 URDF：%1").arg(m_state.current_model.original_imported_urdf_path));
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
        if (m_result_model_label != nullptr)
        {
            m_result_model_label->setText(lines.join(QLatin1Char('\n')));
        }
    }

    // ── 诊断信息 ──────────────────────────────────────────────
    {
        QStringList lines;
        lines.push_back(QStringLiteral("backend_id = %1").arg(m_backend_diagnostic.backend_id));
        lines.push_back(QStringLiteral("语义归一 = %1").arg(FormatReadyState(m_backend_diagnostic.status.normalized_semantics_ready)));
        lines.push_back(QStringLiteral("Frame 语义 = %1").arg(FormatReadyState(m_backend_diagnostic.status.frame_semantics_ready)));
        lines.push_back(QStringLiteral("Joint 顺序 = %1").arg(FormatReadyState(m_backend_diagnostic.status.joint_order_ready)));
        lines.push_back(QStringLiteral("Build-Context = %1").arg(FormatReadyState(m_backend_diagnostic.status.build_context_ready)));
        lines.push_back(QStringLiteral("共享内核 = %1").arg(FormatReadyState(m_backend_diagnostic.status.shared_robot_kernel_ready)));
        if (!m_backend_diagnostic.status.status_message.isEmpty())
        {
            lines.push_back(QStringLiteral("诊断摘要：%1").arg(m_backend_diagnostic.status.status_message));
        }
        if (m_result_diag_label != nullptr)
        {
            m_result_diag_label->setText(lines.join(QLatin1Char('\n')));
        }
    }

    // ── FK 结果 ────────────────────────────────────────────────
    {
        QStringList lines;
        if (m_state.last_fk_result.success)
        {
            const auto& pose = m_state.last_fk_result.tcp_pose;
            lines.push_back(QStringLiteral("状态：成功"));
            lines.push_back(QStringLiteral("TCP 位置[m] = %1").arg(FormatArray3(pose.position_m, 4)));
            lines.push_back(QStringLiteral("TCP 姿态[deg] = %1").arg(FormatArray3(pose.rpy_deg, 3)));
            lines.push_back(QStringLiteral("关节输入 = %1").arg(FormatJointVector(m_state.last_fk_result.joint_positions_deg)));
        }
        else
        {
            lines.push_back(QStringLiteral("状态：%1").arg(m_state.last_fk_result.message.isEmpty()
                ? QStringLiteral("尚未执行")
                : m_state.last_fk_result.message));
        }
        if (m_result_fk_label != nullptr)
        {
            m_result_fk_label->setText(lines.join(QLatin1Char('\n')));
        }

        // 可操作度与条件数显示
        if (m_result_fk_manipulability_label != nullptr)
        {
            if (m_last_jacobian_analysis.success)
            {
                const QString singularFlag = m_last_jacobian_analysis.is_singular
                    ? QStringLiteral(" ⚠ 奇异")
                    : QString();
                m_result_fk_manipulability_label->setText(QStringLiteral(
                    "可操作度 w = %1 | 条件数 κ = %2%3")
                    .arg(m_last_jacobian_analysis.manipulability, 0, 'f', 6)
                    .arg(m_last_jacobian_analysis.condition_number, 0, 'f', 2)
                    .arg(singularFlag));
                m_result_fk_manipulability_label->setStyleSheet(
                    m_last_jacobian_analysis.is_singular
                        ? QStringLiteral("font-size: 11px; color: #dc2626; font-weight: bold;")
                        : QStringLiteral("font-size: 11px; color: #555;"));
                m_result_fk_manipulability_label->setVisible(true);
            }
            else
            {
                m_result_fk_manipulability_label->setVisible(false);
            }
        }
    }

    // ── IK 结果 ────────────────────────────────────────────────
    {
        QStringList lines;
        if (m_state.last_ik_result.success)
        {
            lines.push_back(QStringLiteral("状态：成功"));
            lines.push_back(QStringLiteral("求解器：%1").arg(m_state.last_ik_result.solver_id.isEmpty()
                ? QStringLiteral("默认") : m_state.last_ik_result.solver_id));
            lines.push_back(QStringLiteral("解 = %1").arg(FormatJointVector(m_state.last_ik_result.joint_positions_deg)));
            lines.push_back(
                QStringLiteral("位置误差 = %1 mm，姿态误差 = %2 deg，迭代次数 = %3")
                    .arg(m_state.last_ik_result.position_error_mm, 0, 'f', 3)
                    .arg(m_state.last_ik_result.orientation_error_deg, 0, 'f', 3)
                    .arg(m_state.last_ik_result.iteration_count));
            if (m_state.last_ik_result.total_solutions_found > 1)
            {
                lines.push_back(QStringLiteral("过滤前总解数 = %1，有效解数 = %2")
                    .arg(m_state.last_ik_result.total_solutions_found)
                    .arg(m_state.last_ik_result.valid_solution_count));
            }
        }
        else
        {
            lines.push_back(QStringLiteral("状态：%1").arg(m_state.last_ik_result.message.isEmpty()
                ? QStringLiteral("尚未执行")
                : m_state.last_ik_result.message));
        }
        if (m_result_ik_label != nullptr)
        {
            m_result_ik_label->setText(lines.join(QLatin1Char('\n')));
        }

        // ── 更新多解浏览器 ────────────────────────────────────
        if (m_ik_solution_combo != nullptr)
        {
            const auto& allSol = m_state.last_ik_result.all_solutions_deg;
            const bool hasMultiple = (allSol.size() > 1);
            m_ik_solution_combo->setEnabled(hasMultiple);
            m_ik_solution_combo->blockSignals(true);
            m_ik_solution_combo->clear();

            if (!allSol.empty())
            {
                for (std::size_t i = 0; i < allSol.size(); ++i)
                {
                    m_ik_solution_combo->addItem(
                        QStringLiteral("解 %1/%2").arg(i + 1).arg(allSol.size()),
                        static_cast<qulonglong>(i));
                }
                // 自动选中最佳解（索引 0）
                m_ik_solution_combo->setCurrentIndex(0);
                for (std::size_t j = 0; j < m_ik_solution_spins.size() && j < allSol[0].size(); ++j)
                    m_ik_solution_spins[j]->setValue(allSol[0][j]);
            }
            else
            {
                for (auto* sp : m_ik_solution_spins)
                    sp->setValue(0.0);
            }
            m_ik_solution_combo->blockSignals(false);
        }
    }

    // ── 工作空间结果 ──────────────────────────────────────────
    {
        QStringList lines;
        if (m_state.last_workspace_result.success)
        {
            lines.push_back(QStringLiteral("状态：成功"));
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
            lines.push_back(QStringLiteral("状态：%1").arg(m_state.last_workspace_result.message.isEmpty()
                ? QStringLiteral("尚未执行")
                : m_state.last_workspace_result.message));
        }
        if (m_result_ws_label != nullptr)
        {
            m_result_ws_label->setText(lines.join(QLatin1Char('\n')));
        }
    }

    RefreshModelStatusLabels();
    RefreshValidationState();
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

// =========================================================================
// 通道 A：结构变动处理 (重量级重建)
// =========================================================================
void KinematicsWidget::SyncStructureAndPreview()
{
    if (m_is_populating_form) return;

    // 1. 收集界面草稿
    auto draftModel = CollectModelFromForm();

    // 如果当前是 URDF 只读模式，不允许利用表格直接驱动底层骨架改变
    if (draftModel.master_model_type == QStringLiteral("urdf")) {
        return; 
    }

    // 只要结构发生变动（包含从拓扑更新），强制赋予带时间戳的唯一 ID。
    // 这将迫使物理引擎彻底销毁旧缓存，重新解析新的 DH 参数建树！
    draftModel.unified_robot_model_ref = QStringLiteral("dh_preview::%1_%2")
        .arg(draftModel.meta.kinematic_id.trimmed().isEmpty() ? QStringLiteral("default") : draftModel.meta.kinematic_id.trimmed())
        .arg(QDateTime::currentMSecsSinceEpoch());

    // 2. 调用 Service 的通道 A 接口重建引擎环境与场景
    // 中文说明：实时预览只负责更新内存中的统一模型与中央骨架，不在此时写出派生 URDF 文件。
    // 真正的派生产物落盘只保留在 SaveDraft() 流程中，避免“改一格参数就改工程文件”的副作用。
    auto angles = CollectJointInputs(m_fk_joint_spins);

    const auto buildResult = m_service.BuildDhPreviewScene(draftModel, angles);

    if (buildResult.IsSuccess())
    {
        // 3. 静默吸收引擎分配的内部签名，绝对不反写表格以防打断用户输入
        m_state.current_model.unified_robot_snapshot = buildResult.preview_model.unified_robot_snapshot;
        m_state.current_model.joint_order_signature = buildResult.preview_model.joint_order_signature;
        m_state.current_model.pinocchio_model_ready = buildResult.preview_model.pinocchio_model_ready;
        m_state.current_model.conversion_diagnostics = buildResult.preview_model.conversion_diagnostics;
        m_state.current_model.unified_robot_model_ref = buildResult.preview_model.unified_robot_model_ref;
        
        m_preview_scene = buildResult.preview_scene;
        m_preview_model = buildResult.preview_model;
        m_preview_source_mode = QStringLiteral("dh_preview");

        // 4. 发射重绘信号给 VTK
        emit PreviewSceneGenerated(m_preview_scene);
        
        RefreshBackendDiagnostics();
        RenderResults();
        RefreshModelStatusLabels();
        RefreshValidationState();
    }
    else 
    {
        SetOperationMessage(buildResult.message, false, true);
        EmitTelemetryStatus(QStringLiteral("DH/MDH 参数化设计"), buildResult.message, true);
        RefreshValidationState();
    }
}

// =========================================================================
// 通道 B：姿态变动处理 (轻量级位姿更新 / 骨架实时重绘)
// =========================================================================
void KinematicsWidget::SyncPoseOnly()
{
    if (m_is_populating_form) return;

    // 每次姿态刷新时同步检测关节越界状态并更新视觉提示
    UpdateJointLimitWarningStyle();
    UpdateJointLimitMargins();

    auto angles = CollectJointInputs(m_fk_joint_spins);

    // =====================================================================
    // 路线 1：DH/MDH 参数化设计模型
    // =====================================================================
    if (m_state.current_model.master_model_type == QStringLiteral("dh_mdh"))
    {
        // 【核心修复 1】：必须抓取界面上实时的表格数据，而不是用旧的缓存
        auto liveModel = CollectModelFromForm();
        liveModel.unified_robot_model_ref = m_state.current_model.unified_robot_model_ref;
        liveModel.joint_order_signature = m_state.current_model.joint_order_signature;

        RoboSDP::Kinematics::Dto::FkRequestDto request;
        request.joint_positions_deg = angles;
        // 自动将界面上的 6 个角度截断为模型实际需要的数量（例如 2 个）
        request.joint_positions_deg.resize(static_cast<std::size_t>(liveModel.joint_count), 0.0);

        // 调用基于 Pinocchio 共享内核的 SolveFk
        const auto fkResult = m_service.SolveFk(liveModel, request);

        if (fkResult.success)
        {
            PreviewPoseMap poseMap;
            poseMap[QStringLiteral("base_link")] = liveModel.base_frame;
            for (const auto& linkPose : fkResult.link_poses)
            {
                poseMap[linkPose.link_id] = linkPose.pose;
            }
            poseMap[QStringLiteral("tcp_frame")] = fkResult.tcp_pose;
            
            emit PreviewPosesUpdated(poseMap);

            // 同步更新 FK 区域实时 TCP 位姿读出标签
            m_state.last_fk_result = fkResult;
            UpdateFkPoseReadout(fkResult.tcp_pose);
        }
        else
        {
            // 【核心修复 2】：绝对不能静默失败！把拦截原因红字打在公屏上！
            emit LogMessageGenerated(QStringLiteral("[Kinematics][Error] DH/MDH 姿态刷新被拦截：%1").arg(fkResult.message));
        }
        return;
    }

    // =====================================================================
    // 路线 2：URDF 工程模型
    // =====================================================================
    const auto updateResult = m_service.UpdatePreviewPoses(m_state.current_model, angles);
    if (updateResult.IsSuccess() && !updateResult.link_world_poses.empty())
    {
        emit PreviewPosesUpdated(updateResult.link_world_poses);

        // 从 URDF 预览姿态 map 中提取 TCP 位姿用于实时读出
        auto it = updateResult.link_world_poses.find(QStringLiteral("tcp_frame"));
        if (it != updateResult.link_world_poses.end())
        {
            UpdateFkPoseReadout(it->second);
        }
    }
    else 
    {
        // 同样，把 URDF 失败的原因打印出来
        emit LogMessageGenerated(QStringLiteral("[Kinematics][Warning] URDF 姿态刷新被拦截：%1").arg(updateResult.message));
    }
}

void KinematicsWidget::UpdateFkPoseReadout(
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& tcpPose)
{
    if (m_fk_tcp_pos_label != nullptr)
    {
        m_fk_tcp_pos_label->setText(
            QStringLiteral("P: (%1, %2, %3) m")
                .arg(tcpPose.position_m[0], 8, 'f', 4, QLatin1Char(' '))
                .arg(tcpPose.position_m[1], 8, 'f', 4, QLatin1Char(' '))
                .arg(tcpPose.position_m[2], 8, 'f', 4, QLatin1Char(' ')));
    }
    if (m_fk_tcp_rpy_label != nullptr)
    {
        m_fk_tcp_rpy_label->setText(
            QStringLiteral("R: (%1, %2, %3) deg")
                .arg(tcpPose.rpy_deg[0], 7, 'f', 2, QLatin1Char(' '))
                .arg(tcpPose.rpy_deg[1], 7, 'f', 2, QLatin1Char(' '))
                .arg(tcpPose.rpy_deg[2], 7, 'f', 2, QLatin1Char(' ')));
    }
}

// =========================================================================
// 【逆向驱动】3D 视图滚轮 → FK 滑块联动
// =========================================================================
void KinematicsWidget::HandleJointAngleScrolled(int jointIndex, double deltaDeg)
{
    // 防表单填充期间误触发
    if (m_is_populating_form) return;

    // 边界检查：确保索引在有效范围内
    if (jointIndex < 0 || jointIndex >= static_cast<int>(m_fk_joint_spins.size()))
    {
        return;
    }

    // 中文说明：读取当前值，叠加增量，再设回 spinbox。
    // spinbox 的 setValue 会自动触发 valueChanged → signal → markDirty。
    const double newValue = m_fk_joint_spins[jointIndex]->value() + deltaDeg;
    m_fk_joint_spins[jointIndex]->setValue(newValue);

    // 触发通道 B：轻量级姿态刷新（仅更新 vtkTransform，不重建场景）
    SyncPoseOnly();
}

// =========================================================================
// 【逆向驱动】TCP Gizmo 拖动 → IK 求解联动
// =========================================================================
void KinematicsWidget::HandleTcpPoseDragged(
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& newPose)
{
    // 防表单填充期间误触发
    if (m_is_populating_form) return;

    // 将 Gizmo 拖动产生的新位姿填充到 IK 目标 spinbox 中
    m_ik_target_pose_spins[0]->setValue(newPose.position_m[0]);
    m_ik_target_pose_spins[1]->setValue(newPose.position_m[1]);
    m_ik_target_pose_spins[2]->setValue(newPose.position_m[2]);
    m_ik_target_pose_spins[3]->setValue(newPose.rpy_deg[0]);
    m_ik_target_pose_spins[4]->setValue(newPose.rpy_deg[1]);
    m_ik_target_pose_spins[5]->setValue(newPose.rpy_deg[2]);

    // 触发 IK 求解：求解结果将通过 RenderResults 自动刷新 FK 滑块和 3D 视图
    OnRunIkClicked();
}

void KinematicsWidget::HandlePreviewLinkPicked(const QString& linkName)
{
    if (m_dh_table == nullptr)
    {
        return;
    }

    m_dh_table->clearSelection();
    const QString normalizedLinkName = linkName.trimmed();
    if (normalizedLinkName.isEmpty())
    {
        return;
    }

    for (int row = 0; row < m_dh_table->rowCount(); ++row)
    {
        const QTableWidgetItem* item = m_dh_table->item(row, 0);
        if (item != nullptr && item->text().trimmed() == normalizedLinkName)
        {
            m_dh_table->selectRow(row);
            m_dh_table->scrollToItem(m_dh_table->item(row, 0), QAbstractItemView::PositionAtCenter);
            SetOperationMessage(
                QStringLiteral("已在 DH 参数表中定位 3D 选中连杆：%1").arg(normalizedLinkName),
                true);
            return;
        }
    }
}

void KinematicsWidget::ClearPreviewContext()
{
    // 从 Topology/JSON 切换模型结构时，旧中央预览上下文不能继续留在视图里。
    m_preview_scene = PreviewSceneDto {};
    m_preview_model = RoboSDP::Kinematics::Dto::KinematicModelDto {};
    m_preview_source_mode = QStringLiteral("none");
    emit IkPoseComparisonCleared();
    emit PreviewSceneGenerated(m_preview_scene);
    RefreshModelStatusLabels();
}

void KinematicsWidget::RefreshModelStatusLabels()
{
    if (m_master_model_mode_label != nullptr)
    {
        m_master_model_mode_label->setText(
            RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::BuildUserFacingStateText(
                m_state.current_model));
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

std::vector<double> KinematicsWidget::CollectJointInputs(const std::vector<QDoubleSpinBox*>& spinBoxes) const
{
    std::vector<double> values;
    // 获取当前模型实际的自由度
    int activeCount = m_state.current_model.joint_count; 
    // 只去拿前 activeCount 个滑块的数值，忽略后面被隐藏的滑块
    for (int i = 0; i < activeCount && i < static_cast<int>(spinBoxes.size()); ++i)
    {
        values.push_back(spinBoxes[i]->value());
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

void KinematicsWidget::UpdateFkJointLimitLabels()
{
    // 从关节限位表读取 soft_min/soft_max，更新 FK 滑块标签显示限位范围
    for (int i = 0; i < static_cast<int>(m_fk_joint_labels.size()); ++i)
    {
        if (i < m_joint_limit_table->rowCount())
        {
            const QString jointId = ReadTableString(m_joint_limit_table, i, 0);
            const double softMin = ReadTableDouble(m_joint_limit_table, i, 1);
            const double softMax = ReadTableDouble(m_joint_limit_table, i, 2);
            const QString label = jointId.isEmpty()
                ? QStringLiteral("J%1").arg(i + 1)
                : jointId;
            m_fk_joint_labels[i]->setText(QStringLiteral("%1 [%2, %3]")
                .arg(label)
                .arg(softMin, 0, 'f', 1)
                .arg(softMax, 0, 'f', 1));

            // Sync slider limits to joint limits
            if (i < static_cast<int>(m_fk_joint_sliders.size())) {
                m_fk_joint_sliders[i]->blockSignals(true);
                m_fk_joint_sliders[i]->setRange(static_cast<int>(std::floor(softMin)), static_cast<int>(std::ceil(softMax)));
                m_fk_joint_sliders[i]->blockSignals(false);
            }
        }
        else
        {
            // 限位表尚未填充时显示默认标签
            m_fk_joint_labels[i]->setText(QStringLiteral("J%1").arg(i + 1));
        }
    }
}

void KinematicsWidget::UpdateJointLimitWarningStyle()
{
    // 表单填充期间跳过，避免限位表未就绪时误报
    if (m_is_populating_form) return;

    // 检查 FK 关节角度是否超出 soft_limit，越界时设置红色背景
    for (int i = 0; i < static_cast<int>(m_fk_joint_spins.size()); ++i)
    {
        if (i < m_joint_limit_table->rowCount())
        {
            const double softMin = ReadTableDouble(m_joint_limit_table, i, 1);
            const double softMax = ReadTableDouble(m_joint_limit_table, i, 2);
            const double value = m_fk_joint_spins[i]->value();
            const bool outOfLimit = (value < softMin || value > softMax);
            m_fk_joint_spins[i]->setStyleSheet(outOfLimit
                ? QStringLiteral("background-color: #fecaca; color: #991b1b;")
                : QString());
        }
    }
}

/// @brief 计算每个 FK 关节距离软限位的归一化裕量，更新裕量标签颜色。
/// margin_i = min(q - soft_min, soft_max - q) / (soft_max - soft_min) * 100%
/// 颜色规则：>=30% 绿色, 10~30% 黄色, <10% 红色
void KinematicsWidget::UpdateJointLimitMargins()
{
    if (m_is_populating_form) return;

    for (int i = 0; i < static_cast<int>(m_fk_margin_labels.size()); ++i)
    {
        if (i >= m_joint_limit_table->rowCount() || i >= static_cast<int>(m_fk_joint_spins.size()))
        {
            m_fk_margin_labels[i]->setText(QStringLiteral("—"));
            m_fk_margin_labels[i]->setStyleSheet(QStringLiteral("font-size: 10px; color: #888;"));
            continue;
        }

        const double softMin = ReadTableDouble(m_joint_limit_table, i, 1);
        const double softMax = ReadTableDouble(m_joint_limit_table, i, 2);
        const double range = softMax - softMin;
        if (range <= 0.0)
        {
            m_fk_margin_labels[i]->setText(QStringLiteral("—"));
            m_fk_margin_labels[i]->setStyleSheet(QStringLiteral("font-size: 10px; color: #888;"));
            continue;
        }

        const double value = m_fk_joint_spins[i]->value();
        const double distToMin = value - softMin;
        const double distToMax = softMax - value;
        const double margin = std::min(distToMin, distToMax);
        const double marginPercent = (margin / range) * 100.0;
        const double clamped = std::max(0.0, marginPercent);
        const int displayPercent = static_cast<int>(clamped + 0.5);

        QString color;
        if (marginPercent >= 30.0)
            color = QStringLiteral("#16a34a"); // 绿色
        else if (marginPercent >= 10.0)
            color = QStringLiteral("#ca8a04"); // 黄色
        else
            color = QStringLiteral("#dc2626"); // 红色

        m_fk_margin_labels[i]->setText(QStringLiteral("%1%").arg(displayPercent));
        m_fk_margin_labels[i]->setStyleSheet(
            QStringLiteral("font-size: 10px; font-weight: bold; color: %1;").arg(color));
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
        RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::ApplyImportedUrdfReferenceState(
            m_state.current_model,
            urdfFilePath,
            QStringLiteral("original_imported"));
        m_preview_scene = importResult.preview_scene;
        m_preview_model = importResult.preview_model;
        m_preview_source_mode = QStringLiteral("urdf_preview");
        PopulateForm(m_state.current_model);
        RefreshBackendDiagnostics();
        RenderResults();
        emit PreviewSceneGenerated(m_preview_scene);
        SyncPoseOnly(); // 调用新版矩阵计算通道
        MarkDirty();

        // 🔽🔽🔽 【新增：强警告弹窗逻辑】 🔽🔽🔽
        // 检查提取出的 DH 草案级别
        if (m_state.current_model.dh_draft_extraction_level == QStringLiteral("diagnostic_only"))
        {
            // 提取出错或近似映射时的警告信息，通常保存在 conversion_diagnostics 中
            const QString diagnosticsMsg = m_state.current_model.conversion_diagnostics;
            
            // 如果诊断信息中包含关于 axis 的警告，则说明是非标准 Z 轴引起的
            if (diagnosticsMsg.contains(QStringLiteral("axis")))
            {
                QMessageBox::warning(
                    this,
                    QStringLiteral("非标准旋转轴警告"),
                    QStringLiteral("导入的 URDF 文件中存在非标准 Z 轴旋转的关节。\n\n"
                                   "在 D-H (Denavit-Hartenberg) 运动学模型中，所有的关节旋转必须严格围绕 Z 轴进行。\n\n"
                                   "系统已尝试进行近似映射，但当前的 DH 参数表仅能作为【诊断展示】，强行使用可能导致运动失真。\n\n"
                                   "建议：请在您的 CAD 软件（如 SolidWorks）或 URDF 编辑器中，将所有活动关节的旋转轴 (axis) 对齐到 Z 轴 (0, 0, 1) 后重新导出。")
                );
            }
            else
            {
                // 其他原因导致的 diagnostic_only，也给一个通用提示
                 QMessageBox::information(
                    this,
                    QStringLiteral("DH 参数提取提示"),
                    QStringLiteral("导入的 URDF 结构较复杂（例如多分支或存在不平行的交叉轴）。\n\n"
                                   "系统生成的 DH 参数表仅为【诊断参考】，请仔细核对右侧的连杆参数。")
                );
            }
        }
        // 🔼🔼🔼 【新增结束】 🔼🔼🔼
    }

    SetOperationMessage(importResult.message, importResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(importResult.message));
    emit StatusChanged();
}

/**
 * @brief 槽函数：响应用户点击"从 Topology 生成"按钮的事件。
 * @details 
 * 该函数负责调用底层服务，读取当前项目的拓扑配置文件，将其映射为标准的 DH/MDH 运动学参数，
 * 并驱动界面的数据表格和右侧的 3D VTK 视图进行一次“重量级”的同步刷新（通道 A）。
 */
void KinematicsWidget::OnBuildFromTopologyClicked()
{
    // 1. 获取当前项目的根目录路径，用于底层服务读取对应的 Topology JSON 文件
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
        
    // 2. 调用核心业务层：执行“拓扑到运动学”的核心算法映射
    const auto buildResult = m_service.BuildFromTopology(projectRootPath);
    
    // 3. 检查底层转换是否成功
    if (buildResult.IsSuccess())
    {
        // 3.1 更新当前界面的内存状态机
        m_state = buildResult.state;
        
        // 【重要架构细节】：这里注释掉了 ClearPreviewContext();
        // 原因：ClearPreviewContext 会发射一个“空场景”信号。如果在这里调用，
        // 紧接着下方的 SyncStructureAndPreview() 又会发射一个“新场景”信号。
        // 这在 Qt 和 VTK 的异步渲染队列中极易引发“竞态条件 (Race Condition)”，
        // 导致画面闪烁或者新的渲染指令被丢弃，因此直接复写即可，无需清屏。
        // ClearPreviewContext(); 
        
        // 3.2 数据流向 UI：将后台生成的全新 D-H 模型数据回填到界面的 QTableWidget 和 SpinBox 中
        // 注意：PopulateForm 内部会设置 m_is_populating_form = true，防止触发连环的 cellChanged 信号
        PopulateForm(m_state.current_model);

        // 腕部关节在零位时 J4/J5/J6 轴线重合，三个连杆在视觉上完全重叠为一根。
        // 给 J5 默认 5° 偏转，使 6 个连杆从初始视角就能清晰区分。
        if (m_fk_joint_spins.size() >= 5)
        {
            m_fk_joint_spins[4]->setValue(5.0);
        }

        // 3.3 数据流向 3D 引擎：【核心修改 - 调用通道 A】
        // 由于是从拓扑全新生成的模型，机器人的物理尺寸 ($a, \alpha, d, \theta$) 已经发生了根本改变。
        // 因此必须走”重量级重建”通道：强制底层 Pinocchio 物理引擎销毁旧缓存、重新建树，
        // 并向主窗口发射 PreviewSceneGenerated 信号，让 VTK 重新绘制所有的圆柱体和网格。
        SyncStructureAndPreview(); 
        
        // 3.4 刷新右下角的文本诊断面板和结果摘要
        RefreshBackendDiagnostics();
        RenderResults();
        
        // 3.5 将当前页面标记为“已修改但未保存”（Dirty），激活顶部的“保存草稿”按钮
        MarkDirty();
    }

    // 4. 用户交互反馈：在界面底部的状态栏显示成功或失败的提示文本
    SetOperationMessage(buildResult.message, buildResult.IsSuccess());
    
    // 5. 记录系统日志，抛给主窗口的 Console 面板
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(buildResult.message));
    emit StatusChanged();
}

void KinematicsWidget::OnCopyUrdfDraftToDhClicked()
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
    const bool derivedUrdfReference =
        m_state.current_model.urdf_master_source_type == QStringLiteral("project_derived");

    if (!RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::CanCopyUrdfDraftToDh(
            m_state.current_model))
    {
        const QString message = derivedUrdfReference
            ? QStringLiteral(
                  "当前文件是 DH/MDH 派生的最小 URDF，只用于交换/预览，不能再复制回 DH 模型。\n\n"
                  "请继续使用 Topology + DH/MDH 参数化设计；如需基于外部工程模型建模，请导入原始工程 URDF。")
            : diagnosticOnlyDraft
            ? QStringLiteral(
                  "当前工程 URDF 仅提取到“仅诊断展示”的 DH/MDH 草案，不能复制为可编辑 DH 模型。\n\n"
                  "请继续保持工程 URDF 参考预览；若需要参数化设计，请改用 Topology 生成 DH 骨架，"
                  "或导入一份可稳定提取 DH 链的 URDF。\n\n"
                  "当前草案级别：%1")
                  .arg(extractionLevel)
            : QStringLiteral(
                  "当前工程 URDF 提取出的 DH/MDH 草案不完整，无法安全复制为可编辑 DH 模型。\n\n"
                  "要求：links 与 joint_limits 都非空，且数量一一对应。\n"
                  "当前 links=%1，joint_limits=%2，草案级别=%3")
                  .arg(m_state.current_model.links.size())
                  .arg(m_state.current_model.joint_limits.size())
                  .arg(extractionLevel);

        QMessageBox::warning(
            this,
            QStringLiteral("无法复制 URDF 草案为 DH 模型"),
            message);
        SetOperationMessage(message, false, true);
        EmitTelemetryStatus(QStringLiteral("URDF 草案复制"), message, true);
        emit LogMessageGenerated(QStringLiteral("[Kinematics][Warning] %1").arg(message));
        return;
    }

    const QString warningMessage = QStringLiteral(
        "即将把当前工程 URDF 的 DH/MDH 诊断草案复制为一个可编辑 DH 模型。\n\n"
        "复制后将发生以下变化：\n"
        "1. DH/MDH 参数表将变为可编辑；\n"
        "2. 中央三维主视图区将改由 DH/MDH 骨架驱动；\n"
        "3. 原始工程 URDF 不会被反写；\n"
        "4. 保存时会从 DH/MDH 模型派生一份最小 URDF 交换产物。\n\n"
        "当前草案级别：%1\n"
        "建议先核对 a / alpha / d / theta_offset、Base / Flange / TCP 后再继续。")
        .arg(extractionLevel);

    const int answer = QMessageBox::warning(
        this,
        QStringLiteral("复制 URDF 草案为 DH 模型"),
        warningMessage,
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (answer != QMessageBox::Yes)
    {
        emit LogMessageGenerated(QStringLiteral("[Kinematics] 已取消复制 URDF 草案为 DH 模型。"));
        return;
    }

    m_state.current_model =
        RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::CreateDhCopyFromUrdfDraft(
            CollectModelFromForm());

    ClearPreviewContext();
    PopulateForm(m_state.current_model);
    
    // 【核心修改】：通过通道 A 同步重建三维骨架
    SyncStructureAndPreview();

    MarkDirty();

    const QString message = QStringLiteral("已复制 URDF 诊断草案为可编辑 DH/MDH 参数化模型，原始工程 URDF 不会被反写。");
    SetOperationMessage(message, true);
    EmitTelemetryStatus(QStringLiteral("DH/MDH 参数化设计"), message, false);
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(message));
    emit StatusChanged();
}

void KinematicsWidget::OnPromoteDhDraftToMasterClicked()
{
    OnCopyUrdfDraftToDhClicked();
}

void KinematicsWidget::OnReturnToUrdfReferenceClicked()
{
    if (m_state.current_model.master_model_type != QStringLiteral("dh_mdh"))
    {
        return;
    }

    const QString originalImportedPath = ResolveOriginalImportedUrdfMasterPath();
    const QString derivedUrdfPath = ResolveDerivedUrdfMasterPath();
    const QString urdfFilePath = originalImportedPath;
    const QString selectedSourceType = QStringLiteral("original_imported");

    if (urdfFilePath.trimmed().isEmpty() || !QFileInfo::exists(urdfFilePath))
    {
        const QString message = derivedUrdfPath.trimmed().isEmpty()
            ? QStringLiteral("当前没有可用的原始工程 URDF，无法回到 URDF 参考。请继续使用 Topology + DH/MDH 参数化设计，或重新导入工程 URDF。")
            : QStringLiteral("当前只有 DH 派生最小 URDF。该文件仅用于交换/预览，不允许作为可回切参考，以避免 DH -> URDF -> DH 的循环建模。请继续使用当前 DH/MDH 参数化设计。");
        SetOperationMessage(message, false, true);
        QMessageBox::information(this, QStringLiteral("无法回到 URDF 参考"), message);
        emit LogMessageGenerated(QStringLiteral("[Kinematics][Warning] %1").arg(message));
        return;
    }

    const QString warningMessage = QStringLiteral(
        "即将回到 URDF 参考视图。\n\n"
        "切换后将发生以下变化：\n"
        "1. 中央三维主视图区将改由原始工程 URDF 参考模型预览驱动；\n"
        "2. DH/MDH 参数表将重新成为只读诊断草案；\n"
        "3. 当前 DH/MDH 参数化编辑不会反写原始工程 URDF。\n\n"
        "本次 URDF 来源：%1\n"
        "本次将使用以下 URDF：\n%2")
        .arg(FormatUrdfMasterSourceType(selectedSourceType), urdfFilePath);

    const int answer = QMessageBox::warning(
        this,
        QStringLiteral("回到 URDF 参考"),
        warningMessage,
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (answer != QMessageBox::Yes)
    {
        emit LogMessageGenerated(QStringLiteral("[Kinematics] 已取消回到 URDF 参考。"));
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
    m_state.current_model.original_imported_urdf_path =
        selectedSourceType == QStringLiteral("original_imported")
            ? urdfFilePath
            : previousOriginalImportedPath;
    RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::ApplyImportedUrdfReferenceState(
        m_state.current_model,
        urdfFilePath,
        selectedSourceType);
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

    MarkDirty();

    const QString message = QStringLiteral("已回到原始工程 URDF 参考视图，DH/MDH 参数表为只读诊断草案。");
    SetOperationMessage(message, true);
    EmitTelemetryStatus(QStringLiteral("URDF 工程参考"), message, false);
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(message));
    emit StatusChanged();
}

void KinematicsWidget::OnSwitchToUrdfMasterClicked()
{
    OnReturnToUrdfReferenceClicked();
}

void KinematicsWidget::OnRunFkClicked()
{
    m_state.current_model = CollectModelFromForm();
    RefreshBackendDiagnostics();

    // 【IK DEBUG】记录 FK 请求的模型缓存字段
    {
        QString angleSummary;
        const auto angles = CollectJointInputs(m_fk_joint_spins);
        for (std::size_t i = 0; i < angles.size(); ++i)
            angleSummary += (i > 0 ? QStringLiteral(", ") : QString()) + QString::number(angles[i], 'f', 3);
        emit LogMessageGenerated(QStringLiteral(
            "[Kinematics][Debug] FK Request: model_ref=%1 joint_sig=%2 joint_count=%3 "
            "angles=[%4]")
            .arg(m_state.current_model.unified_robot_model_ref)
            .arg(m_state.current_model.joint_order_signature)
            .arg(m_state.current_model.joint_count)
            .arg(angleSummary));
    }

    RoboSDP::Kinematics::Dto::FkRequestDto request;
    request.joint_positions_deg = CollectJointInputs(m_fk_joint_spins);
    m_state.last_fk_result = m_service.SolveFk(m_state.current_model, request);
    MarkDirty();

    // 【IK DEBUG】记录 FK 结果的 TCP 位姿，便于与 IK 目标对比
    if (m_state.last_fk_result.success)
    {
        UpdateFkPoseReadout(m_state.last_fk_result.tcp_pose);

        emit LogMessageGenerated(QStringLiteral(
            "[Kinematics][Debug] FK Result TCP: pos=[%1,%2,%3] rpy=[%4,%5,%6]")
            .arg(m_state.last_fk_result.tcp_pose.position_m[0], 0, 'f', 6)
            .arg(m_state.last_fk_result.tcp_pose.position_m[1], 0, 'f', 6)
            .arg(m_state.last_fk_result.tcp_pose.position_m[2], 0, 'f', 6)
            .arg(m_state.last_fk_result.tcp_pose.rpy_deg[0], 0, 'f', 6)
            .arg(m_state.last_fk_result.tcp_pose.rpy_deg[1], 0, 'f', 6)
            .arg(m_state.last_fk_result.tcp_pose.rpy_deg[2], 0, 'f', 6));
    }

    // FK 成功后同步计算 Jacobian 分析（可操作度/条件数）
    if (m_state.last_fk_result.success)
    {
        m_last_jacobian_analysis = m_service.ComputeJacobianAnalysis(
            m_state.current_model, request.joint_positions_deg);
    }

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
    emit StatusChanged();
}

void KinematicsWidget::OnRunIkClicked()
{
    m_state.current_model = CollectModelFromForm();
    RefreshBackendDiagnostics();

    // 【IK DEBUG】记录当前模型的关键缓存字段和种子关节值
    {
        QString seedSummary;
        const auto seedVals = CollectJointInputs(m_ik_seed_joint_spins);
        for (std::size_t i = 0; i < seedVals.size(); ++i)
            seedSummary += (i > 0 ? QStringLiteral(", ") : QString()) + QString::number(seedVals[i], 'f', 3);
        emit LogMessageGenerated(QStringLiteral(
            "[Kinematics][Debug] IK Request: model_ref=%1 joint_sig=%2 joint_count=%3 limits=%4 "
            "seed=[%5] target_pos=[%6,%7,%8] rpy=[%9,%10,%11]")
            .arg(m_state.current_model.unified_robot_model_ref)
            .arg(m_state.current_model.joint_order_signature)
            .arg(m_state.current_model.joint_count)
            .arg(m_state.current_model.joint_limits.size())
            .arg(seedSummary)
            .arg(m_ik_target_pose_spins[0]->value(), 0, 'f', 6)
            .arg(m_ik_target_pose_spins[1]->value(), 0, 'f', 6)
            .arg(m_ik_target_pose_spins[2]->value(), 0, 'f', 6)
            .arg(m_ik_target_pose_spins[3]->value(), 0, 'f', 6)
            .arg(m_ik_target_pose_spins[4]->value(), 0, 'f', 6)
            .arg(m_ik_target_pose_spins[5]->value(), 0, 'f', 6));
    }

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

    // IK 求解成功后自动将结果填入 FK 滑块，方便用户微调后重新执行 FK 验证
    if (m_state.last_ik_result.success)
    {
        const auto& ikJoints = m_state.last_ik_result.joint_positions_deg;
        for (std::size_t i = 0; i < m_fk_joint_spins.size() && i < ikJoints.size(); ++i)
        {
            m_fk_joint_spins[i]->setValue(ikJoints[i]);
        }

        RoboSDP::Kinematics::Dto::FkRequestDto fkVerifyRequest;
        fkVerifyRequest.joint_positions_deg = ikJoints;
        m_state.last_fk_result = m_service.SolveFk(m_state.current_model, fkVerifyRequest);
        if (m_state.last_fk_result.success)
        {
            PreviewPoseMap poseMap;
            poseMap[QStringLiteral("base_link")] = m_state.current_model.base_frame;
            for (const auto& linkPose : m_state.last_fk_result.link_poses)
            {
                poseMap[linkPose.link_id] = linkPose.pose;
            }
            poseMap[QStringLiteral("tcp_frame")] = m_state.last_fk_result.tcp_pose;
            emit PreviewPosesUpdated(poseMap);

            const bool withinTolerance =
                m_state.last_ik_result.position_error_mm <= m_state.current_model.ik_solver_config.position_tolerance_mm &&
                m_state.last_ik_result.orientation_error_deg <= m_state.current_model.ik_solver_config.orientation_tolerance_deg;
            emit IkPoseComparisonUpdated(
                request.target_pose,
                m_state.last_fk_result.tcp_pose,
                m_state.last_ik_result.position_error_mm,
                m_state.last_ik_result.orientation_error_deg,
                withinTolerance,
                true);
        }
        else
        {
            emit LogMessageGenerated(
                QStringLiteral("[Kinematics][Warning] IK 解 FK 验证失败：%1")
                    .arg(m_state.last_fk_result.message));
            emit IkPoseComparisonUpdated(
                request.target_pose,
                RoboSDP::Kinematics::Dto::CartesianPoseDto {},
                m_state.last_ik_result.position_error_mm,
                m_state.last_ik_result.orientation_error_deg,
                false,
                false);
        }
    }
    else
    {
        emit IkPoseComparisonUpdated(
            request.target_pose,
            RoboSDP::Kinematics::Dto::CartesianPoseDto {},
            m_state.last_ik_result.position_error_mm,
            m_state.last_ik_result.orientation_error_deg,
            false,
            false);
    }

    RenderResults();

    const bool warning = !m_state.last_ik_result.success;
    SetOperationMessage(m_state.last_ik_result.message, m_state.last_ik_result.success, warning);
    EmitTelemetryStatus(
        m_state.last_ik_result.solver_id.isEmpty()
            ? QStringLiteral("IK 求解器")
            : m_state.last_ik_result.solver_id,
        m_state.last_ik_result.message,
        warning);
    emit LogMessageGenerated(QStringLiteral("%1 %2").arg(
        warning ? QStringLiteral("[Kinematics][Warning]") : QStringLiteral("[Kinematics]"),
        m_state.last_ik_result.message));
    emit StatusChanged();
}

void KinematicsWidget::OnSampleWorkspaceClicked()
{
    m_state.current_model = CollectModelFromForm();
    RefreshBackendDiagnostics();

    RoboSDP::Kinematics::Dto::WorkspaceRequestDto request;
    request.sample_count = m_workspace_sample_count_spin->value();
    m_state.last_workspace_result = m_service.SampleWorkspace(m_state.current_model, request);
    MarkDirty();

    // 工作空间采样成功后，发射点云信号供 VTK 渲染 3D 散点图
    if (m_state.last_workspace_result.success)
    {
        std::vector<std::array<double, 3>> tcpPositions;
        tcpPositions.reserve(m_state.last_workspace_result.sampled_points.size());
        for (const auto& pt : m_state.last_workspace_result.sampled_points)
        {
            tcpPositions.push_back(pt.tcp_pose.position_m);
        }
        emit WorkspacePointCloudGenerated(tcpPositions);
    }

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
    emit StatusChanged();
}

/// @brief 关键工位可达性检测：读取目标位姿，调用服务层多种子 IK 求解。
void KinematicsWidget::OnCheckReachabilityClicked()
{
    m_state.current_model = CollectModelFromForm();

    RoboSDP::Kinematics::Dto::CartesianPoseDto targetPose;
    targetPose.position_m = {
        m_reach_target_spins[0]->value(),
        m_reach_target_spins[1]->value(),
        m_reach_target_spins[2]->value()};
    targetPose.rpy_deg = {
        m_reach_target_spins[3]->value(),
        m_reach_target_spins[4]->value(),
        m_reach_target_spins[5]->value()};

    const int seedCount = m_reach_seed_count_spin != nullptr
        ? m_reach_seed_count_spin->value() : 20;

    const auto result = m_service.CheckReachability(m_state.current_model, targetPose, seedCount);

    if (result.reachable)
    {
        m_reach_result_label->setStyleSheet(QStringLiteral("color: #16a34a; font-weight: bold;"));
        m_reach_result_label->setText(QStringLiteral(
            "[可达] 收敛 %1/%2 种子 | 位置误差 %3 mm | 姿态误差 %4°")
            .arg(result.converged_count)
            .arg(result.total_seeds)
            .arg(result.best_position_error_mm, 0, 'f', 3)
            .arg(result.best_orientation_error_deg, 0, 'f', 3));
    }
    else
    {
        m_reach_result_label->setStyleSheet(QStringLiteral("color: #dc2626; font-weight: bold;"));
        m_reach_result_label->setText(QStringLiteral(
            "[不可达] %1 种子均未收敛 | 最小位置误差 %2 mm | 最小姿态误差 %3°")
            .arg(result.total_seeds)
            .arg(result.best_position_error_mm, 0, 'f', 3)
            .arg(result.best_orientation_error_deg, 0, 'f', 3));
    }

    SetOperationMessage(result.message, result.reachable, !result.reachable);
    emit LogMessageGenerated(QStringLiteral("[Kinematics] 可达性检测: %1").arg(result.message));
}

/// @brief 奇异区分析：带 Jacobian 条件数计算的 MC 工作空间采样。
void KinematicsWidget::OnSingularityAnalysisClicked()
{
    m_state.current_model = CollectModelFromForm();
    RefreshBackendDiagnostics();

    RoboSDP::Kinematics::Dto::SingularityAnalysisRequestDto request;
    request.sample_count = m_workspace_sample_count_spin != nullptr
        ? m_workspace_sample_count_spin->value() : 2000;
    request.condition_threshold = m_singularity_threshold_spin != nullptr
        ? m_singularity_threshold_spin->value() : 1000.0;

    const auto result = m_service.SampleWorkspaceWithSingularity(m_state.current_model, request);

    if (result.success)
    {
        // 统计奇异点数量
        int singularCount = 0;
        double maxCondition = 0.0, sumManipulability = 0.0;
        for (const auto& pt : result.sampled_points)
        {
            if (pt.condition_number > request.condition_threshold) ++singularCount;
            maxCondition = std::max(maxCondition, pt.condition_number);
            sumManipulability += pt.manipulability;
        }
        const double avgManip = result.sampled_points.empty() ? 0.0
            : sumManipulability / result.sampled_points.size();

        m_singularity_result_label->setStyleSheet(QStringLiteral("color: #333;"));
        m_singularity_result_label->setText(QStringLiteral(
            "采样 %1 点 | 奇异 %2 点 (%3%) | 最大条件数 %4 | 平均可操作度 %5")
            .arg(result.reachable_sample_count)
            .arg(singularCount)
            .arg(100.0 * singularCount / std::max(1, result.reachable_sample_count), 0, 'f', 1)
            .arg(maxCondition, 0, 'f', 1)
            .arg(avgManip, 0, 'f', 6));

        // 发射奇异点云（位置 + 奇异标记）
        std::vector<std::array<double, 3>> tcpPositions;
        std::vector<bool> isSingular;
        tcpPositions.reserve(result.sampled_points.size());
        isSingular.reserve(result.sampled_points.size());
        for (const auto& pt : result.sampled_points)
        {
            tcpPositions.push_back(pt.tcp_pose.position_m);
            isSingular.push_back(pt.condition_number > request.condition_threshold);
        }
        emit SingularityPointCloudGenerated(tcpPositions, isSingular);
    }
    else
    {
        m_singularity_result_label->setStyleSheet(QStringLiteral("color: #dc2626;"));
        m_singularity_result_label->setText(QStringLiteral("分析失败：%1").arg(result.message));
    }

    SetOperationMessage(result.message, result.success, !result.success);
    emit LogMessageGenerated(QStringLiteral("[Kinematics] 奇异区分析: %1").arg(result.message));
    emit StatusChanged();
}

/// @brief 姿态可达性分析：固定 TCP 位置，网格采样 RPY 姿态，IK 求解判定可达性。
void KinematicsWidget::OnOrientationReachabilityClicked()
{
    m_state.current_model = CollectModelFromForm();

    std::array<double, 3> pos = {
        m_orient_pos_spins[0]->value(),
        m_orient_pos_spins[1]->value(),
        m_orient_pos_spins[2]->value()};
    const int steps = m_orient_steps_spin != nullptr ? m_orient_steps_spin->value() : 7;
    const double range = m_orient_range_spin != nullptr ? m_orient_range_spin->value() : 180.0;

    const auto result = m_service.CheckOrientationReachability(
        m_state.current_model, pos, steps, range);

    if (result.success)
    {
        const double pct = result.reachability_ratio * 100.0;
        QString color;
        if (pct >= 70.0) color = QStringLiteral("#16a34a");
        else if (pct >= 30.0) color = QStringLiteral("#ca8a04");
        else color = QStringLiteral("#dc2626");

        m_orient_result_label->setStyleSheet(
            QStringLiteral("color: %1; font-weight: bold;").arg(color));
        m_orient_result_label->setText(QStringLiteral(
            "可达率 %1% (%2/%3) | 范围 ±%4° × %5 步")
            .arg(pct, 0, 'f', 1)
            .arg(result.reachable_count)
            .arg(result.total_samples)
            .arg(range, 0, 'f', 0)
            .arg(steps));
    }
    else
    {
        m_orient_result_label->setStyleSheet(QStringLiteral("color: #dc2626;"));
        m_orient_result_label->setText(QStringLiteral("分析失败：%1").arg(result.message));
    }

    SetOperationMessage(result.message, result.success, !result.success);
    emit LogMessageGenerated(QStringLiteral("[Kinematics] 姿态可达性: %1").arg(result.message));
    emit StatusChanged();
}

void KinematicsWidget::OnSaveDraftClicked()
{
    const auto saveResult = SaveCurrentDraft();
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(saveResult.message));
    emit StatusChanged();
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
            // 【核心修改】：通过通道 A 重建三维骨架
            SyncStructureAndPreview();
        }
        else 
        {
            // 【核心修改】：通过通道 B 恢复三维姿态
            SyncPoseOnly();
        }
        RefreshBackendDiagnostics();
        RenderResults();
        MarkClean();
    }

    SetOperationMessage(loadResult.message, loadResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Kinematics] %1").arg(loadResult.message));
    emit StatusChanged();
}

void KinematicsWidget::AdjustJointInputCount(int jointCount)
{
    // 1. 动态补充 FK 关节行：侧边栏优先采用纵向列表，避免 J1~J6 横向挤压。
    while (m_fk_joint_spins.size() < static_cast<std::size_t>(jointCount))
    {
        int index = static_cast<int>(m_fk_joint_spins.size());
        auto* spinBox = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
        spinBox->setMinimumWidth(86);
        spinBox->setMaximumWidth(118);
        
        auto* slider = new QSlider(Qt::Horizontal, m_fk_grid->parentWidget());
        slider->setRange(-360, 360);
        slider->setValue(0);
        slider->setPageStep(10);
        slider->setToolTip(QStringLiteral("拖动调整当前关节角度，中央视图会实时刷新。"));
        slider->setMinimumWidth(120);

        connect(slider, &QSlider::valueChanged, spinBox, [spinBox](int value) {
            if (std::abs(spinBox->value() - static_cast<double>(value)) > 0.01) {
                spinBox->setValue(static_cast<double>(value));
            }
        });
        
        connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, [this, slider](double val) {
                if (slider->value() != static_cast<int>(std::round(val))) {
                    slider->blockSignals(true);
                    slider->setValue(static_cast<int>(std::round(val)));
                    slider->blockSignals(false);
                }
                MarkDirty();
                UpdateJointLimitWarningStyle(); // ʵʱԽ粢
                UpdateJointLimitMargins();      // ʵʱԣǩ
                SyncPoseOnly(); // άʵʱ
            });

        auto* label = new QLabel(QStringLiteral("J%1").arg(index + 1), m_fk_grid->parentWidget());
        label->setMinimumWidth(86);

        const int row = index + 1;
        m_fk_grid->addWidget(label, row, 0);
        m_fk_grid->addWidget(spinBox, row, 1);
        m_fk_grid->addWidget(slider, row, 2);

        auto* marginLabel = new QLabel(QStringLiteral(""), m_fk_grid->parentWidget());
        marginLabel->setAlignment(Qt::AlignCenter);
        marginLabel->setMinimumWidth(46);
        marginLabel->setStyleSheet(QStringLiteral("font-size: 11px; color: #888;"));
        m_fk_grid->addWidget(marginLabel, row, 3);

        m_fk_joint_spins.push_back(spinBox);
        m_fk_joint_sliders.push_back(slider);
        m_fk_joint_labels.push_back(label);
        m_fk_margin_labels.push_back(marginLabel);
    }

    // 2. 动态补充 IK 种子滑块
    while (m_ik_seed_joint_spins.size() < static_cast<std::size_t>(jointCount))
    {
        int index = static_cast<int>(m_ik_seed_joint_spins.size());
        auto* spinBox = CreateDoubleSpinBox(-360.0, 360.0, 3, 1.0);
        spinBox->setMinimumWidth(82);
        
        connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, [this](double) { MarkDirty(); });

        auto* label = new QLabel(QStringLiteral("J%1").arg(index + 1), m_ik_seed_grid->parentWidget());
        const int row = 1 + (index / 2);
        const int colOffset = (index % 2) * 2;
        m_ik_seed_grid->addWidget(label, row, colOffset);
        m_ik_seed_grid->addWidget(spinBox, row, colOffset + 1);

        m_ik_seed_joint_spins.push_back(spinBox);
        m_ik_seed_joint_labels.push_back(label);
    }

    // 3. 控制显示与隐藏（比如导入了2轴机器人，就把 J3~J6 隐藏并归零）
    for (std::size_t i = 0; i < m_fk_joint_spins.size(); ++i)
    {
        const bool visible = (i < static_cast<std::size_t>(jointCount));
        m_fk_joint_spins[i]->setVisible(visible);
        m_fk_joint_labels[i]->setVisible(visible);
        if (i < m_fk_joint_sliders.size())
            m_fk_joint_sliders[i]->setVisible(visible);
        if (i < m_fk_margin_labels.size())
            m_fk_margin_labels[i]->setVisible(visible);
        m_ik_seed_joint_spins[i]->setVisible(visible);
        m_ik_seed_joint_labels[i]->setVisible(visible);

        // 如果被隐藏了，把数值归零并阻断信号，防止旧数据残留污染后端矩阵
        if (!visible)
        {
            m_fk_joint_spins[i]->blockSignals(true);
            m_fk_joint_spins[i]->setValue(0.0);
            m_fk_joint_spins[i]->blockSignals(false);
            if (i < m_fk_joint_sliders.size())
            {
                m_fk_joint_sliders[i]->blockSignals(true);
                m_fk_joint_sliders[i]->setValue(0);
                m_fk_joint_sliders[i]->blockSignals(false);
            }

            m_ik_seed_joint_spins[i]->blockSignals(true);
            m_ik_seed_joint_spins[i]->setValue(0.0);
            m_ik_seed_joint_spins[i]->blockSignals(false);
        }
    }

    // 4. 同步当前灵敏度值到所有 FK 输入框（确保新建的输入框跟随当前设置）
    if (m_scroll_step_spin != nullptr)
    {
        ApplyStepToAllSpinBoxes(m_scroll_step_spin->value());
    }

    // 5. 更新 FK 滑块标签显示限位范围（如果限位表已填充）
    UpdateFkJointLimitLabels();
}

/// @brief 将所有 FK 关节输入框的 singleStep 设置为指定值。
void KinematicsWidget::ApplyStepToAllSpinBoxes(double stepDeg)
{
    for (auto* spinBox : m_fk_joint_spins)
    {
        if (spinBox != nullptr)
        {
            spinBox->setSingleStep(stepDeg);
        }
    }
}

// ── Ribbon 按钮状态查询 ────────────────────────────────────────────────────

bool KinematicsWidget::CanBuildFromTopology() const
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    if (projectRootPath.trimmed().isEmpty())
    {
        return false;
    }

    const QString topologyFilePath = m_topology_storage.BuildAbsoluteFilePath(projectRootPath);
    return QFileInfo::exists(topologyFilePath);
}

bool KinematicsWidget::CanPromoteToDhMaster() const
{
    return RoboSDP::Kinematics::Domain::KinematicModelStatePolicy::CanCopyUrdfDraftToDh(
        m_state.current_model);
}

bool KinematicsWidget::CanSwitchToUrdfMaster() const
{
    // 只允许回到外部导入的工程 URDF；DH 派生最小 URDF 仅用于交换/预览，避免 DH -> URDF -> DH 循环。
    if (m_state.current_model.master_model_type != QStringLiteral("dh_mdh"))
    {
        return false;
    }
    const QString urdfPath = ResolveOriginalImportedUrdfMasterPath();
    return !urdfPath.trimmed().isEmpty() && QFileInfo::exists(urdfPath);
}

bool KinematicsWidget::CanRunFk() const
{
    return !m_state.current_model.links.empty();
}

bool KinematicsWidget::CanRunIk() const
{
    return !m_state.current_model.links.empty();
}

bool KinematicsWidget::CanSampleWorkspace() const
{
    return !m_state.current_model.links.empty();
}

bool KinematicsWidget::CanSaveDraft() const
{
    return !m_state.current_model.links.empty() && m_has_unsaved_changes;
}

} // namespace RoboSDP::Kinematics::Ui
