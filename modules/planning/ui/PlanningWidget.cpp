#include "modules/planning/ui/PlanningWidget.h"

#include "core/infrastructure/ProjectManager.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace RoboSDP::Planning::Ui
{

namespace
{

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

void SetReadOnlyItem(QTableWidgetItem* item)
{
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
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

} // namespace

PlanningWidget::PlanningWidget(QWidget* parent)
    : QWidget(parent)
    , m_planning_storage(m_repository)
    , m_kinematic_storage(m_repository)
    , m_selection_storage(m_repository)
    , m_service(m_planning_storage, m_kinematic_storage, m_selection_storage, &m_logger)
    , m_state(m_service.CreateDefaultState())
{
    BuildUi();
    PopulateForm(m_state.current_scene);
    RenderResults();
    ConnectDirtyTracking();
    MarkClean();
}

QString PlanningWidget::ModuleName() const
{
    return QStringLiteral("Planning");
}

bool PlanningWidget::HasUnsavedChanges() const
{
    return m_has_unsaved_changes;
}

RoboSDP::Infrastructure::ProjectSaveItemResult PlanningWidget::SaveCurrentDraft()
{
    QString validationMessage;
    if (!ValidateJointTableAndHighlight(&validationMessage))
    {
        SetOperationMessage(validationMessage, false);
        return {ModuleName(), false, validationMessage};
    }

    m_state.current_scene = CollectSceneFromForm();
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

void PlanningWidget::ConnectDirtyTracking()
{
    for (QLineEdit* editor : findChildren<QLineEdit*>())
    {
        if (editor->isReadOnly())
        {
            continue;
        }
        connect(editor, &QLineEdit::textEdited, this, [this]() { MarkDirty(); });
    }
    for (QDoubleSpinBox* editor : findChildren<QDoubleSpinBox*>())
    {
        connect(editor, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [this](double) {
            MarkDirty();
        });
    }
    for (QTableWidget* table : findChildren<QTableWidget*>())
    {
        connect(table, &QTableWidget::cellChanged, this, [this](int, int) { MarkDirty(); });
    }
}

void PlanningWidget::MarkDirty()
{
    m_has_unsaved_changes = true;
}

void PlanningWidget::MarkClean()
{
    m_has_unsaved_changes = false;
}

void PlanningWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    m_operation_label = new QLabel(QStringLiteral("就绪：请先保存上游草稿，再构建 PlanningScene 并执行验证。"), this);
    m_operation_label->setWordWrap(true);

    auto* tabs = new QTabWidget(this);
    tabs->setDocumentMode(true);

    auto* sceneGroup = new QGroupBox(QStringLiteral("场景与服务配置"), this);
    auto* sceneFormLayout = new QFormLayout(sceneGroup);
    m_kinematic_ref_edit = new QLineEdit(sceneGroup);
    m_kinematic_ref_edit->setReadOnly(true);
    m_dynamic_ref_edit = new QLineEdit(sceneGroup);
    m_dynamic_ref_edit->setReadOnly(true);
    m_selection_ref_edit = new QLineEdit(sceneGroup);
    m_selection_ref_edit->setReadOnly(true);
    m_service_endpoint_edit = new QLineEdit(sceneGroup);
    m_planner_id_edit = new QLineEdit(sceneGroup);
    m_allowed_time_spin = new QDoubleSpinBox(sceneGroup);
    m_allowed_time_spin->setRange(0.05, 30.0);
    m_allowed_time_spin->setDecimals(2);
    m_allowed_time_spin->setSingleStep(0.1);
    m_target_cycle_time_spin = new QDoubleSpinBox(sceneGroup);
    m_target_cycle_time_spin->setRange(0.1, 120.0);
    m_target_cycle_time_spin->setDecimals(2);
    m_target_cycle_time_spin->setSingleStep(0.1);
    sceneFormLayout->addRow(QStringLiteral("Kinematics 引用"), m_kinematic_ref_edit);
    sceneFormLayout->addRow(QStringLiteral("Dynamics 引用"), m_dynamic_ref_edit);
    sceneFormLayout->addRow(QStringLiteral("Selection 引用"), m_selection_ref_edit);
    sceneFormLayout->addRow(QStringLiteral("gRPC 服务地址"), m_service_endpoint_edit);
    sceneFormLayout->addRow(QStringLiteral("规划器"), m_planner_id_edit);
    sceneFormLayout->addRow(QStringLiteral("允许规划时间 [s]"), m_allowed_time_spin);
    sceneFormLayout->addRow(QStringLiteral("目标节拍 [s]"), m_target_cycle_time_spin);

    auto* jointGroup = new QGroupBox(QStringLiteral("起点 / 终点关节表"), this);
    auto* jointLayout = new QVBoxLayout(jointGroup);
    m_joint_table = new QTableWidget(jointGroup);
    SetupJointTableColumns();
    jointLayout->addWidget(m_joint_table);

    auto* collisionGroup = new QGroupBox(QStringLiteral("碰撞结果"), this);
    auto* collisionLayout = new QVBoxLayout(collisionGroup);
    m_collision_table = new QTableWidget(collisionGroup);
    SetupCollisionTableColumns();
    collisionLayout->addWidget(m_collision_table);

    auto* selfCollisionGroup = new QGroupBox(QStringLiteral("自碰撞结果"), this);
    auto* selfCollisionLayout = new QVBoxLayout(selfCollisionGroup);
    m_self_collision_table = new QTableWidget(selfCollisionGroup);
    SetupSelfCollisionTableColumns();
    selfCollisionLayout->addWidget(m_self_collision_table);

    auto* resultGroup = new QGroupBox(QStringLiteral("规划验证摘要"), this);
    auto* resultLayout = new QVBoxLayout(resultGroup);
    m_result_summary_edit = new QPlainTextEdit(resultGroup);
    m_result_summary_edit->setReadOnly(true);
    resultLayout->addWidget(m_result_summary_edit);

    // 中文说明：Planning 页面按验证流程拆为页签，场景收集、表格校验和结果渲染逻辑保持原有入口。
    tabs->addTab(CreateScrollableTab(sceneGroup), QStringLiteral("场景配置"));
    tabs->addTab(CreateScrollableTab(jointGroup), QStringLiteral("起止状态"));
    tabs->addTab(CreateScrollableTab(collisionGroup), QStringLiteral("碰撞检测"));
    tabs->addTab(CreateScrollableTab(selfCollisionGroup), QStringLiteral("自碰撞检测"));
    tabs->addTab(CreateScrollableTab(resultGroup), QStringLiteral("验证摘要"));

    rootLayout->addWidget(m_operation_label);
    rootLayout->addWidget(tabs, 1);

    connect(m_joint_table, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*) { ValidateJointTableAndHighlight(); });
}

QWidget* PlanningWidget::CreateScrollableTab(QWidget* contentWidget)
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

void PlanningWidget::SetupJointTableColumns()
{
    m_joint_table->setColumnCount(5);
    m_joint_table->setHorizontalHeaderLabels({
        QStringLiteral("joint_id"),
        QStringLiteral("start_deg"),
        QStringLiteral("goal_deg"),
        QStringLiteral("lower_deg"),
        QStringLiteral("upper_deg")});
    m_joint_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void PlanningWidget::SetupCollisionTableColumns()
{
    m_collision_table->setColumnCount(4);
    m_collision_table->setHorizontalHeaderLabels({
        QStringLiteral("request_id"),
        QStringLiteral("object_id"),
        QStringLiteral("in_collision"),
        QStringLiteral("message")});
    m_collision_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void PlanningWidget::SetupSelfCollisionTableColumns()
{
    m_self_collision_table->setColumnCount(5);
    m_self_collision_table->setHorizontalHeaderLabels({
        QStringLiteral("request_id"),
        QStringLiteral("link_a"),
        QStringLiteral("link_b"),
        QStringLiteral("in_self_collision"),
        QStringLiteral("message")});
    m_self_collision_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void PlanningWidget::PopulateForm(const RoboSDP::Planning::Dto::PlanningSceneDto& scene)
{
    m_kinematic_ref_edit->setText(scene.meta.kinematic_ref);
    m_dynamic_ref_edit->setText(scene.meta.dynamic_ref);
    m_selection_ref_edit->setText(scene.meta.selection_ref);
    m_service_endpoint_edit->setText(scene.planning_config.service_endpoint);
    m_planner_id_edit->setText(scene.planning_config.planner_id);
    m_allowed_time_spin->setValue(scene.planning_config.allowed_planning_time_s);
    m_target_cycle_time_spin->setValue(scene.planning_config.target_cycle_time_s);

    m_joint_table->setRowCount(static_cast<int>(scene.joint_limits.size()));
    for (int row = 0; row < static_cast<int>(scene.joint_limits.size()); ++row)
    {
        const auto& limit = scene.joint_limits.at(static_cast<std::size_t>(row));
        auto* jointIdItem = EnsureItem(m_joint_table, row, 0);
        jointIdItem->setText(limit.joint_id);
        SetReadOnlyItem(jointIdItem);

        EnsureItem(m_joint_table, row, 1)->setText(
            QString::number(scene.start_state.joint_values_deg.at(static_cast<std::size_t>(row)), 'f', 3));
        EnsureItem(m_joint_table, row, 2)->setText(
            QString::number(scene.goal_state.joint_values_deg.at(static_cast<std::size_t>(row)), 'f', 3));

        auto* lowerItem = EnsureItem(m_joint_table, row, 3);
        lowerItem->setText(QString::number(limit.lower_deg, 'f', 3));
        SetReadOnlyItem(lowerItem);

        auto* upperItem = EnsureItem(m_joint_table, row, 4);
        upperItem->setText(QString::number(limit.upper_deg, 'f', 3));
        SetReadOnlyItem(upperItem);
    }

    ValidateJointTableAndHighlight();
}

RoboSDP::Planning::Dto::PlanningSceneDto PlanningWidget::CollectSceneFromForm() const
{
    RoboSDP::Planning::Dto::PlanningSceneDto scene = m_state.current_scene;
    scene.planning_config.service_endpoint = m_service_endpoint_edit->text().trimmed();
    scene.planning_config.planner_id = m_planner_id_edit->text().trimmed();
    scene.planning_config.allowed_planning_time_s = m_allowed_time_spin->value();
    scene.planning_config.target_cycle_time_s = m_target_cycle_time_spin->value();

    scene.start_state.joint_values_deg.clear();
    scene.goal_state.joint_values_deg.clear();
    for (int row = 0; row < m_joint_table->rowCount(); ++row)
    {
        scene.start_state.joint_values_deg.push_back(ReadTableDouble(m_joint_table, row, 1));
        scene.goal_state.joint_values_deg.push_back(ReadTableDouble(m_joint_table, row, 2));
    }

    return scene;
}

void PlanningWidget::RenderResults()
{
    const auto* latestResult = m_state.results.empty() ? nullptr : &m_state.results.back();

    m_collision_table->setRowCount(latestResult != nullptr ? static_cast<int>(latestResult->collision_results.size()) : 0);
      if (latestResult != nullptr)
    {
        for (int row = 0; row < static_cast<int>(latestResult->collision_results.size()); ++row)
        {
            const auto& result = latestResult->collision_results.at(static_cast<std::size_t>(row));
            EnsureItem(m_collision_table, row, 0)->setText(result.request_id);
            EnsureItem(m_collision_table, row, 1)->setText(result.object_id);
            EnsureItem(m_collision_table, row, 2)->setText(result.in_collision ? QStringLiteral("是") : QStringLiteral("否"));
            EnsureItem(m_collision_table, row, 3)->setText(result.message);
            for (int column = 0; column < m_collision_table->columnCount(); ++column)
            {
                SetReadOnlyItem(EnsureItem(m_collision_table, row, column));
            }
        }
    }

    m_self_collision_table->setRowCount(latestResult != nullptr ? static_cast<int>(latestResult->self_collision_results.size()) : 0);
    if (latestResult != nullptr)
    {
        for (int row = 0; row < static_cast<int>(latestResult->self_collision_results.size()); ++row)
        {
            const auto& result = latestResult->self_collision_results.at(static_cast<std::size_t>(row));
            EnsureItem(m_self_collision_table, row, 0)->setText(result.request_id);
            EnsureItem(m_self_collision_table, row, 1)->setText(result.link_a);
            EnsureItem(m_self_collision_table, row, 2)->setText(result.link_b);
            EnsureItem(m_self_collision_table, row, 3)->setText(result.in_self_collision ? QStringLiteral("是") : QStringLiteral("否"));
            EnsureItem(m_self_collision_table, row, 4)->setText(result.message);
            for (int column = 0; column < m_self_collision_table->columnCount(); ++column)
            {
                SetReadOnlyItem(EnsureItem(m_self_collision_table, row, column));
            }
        }
    }

    QStringList lines;
    lines.push_back(QStringLiteral("Unified Ref = %1").arg(
        m_state.current_scene.meta.unified_robot_snapshot.unified_robot_model_ref.isEmpty()
            ? QStringLiteral("not_generated")
            : m_state.current_scene.meta.unified_robot_snapshot.unified_robot_model_ref));
    lines.push_back(QStringLiteral("Kinematic Kernel Ready = %1").arg(
        m_state.current_scene.meta.unified_robot_snapshot.pinocchio_model_ready ? QStringLiteral("true") : QStringLiteral("false")));
    lines.push_back(QStringLiteral("Snapshot Mode = %1 / %2").arg(
        m_state.current_scene.meta.unified_robot_snapshot.master_model_type,
        m_state.current_scene.meta.unified_robot_snapshot.modeling_mode));
    lines.push_back(QStringLiteral("Snapshot Source = %1 / %2").arg(
        m_state.current_scene.meta.unified_robot_snapshot.source_kinematic_id.trimmed().isEmpty()
            ? QStringLiteral("unknown_model")
            : m_state.current_scene.meta.unified_robot_snapshot.source_kinematic_id,
        m_state.current_scene.meta.unified_robot_snapshot.model_source_mode.trimmed().isEmpty()
            ? QStringLiteral("unknown_source")
            : m_state.current_scene.meta.unified_robot_snapshot.model_source_mode));
    lines.push_back(QStringLiteral("Derived Artifact State = %1").arg(
        m_state.current_scene.meta.unified_robot_snapshot.derived_artifact_state_code.trimmed().isEmpty()
            ? QStringLiteral("unknown_artifact")
            : m_state.current_scene.meta.unified_robot_snapshot.derived_artifact_state_code));
    lines.push_back(QStringLiteral("Derived Artifact Path = %1").arg(
        m_state.current_scene.meta.unified_robot_snapshot.derived_artifact_relative_path.trimmed().isEmpty()
            ? QStringLiteral("not_materialized")
            : m_state.current_scene.meta.unified_robot_snapshot.derived_artifact_relative_path));
    lines.push_back(QStringLiteral("Derived Artifact Version = %1").arg(
        m_state.current_scene.meta.unified_robot_snapshot.derived_artifact_version.trimmed().isEmpty()
            ? QStringLiteral("unknown_version")
            : m_state.current_scene.meta.unified_robot_snapshot.derived_artifact_version));
    lines.push_back(QStringLiteral("Derived Artifact Generated At = %1").arg(
        m_state.current_scene.meta.unified_robot_snapshot.derived_artifact_generated_at_utc.trimmed().isEmpty()
            ? QStringLiteral("not_recorded")
            : m_state.current_scene.meta.unified_robot_snapshot.derived_artifact_generated_at_utc));
    lines.push_back(QStringLiteral("Derived Artifact Exists = %1").arg(
        m_state.current_scene.meta.unified_robot_snapshot.derived_artifact_exists ? QStringLiteral("true") : QStringLiteral("false")));
    lines.push_back(QStringLiteral("Derived Artifact Fresh = %1").arg(
        m_state.current_scene.meta.unified_robot_snapshot.derived_artifact_fresh ? QStringLiteral("true") : QStringLiteral("false")));
    if (!m_state.current_scene.meta.unified_robot_snapshot.conversion_diagnostics.trimmed().isEmpty())
    {
        lines.push_back(QStringLiteral("Kinematic Chain Summary = %1")
                            .arg(m_state.current_scene.meta.unified_robot_snapshot.conversion_diagnostics));
    }
    lines.push_back(QStringLiteral("规划场景：%1").arg(m_state.current_scene.meta.name));
    lines.push_back(QStringLiteral("请求数量：%1").arg(m_state.requests.size()));
    lines.push_back(QStringLiteral("验证结果数量：%1").arg(m_state.results.size()));

    if (latestResult != nullptr)
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[最新验证结果]"));
        lines.push_back(QStringLiteral("状态：%1").arg(latestResult->success ? QStringLiteral("成功") : QStringLiteral("失败")));
        lines.push_back(QStringLiteral("消息：%1").arg(latestResult->message));
        lines.push_back(QStringLiteral("节拍：%1")
                            .arg(latestResult->cycle_time_result.message));
        lines.push_back(QStringLiteral("节拍余量：%1 s").arg(latestResult->cycle_time_result.margin_s, 0, 'f', 3));
        lines.push_back(QStringLiteral("可行性摘要：%1").arg(latestResult->feasibility_summary));

        if (!latestResult->trajectory_results.empty())
        {
            const auto& trajectory = latestResult->trajectory_results.front();
            lines.push_back(QStringLiteral("规划器：%1").arg(trajectory.planner_id));
            lines.push_back(QStringLiteral("规划时长：%1 s").arg(trajectory.planning_time_s, 0, 'f', 3));
            lines.push_back(QStringLiteral("轨迹时长：%1 s").arg(trajectory.trajectory_duration_s, 0, 'f', 3));
            lines.push_back(QStringLiteral("路点数量：%1").arg(trajectory.waypoint_count));
        }
    }
    else
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("尚未执行点到点规划验证。"));
    }

    m_result_summary_edit->setPlainText(lines.join(QLatin1Char('\n')));
}

void PlanningWidget::SetOperationMessage(const QString& message, bool success)
{
    m_operation_label->setText(message);
    m_operation_label->setStyleSheet(success ? QStringLiteral("color: #1b7f3b;")
                                             : QStringLiteral("color: #b42318;"));
}

bool PlanningWidget::ValidateJointTableAndHighlight(QString* message)
{
    QStringList errors;
    const QSignalBlocker blocker(m_joint_table);

    for (int row = 0; row < m_joint_table->rowCount(); ++row)
    {
        const double startValue = ReadTableDouble(m_joint_table, row, 1);
        const double goalValue = ReadTableDouble(m_joint_table, row, 2);
        const double lower = ReadTableDouble(m_joint_table, row, 3);
        const double upper = ReadTableDouble(m_joint_table, row, 4);

        const bool startValid = startValue + 1e-6 >= lower && startValue - 1e-6 <= upper;
        const bool goalValid = goalValue + 1e-6 >= lower && goalValue - 1e-6 <= upper;

        HighlightItem(EnsureItem(m_joint_table, row, 1), startValid, QStringLiteral("起点角度必须位于限位区间内。"));
        HighlightItem(EnsureItem(m_joint_table, row, 2), goalValid, QStringLiteral("终点角度必须位于限位区间内。"));

        if (!startValid)
        {
            errors.push_back(QStringLiteral("%1 的起点角度超出限位区间。").arg(ReadTableString(m_joint_table, row, 0)));
        }
        if (!goalValid)
        {
            errors.push_back(QStringLiteral("%1 的终点角度超出限位区间。").arg(ReadTableString(m_joint_table, row, 0)));
        }
    }

    if (message != nullptr)
    {
        *message = errors.isEmpty() ? QString() : errors.front();
    }
    return errors.isEmpty();
}

void PlanningWidget::HighlightItem(QTableWidgetItem* item, bool isValid, const QString& tooltip)
{
    if (item == nullptr)
    {
        return;
    }

    item->setBackground(isValid ? QColor(Qt::white) : QColor(QStringLiteral("#FEE4E2")));
    item->setToolTip(isValid ? QString() : tooltip);
}

void PlanningWidget::OnBuildSceneClicked()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto buildResult = m_service.BuildPlanningScene(projectRootPath);
    if (buildResult.IsSuccess())
    {
        m_state = buildResult.state;
        PopulateForm(m_state.current_scene);
        RenderResults();
        MarkDirty();
    }

    SetOperationMessage(buildResult.message, buildResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Planning] %1").arg(buildResult.message));
}

void PlanningWidget::OnRunVerificationClicked()
{
    QString validationMessage;
    if (!ValidateJointTableAndHighlight(&validationMessage))
    {
        SetOperationMessage(validationMessage, false);
        emit LogMessageGenerated(QStringLiteral("[Planning] %1").arg(validationMessage));
        return;
    }

    m_state.current_scene = CollectSceneFromForm();
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto runResult = m_service.RunPointToPointVerification(
        projectRootPath,
        m_state);
    if (runResult.IsSuccess())
    {
        m_state = runResult.state;
        PopulateForm(m_state.current_scene);
        RenderResults();
        MarkDirty();
    }

    SetOperationMessage(runResult.message, runResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Planning] %1").arg(runResult.message));
}

void PlanningWidget::OnSaveDraftClicked()
{
    const auto saveResult = SaveCurrentDraft();
    emit LogMessageGenerated(QStringLiteral("[Planning] %1").arg(saveResult.message));
}

void PlanningWidget::OnLoadClicked()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto loadResult = m_service.LoadDraft(projectRootPath);
    if (loadResult.IsSuccess())
    {
        m_state = loadResult.state;
        PopulateForm(m_state.current_scene);
        RenderResults();
        MarkClean();
    }

    SetOperationMessage(loadResult.message, loadResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Planning] %1").arg(loadResult.message));
}

} // namespace RoboSDP::Planning::Ui
