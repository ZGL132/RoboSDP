#include "modules/dynamics/ui/DynamicsWidget.h"

#include "apps/desktop-qt/third_party/qcustomplot/qcustomplot.h"

#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QScrollArea>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVector>
#include <QVBoxLayout>

namespace RoboSDP::Dynamics::Ui
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

double ReadTableDouble(QTableWidget* table, int row, int column, double defaultValue = 0.0)
{
    QTableWidgetItem* item = table->item(row, column);
    return item != nullptr ? item->text().toDouble() : defaultValue;
}

int ReadTableInt(QTableWidget* table, int row, int column, int defaultValue = 0)
{
    QTableWidgetItem* item = table->item(row, column);
    return item != nullptr ? item->text().toInt() : defaultValue;
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

QString BuildBackendListText(
    const std::vector<RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto>& results)
{
    QStringList backends;
    for (const auto& result : results)
    {
        if (!result.solver_backend.isEmpty() && !backends.contains(result.solver_backend))
        {
            backends.push_back(result.solver_backend);
        }
    }

    return backends.isEmpty() ? QStringLiteral("未执行") : backends.join(QStringLiteral(", "));
}

bool HasFailedResults(const std::vector<RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto>& results)
{
    for (const auto& result : results)
    {
        if (!result.success)
        {
            return true;
        }
    }
    return false;
}

QString BuildPrimaryResultMessage(
    const std::vector<RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto>& results)
{
    return results.empty() ? QStringLiteral("尚未执行逆动力学主链。") : results.front().message;
}

QString FormatEnvelopeLine(const RoboSDP::Dynamics::Dto::LoadEnvelopeJointDto& joint)
{
    return QStringLiteral("%1: 峰值扭矩 %2 Nm, RMS %3 Nm, 峰值功率 %4 W, 来源 %5")
        .arg(joint.joint_id)
        .arg(joint.peak_torque_nm, 0, 'f', 3)
        .arg(joint.rms_torque_nm, 0, 'f', 3)
        .arg(joint.peak_power_w, 0, 'f', 3)
        .arg(joint.source_trajectory_id);
}

} // namespace

DynamicsWidget::DynamicsWidget(QWidget* parent)
    : QWidget(parent)
    , m_dynamic_storage(m_repository)
    , m_kinematic_storage(m_repository)
    , m_service(m_dynamic_storage, m_kinematic_storage, &m_logger)
    , m_state(m_service.CreateDefaultState())
{
    BuildUi();
    PopulateForm(m_state.current_model);
    RenderBackendStatus();
    RenderTorquePlot();
    RenderResults();
}

void DynamicsWidget::TriggerRunAnalysis()
{
    // 中文说明：外部顶部功能区只能触发页面既有入口，避免绕过表格校验、状态遥测和图表刷新流程。
    OnRunAnalysisClicked();
}

void DynamicsWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    auto* projectPathLayout = new QHBoxLayout();
    auto* projectPathLabel = new QLabel(QStringLiteral("项目目录"), this);
    m_project_root_edit = new QLineEdit(this);
    m_project_root_edit->setObjectName(QStringLiteral("dynamics_project_root_edit"));
    m_project_root_edit->setText(QDir::current().filePath(QStringLiteral("requirement-draft-project")));
    m_browse_button = new QPushButton(QStringLiteral("选择目录"), this);
    projectPathLayout->addWidget(projectPathLabel);
    projectPathLayout->addWidget(m_project_root_edit, 1);
    projectPathLayout->addWidget(m_browse_button);

    auto* actionLayout = new QHBoxLayout();
    m_build_from_kinematics_button = new QPushButton(QStringLiteral("从 Kinematics 生成"), this);
    m_run_analysis_button = new QPushButton(QStringLiteral("执行逆动力学"), this);
    m_save_button = new QPushButton(QStringLiteral("保存草稿"), this);
    m_load_button = new QPushButton(QStringLiteral("重新加载"), this);
    m_build_from_kinematics_button->setObjectName(QStringLiteral("dynamics_build_from_kinematics_button"));
    m_run_analysis_button->setObjectName(QStringLiteral("dynamics_run_analysis_button"));
    actionLayout->addWidget(m_build_from_kinematics_button);
    actionLayout->addWidget(m_run_analysis_button);
    actionLayout->addWidget(m_save_button);
    actionLayout->addWidget(m_load_button);
    actionLayout->addStretch();

    m_operation_label = new QLabel(QStringLiteral("就绪：请先保存 Kinematics，再生成 Dynamics 草稿。"), this);
    m_operation_label->setObjectName(QStringLiteral("dynamics_operation_label"));
    m_operation_label->setWordWrap(true);

    auto* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);

    auto* scrollContent = new QWidget(scrollArea);
    auto* contentLayout = new QVBoxLayout(scrollContent);
    contentLayout->setContentsMargins(4, 4, 4, 4);
    contentLayout->setSpacing(8);
    contentLayout->addWidget(CreateModelGroup());
    contentLayout->addWidget(CreateLinkTableGroup());
    contentLayout->addWidget(CreateJointDriveTableGroup());
    contentLayout->addWidget(CreateTrajectoryGroup());
    contentLayout->addWidget(CreateBackendStatusGroup());
    contentLayout->addWidget(CreatePlotGroup());
    contentLayout->addWidget(CreateResultGroup());
    contentLayout->addStretch();
    scrollArea->setWidget(scrollContent);

    rootLayout->addLayout(projectPathLayout);
    rootLayout->addLayout(actionLayout);
    rootLayout->addWidget(m_operation_label);
    rootLayout->addWidget(scrollArea, 1);

    connect(m_browse_button, &QPushButton::clicked, this, [this]() { OnBrowseProjectRootClicked(); });
    connect(m_build_from_kinematics_button, &QPushButton::clicked, this, [this]() { OnBuildFromKinematicsClicked(); });
    connect(m_run_analysis_button, &QPushButton::clicked, this, [this]() { OnRunAnalysisClicked(); });
    connect(m_save_button, &QPushButton::clicked, this, [this]() { OnSaveDraftClicked(); });
    connect(m_load_button, &QPushButton::clicked, this, [this]() { OnLoadClicked(); });
    connect(m_link_table, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*) { ValidateTablesAndHighlight(); });
    connect(m_joint_drive_table, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*) { ValidateTablesAndHighlight(); });
    connect(m_trajectory_table, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*) { ValidateTablesAndHighlight(); });
}

QGroupBox* DynamicsWidget::CreateModelGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("模型摘要"), this);
    auto* layout = new QFormLayout(groupBox);

    m_model_name_edit = new QLineEdit(groupBox);
    m_kinematic_ref_edit = new QLineEdit(groupBox);
    m_kinematic_ref_edit->setReadOnly(true);
    m_topology_ref_edit = new QLineEdit(groupBox);
    m_topology_ref_edit->setReadOnly(true);
    m_requirement_ref_edit = new QLineEdit(groupBox);
    m_requirement_ref_edit->setReadOnly(true);
    m_gravity_z_spin = CreateDoubleSpinBox(-50.0, 0.0, 3, 0.01);
    m_end_effector_mass_spin = CreateDoubleSpinBox(0.0, 500.0, 3, 0.1);

    layout->addRow(QStringLiteral("模型名称"), m_model_name_edit);
    layout->addRow(QStringLiteral("Kinematics 引用"), m_kinematic_ref_edit);
    layout->addRow(QStringLiteral("Topology 引用"), m_topology_ref_edit);
    layout->addRow(QStringLiteral("Requirement 引用"), m_requirement_ref_edit);
    layout->addRow(QStringLiteral("重力 Z [m/s^2]"), m_gravity_z_spin);
    layout->addRow(QStringLiteral("末端负载质量 [kg]"), m_end_effector_mass_spin);
    return groupBox;
}

QGroupBox* DynamicsWidget::CreateLinkTableGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("连杆质量 / 惯量表"), this);
    auto* layout = new QVBoxLayout(groupBox);
    m_link_table = new QTableWidget(groupBox);
    SetupLinkTableColumns();
    layout->addWidget(m_link_table);
    return groupBox;
}

QGroupBox* DynamicsWidget::CreateJointDriveTableGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("关节传动参数表"), this);
    auto* layout = new QVBoxLayout(groupBox);
    m_joint_drive_table = new QTableWidget(groupBox);
    SetupJointDriveTableColumns();
    layout->addWidget(m_joint_drive_table);
    return groupBox;
}

QGroupBox* DynamicsWidget::CreateTrajectoryGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("基准轨迹最小集"), this);
    auto* layout = new QVBoxLayout(groupBox);
    m_trajectory_table = new QTableWidget(groupBox);
    SetupTrajectoryTableColumns();
    layout->addWidget(m_trajectory_table);
    return groupBox;
}

QGroupBox* DynamicsWidget::CreateBackendStatusGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("后端状态"), this);
    auto* layout = new QFormLayout(groupBox);

    m_solver_backend_edit = new QLineEdit(groupBox);
    m_solver_backend_edit->setReadOnly(true);
    m_used_fallback_edit = new QLineEdit(groupBox);
    m_used_fallback_edit->setReadOnly(true);
    m_backend_message_label = new QLabel(groupBox);
    m_backend_message_label->setWordWrap(true);

    layout->addRow(QStringLiteral("solver_backend"), m_solver_backend_edit);
    layout->addRow(QStringLiteral("used_fallback"), m_used_fallback_edit);
    layout->addRow(QStringLiteral("backend_message"), m_backend_message_label);
    return groupBox;
}

QGroupBox* DynamicsWidget::CreatePlotGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("最小结果曲线"), this);
    auto* layout = new QVBoxLayout(groupBox);

    m_torque_plot_status_label = new QLabel(QStringLiteral("当前显示：首条轨迹的首个关节扭矩-时间曲线。"), groupBox);
    m_torque_plot_status_label->setWordWrap(true);

    m_torque_plot = new QCustomPlot(groupBox);
    m_torque_plot->xAxis->setLabel(QStringLiteral("时间 [s]"));
    m_torque_plot->yAxis->setLabel(QStringLiteral("扭矩 [Nm]"));
    m_torque_plot->legend->setVisible(true);

    layout->addWidget(m_torque_plot_status_label);
    layout->addWidget(m_torque_plot);
    return groupBox;
}

QGroupBox* DynamicsWidget::CreateResultGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("逆动力学结果摘要"), this);
    auto* layout = new QVBoxLayout(groupBox);
    m_result_summary_edit = new QPlainTextEdit(groupBox);
    m_result_summary_edit->setReadOnly(true);
    layout->addWidget(m_result_summary_edit);
    return groupBox;
}

QDoubleSpinBox* DynamicsWidget::CreateDoubleSpinBox(double minimum, double maximum, int decimals, double step)
{
    auto* spinBox = new QDoubleSpinBox(this);
    spinBox->setRange(minimum, maximum);
    spinBox->setDecimals(decimals);
    spinBox->setSingleStep(step);
    spinBox->setAccelerated(true);
    return spinBox;
}

void DynamicsWidget::SetupLinkTableColumns()
{
    m_link_table->setColumnCount(11);
    m_link_table->setHorizontalHeaderLabels({
        QStringLiteral("link_id"),
        QStringLiteral("mass [kg]"),
        QStringLiteral("cog_x [m]"),
        QStringLiteral("cog_y [m]"),
        QStringLiteral("cog_z [m]"),
        QStringLiteral("Ixx"),
        QStringLiteral("Iyy"),
        QStringLiteral("Izz"),
        QStringLiteral("Ixy"),
        QStringLiteral("Ixz"),
        QStringLiteral("Iyz")});
    m_link_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void DynamicsWidget::SetupJointDriveTableColumns()
{
    m_joint_drive_table->setColumnCount(6);
    m_joint_drive_table->setHorizontalHeaderLabels({
        QStringLiteral("joint_id"),
        QStringLiteral("ratio"),
        QStringLiteral("efficiency"),
        QStringLiteral("viscous"),
        QStringLiteral("coulomb"),
        QStringLiteral("static")});
    m_joint_drive_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void DynamicsWidget::SetupTrajectoryTableColumns()
{
    m_trajectory_table->setColumnCount(6);
    m_trajectory_table->setHorizontalHeaderLabels({
        QStringLiteral("trajectory_id"),
        QStringLiteral("name"),
        QStringLiteral("profile_type"),
        QStringLiteral("active_joint"),
        QStringLiteral("duration [s]"),
        QStringLiteral("sample_count")});
    m_trajectory_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

RoboSDP::Dynamics::Dto::DynamicModelDto DynamicsWidget::CollectModelFromForm() const
{
    RoboSDP::Dynamics::Dto::DynamicModelDto model = m_state.current_model;
    model.meta.name = m_model_name_edit->text().trimmed();
    model.meta.kinematic_ref = m_kinematic_ref_edit->text().trimmed();
    model.meta.topology_ref = m_topology_ref_edit->text().trimmed();
    model.meta.requirement_ref = m_requirement_ref_edit->text().trimmed();
    model.gravity[2] = m_gravity_z_spin->value();
    model.end_effector.mass = m_end_effector_mass_spin->value();

    model.links.clear();
    for (int row = 0; row < m_link_table->rowCount(); ++row)
    {
        RoboSDP::Dynamics::Dto::DynamicLinkDto link;
        link.link_id = ReadTableString(m_link_table, row, 0);
        link.mass = ReadTableDouble(m_link_table, row, 1);
        link.cog = {
            ReadTableDouble(m_link_table, row, 2),
            ReadTableDouble(m_link_table, row, 3),
            ReadTableDouble(m_link_table, row, 4)};
        link.inertia_tensor = {
            ReadTableDouble(m_link_table, row, 5),
            ReadTableDouble(m_link_table, row, 6),
            ReadTableDouble(m_link_table, row, 7),
            ReadTableDouble(m_link_table, row, 8),
            ReadTableDouble(m_link_table, row, 9),
            ReadTableDouble(m_link_table, row, 10)};
        model.links.push_back(link);
    }

    model.joints.clear();
    for (int row = 0; row < m_joint_drive_table->rowCount(); ++row)
    {
        RoboSDP::Dynamics::Dto::DynamicJointDriveDto joint;
        joint.joint_id = ReadTableString(m_joint_drive_table, row, 0);
        joint.transmission_ratio = ReadTableDouble(m_joint_drive_table, row, 1, 100.0);
        joint.efficiency = ReadTableDouble(m_joint_drive_table, row, 2, 0.92);
        joint.friction.viscous = ReadTableDouble(m_joint_drive_table, row, 3);
        joint.friction.coulomb = ReadTableDouble(m_joint_drive_table, row, 4);
        joint.friction.statik = ReadTableDouble(m_joint_drive_table, row, 5);
        model.joints.push_back(joint);
    }

    model.trajectories.clear();
    for (int row = 0; row < m_trajectory_table->rowCount(); ++row)
    {
        RoboSDP::Dynamics::Dto::BenchmarkTrajectoryDto trajectory;
        trajectory.trajectory_id = ReadTableString(m_trajectory_table, row, 0);
        trajectory.name = ReadTableString(m_trajectory_table, row, 1);
        trajectory.profile_type = ReadTableString(m_trajectory_table, row, 2);
        trajectory.active_joint_id = ReadTableString(m_trajectory_table, row, 3);
        trajectory.duration_s = ReadTableDouble(m_trajectory_table, row, 4, 1.0);
        trajectory.sample_count = ReadTableInt(m_trajectory_table, row, 5, 64);

        if (row < static_cast<int>(m_state.current_model.trajectories.size()))
        {
            const auto& original = m_state.current_model.trajectories.at(static_cast<std::size_t>(row));
            trajectory.joint_start_deg = original.joint_start_deg;
            trajectory.joint_target_deg = original.joint_target_deg;
        }

        model.trajectories.push_back(trajectory);
    }

    return model;
}

void DynamicsWidget::PopulateForm(const RoboSDP::Dynamics::Dto::DynamicModelDto& model)
{
    m_model_name_edit->setText(model.meta.name);
    m_kinematic_ref_edit->setText(model.meta.kinematic_ref);
    m_topology_ref_edit->setText(model.meta.topology_ref);
    m_requirement_ref_edit->setText(model.meta.requirement_ref);
    m_gravity_z_spin->setValue(model.gravity[2]);
    m_end_effector_mass_spin->setValue(model.end_effector.mass);

    m_link_table->setRowCount(static_cast<int>(model.links.size()));
    for (int row = 0; row < static_cast<int>(model.links.size()); ++row)
    {
        const auto& link = model.links.at(static_cast<std::size_t>(row));
        auto* idItem = EnsureItem(m_link_table, row, 0);
        idItem->setText(link.link_id);
        SetReadOnlyItem(idItem);
        EnsureItem(m_link_table, row, 1)->setText(QString::number(link.mass, 'f', 4));
        EnsureItem(m_link_table, row, 2)->setText(QString::number(link.cog[0], 'f', 4));
        EnsureItem(m_link_table, row, 3)->setText(QString::number(link.cog[1], 'f', 4));
        EnsureItem(m_link_table, row, 4)->setText(QString::number(link.cog[2], 'f', 4));
        EnsureItem(m_link_table, row, 5)->setText(QString::number(link.inertia_tensor[0], 'f', 5));
        EnsureItem(m_link_table, row, 6)->setText(QString::number(link.inertia_tensor[1], 'f', 5));
        EnsureItem(m_link_table, row, 7)->setText(QString::number(link.inertia_tensor[2], 'f', 5));
        EnsureItem(m_link_table, row, 8)->setText(QString::number(link.inertia_tensor[3], 'f', 5));
        EnsureItem(m_link_table, row, 9)->setText(QString::number(link.inertia_tensor[4], 'f', 5));
        EnsureItem(m_link_table, row, 10)->setText(QString::number(link.inertia_tensor[5], 'f', 5));
    }

    m_joint_drive_table->setRowCount(static_cast<int>(model.joints.size()));
    for (int row = 0; row < static_cast<int>(model.joints.size()); ++row)
    {
        const auto& joint = model.joints.at(static_cast<std::size_t>(row));
        auto* idItem = EnsureItem(m_joint_drive_table, row, 0);
        idItem->setText(joint.joint_id);
        SetReadOnlyItem(idItem);
        EnsureItem(m_joint_drive_table, row, 1)->setText(QString::number(joint.transmission_ratio, 'f', 3));
        EnsureItem(m_joint_drive_table, row, 2)->setText(QString::number(joint.efficiency, 'f', 4));
        EnsureItem(m_joint_drive_table, row, 3)->setText(QString::number(joint.friction.viscous, 'f', 4));
        EnsureItem(m_joint_drive_table, row, 4)->setText(QString::number(joint.friction.coulomb, 'f', 4));
        EnsureItem(m_joint_drive_table, row, 5)->setText(QString::number(joint.friction.statik, 'f', 4));
    }

    m_trajectory_table->setRowCount(static_cast<int>(model.trajectories.size()));
    for (int row = 0; row < static_cast<int>(model.trajectories.size()); ++row)
    {
        const auto& trajectory = model.trajectories.at(static_cast<std::size_t>(row));
        auto* idItem = EnsureItem(m_trajectory_table, row, 0);
        idItem->setText(trajectory.trajectory_id);
        SetReadOnlyItem(idItem);
        auto* nameItem = EnsureItem(m_trajectory_table, row, 1);
        nameItem->setText(trajectory.name);
        SetReadOnlyItem(nameItem);
        auto* profileItem = EnsureItem(m_trajectory_table, row, 2);
        profileItem->setText(trajectory.profile_type);
        SetReadOnlyItem(profileItem);
        auto* jointItem = EnsureItem(m_trajectory_table, row, 3);
        jointItem->setText(trajectory.active_joint_id);
        SetReadOnlyItem(jointItem);
        EnsureItem(m_trajectory_table, row, 4)->setText(QString::number(trajectory.duration_s, 'f', 3));
        EnsureItem(m_trajectory_table, row, 5)->setText(QString::number(trajectory.sample_count));
    }

    ValidateTablesAndHighlight();
}

void DynamicsWidget::RenderBackendStatus()
{
    if (m_solver_backend_edit == nullptr || m_used_fallback_edit == nullptr || m_backend_message_label == nullptr)
    {
        return;
    }

    m_solver_backend_edit->setText(BuildBackendListText(m_state.trajectory_results));
    m_used_fallback_edit->setText(m_state.trajectory_results.empty() ? QStringLiteral("未执行") : QStringLiteral("否"));
    const bool warning = HasFailedResults(m_state.trajectory_results);
    m_backend_message_label->setText(BuildPrimaryResultMessage(m_state.trajectory_results));
    m_backend_message_label->setStyleSheet(
        warning ? QStringLiteral("color: #b54708; font-weight: 600;")
                : QStringLiteral("color: #344054;"));
}

void DynamicsWidget::RenderTorquePlot()
{
    if (m_torque_plot == nullptr || m_torque_plot_status_label == nullptr)
    {
        return;
    }

    m_torque_plot->clearGraphs();

    if (m_state.trajectory_results.empty()
        || m_state.trajectory_results.front().joint_curves.empty()
        || m_state.trajectory_results.front().joint_curves.front().samples.empty())
    {
        m_torque_plot_status_label->setText(QStringLiteral("当前无可绘制曲线，请先执行逆动力学或加载已有结果。"));
        m_torque_plot->xAxis->setLabel(QStringLiteral("时间 [s]"));
        m_torque_plot->yAxis->setLabel(QStringLiteral("扭矩 [Nm]"));
        m_torque_plot->replot();
        return;
    }

    const auto& trajectoryResult = m_state.trajectory_results.front();
    const auto& curve = trajectoryResult.joint_curves.front();

    QVector<double> xValues;
    QVector<double> yValues;
    xValues.reserve(static_cast<int>(curve.samples.size()));
    yValues.reserve(static_cast<int>(curve.samples.size()));
    for (const auto& sample : curve.samples)
    {
        xValues.push_back(sample.time_s);
        yValues.push_back(sample.torque_nm);
    }

    QCPGraph* graph = m_torque_plot->addGraph();
    graph->setName(QStringLiteral("%1 / %2").arg(trajectoryResult.name, curve.joint_id));
    graph->setPen(QPen(QColor(QStringLiteral("#175CD3")), 2.0));
    graph->setData(xValues, yValues);

    m_torque_plot->xAxis->setLabel(QStringLiteral("时间 [s]"));
    m_torque_plot->yAxis->setLabel(QStringLiteral("扭矩 [Nm]"));
    m_torque_plot->rescaleAxes();
    m_torque_plot->replot();

    m_torque_plot_status_label->setText(
        QStringLiteral("当前显示：%1 的 %2 扭矩-时间曲线。").arg(trajectoryResult.name, curve.joint_id));
}

void DynamicsWidget::RenderResults()
{
    QStringList lines;
    lines.push_back(QStringLiteral("模型：%1").arg(m_state.current_model.meta.name));
    lines.push_back(QStringLiteral("轨迹数量：%1").arg(m_state.current_model.trajectories.size()));
    lines.push_back(QStringLiteral("参数化结果：%1").arg(m_state.parameterized_trajectories.size()));
    lines.push_back(QStringLiteral("逆动力学结果：%1").arg(m_state.trajectory_results.size()));

    if (!m_state.trajectory_results.empty())
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[轨迹执行摘要]"));
        lines.push_back(QStringLiteral("后端 = %1").arg(BuildBackendListText(m_state.trajectory_results)));
        lines.push_back(QStringLiteral("是否存在失败轨迹 = %1").arg(
            HasFailedResults(m_state.trajectory_results) ? QStringLiteral("是") : QStringLiteral("否")));
        for (const auto& result : m_state.trajectory_results)
        {
            lines.push_back(
                QStringLiteral("%1: %2, 关节曲线 %3 条")
                    .arg(result.name)
                    .arg(result.success ? QStringLiteral("成功") : result.message)
                    .arg(result.joint_curves.size()));
        }
    }

    if (!m_state.peak_stats.empty())
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[峰值统计]"));
        for (const auto& peak : m_state.peak_stats)
        {
            lines.push_back(
                QStringLiteral("%1 / %2: 峰值扭矩 %3 Nm, 峰值速度 %4 deg/s, 峰值功率 %5 W")
                    .arg(peak.trajectory_id)
                    .arg(peak.joint_id)
                    .arg(peak.peak_torque_nm, 0, 'f', 3)
                    .arg(peak.peak_speed_deg_s, 0, 'f', 3)
                    .arg(peak.peak_power_w, 0, 'f', 3));
        }
    }

    if (!m_state.rms_stats.empty())
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[RMS 统计]"));
        for (const auto& rms : m_state.rms_stats)
        {
            lines.push_back(
                QStringLiteral("%1 / %2: RMS 扭矩 %3 Nm")
                    .arg(rms.trajectory_id)
                    .arg(rms.joint_id)
                    .arg(rms.rms_torque_nm, 0, 'f', 3));
        }
    }

    if (!m_state.load_envelope.joints.empty())
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("[负载包络]"));
        for (const auto& joint : m_state.load_envelope.joints)
        {
            lines.push_back(FormatEnvelopeLine(joint));
        }
    }

    if (lines.size() <= 4)
    {
        lines.push_back(QString());
        lines.push_back(QStringLiteral("尚未执行逆动力学主链。"));
    }

    m_result_summary_edit->setPlainText(lines.join(QLatin1Char('\n')));
}

void DynamicsWidget::SetOperationMessage(const QString& message, bool success, bool warning)
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

void DynamicsWidget::EmitTelemetryStatus(const QString& engineName, const QString& message, bool warning)
{
    const QString prefix = warning ? QStringLiteral("[警告]") : QStringLiteral("[计算完成]");
    emit TelemetryStatusGenerated(QStringLiteral("%1 引擎: %2。%3").arg(prefix, engineName, message), warning);
}

bool DynamicsWidget::ValidateTablesAndHighlight(QString* message)
{
    QStringList errors;

    const QSignalBlocker linkBlocker(m_link_table);
    const QSignalBlocker jointBlocker(m_joint_drive_table);
    const QSignalBlocker trajectoryBlocker(m_trajectory_table);

    for (int row = 0; row < m_link_table->rowCount(); ++row)
    {
        QTableWidgetItem* massItem = EnsureItem(m_link_table, row, 1);
        const bool massValid = ReadTableDouble(m_link_table, row, 1) >= 0.0;
        HighlightItem(massItem, massValid, QStringLiteral("质量必须大于等于 0。"));
        if (!massValid)
        {
            errors.push_back(QStringLiteral("连杆 %1 的质量必须大于等于 0。").arg(ReadTableString(m_link_table, row, 0)));
        }

        for (int column = 5; column <= 10; ++column)
        {
            QTableWidgetItem* inertiaItem = EnsureItem(m_link_table, row, column);
            const bool inertiaValid = ReadTableDouble(m_link_table, row, column) >= 0.0;
            HighlightItem(inertiaItem, inertiaValid, QStringLiteral("惯量参数必须大于等于 0。"));
            if (!inertiaValid)
            {
                errors.push_back(QStringLiteral("连杆 %1 的惯量参数必须大于等于 0。").arg(ReadTableString(m_link_table, row, 0)));
            }
        }
    }

    for (int row = 0; row < m_joint_drive_table->rowCount(); ++row)
    {
        QTableWidgetItem* ratioItem = EnsureItem(m_joint_drive_table, row, 1);
        const bool ratioValid = ReadTableDouble(m_joint_drive_table, row, 1) > 0.0;
        HighlightItem(ratioItem, ratioValid, QStringLiteral("传动比必须大于 0。"));
        if (!ratioValid)
        {
            errors.push_back(QStringLiteral("关节 %1 的传动比必须大于 0。").arg(ReadTableString(m_joint_drive_table, row, 0)));
        }

        QTableWidgetItem* efficiencyItem = EnsureItem(m_joint_drive_table, row, 2);
        const double efficiency = ReadTableDouble(m_joint_drive_table, row, 2);
        const bool efficiencyValid = efficiency > 0.0 && efficiency <= 1.0;
        HighlightItem(efficiencyItem, efficiencyValid, QStringLiteral("效率必须位于 (0, 1] 区间。"));
        if (!efficiencyValid)
        {
            errors.push_back(QStringLiteral("关节 %1 的效率必须位于 (0, 1] 区间。").arg(ReadTableString(m_joint_drive_table, row, 0)));
        }
    }

    for (int row = 0; row < m_trajectory_table->rowCount(); ++row)
    {
        bool sampleOk = false;
        const QString sampleText = ReadTableString(m_trajectory_table, row, 5);
        sampleText.toInt(&sampleOk);

        QTableWidgetItem* durationItem = EnsureItem(m_trajectory_table, row, 4);
        const bool durationValid = ReadTableDouble(m_trajectory_table, row, 4) > 0.0;
        HighlightItem(durationItem, durationValid, QStringLiteral("轨迹持续时间必须大于 0。"));
        if (!durationValid)
        {
            errors.push_back(QStringLiteral("轨迹 %1 的持续时间必须大于 0。").arg(ReadTableString(m_trajectory_table, row, 0)));
        }

        QTableWidgetItem* sampleItem = EnsureItem(m_trajectory_table, row, 5);
        const int sampleCount = sampleText.toInt();
        const bool sampleValid = sampleOk && sampleCount >= 2;
        HighlightItem(sampleItem, sampleValid, QStringLiteral("采样点数必须是不小于 2 的整数。"));
        if (!sampleValid)
        {
            errors.push_back(QStringLiteral("轨迹 %1 的采样点数必须是不小于 2 的整数。").arg(ReadTableString(m_trajectory_table, row, 0)));
        }
    }

    if (message != nullptr)
    {
        *message = errors.isEmpty() ? QString() : errors.front();
    }
    return errors.isEmpty();
}

void DynamicsWidget::HighlightItem(QTableWidgetItem* item, bool isValid, const QString& tooltip)
{
    if (item == nullptr)
    {
        return;
    }

    item->setBackground(isValid ? QColor(Qt::white) : QColor(QStringLiteral("#FEE4E2")));
    item->setToolTip(isValid ? QString() : tooltip);
}

void DynamicsWidget::OnBrowseProjectRootClicked()
{
    const QString selectedDirectory = QFileDialog::getExistingDirectory(
        this,
        QStringLiteral("选择 Dynamics 项目目录"),
        m_project_root_edit->text().trimmed());

    if (!selectedDirectory.isEmpty())
    {
        m_project_root_edit->setText(QDir::toNativeSeparators(selectedDirectory));
    }
}

void DynamicsWidget::OnBuildFromKinematicsClicked()
{
    const auto buildResult = m_service.BuildFromKinematics(m_project_root_edit->text().trimmed());
    if (buildResult.IsSuccess())
    {
        m_state = buildResult.state;
        PopulateForm(m_state.current_model);
        RenderBackendStatus();
        RenderTorquePlot();
        RenderResults();
    }

    SetOperationMessage(buildResult.message, buildResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Dynamics] %1").arg(buildResult.message));
}

void DynamicsWidget::OnRunAnalysisClicked()
{
    QString validationMessage;
    if (!ValidateTablesAndHighlight(&validationMessage))
    {
        SetOperationMessage(validationMessage, false);
        emit LogMessageGenerated(QStringLiteral("[Dynamics] %1").arg(validationMessage));
        return;
    }

    m_state.current_model = CollectModelFromForm();
    const auto analyzeResult = m_service.RunInverseDynamicsChain(
        m_project_root_edit->text().trimmed(),
        m_state);

    if (analyzeResult.IsSuccess())
    {
        m_state = analyzeResult.state;
        PopulateForm(m_state.current_model);
        RenderBackendStatus();
        RenderTorquePlot();
        RenderResults();
    }

    const bool warning = !analyzeResult.IsSuccess();
    SetOperationMessage(analyzeResult.message, analyzeResult.IsSuccess(), warning);
    EmitTelemetryStatus(QStringLiteral("Pinocchio 共享内核"), analyzeResult.message, warning);
    emit LogMessageGenerated(QStringLiteral("%1 %2").arg(
        warning ? QStringLiteral("[Dynamics][Warning]") : QStringLiteral("[Dynamics]"),
        analyzeResult.message));
}

void DynamicsWidget::OnSaveDraftClicked()
{
    QString validationMessage;
    if (!ValidateTablesAndHighlight(&validationMessage))
    {
        SetOperationMessage(validationMessage, false);
        emit LogMessageGenerated(QStringLiteral("[Dynamics] %1").arg(validationMessage));
        return;
    }

    m_state.current_model = CollectModelFromForm();
    const auto saveResult = m_service.SaveDraft(m_project_root_edit->text().trimmed(), m_state);
    SetOperationMessage(saveResult.message, saveResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Dynamics] %1").arg(saveResult.message));
}

void DynamicsWidget::OnLoadClicked()
{
    const auto loadResult = m_service.LoadDraft(m_project_root_edit->text().trimmed());
    if (loadResult.IsSuccess())
    {
        m_state = loadResult.state;
        PopulateForm(m_state.current_model);
        RenderBackendStatus();
        RenderTorquePlot();
        RenderResults();
    }

    SetOperationMessage(loadResult.message, loadResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Dynamics] %1").arg(loadResult.message));
}

} // namespace RoboSDP::Dynamics::Ui
