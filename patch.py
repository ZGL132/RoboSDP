import sys

with open('modules/kinematics/ui/KinematicsWidget.cpp', 'r', encoding='utf-8') as f:
    content = f.read()

start_idx = content.find('QWidget* KinematicsWidget::CreateModelGroup()')
if start_idx == -1:
    start_idx = content.find('QGroupBox* KinematicsWidget::CreateModelGroup()')

end_idx = content.find('QGroupBox* KinematicsWidget::CreateDhTableGroup()')

if start_idx != -1 and end_idx != -1:
    new_func = r'''QWidget* KinematicsWidget::CreateModelGroup()
{
    auto* pageWidget = new QWidget(this);
    auto* pageLayout = new QVBoxLayout(pageWidget);
    pageLayout->setContentsMargins(0, 0, 0, 0);

    // 1. Basic properties
    auto* basicPropsWidget = new QWidget(pageWidget);
    auto* formLayout = new QFormLayout(basicPropsWidget);
    m_model_name_edit = new QLineEdit(basicPropsWidget);
    m_parameter_convention_combo = new QComboBox(basicPropsWidget);
    m_parameter_convention_combo->addItem(QStringLiteral("DH"), QStringLiteral("DH"));
    m_parameter_convention_combo->addItem(QStringLiteral("MDH"), QStringLiteral("MDH"));
    formLayout->addRow(QStringLiteral("模型名称"), m_model_name_edit);
    formLayout->addRow(QStringLiteral("参数约定"), m_parameter_convention_combo);

    auto* basicSection = new CollapsibleSectionWidget(QStringLiteral("基础模型属性"), pageWidget);
    basicSection->SetContent(basicPropsWidget);
    basicSection->SetCollapsed(false);
    pageLayout->addWidget(basicSection);

    // 2. Physical frames
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

    auto* calcTcpBtn = new QPushButton(QStringLiteral("从法兰+工具计算TCP"), framesWidget);
    framesLayout->addWidget(calcTcpBtn, 3, 1);
    connect(calcTcpBtn, &QPushButton::clicked, this, [this]() {
        if (m_logger) {
            m_logger->LogInfo(ModuleName(), QStringLiteral("该功能尚未实现: 自动计算TCP"));
        }
        QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("待实现：根据 Flange * Tool 自动计算 TCP 位姿。"));
    });

    auto* framesSection = new CollapsibleSectionWidget(QStringLiteral("物理坐标系设置"), pageWidget);
    framesSection->SetContent(framesWidget);
    framesSection->SetCollapsed(false);
    pageLayout->addWidget(framesSection);

    // 3. Debug / Metadata section
    auto* debugWidget = new QWidget(pageWidget);
    auto* debugForm = new QFormLayout(debugWidget);
    
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
    
    debugForm->addRow(QStringLiteral("Topology 引用"), m_topology_ref_edit);
    debugForm->addRow(QStringLiteral("Requirement 引用"), m_requirement_ref_edit);
    debugForm->addRow(QStringLiteral("主控模型模式"), m_master_model_mode_label);
    debugForm->addRow(QStringLiteral("派生状态"), m_derived_model_state_label);
    debugForm->addRow(QStringLiteral("预览数据源"), m_preview_source_label);
    debugForm->addRow(QStringLiteral("主模型切换状态"), m_master_switch_state_label);
    debugForm->addRow(QStringLiteral("URDF 来源类型"), m_urdf_source_type_label);
    debugForm->addRow(QStringLiteral("DH 草稿提取级别"), m_dh_draft_level_label);
    debugForm->addRow(QStringLiteral("DH 草稿状态"), m_dh_draft_status_label);

    auto* debugSection = new CollapsibleSectionWidget(QStringLiteral("模型诊断与高级属性 (调试)"), pageWidget);
    debugSection->SetContent(debugWidget);
    debugSection->SetCollapsed(true);
    pageLayout->addWidget(debugSection);

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

'''
    new_content = content[:start_idx] + new_func + content[end_idx:]
    with open('modules/kinematics/ui/KinematicsWidget.cpp', 'w', encoding='utf-8') as f:
        f.write(new_content)
    print("Replaced successfully")
else:
    print("Could not find bounds")