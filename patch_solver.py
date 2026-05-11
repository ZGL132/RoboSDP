import sys

with open('modules/kinematics/ui/KinematicsWidget.cpp', 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Replace CreateSolverGroup to eliminate QToolBox
old_solver_group = r'''QGroupBox* KinematicsWidget::CreateSolverGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("正逆解与求解器"), this);
    auto* layout = new QVBoxLayout(groupBox);

    auto* toolbox = new QToolBox(groupBox);
    toolbox->addItem(CreateSolverConfigPage(), QStringLiteral("配置参数"));
    toolbox->addItem(CreateInteractivePage(), QStringLiteral("交互演示"));
    toolbox->addItem(CreateAdvancedAnalysisPage(), QStringLiteral("高级分析"));
    toolbox->setCurrentIndex(1); // 默认展开交互演示
    toolbox->setStyleSheet(QStringLiteral(
        "QToolBox::tab { padding: 6px 12px; font-weight: bold; }"));

    layout->addWidget(toolbox);
    return groupBox;
}'''

# Note: The Chinese strings might differ due to encoding issues in my python read.
# Let's find bounds programmatically instead.

start_idx = content.find('QGroupBox* KinematicsWidget::CreateSolverGroup()')
end_idx = content.find('QWidget* KinematicsWidget::CreateSolverConfigPage()')

if start_idx != -1 and end_idx != -1:
    new_solver_group = r'''QGroupBox* KinematicsWidget::CreateSolverGroup()
{
    auto* groupBox = new QGroupBox(QStringLiteral("正逆解与验证"), this);
    auto* layout = new QVBoxLayout(groupBox);

    auto* interactivePage = CreateInteractivePage();
    layout->addWidget(interactivePage);

    auto* advBox = new QGroupBox(QStringLiteral("高级分析 (工作空间、奇异性等)"));
    advBox->setCheckable(true);
    advBox->setChecked(false);
    auto* advLayout = new QVBoxLayout(advBox);
    advLayout->addWidget(CreateAdvancedAnalysisPage());
    layout->addWidget(advBox);

    return groupBox;
}

'''
    content = content[:start_idx] + new_solver_group + content[end_idx:]

# 2. Modify CreateInteractivePage to include SolverConfigPage
target_grid_start = content.find('auto* ikTargetGrid = new QGridLayout();')
target_grid_end = content.find('layout->addLayout(ikTargetGrid);') + len('layout->addLayout(ikTargetGrid);')

if target_grid_start != -1 and target_grid_end != -1:
    old_target_grid = content[target_grid_start:target_grid_end]
    new_target_grid = r'''auto* targetAndConfigLayout = new QHBoxLayout();
    
    auto* ikTargetGrid = new QGridLayout();
''' + old_target_grid.replace('auto* ikTargetGrid = new QGridLayout();\n', '').replace('layout->addLayout(ikTargetGrid);', '') + r'''
    targetAndConfigLayout->addLayout(ikTargetGrid, 2);

    auto* solverConfigBox = new QGroupBox(QStringLiteral("⚙️ 求解器设置"));
    solverConfigBox->setCheckable(true);
    solverConfigBox->setChecked(false);
    auto* scLayout = new QVBoxLayout(solverConfigBox);
    scLayout->addWidget(CreateSolverConfigPage());
    targetAndConfigLayout->addWidget(solverConfigBox, 1);

    layout->addLayout(targetAndConfigLayout);'''
    
    content = content[:target_grid_start] + new_target_grid + content[target_grid_end:]

with open('modules/kinematics/ui/KinematicsWidget.cpp', 'w', encoding='utf-8') as f:
    f.write(content)
print("Solver UI updated")
