import sys

with open('modules/kinematics/ui/KinematicsWidget.cpp', 'r', encoding='utf-8') as f:
    content = f.read()

# Replace CollapsibleSectionWidget with QGroupBox
content = content.replace('auto* basicSection = new CollapsibleSectionWidget', 'auto* basicSection = new QGroupBox')
content = content.replace('basicSection->SetContent(basicPropsWidget);', 'auto* bsLayout = new QVBoxLayout(basicSection); bsLayout->addWidget(basicPropsWidget);')
content = content.replace('basicSection->SetCollapsed(false);', '')

content = content.replace('auto* framesSection = new CollapsibleSectionWidget', 'auto* framesSection = new QGroupBox')
content = content.replace('framesSection->SetContent(framesWidget);', 'auto* fsLayout = new QVBoxLayout(framesSection); fsLayout->addWidget(framesWidget);')
content = content.replace('framesSection->SetCollapsed(false);', '')

content = content.replace('auto* debugSection = new CollapsibleSectionWidget', 'auto* debugSection = new QGroupBox')
content = content.replace('debugSection->SetContent(debugWidget);', 'auto* dsLayout = new QVBoxLayout(debugSection); dsLayout->addWidget(debugWidget);')
content = content.replace('debugSection->SetCollapsed(true);', 'debugSection->setCheckable(true); debugSection->setChecked(false); // 模拟折叠')

with open('modules/kinematics/ui/KinematicsWidget.cpp', 'w', encoding='utf-8') as f:
    f.write(content)
print("Replaced CollapsibleSectionWidget with QGroupBox")