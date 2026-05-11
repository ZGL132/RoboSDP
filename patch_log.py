import sys

with open('modules/kinematics/ui/KinematicsWidget.cpp', 'r', encoding='utf-8') as f:
    content = f.read()

content = content.replace('m_logger->LogInfo(ModuleName(), QStringLiteral("该功能尚未实现: 自动计算TCP"));', 
                          'm_logger->Log(RoboSDP::Logging::LogLevel::Info, QStringLiteral("该功能尚未实现: 自动计算TCP"), RoboSDP::Errors::ErrorCode::Ok, {ModuleName(), QString(), QString()});')

with open('modules/kinematics/ui/KinematicsWidget.cpp', 'w', encoding='utf-8') as f:
    f.write(content)
print("Fixed LogInfo")