#pragma once

#include "core/config/ConfigLoader.h"

namespace RoboSDP::Config
{

/**
 * @brief 统一应用配置入口。
 * 该门面负责收口“默认配置 + 统一文件路径加载”两种入口，
 * 不改变现有 ConfigLoader 的解析逻辑，只为主链提供稳定访问点。
 */
class AppConfigLoader
{
public:
    /// 返回当前统一应用配置文件的默认相对路径。
    QString DefaultAppConfigRelativePath() const;

    /// 从应用根目录加载配置；缺失时自动回退到默认配置。
    ConfigLoadResult LoadFromApplicationRoot(const QString& applicationRootPath) const;

private:
    ConfigLoader m_configLoader;
};

} // namespace RoboSDP::Config
