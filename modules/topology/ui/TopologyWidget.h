#pragma once

#include "core/infrastructure/ProjectSaveCoordinator.h"
#include "core/logging/ConsoleLogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"
#include "modules/topology/service/TopologyService.h"
#include "modules/topology/service/TopologyTemplateLoader.h"
// 引入运动学模块的预览场景数据传输对象（DTO），用于向主渲染器（如VTK）传递 3D 骨架数据
#include "modules/kinematics/dto/UrdfPreviewSceneDto.h"

#include <QHash>
#include <QString>
#include <QWidget>
#include <array>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;

namespace RoboSDP::Topology::Ui
{

/**
 * @brief Topology（构型/拓扑）最小可用页面类。
 *
 * 这是一个 Qt 视图组件，主要负责：
 * 1. 构型模板的选择。
 * 2. 机器人基础尺寸（如基座高度、臂长等 DH 参数）的输入与编辑。
 * 3. 候选构型的生成与推荐方案展示。
 * 4. 草稿数据的保存与加载（实现 IProjectSaveParticipant 接口纳入全局保存体系）。
 * 5. 将当前设定的参数实时转化为 3D 骨架，并广播给主界面的渲染引擎。
 */
class TopologyWidget : public QWidget, public RoboSDP::Infrastructure::IProjectSaveParticipant
{
    Q_OBJECT

public:
    explicit TopologyWidget(QWidget* parent = nullptr);

    /// @brief 实现接口：返回全局保存日志使用的模块名称。
    QString ModuleName() const override;

    /// @brief 实现接口：保存当前 Topology 草稿，供模块内按钮和全局快捷键(Ctrl+S)共用。
    RoboSDP::Infrastructure::ProjectSaveItemResult SaveCurrentDraft() override;

    /// @brief 实现接口：返回当前 Topology 表单是否存在未保存的脏数据（供退出提示使用）。
    bool HasUnsavedChanges() const override;

public:
    /// @brief 外部接口：强制触发一次实时预览信号（可供主窗口加载完毕后主动拉取画面）。
    void ForceEmitPreview() { UpdateLivePreview(); }

    // ── 以下方法由 RibbonBarWidget 工具按钮信号触发，委托到内部槽函数 ──
    /// @brief Ribbon 接口：刷新构型模板列表。
    void TriggerRefreshTemplates() { OnRefreshTemplatesClicked(); }
    /// @brief Ribbon 接口：执行构型生成。
    void TriggerGenerate() { OnGenerateClicked(); }
    /// @brief Ribbon 接口：校验当前构型。
    void TriggerValidate() { OnValidateClicked(); }
    /// @brief Ribbon 接口：保存构型草稿。
    void TriggerSaveDraft() { OnSaveDraftClicked(); }

    // ── Ribbon 按钮状态查询方法 ──
    /// @brief Ribbon 按钮状态查询：刷新模板列表按钮是否可用（始终可用）。
    bool CanRefreshTemplates() const { return true; }
    /// @brief Ribbon 按钮状态查询：执行构型生成按钮是否可用（需已选中有效模板）。
    bool CanGenerate() const;
    /// @brief Ribbon 按钮状态查询：校验构型按钮是否可用（需已填写至少构型名称）。
    bool CanValidate() const;
    /// @brief Ribbon 按钮状态查询：保存草稿按钮是否可用（需存在未保存变更）。
    bool CanSaveDraft() const { return m_has_unsaved_changes; }

signals:
    /// @brief 信号：将 Topology 模块的操作日志发给主窗口底部的日志面板。
    void LogMessageGenerated(const QString& message);

    /// @brief 信号：向外（通常是主窗口的三维视图区）广播当前的拓扑骨架预览场景，用于实时渲染。
    void TopologyPreviewGenerated(const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene);

    /// @brief 信号：模块内部状态（表单字段、脏标志）发生变化时发射，通知 MainWindow 刷新 Ribbon 按钮启用/禁用状态。
    void StatusChanged();

private:
    // ==================== UI 构建与初始化 ====================
    void BuildUi();
    void RefreshTemplateOptions();

    // 分块构建 UI 面板
    QGroupBox* CreateTopologyGroup();
    QGroupBox* CreateCandidateGroup();
    QGroupBox* CreateValidationGroup();
    QWidget* CreateScrollableTab(QWidget* contentWidget);

    // UI 辅助工具
    QDoubleSpinBox* CreateDoubleSpinBox(double minimum, double maximum, int decimals, double step);
    
    /// @brief 将数据字段路径与对应的 UI 控件绑定，用于在校验失败时高亮特定控件。
    void RegisterFieldWidget(const QString& fieldPath, QWidget* widget);
    
    // ==================== 状态追踪 ====================
    /// @brief 绑定所有输入控件的值改变信号，用于触发 MarkDirty 和实时预览。
    void ConnectDirtyTracking();
    void MarkDirty();
    void MarkClean();

    // ==================== 数据流转 ====================
    /// @brief 从当前 UI 表单中收集所有的值，组装成 Topology DTO 模型。
    RoboSDP::Topology::Dto::RobotTopologyModelDto CollectModelFromForm() const;
    /// @brief 将后端的 Topology DTO 模型数据回填到界面的各个输入控件中。
    void PopulateForm(const RoboSDP::Topology::Dto::RobotTopologyModelDto& model);
    
    // ==================== 结果展示 ====================
    void RenderCandidates();
    void RenderRecommendation();

    // ==================== 校验与提示 ====================
    void ClearValidationState();
    void ApplyValidationResult(const RoboSDP::Topology::Validation::TopologyValidationResult& result);
    void SetOperationMessage(const QString& message, bool success);

    // ==================== 槽函数 (用户交互事件) ====================
    void OnRefreshTemplatesClicked();
    void OnGenerateClicked();
    void OnValidateClicked();
    void OnSaveDraftClicked();
    void OnLoadClicked();

    // ==================== 核心逻辑 ====================
    /// @brief 收集当前参数，调用底层数学接口生成 3D 骨架并发送信号，实现参数变动时画面实时刷新。
    void UpdateLivePreview();

private:
    // 后端依赖与服务注入
    RoboSDP::Logging::ConsoleLogger m_logger;
    RoboSDP::Repository::LocalJsonRepository m_repository;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage m_requirement_storage;
    RoboSDP::Topology::Persistence::TopologyJsonStorage m_topology_storage;
    RoboSDP::Topology::Validation::TopologyValidator m_validator;
    RoboSDP::Topology::Service::TopologyTemplateLoader m_template_loader;
    RoboSDP::Topology::Service::TopologyService m_service;

    // 页面状态
    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto m_state;
    /// @brief 存储 DTO 字段名与对应 UI 控件的映射表（供校验飘红使用）
    QHash<QString, QWidget*> m_field_widgets;
    bool m_has_unsaved_changes = false;

    // ==================== UI 控件指针 ====================
    QComboBox* m_template_combo = nullptr;
    QLabel* m_operation_label = nullptr;

    QLineEdit* m_topology_name_edit = nullptr;
    QComboBox* m_base_mount_combo = nullptr;
    QDoubleSpinBox* m_base_height_spin = nullptr;
    QDoubleSpinBox* m_base_orientation_x_spin = nullptr;
    QDoubleSpinBox* m_base_orientation_y_spin = nullptr;
    QDoubleSpinBox* m_base_orientation_z_spin = nullptr;
    QDoubleSpinBox* m_j1_range_min_spin = nullptr;
    QDoubleSpinBox* m_j1_range_max_spin = nullptr;

    // 运动学 DH 尺寸输入控件
    QDoubleSpinBox* m_shoulder_offset_spin = nullptr;
    QDoubleSpinBox* m_upper_arm_length_spin = nullptr;
    QDoubleSpinBox* m_forearm_length_spin = nullptr;
    QDoubleSpinBox* m_wrist_offset_spin = nullptr;
    
    // 机械/走线预留控件
    QCheckBox* m_internal_routing_check = nullptr;
    QCheckBox* m_hollow_wrist_check = nullptr;
    QDoubleSpinBox* m_reserved_channel_spin = nullptr;
    QCheckBox* m_seventh_axis_reserved_check = nullptr;
    QLineEdit* m_hollow_joint_ids_edit = nullptr;

    // 列表与信息展示面板
    QLabel* m_candidate_summary_label = nullptr;
    QListWidget* m_candidate_list = nullptr;
    QLabel* m_recommendation_summary_label = nullptr;
    QListWidget* m_recommendation_reason_list = nullptr;

    QLabel* m_validation_summary_label = nullptr;
    QListWidget* m_validation_issue_list = nullptr;
};

} // namespace RoboSDP::Topology::Ui