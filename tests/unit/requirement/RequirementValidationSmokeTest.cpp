#include "modules/requirement/dto/RequirementModelDto.h"
#include "modules/requirement/validator/RequirementValidator.h"

#include <QCoreApplication>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    RoboSDP::Requirement::Validation::RequirementValidator validator;

    // 测试 1：默认 DTO 应通过基础校验（项目名为空，预期有 Error）
    {
        auto model = RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
        auto result = validator.Validate(model);
        // 默认 CreateDefault 的 project_name 为空字符串，应产生至少 1 个 Error
        if (result.IsValid())
        {
            return 1;
        }
    }

    // 测试 2：合法 DTO 应通过校验
    {
        auto model = RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
        model.project_meta.project_name = QStringLiteral("测试项目");
        model.project_meta.scenario_type = QStringLiteral("handling");
        model.load_requirements.rated_payload = 10.0;
        model.load_requirements.max_payload = 15.0;
        model.load_requirements.tool_mass = 2.0;
        model.workspace_requirements.max_radius = 1.5;
        model.workspace_requirements.min_radius = 0.3;
        model.workspace_requirements.max_height = 2.0;
        model.workspace_requirements.min_height = 0.0;
        model.motion_requirements.max_linear_speed = 2.0;
        model.motion_requirements.max_angular_speed = 180.0;
        model.motion_requirements.max_acceleration = 5.0;
        model.motion_requirements.max_angular_acceleration = 360.0;
        model.motion_requirements.jerk_limit = 10.0;
        model.motion_requirements.takt_time = 30.0;

        auto result = validator.Validate(model);
        // 应仅有 jerk 为 10 而不是 0 的警告消除，但如果 takt_time > 0 则不应有错误
        // 实际上 jerk_limit = 10 > 0，所以不会有 warning 和 error
        if (!result.IsValid())
        {
            return 2;
        }
    }

    // 测试 3：无效数据应产生校验错误（max_payload < rated_payload）
    {
        auto model = RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
        model.project_meta.project_name = QStringLiteral("测试项目");
        model.project_meta.scenario_type = QStringLiteral("handling");
        model.load_requirements.rated_payload = 20.0;
        model.load_requirements.max_payload = 10.0; // 小于额定负载

        auto result = validator.Validate(model);
        // 应检测到 max_payload < rated_payload
        bool foundExpectedError = false;
        for (const auto& issue : result.issues)
        {
            if (issue.code == QStringLiteral("REQ_MAX_PAYLOAD_LT_RATED"))
            {
                foundExpectedError = true;
                break;
            }
        }
        if (!foundExpectedError)
        {
            return 3;
        }
    }

    // 测试 4：节拍时间为 0 时应产生错误
    {
        auto model = RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
        model.project_meta.project_name = QStringLiteral("测试项目");
        model.project_meta.scenario_type = QStringLiteral("welding");
        model.load_requirements.rated_payload = 10.0;
        model.load_requirements.max_payload = 15.0;
        model.workspace_requirements.max_radius = 1.5;
        model.motion_requirements.max_linear_speed = 2.0;
        model.motion_requirements.max_angular_speed = 180.0;
        model.motion_requirements.max_acceleration = 5.0;
        model.motion_requirements.max_angular_acceleration = 360.0;
        // takt_time 保持默认 0

        auto result = validator.Validate(model);
        bool foundTaktError = false;
        for (const auto& issue : result.issues)
        {
            if (issue.code == QStringLiteral("REQ_TAKT_TIME_INVALID"))
            {
                foundTaktError = true;
                break;
            }
        }
        if (!foundTaktError)
        {
            return 4;
        }
    }

    return 0;
}
