#include "modules/topology/dto/RobotTopologyModelDto.h"
#include "modules/topology/validator/TopologyValidator.h"

#include <QCoreApplication>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    RoboSDP::Topology::Validation::TopologyValidator validator;

    // 测试 1：默认 CreateDefault() 的 6R 构型应通过校验
    {
        auto model = RoboSDP::Topology::Dto::RobotTopologyModelDto::CreateDefault();
        auto result = validator.Validate(model);
        if (!result.IsValid())
        {
            return 1;
        }
    }

    // 测试 2：空 ID 的构型应产生错误
    {
        auto model = RoboSDP::Topology::Dto::RobotTopologyModelDto::CreateDefault();
        model.meta.topology_id = QStringLiteral("");
        model.meta.name = QStringLiteral("测试构型");
        auto result = validator.Validate(model);
        if (result.IsValid())
        {
            return 2;
        }
    }

    // 测试 3：joint_count != 6 应产生错误
    {
        auto model = RoboSDP::Topology::Dto::RobotTopologyModelDto::CreateDefault();
        model.robot_definition.joint_count = 4;
        auto result = validator.Validate(model);
        bool foundJointCountError = false;
        for (const auto& issue : result.issues)
        {
            if (issue.code == QStringLiteral("TOPOLOGY_JOINT_COUNT_INVALID"))
            {
                foundJointCountError = true;
                break;
            }
        }
        if (!foundJointCountError)
        {
            return 3;
        }
    }

    // 测试 4：joints 数量与 joint_count 不一致应产生错误
    {
        auto model = RoboSDP::Topology::Dto::RobotTopologyModelDto::CreateDefault();
        model.joints.pop_back(); // 移除一个关节，变成 5 个
        auto result = validator.Validate(model);
        bool foundSizeMismatch = false;
        for (const auto& issue : result.issues)
        {
            if (issue.code == QStringLiteral("TOPOLOGY_JOINT_SIZE_MISMATCH"))
            {
                foundSizeMismatch = true;
                break;
            }
        }
        if (!foundSizeMismatch)
        {
            return 4;
        }
    }

    // 测试 5：大臂长度 <= 0 应产生错误
    {
        auto model = RoboSDP::Topology::Dto::RobotTopologyModelDto::CreateDefault();
        model.robot_definition.upper_arm_length_m = 0.0;
        auto result = validator.Validate(model);
        bool foundUpperArmError = false;
        for (const auto& issue : result.issues)
        {
            if (issue.code == QStringLiteral("TOPOLOGY_UPPER_ARM_INVALID"))
            {
                foundUpperArmError = true;
                break;
            }
        }
        if (!foundUpperArmError)
        {
            return 5;
        }
    }

    // 测试 6：错误的基座安装方式应产生错误
    {
        auto model = RoboSDP::Topology::Dto::RobotTopologyModelDto::CreateDefault();
        model.robot_definition.base_mount_type = QStringLiteral("invalid_mount");
        auto result = validator.Validate(model);
        bool foundMountError = false;
        for (const auto& issue : result.issues)
        {
            if (issue.code == QStringLiteral("TOPOLOGY_BASE_MOUNT_INVALID"))
            {
                foundMountError = true;
                break;
            }
        }
        if (!foundMountError)
        {
            return 6;
        }
    }

    // 测试 7：默认构型的 topology_graph.links 应包含 7 个连杆
    {
        auto model = RoboSDP::Topology::Dto::RobotTopologyModelDto::CreateDefault();
        if (model.topology_graph.links.size() != 7)
        {
            return 7;
        }
        if (model.joints.size() != 6)
        {
            return 8;
        }
    }

    return 0;
}
