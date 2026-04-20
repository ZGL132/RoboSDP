"""
Planning gRPC Python 服务骨架。

当前文件只提供第 6 阶段需要的最小入口说明：
1. 服务端技术路线固定为 Python + MoveIt + gRPC。
2. 本轮先完成协议和调用链骨架，不在仓库中引入完整 MoveIt 运行依赖。
3. 后续接入真实 MoveIt 时，只需把 skeleton 响应替换为真实规划调用。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List


@dataclass
class TrajectoryWaypoint:
    """最小轨迹路点对象，供后续真实 gRPC 实现替换。"""

    time_from_start_s: float
    joint_values_deg: List[float]


@dataclass
class PointToPointPlanningResponse:
    """最小响应对象，对齐 proto 的核心口径。"""

    request_id: str
    transport_success: bool
    planning_success: bool
    message: str
    planner_id: str
    planning_time_s: float
    trajectory_duration_s: float
    waypoints: List[TrajectoryWaypoint]
    in_collision: bool
    collision_object_id: str
    collision_message: str
    in_self_collision: bool
    self_collision_link_a: str
    self_collision_link_b: str
    self_collision_message: str


class PlanningVerificationServiceSkeleton:
    """
    Python MoveIt 服务骨架。

    当前先返回固定的 skeleton 结果，后续真实接入时可在这里：
    1. 构建 PlanningScene；
    2. 调用 MoveIt 做点到点规划；
    3. 产出碰撞、自碰撞和轨迹摘要；
    4. 通过 gRPC 返回给 Qt 主程序。
    """

    def verify_point_to_point(
        self,
        request_id: str,
        planner_id: str,
        start_joint_values_deg: Iterable[float],
        goal_joint_values_deg: Iterable[float],
    ) -> PointToPointPlanningResponse:
        waypoints = [
            TrajectoryWaypoint(time_from_start_s=0.0, joint_values_deg=list(start_joint_values_deg)),
            TrajectoryWaypoint(time_from_start_s=1.0, joint_values_deg=list(goal_joint_values_deg)),
        ]
        return PointToPointPlanningResponse(
            request_id=request_id,
            transport_success=True,
            planning_success=True,
            message="planning-grpc skeleton returned success.",
            planner_id=planner_id,
            planning_time_s=0.1,
            trajectory_duration_s=1.0,
            waypoints=waypoints,
            in_collision=False,
            collision_object_id="",
            collision_message="no collision in skeleton response",
            in_self_collision=False,
            self_collision_link_a="",
            self_collision_link_b="",
            self_collision_message="no self collision in skeleton response",
        )


def main() -> int:
    """
    当前阶段只保留可读的服务入口占位。

    真正接入 grpcio 和 MoveIt 之后，可在这里：
    1. 加载 proto 生成代码；
    2. 注册 PlanningVerificationServiceSkeleton 的真实实现；
    3. 启动 gRPC server。
    """

    service = PlanningVerificationServiceSkeleton()
    response = service.verify_point_to_point(
        request_id="planning_request_demo",
        planner_id="RRTConnect",
        start_joint_values_deg=[0.0, -20.0, 30.0, 0.0, 20.0, 0.0],
        goal_joint_values_deg=[15.0, -35.0, 45.0, 10.0, 15.0, 20.0],
    )
    print(response.message)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
