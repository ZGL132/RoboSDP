from __future__ import annotations

import argparse
import sys
import time
from concurrent import futures
from pathlib import Path
import subprocess


def ensure_grpc_modules(proto_file: Path, output_dir: Path):
    """
    运行时生成最小 gRPC Python 代码。
    本轮只支持单个 proto 文件，避免把大量生成代码提交进仓库。
    """

    output_dir.mkdir(parents=True, exist_ok=True)
    include_dir = proto_file.parent.as_posix()
    output_dir_posix = output_dir.as_posix()
    proto_file_posix = proto_file.as_posix()
    command = [
        sys.executable,
        "-m",
        "grpc_tools.protoc",
        f"-I{include_dir}",
        f"--python_out={output_dir_posix}",
        f"--grpc_python_out={output_dir_posix}",
        proto_file_posix,
    ]
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        raise RuntimeError(
            "grpc_tools.protoc 生成 Python 代码失败："
            + (completed.stderr.strip() or completed.stdout.strip())
        )

    if str(output_dir) not in sys.path:
        sys.path.insert(0, str(output_dir))

    import planning_verification_pb2  # type: ignore
    import planning_verification_pb2_grpc  # type: ignore

    return planning_verification_pb2, planning_verification_pb2_grpc


def load_grpc_modules(proto_file: Path, output_dir: Path):
    """
    优先加载仓库内已经生成的模块。
    若用户首次启动且仓库内还没有生成文件，再在服务目录下执行一次最小生成。
    """

    if str(output_dir) not in sys.path:
        sys.path.insert(0, str(output_dir))

    try:
        import planning_verification_pb2  # type: ignore
        import planning_verification_pb2_grpc  # type: ignore

        return planning_verification_pb2, planning_verification_pb2_grpc
    except ModuleNotFoundError:
        return ensure_grpc_modules(proto_file, output_dir)


def build_waypoints(start_values: list[float], goal_values: list[float], count: int, duration_s: float):
    """生成最小线性轨迹，用于阶段 B1 的真实 gRPC 联调。"""

    safe_count = max(2, count)
    waypoints = []
    for index in range(safe_count):
        ratio = index / float(safe_count - 1)
        joint_values = []
        for start_value, goal_value in zip(start_values, goal_values):
            joint_values.append(start_value + (goal_value - start_value) * ratio)
        waypoints.append((duration_s * ratio, joint_values))
    return waypoints


def estimate_duration(start_values: list[float], goal_values: list[float]) -> float:
    """按最小规则估算轨迹时长，先保证链路可用，不追求真实 MoveIt 动力学时标。"""

    max_delta = 0.0
    for start_value, goal_value in zip(start_values, goal_values):
        max_delta = max(max_delta, abs(goal_value - start_value))
    return max(0.6, max_delta / 90.0 + 0.4)


class PlanningVerificationServiceServicer:
    """最小 Planning gRPC 服务实现。"""

    def __init__(self, pb2):
        self._pb2 = pb2

    def VerifyPointToPoint(self, request, context):  # noqa: N802
        """
        当前只实现最小点到点请求：
        1. 校验关节数量；
        2. 生成线性轨迹；
        3. 返回最小碰撞 / 自碰撞 / 节拍结果。
        """

        if not request.joint_ids:
            return self._pb2.PointToPointPlanningResponse(
                request_id=request.request_id,
                transport_success=True,
                planning_success=False,
                message="规划请求缺少关节 ID。",
                planner_id=request.planner_id,
                planning_time_s=0.0,
                trajectory_duration_s=0.0,
                in_collision=False,
                collision_object_id="environment",
                collision_message="未执行碰撞检查。",
                in_self_collision=False,
                self_collision_link_a="link_2",
                self_collision_link_b="link_4",
                self_collision_message="未执行自碰撞检查。",
            )

        if len(request.joint_ids) != len(request.start_joint_values_deg) or len(request.joint_ids) != len(
            request.goal_joint_values_deg
        ):
            return self._pb2.PointToPointPlanningResponse(
                request_id=request.request_id,
                transport_success=True,
                planning_success=False,
                message="关节 ID、起点和终点数量不一致。",
                planner_id=request.planner_id,
                planning_time_s=0.0,
                trajectory_duration_s=0.0,
                in_collision=False,
                collision_object_id="environment",
                collision_message="未执行碰撞检查。",
                in_self_collision=False,
                self_collision_link_a="link_2",
                self_collision_link_b="link_4",
                self_collision_message="未执行自碰撞检查。",
            )

        if request.allowed_planning_time_s <= 0.05:
            return self._pb2.PointToPointPlanningResponse(
                request_id=request.request_id,
                transport_success=True,
                planning_success=False,
                message="允许规划时间过小，最小服务直接返回失败。",
                planner_id=request.planner_id,
                planning_time_s=request.allowed_planning_time_s,
                trajectory_duration_s=0.0,
                in_collision=False,
                collision_object_id="environment",
                collision_message="未执行碰撞检查。",
                in_self_collision=False,
                self_collision_link_a="link_2",
                self_collision_link_b="link_4",
                self_collision_message="未执行自碰撞检查。",
            )

        trajectory_duration_s = estimate_duration(
            list(request.start_joint_values_deg),
            list(request.goal_joint_values_deg),
        )
        in_collision = bool(request.check_collision and len(request.goal_joint_values_deg) > 1 and request.goal_joint_values_deg[1] < -115.0)
        in_self_collision = bool(
            request.check_self_collision
            and len(request.goal_joint_values_deg) > 2
            and request.goal_joint_values_deg[1] < -110.0
            and request.goal_joint_values_deg[2] > 120.0
        )
        planning_success = not in_collision and not in_self_collision

        waypoints = [
            self._pb2.TrajectoryWaypoint(time_from_start_s=time_from_start_s, joint_values_deg=joint_values)
            for time_from_start_s, joint_values in build_waypoints(
                list(request.start_joint_values_deg),
                list(request.goal_joint_values_deg),
                11,
                trajectory_duration_s,
            )
        ]

        return self._pb2.PointToPointPlanningResponse(
            request_id=request.request_id,
            transport_success=True,
            planning_success=planning_success,
            message="Python planning-grpc 最小服务已返回点到点规划结果。"
            if planning_success
            else "Python planning-grpc 最小服务检测到碰撞或自碰撞。",
            planner_id=request.planner_id,
            planning_time_s=min(0.25, max(0.05, request.allowed_planning_time_s * 0.25)),
            trajectory_duration_s=trajectory_duration_s,
            waypoints=waypoints,
            in_collision=in_collision,
            collision_object_id="environment_box",
            collision_message="检测到环境碰撞。" if in_collision else "未发现环境碰撞。",
            in_self_collision=in_self_collision,
            self_collision_link_a="link_2",
            self_collision_link_b="link_4",
            self_collision_message="检测到机械臂自碰撞。" if in_self_collision else "未发现机械臂自碰撞。",
        )


def serve(host: str, port: int) -> int:
    """启动最小 gRPC 服务。"""

    try:
        import grpc
    except ImportError as exc:  # pragma: no cover - 仅在缺依赖时触发
        raise RuntimeError("缺少 grpcio，请先安装 services/planning-grpc/server/requirements.txt。") from exc

    repo_root = Path(__file__).resolve().parents[3]
    proto_file = repo_root / "services" / "planning-grpc" / "proto" / "planning_verification.proto"
    generated_dir = Path(__file__).resolve().parent
    pb2, pb2_grpc = load_grpc_modules(proto_file, generated_dir)

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    servicer = PlanningVerificationServiceServicer(pb2)
    pb2_grpc.add_PlanningVerificationServiceServicer_to_server(servicer, server)
    bind_address = f"{host}:{port}"
    server.add_insecure_port(bind_address)
    server.start()

    print(f"Planning gRPC server started at {bind_address}", flush=True)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Planning gRPC server stopping...", flush=True)
        server.stop(grace=0)
    return 0


def main() -> int:
    """命令行入口。"""

    parser = argparse.ArgumentParser(description="RoboSDP Planning 最小 gRPC 服务")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50051)
    args = parser.parse_args()
    return serve(args.host, args.port)


if __name__ == "__main__":
    raise SystemExit(main())
