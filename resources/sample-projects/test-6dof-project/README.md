# 6DOF Test Project

这个目录是一套用于 RoboSDP 第一阶段演示与手动测试的 6 自由度样例项目。

已包含模块：
- Requirement
- Topology
- Kinematics
- Dynamics
- Selection
- Planning
- Scheme Snapshot / Export

推荐手动验证路径：
1. 在桌面程序中通过“打开项目”选择本目录。
2. 依次进入 Requirement / Topology / Kinematics / Dynamics / Selection / Planning / Scheme 页面。
3. 先尝试“重新加载”，确认各模块都能读到现有 JSON。
4. 在 Dynamics 页面执行一次逆动力学，在 Selection 页面重新执行一次选型，在 Planning 页面重新执行一次最小规划验证。
5. 在 Scheme 页面执行一次“重新生成并保存”或“导出 JSON”，确认整条链路可再次聚合。

说明：
- 这套样例优先用于演示当前仓库第一阶段最小闭环。
- 目录根下已补齐 `project.json`，可直接通过当前桌面程序的项目打开校验。
- 结果数据以现有 smoke 数据和静态演示数据为基础，不代表真实工程定型结果。
