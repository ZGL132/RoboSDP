# RoboSDP 管理员构建脚本

## 作用

当前机器在普通权限下运行 `CMake try_compile / try_run` 时，会被系统级文件锁拦截。
在问题彻底根治前，项目已经验证通过的稳定路径是：

1. 管理员权限 `configure`
2. 管理员权限 `build`
3. 用固定的 `Qt / MinGW / VTK` 路径运行桌面程序

本说明文档只记录本轮已经验通的稳定脚本入口。

## 前提

1. 使用 **管理员 PowerShell** 运行脚本
2. 当前机器使用：
   - `Qt 5.15.2 + MinGW 8.1.0`
   - `VTK` 安装目录：`D:\10_Source_Repos\21_robot\_thirdparty-build\vtk-qt5-mingw81\install-ready`

## 稳定脚本入口

当前目录请只使用 `*-safe.ps1` 这组脚本入口。

### 1. 配置

```powershell
.\tools\build\configure-admin-safe.ps1
```

### 2. 编译桌面端

```powershell
.\tools\build\build-desktop-admin-safe.ps1
```

### 3. 运行桌面端

```powershell
.\tools\build\run-desktop-admin-safe.ps1
```

### 4. 5 秒启动自检

```powershell
.\tools\build\test-desktop-admin-safe.ps1
```

该脚本会启动 `RoboSDPDesktop.exe`，等待 5 秒后输出：

- `RUNNING_OK`
- 或 `EXITED:<code>`

## 说明

1. 旧的 `configure-admin.ps1`、`build-desktop-admin.ps1`、`run-desktop-admin.ps1` 曾受编码污染影响。
2. 本轮请优先使用 `*-safe.ps1` 这组稳定脚本。
3. 这些脚本是当前机器问题下的 workaround，不是永久替代方案。
