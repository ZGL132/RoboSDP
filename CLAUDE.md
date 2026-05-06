# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Robotic arm design and simulation platform (机械臂设计与仿真验证软件). A desktop Qt/C++ application for end-to-end robot design: requirement definition → topology design → kinematics/dynamics analysis → motor/reducer selection → motion planning verification → scheme export.

## Build & Test

### Prerequisites (Windows + MinGW)
- Qt 5.15.2 (mingw81_64) at `D:/software/Qt/5.15.2/mingw81_64`
- MinGW 8.1.0 at `D:/software/Qt/Tools/mingw810_64`
- VTK 9.2 (MinGW build) at a custom `ROBOSDP_VTK_DIR`
- CMake (VS 2022 bundled) + Ninja
- Pinocchio (conda env at `.deps/pinocchio-env/`)

### Build Commands (Admin Required)
```powershell
# Configure (first time or after CMakeLists changes)
tools/build/configure-admin-safe.ps1

# Build desktop executable
tools/build/build-desktop-admin-safe.ps1

# Quick self-check (launches exe for 5s)
tools/build/test-desktop-admin-safe.ps1

# Manual CMake (admin PS)
cmake -S . -B bdv3-admin -G Ninja `
  -DCMAKE_PREFIX_PATH=D:/software/Qt/5.15.2/mingw81_64 `
  -DCMAKE_C_COMPILER=D:/software/Qt/Tools/mingw810_64/bin/gcc.exe `
  -DCMAKE_CXX_COMPILER=D:/software/Qt/Tools/mingw810_64/bin/g++.exe `
  -DROBOSDP_VTK_DIR=path/to/vtk-9.2/lib/cmake/vtk-9.2

cmake --build bdv3-admin --target RoboSDPDesktop --parallel 2
```

### Running Tests
```powershell
# Build tests first (ROBOSDP_INCLUDE_TESTS_IN_ALL=ON in CMake or build test target)
cmake --build bdv3-admin --target robosdp_test_kinematics_smoke

# Run via ctest
cd bdv3-admin && ctest -R kinematics

# Run single test binary directly
bdv3-admin/tests/unit/kinematics/robosdp_test_kinematics_smoke.exe
```

Test targets follow the pattern: `robosdp_test_<module>_<testname>`. Unit tests live in `tests/unit/<module>/`, integration tests in `tests/integration/`.

### Release Packaging
```powershell
tools/release/package-desktop-release.ps1
```

## Architecture

### Module Pipeline (Strict Order)
Each stage depends on the previous module's outputs:

```
Requirement → Topology → Kinematics → Dynamics → Selection → Planning → Scheme
```

### Module Internal Structure
Every module follows the same pattern:
```
modules/<name>/
  dto/              # Data transfer objects (pure structs, no behavior)
  service/          # Business logic orchestration
  ui/               # Qt Widgets for the right-side property panel
  adapter/          # Algorithm backend abstraction (e.g., Pinocchio)
  persistence/      # JSON read/write via IJsonRepository
  validator/        # DTO validation (some modules)
```

### Core Infrastructure
```
core/
  errors/          # Unified ErrorCode enum with Chinese error messages
  logging/         # ILogger interface + ConsoleLogger
  config/          # ConfigLoader + AppConfig
  infrastructure/  # ProjectManager (singleton, project root context)
                   # ProjectSaveCoordinator (orchestrates cross-module save)
                   # ProjectDirtyDependencyGraph (upstream change tracking)
  repository/      # IJsonRepository + LocalJsonRepository (project folder JSON storage)
  schema/          # DTO consistency checks, project structure validation
  kinematics/      # SharedRobotKernelRegistry (Pinocchio shared kernel cache)
```

### Desktop App (apps/desktop-qt/)
```
apps/desktop-qt/
  main.cpp                 # Entry point
  AppBootstrap.{h,cpp}     # Qt/VTK surface format config, app metadata
  MainWindow.{h,cpp}       # QMainWindow: ribbon, project tree, property dock (stacked), 3D view, log panel
  VtkSceneWidget.{h,cpp}   # Legacy VTK widget (being phased out)
  widgets/
    ribbon/RibbonBarWidget.{h,cpp}
    vtk/RobotVtkView.{h,cpp}       # Central 3D VTK view (current)
    vtk/VtkSceneBuilder.{h,cpp}    # Build VTK scene from DTOs
    dialogs/GlobalSaveResultDialog.{h,cpp}
    empty/ProjectEmptyStateWidget.{h,cpp}
    common/CollapsibleSectionWidget.{h,cpp}
  third_party/qcustomplot/  # QCustomPlot chart library
```

### Key Design Patterns
- **Adapter Pattern**: All algorithm backends (Pinocchio for kinematics/dynamics, MoveIt for planning) are behind abstract adapter interfaces — `IKinematicBackendAdapter`, `IIkSolverAdapter`, `IDynamicsBackendAdapter`, `MoveItGrpcAdapter`
- **DTO/Service/UI Triad**: Each module separates data (DTO), logic (Service), and presentation (Widget)
- **Project Save Coordinator**: Register via `IProjectSaveParticipant` interface; saves are orchestrated in dependency order
- **Singleton ProjectManager**: Single source of truth for current project root path
- **Shared Robot Kernel**: `SharedRobotKernelRegistry` caches Pinocchio models to avoid reparsing URDF

### Services
- `services/planning-grpc/`: Python gRPC server wrapping MoveIt for motion planning verification
  - Proto: `proto/planning_verification.proto`
  - Only P2P planning exposed: `VerifyPointToPoint` RPC

### Technology Stack (Frozen)
| Component | Choice |
|-----------|--------|
| Desktop UI | Qt Widgets + C++17 |
| Build | CMake + Ninja |
| 3D View | VTK + Qt integration |
| Planning backend | Python + MoveIt (via gRPC) |
| Communication | gRPC |
| Dynamics kernel | Pinocchio (via adapter) |
| Project storage | Project folder + JSON |
| Charts | QCustomPlot |
| Logging | Unified ILogger + file output |
| Config | Centralized ConfigLoader |

### DTO Conventions
- All DTOs are plain structs in `namespace RoboSDP::<Module>::Dto`
- Models = domain objects (represent real business objects, persistable)
- DTOs = input/output carriers (serialization, no complex behavior)
- Results = computation output (analysis/simulation results)
- ViewModels = UI-only (never persisted, never cross-module)
- Fields are annotated with Chinese comments explaining unit, default, and semantics

### Coding Rules
- All core classes, methods, complex logic, DTO fields, and adapter mappings must have Chinese comments (中文注释)
- UI layer must not contain complex business logic
- Service layer handles orchestration
- External services accessed only through Adapters
- No God Objects / God Services
- Implement in minimal incremental steps, no batch full-module generation
