# Raw Result Traceability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a traceability layer where every calculation result point can be traced back to the exact raw data session, channel, sample range, time range, algorithm version, and parameters used to compute it.

**Architecture:** Add a new `modules/traceability` library with DTOs, JSON persistence, and a service API. Add a desktop widget that lists calculation runs, plots result series by `frame_seq`, and highlights the corresponding raw waveform window when a result point is selected.

**Tech Stack:** C++17, Qt Core/Widgets, JSON persistence through existing repository patterns, the existing minimal `QCustomPlot` compatibility widget, CMake.

---

## Current Context

The current repository does not contain a RAWD acquisition module or result structures named `frame_seq`, `DropS`, `DB/impact`, or `warn_level`. Existing chart support is a minimal local `QCustomPlot` implementation under `apps/desktop-qt/third_party/qcustomplot`. The implementation should therefore introduce a focused traceability module first, then wire it into the desktop UI.

The working tree is currently dirty from unrelated VTK/sample-project changes. Do not revert those changes while implementing this plan. Keep traceability changes in new files or clearly scoped integration files.

## File Structure

- Create `modules/traceability/CMakeLists.txt`
  - Builds `robosdp_traceability`.
- Create `modules/traceability/dto/TraceabilityDto.h`
  - Defines raw session, calculation run, calculation result, raw sample, and selected result DTOs.
- Create `modules/traceability/persistence/TraceabilityJsonStorage.h`
  - Declares JSON read/write functions for `raw_session`, `calc_run`, and `calc_result`.
- Create `modules/traceability/persistence/TraceabilityJsonStorage.cpp`
  - Implements stable JSON serialization.
- Create `modules/traceability/service/TraceabilityService.h`
  - Declares high-level APIs used by UI and future algorithms.
- Create `modules/traceability/service/TraceabilityService.cpp`
  - Implements run creation, result lookup, result-to-raw-window mapping, and fixture data generation for initial verification.
- Modify `CMakeLists.txt`
  - Adds `add_subdirectory(modules/traceability)`.
- Create `apps/desktop-qt/widgets/traceability/TraceabilityWidget.h`
  - Declares the UI widget.
- Create `apps/desktop-qt/widgets/traceability/TraceabilityWidget.cpp`
  - Implements run list, result table, result plot, raw waveform plot, and selection linkage.
- Modify `apps/desktop-qt/CMakeLists.txt`
  - Adds the widget sources and links `robosdp_traceability`.
- Modify `apps/desktop-qt/MainWindow.h`
  - Adds a member pointer for the traceability widget if integrating into the main window.
- Modify `apps/desktop-qt/MainWindow.cpp`
  - Adds a tab/panel entry for "数据追溯".
- Optional later: create `resources/sample-projects/traceability-demo/`
  - Stores example raw/run/result JSON for manual testing.

## Data Contract

The core relation is:

```text
raw_session.raw_id
  -> calc_run.raw_id
  -> calc_result.run_id
  -> calc_result.raw_start_index/raw_end_index
```

Each result row must carry both `frame_seq` and raw range fields. `frame_seq` alone is not enough because acquisition streams may use global frame numbers, dropped frames, or non-zero offsets.

---

### Task 1: Add Traceability DTOs

**Files:**
- Create: `modules/traceability/dto/TraceabilityDto.h`
- Create: `modules/traceability/CMakeLists.txt`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create DTO header**

Create `modules/traceability/dto/TraceabilityDto.h` with:

```cpp
#pragma once

#include <QDateTime>
#include <QHash>
#include <QString>

#include <vector>

namespace RoboSDP::Traceability::Dto
{

struct RawSessionDto
{
    QString raw_id;
    QString file_path;
    double sample_rate_hz = 0.0;
    int channel_count = 0;
    QDateTime start_time_utc;
    double duration_s = 0.0;
    QString data_hash;
};

struct CalcRunDto
{
    QString run_id;
    QString raw_id;
    QString algorithm_name;
    QString algorithm_version;
    int window_size_samples = 0;
    int step_size_samples = 0;
    QString params_json;
    QString params_hash;
    QDateTime created_at_utc;
};

struct CalcResultDto
{
    QString run_id;
    int frame_seq = 0;
    int local_window_index = 0;
    QString channel;
    qint64 raw_start_index = 0;
    qint64 raw_end_index = 0;
    double start_time_s = 0.0;
    double end_time_s = 0.0;
    double rms = 0.0;
    double db_impact = 0.0;
    double drops = 0.0;
    int warn_level = 0;
};

struct RawSampleDto
{
    qint64 sample_index = 0;
    double time_s = 0.0;
    QString channel;
    double voltage = 0.0;
};

struct ResultSelectionDto
{
    RawSessionDto raw_session;
    CalcRunDto calc_run;
    CalcResultDto calc_result;
    std::vector<RawSampleDto> raw_window_samples;
};

} // namespace RoboSDP::Traceability::Dto
```

- [ ] **Step 2: Create traceability CMake file**

Create `modules/traceability/CMakeLists.txt` with:

```cmake
add_library(robosdp_traceability STATIC
    dto/TraceabilityDto.h
)

target_include_directories(robosdp_traceability
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_traceability
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
)
```

- [ ] **Step 3: Register module in root CMake**

Modify root `CMakeLists.txt` after `add_subdirectory(modules/planning)`:

```cmake
add_subdirectory(modules/traceability)
```

- [ ] **Step 4: Build to verify DTO-only module**

Run:

```powershell
cmake --build build/cmake_de-Debug --target robosdp_traceability --parallel 4
```

Expected: target builds without compiling UI.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt modules/traceability/CMakeLists.txt modules/traceability/dto/TraceabilityDto.h
git commit -m "feat: add traceability data contracts"
```

---

### Task 2: Add JSON Persistence

**Files:**
- Create: `modules/traceability/persistence/TraceabilityJsonStorage.h`
- Create: `modules/traceability/persistence/TraceabilityJsonStorage.cpp`
- Modify: `modules/traceability/CMakeLists.txt`

- [ ] **Step 1: Extend CMake sources**

Change `modules/traceability/CMakeLists.txt` to:

```cmake
add_library(robosdp_traceability STATIC
    dto/TraceabilityDto.h
    persistence/TraceabilityJsonStorage.h
    persistence/TraceabilityJsonStorage.cpp
)

target_include_directories(robosdp_traceability
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_traceability
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
)
```

- [ ] **Step 2: Create persistence header**

Create `modules/traceability/persistence/TraceabilityJsonStorage.h` with:

```cpp
#pragma once

#include "modules/traceability/dto/TraceabilityDto.h"

#include <QJsonArray>
#include <QJsonObject>

namespace RoboSDP::Traceability::Persistence
{

QJsonObject ToJson(const RoboSDP::Traceability::Dto::RawSessionDto& value);
QJsonObject ToJson(const RoboSDP::Traceability::Dto::CalcRunDto& value);
QJsonObject ToJson(const RoboSDP::Traceability::Dto::CalcResultDto& value);

RoboSDP::Traceability::Dto::RawSessionDto RawSessionFromJson(const QJsonObject& object);
RoboSDP::Traceability::Dto::CalcRunDto CalcRunFromJson(const QJsonObject& object);
RoboSDP::Traceability::Dto::CalcResultDto CalcResultFromJson(const QJsonObject& object);

QJsonArray ToJsonArray(const std::vector<RoboSDP::Traceability::Dto::CalcResultDto>& values);
std::vector<RoboSDP::Traceability::Dto::CalcResultDto> CalcResultsFromJsonArray(const QJsonArray& array);

} // namespace RoboSDP::Traceability::Persistence
```

- [ ] **Step 3: Create persistence implementation**

Create `modules/traceability/persistence/TraceabilityJsonStorage.cpp` with:

```cpp
#include "modules/traceability/persistence/TraceabilityJsonStorage.h"

namespace RoboSDP::Traceability::Persistence
{

namespace
{

QString ReadString(const QJsonObject& object, const QString& key)
{
    return object.value(key).toString();
}

int ReadInt(const QJsonObject& object, const QString& key)
{
    return object.value(key).toInt();
}

qint64 ReadInt64(const QJsonObject& object, const QString& key)
{
    return static_cast<qint64>(object.value(key).toDouble());
}

double ReadDouble(const QJsonObject& object, const QString& key)
{
    return object.value(key).toDouble();
}

QDateTime ReadUtcDateTime(const QJsonObject& object, const QString& key)
{
    const QDateTime value = QDateTime::fromString(object.value(key).toString(), Qt::ISODateWithMs);
    return value.isValid() ? value.toUTC() : QDateTime();
}

QString WriteUtcDateTime(const QDateTime& value)
{
    return value.isValid() ? value.toUTC().toString(Qt::ISODateWithMs) : QString();
}

} // namespace

QJsonObject ToJson(const RoboSDP::Traceability::Dto::RawSessionDto& value)
{
    QJsonObject object;
    object.insert(QStringLiteral("raw_id"), value.raw_id);
    object.insert(QStringLiteral("file_path"), value.file_path);
    object.insert(QStringLiteral("sample_rate_hz"), value.sample_rate_hz);
    object.insert(QStringLiteral("channel_count"), value.channel_count);
    object.insert(QStringLiteral("start_time_utc"), WriteUtcDateTime(value.start_time_utc));
    object.insert(QStringLiteral("duration_s"), value.duration_s);
    object.insert(QStringLiteral("data_hash"), value.data_hash);
    return object;
}

QJsonObject ToJson(const RoboSDP::Traceability::Dto::CalcRunDto& value)
{
    QJsonObject object;
    object.insert(QStringLiteral("run_id"), value.run_id);
    object.insert(QStringLiteral("raw_id"), value.raw_id);
    object.insert(QStringLiteral("algorithm_name"), value.algorithm_name);
    object.insert(QStringLiteral("algorithm_version"), value.algorithm_version);
    object.insert(QStringLiteral("window_size_samples"), value.window_size_samples);
    object.insert(QStringLiteral("step_size_samples"), value.step_size_samples);
    object.insert(QStringLiteral("params_json"), value.params_json);
    object.insert(QStringLiteral("params_hash"), value.params_hash);
    object.insert(QStringLiteral("created_at_utc"), WriteUtcDateTime(value.created_at_utc));
    return object;
}

QJsonObject ToJson(const RoboSDP::Traceability::Dto::CalcResultDto& value)
{
    QJsonObject object;
    object.insert(QStringLiteral("run_id"), value.run_id);
    object.insert(QStringLiteral("frame_seq"), value.frame_seq);
    object.insert(QStringLiteral("local_window_index"), value.local_window_index);
    object.insert(QStringLiteral("channel"), value.channel);
    object.insert(QStringLiteral("raw_start_index"), static_cast<double>(value.raw_start_index));
    object.insert(QStringLiteral("raw_end_index"), static_cast<double>(value.raw_end_index));
    object.insert(QStringLiteral("start_time_s"), value.start_time_s);
    object.insert(QStringLiteral("end_time_s"), value.end_time_s);
    object.insert(QStringLiteral("rms"), value.rms);
    object.insert(QStringLiteral("db_impact"), value.db_impact);
    object.insert(QStringLiteral("drops"), value.drops);
    object.insert(QStringLiteral("warn_level"), value.warn_level);
    return object;
}

RoboSDP::Traceability::Dto::RawSessionDto RawSessionFromJson(const QJsonObject& object)
{
    RoboSDP::Traceability::Dto::RawSessionDto value;
    value.raw_id = ReadString(object, QStringLiteral("raw_id"));
    value.file_path = ReadString(object, QStringLiteral("file_path"));
    value.sample_rate_hz = ReadDouble(object, QStringLiteral("sample_rate_hz"));
    value.channel_count = ReadInt(object, QStringLiteral("channel_count"));
    value.start_time_utc = ReadUtcDateTime(object, QStringLiteral("start_time_utc"));
    value.duration_s = ReadDouble(object, QStringLiteral("duration_s"));
    value.data_hash = ReadString(object, QStringLiteral("data_hash"));
    return value;
}

RoboSDP::Traceability::Dto::CalcRunDto CalcRunFromJson(const QJsonObject& object)
{
    RoboSDP::Traceability::Dto::CalcRunDto value;
    value.run_id = ReadString(object, QStringLiteral("run_id"));
    value.raw_id = ReadString(object, QStringLiteral("raw_id"));
    value.algorithm_name = ReadString(object, QStringLiteral("algorithm_name"));
    value.algorithm_version = ReadString(object, QStringLiteral("algorithm_version"));
    value.window_size_samples = ReadInt(object, QStringLiteral("window_size_samples"));
    value.step_size_samples = ReadInt(object, QStringLiteral("step_size_samples"));
    value.params_json = ReadString(object, QStringLiteral("params_json"));
    value.params_hash = ReadString(object, QStringLiteral("params_hash"));
    value.created_at_utc = ReadUtcDateTime(object, QStringLiteral("created_at_utc"));
    return value;
}

RoboSDP::Traceability::Dto::CalcResultDto CalcResultFromJson(const QJsonObject& object)
{
    RoboSDP::Traceability::Dto::CalcResultDto value;
    value.run_id = ReadString(object, QStringLiteral("run_id"));
    value.frame_seq = ReadInt(object, QStringLiteral("frame_seq"));
    value.local_window_index = ReadInt(object, QStringLiteral("local_window_index"));
    value.channel = ReadString(object, QStringLiteral("channel"));
    value.raw_start_index = ReadInt64(object, QStringLiteral("raw_start_index"));
    value.raw_end_index = ReadInt64(object, QStringLiteral("raw_end_index"));
    value.start_time_s = ReadDouble(object, QStringLiteral("start_time_s"));
    value.end_time_s = ReadDouble(object, QStringLiteral("end_time_s"));
    value.rms = ReadDouble(object, QStringLiteral("rms"));
    value.db_impact = ReadDouble(object, QStringLiteral("db_impact"));
    value.drops = ReadDouble(object, QStringLiteral("drops"));
    value.warn_level = ReadInt(object, QStringLiteral("warn_level"));
    return value;
}

QJsonArray ToJsonArray(const std::vector<RoboSDP::Traceability::Dto::CalcResultDto>& values)
{
    QJsonArray array;
    for (const auto& value : values)
    {
        array.append(ToJson(value));
    }
    return array;
}

std::vector<RoboSDP::Traceability::Dto::CalcResultDto> CalcResultsFromJsonArray(const QJsonArray& array)
{
    std::vector<RoboSDP::Traceability::Dto::CalcResultDto> values;
    values.reserve(static_cast<std::size_t>(array.size()));
    for (const QJsonValue& item : array)
    {
        values.push_back(CalcResultFromJson(item.toObject()));
    }
    return values;
}

} // namespace RoboSDP::Traceability::Persistence
```

- [ ] **Step 4: Build persistence**

Run:

```powershell
cmake --build build/cmake_de-Debug --target robosdp_traceability --parallel 4
```

Expected: `robosdp_traceability` builds successfully.

- [ ] **Step 5: Commit**

```powershell
git add modules/traceability
git commit -m "feat: add traceability JSON storage"
```

---

### Task 3: Add Traceability Service

**Files:**
- Create: `modules/traceability/service/TraceabilityService.h`
- Create: `modules/traceability/service/TraceabilityService.cpp`
- Modify: `modules/traceability/CMakeLists.txt`

- [ ] **Step 1: Extend CMake sources**

Add service files to `modules/traceability/CMakeLists.txt`:

```cmake
add_library(robosdp_traceability STATIC
    dto/TraceabilityDto.h
    persistence/TraceabilityJsonStorage.h
    persistence/TraceabilityJsonStorage.cpp
    service/TraceabilityService.h
    service/TraceabilityService.cpp
)
```

- [ ] **Step 2: Create service header**

Create `modules/traceability/service/TraceabilityService.h` with:

```cpp
#pragma once

#include "modules/traceability/dto/TraceabilityDto.h"

#include <optional>

namespace RoboSDP::Traceability::Service
{

class TraceabilityService
{
public:
    void SetRawSession(const RoboSDP::Traceability::Dto::RawSessionDto& rawSession);
    void SetCalcRun(const RoboSDP::Traceability::Dto::CalcRunDto& calcRun);
    void SetCalcResults(const std::vector<RoboSDP::Traceability::Dto::CalcResultDto>& results);

    const RoboSDP::Traceability::Dto::RawSessionDto& RawSession() const;
    const RoboSDP::Traceability::Dto::CalcRunDto& CalcRun() const;
    const std::vector<RoboSDP::Traceability::Dto::CalcResultDto>& CalcResults() const;

    std::optional<RoboSDP::Traceability::Dto::CalcResultDto> FindResult(
        const QString& channel,
        int frameSeq) const;

    RoboSDP::Traceability::Dto::ResultSelectionDto BuildSelection(
        const QString& channel,
        int frameSeq) const;

    static std::vector<RoboSDP::Traceability::Dto::RawSampleDto> BuildSyntheticRawWindow(
        const RoboSDP::Traceability::Dto::RawSessionDto& rawSession,
        const RoboSDP::Traceability::Dto::CalcResultDto& result);

private:
    RoboSDP::Traceability::Dto::RawSessionDto m_rawSession;
    RoboSDP::Traceability::Dto::CalcRunDto m_calcRun;
    std::vector<RoboSDP::Traceability::Dto::CalcResultDto> m_results;
};

RoboSDP::Traceability::Dto::CalcResultDto BuildCalcResultFromWindow(
    const QString& runId,
    const QString& channel,
    int frameSeq,
    int localWindowIndex,
    qint64 rawStartIndex,
    int windowSizeSamples,
    double sampleRateHz,
    double rms,
    double dbImpact,
    double drops,
    int warnLevel);

} // namespace RoboSDP::Traceability::Service
```

- [ ] **Step 3: Create service implementation**

Create `modules/traceability/service/TraceabilityService.cpp` with:

```cpp
#include "modules/traceability/service/TraceabilityService.h"

#include <cmath>

namespace RoboSDP::Traceability::Service
{

void TraceabilityService::SetRawSession(const RoboSDP::Traceability::Dto::RawSessionDto& rawSession)
{
    m_rawSession = rawSession;
}

void TraceabilityService::SetCalcRun(const RoboSDP::Traceability::Dto::CalcRunDto& calcRun)
{
    m_calcRun = calcRun;
}

void TraceabilityService::SetCalcResults(const std::vector<RoboSDP::Traceability::Dto::CalcResultDto>& results)
{
    m_results = results;
}

const RoboSDP::Traceability::Dto::RawSessionDto& TraceabilityService::RawSession() const
{
    return m_rawSession;
}

const RoboSDP::Traceability::Dto::CalcRunDto& TraceabilityService::CalcRun() const
{
    return m_calcRun;
}

const std::vector<RoboSDP::Traceability::Dto::CalcResultDto>& TraceabilityService::CalcResults() const
{
    return m_results;
}

std::optional<RoboSDP::Traceability::Dto::CalcResultDto> TraceabilityService::FindResult(
    const QString& channel,
    int frameSeq) const
{
    for (const auto& result : m_results)
    {
        if (result.channel == channel && result.frame_seq == frameSeq)
        {
            return result;
        }
    }
    return std::nullopt;
}

RoboSDP::Traceability::Dto::ResultSelectionDto TraceabilityService::BuildSelection(
    const QString& channel,
    int frameSeq) const
{
    RoboSDP::Traceability::Dto::ResultSelectionDto selection;
    selection.raw_session = m_rawSession;
    selection.calc_run = m_calcRun;

    const auto result = FindResult(channel, frameSeq);
    if (!result.has_value())
    {
        return selection;
    }

    selection.calc_result = *result;
    selection.raw_window_samples = BuildSyntheticRawWindow(m_rawSession, *result);
    return selection;
}

std::vector<RoboSDP::Traceability::Dto::RawSampleDto> TraceabilityService::BuildSyntheticRawWindow(
    const RoboSDP::Traceability::Dto::RawSessionDto& rawSession,
    const RoboSDP::Traceability::Dto::CalcResultDto& result)
{
    std::vector<RoboSDP::Traceability::Dto::RawSampleDto> samples;
    if (rawSession.sample_rate_hz <= 0.0 || result.raw_end_index < result.raw_start_index)
    {
        return samples;
    }

    const qint64 maxPreviewSamples = 2000;
    const qint64 totalSamples = result.raw_end_index - result.raw_start_index + 1;
    const qint64 stride = std::max<qint64>(1, totalSamples / maxPreviewSamples);

    for (qint64 index = result.raw_start_index; index <= result.raw_end_index; index += stride)
    {
        RoboSDP::Traceability::Dto::RawSampleDto sample;
        sample.sample_index = index;
        sample.time_s = static_cast<double>(index) / rawSession.sample_rate_hz;
        sample.channel = result.channel;
        sample.voltage = 12.5 + 0.28 * std::sin(2.0 * 3.14159265358979323846 * 40.0 * sample.time_s);
        samples.push_back(sample);
    }
    return samples;
}

RoboSDP::Traceability::Dto::CalcResultDto BuildCalcResultFromWindow(
    const QString& runId,
    const QString& channel,
    int frameSeq,
    int localWindowIndex,
    qint64 rawStartIndex,
    int windowSizeSamples,
    double sampleRateHz,
    double rms,
    double dbImpact,
    double drops,
    int warnLevel)
{
    RoboSDP::Traceability::Dto::CalcResultDto result;
    result.run_id = runId;
    result.channel = channel;
    result.frame_seq = frameSeq;
    result.local_window_index = localWindowIndex;
    result.raw_start_index = rawStartIndex;
    result.raw_end_index = rawStartIndex + std::max(0, windowSizeSamples) - 1;
    result.start_time_s = sampleRateHz > 0.0 ? static_cast<double>(result.raw_start_index) / sampleRateHz : 0.0;
    result.end_time_s = sampleRateHz > 0.0 ? static_cast<double>(result.raw_end_index) / sampleRateHz : 0.0;
    result.rms = rms;
    result.db_impact = dbImpact;
    result.drops = drops;
    result.warn_level = warnLevel;
    return result;
}

} // namespace RoboSDP::Traceability::Service
```

- [ ] **Step 4: Build service**

Run:

```powershell
cmake --build build/cmake_de-Debug --target robosdp_traceability --parallel 4
```

Expected: service builds successfully.

- [ ] **Step 5: Commit**

```powershell
git add modules/traceability
git commit -m "feat: add traceability service"
```

---

### Task 4: Add Desktop Traceability Widget

**Files:**
- Create: `apps/desktop-qt/widgets/traceability/TraceabilityWidget.h`
- Create: `apps/desktop-qt/widgets/traceability/TraceabilityWidget.cpp`
- Modify: `apps/desktop-qt/CMakeLists.txt`

- [ ] **Step 1: Add widget files to desktop CMake**

Add these paths to `ROBOSDP_DESKTOP_SOURCES`:

```cmake
    widgets/traceability/TraceabilityWidget.cpp
    widgets/traceability/TraceabilityWidget.h
```

Add `robosdp_traceability` to `target_link_libraries(RoboSDPDesktop PRIVATE ...)`.

- [ ] **Step 2: Create widget header**

Create `apps/desktop-qt/widgets/traceability/TraceabilityWidget.h` with:

```cpp
#pragma once

#include "modules/traceability/service/TraceabilityService.h"

#include <QTableWidget>
#include <QWidget>

class QLabel;
class QCustomPlot;

class TraceabilityWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TraceabilityWidget(QWidget* parent = nullptr);

private:
    void BuildUi();
    void LoadDemoData();
    void RefreshResultTable();
    void RefreshResultPlot();
    void ShowSelection(const QString& channel, int frameSeq);
    void RefreshRawWindowPlot(const RoboSDP::Traceability::Dto::ResultSelectionDto& selection);
    void RefreshSelectionDetails(const RoboSDP::Traceability::Dto::ResultSelectionDto& selection);

    RoboSDP::Traceability::Service::TraceabilityService m_service;
    QTableWidget* m_result_table = nullptr;
    QCustomPlot* m_result_plot = nullptr;
    QCustomPlot* m_raw_plot = nullptr;
    QLabel* m_detail_label = nullptr;
};
```

- [ ] **Step 3: Create widget implementation**

Create `apps/desktop-qt/widgets/traceability/TraceabilityWidget.cpp` with a compact implementation that:

```cpp
#include "apps/desktop-qt/widgets/traceability/TraceabilityWidget.h"
#include "apps/desktop-qt/third_party/qcustomplot/qcustomplot.h"

#include <QDateTime>
#include <QHeaderView>
#include <QLabel>
#include <QSplitter>
#include <QTableWidget>
#include <QVBoxLayout>

TraceabilityWidget::TraceabilityWidget(QWidget* parent)
    : QWidget(parent)
{
    BuildUi();
    LoadDemoData();
    RefreshResultTable();
    RefreshResultPlot();
}
```

The implementation must create:

```text
Top: result plot, x = frame_seq, y = RMS
Middle: result table with frame_seq/channel/raw range/result columns
Bottom: raw waveform plot for selected result
Right or bottom details: run_id, raw_id, algorithm version, time range, sample range
```

Use this selection connection:

```cpp
connect(m_result_table, &QTableWidget::cellClicked, this, [this](int row, int) {
    const QString channel = m_result_table->item(row, 1)->text();
    const int frameSeq = m_result_table->item(row, 0)->text().toInt();
    ShowSelection(channel, frameSeq);
});
```

Because the current local `QCustomPlot` subset has no point-click interaction API, table selection is the first implementation path. Plot click selection can be added after extending `QCustomPlot`.

- [ ] **Step 4: Build desktop**

Run:

```powershell
cmake --build build/cmake_de-Debug --target RoboSDPDesktop --parallel 4
```

Expected: desktop target builds successfully.

- [ ] **Step 5: Commit**

```powershell
git add apps/desktop-qt/CMakeLists.txt apps/desktop-qt/widgets/traceability modules/traceability
git commit -m "feat: add traceability desktop widget"
```

---

### Task 5: Integrate Widget Into Main Window

**Files:**
- Modify: `apps/desktop-qt/MainWindow.h`
- Modify: `apps/desktop-qt/MainWindow.cpp`

- [ ] **Step 1: Add include**

In `apps/desktop-qt/MainWindow.cpp`, add:

```cpp
#include "apps/desktop-qt/widgets/traceability/TraceabilityWidget.h"
```

- [ ] **Step 2: Add member**

In `apps/desktop-qt/MainWindow.h`, add a private member near other widget pointers:

```cpp
TraceabilityWidget* m_traceabilityWidget = nullptr;
```

Forward declare before `class MainWindow`:

```cpp
class TraceabilityWidget;
```

- [ ] **Step 3: Add the page to the existing central navigation**

Find where the main module widgets are created in `MainWindow.cpp`. Add:

```cpp
m_traceabilityWidget = new TraceabilityWidget(this);
```

Then add it to the same stack/tab/navigation container used by Requirement, Kinematics, Dynamics, Selection, Planning, and Scheme pages. Use the label:

```cpp
QStringLiteral("数据追溯")
```

If the main window uses a tree node or ribbon action, add the node/action with stable key:

```cpp
QStringLiteral("traceability")
```

- [ ] **Step 4: Build desktop**

Run:

```powershell
cmake --build build/cmake_de-Debug --target RoboSDPDesktop --parallel 4
```

Expected: desktop target builds successfully and the "数据追溯" page opens.

- [ ] **Step 5: Commit**

```powershell
git add apps/desktop-qt/MainWindow.cpp apps/desktop-qt/MainWindow.h
git commit -m "feat: expose traceability page"
```

---

### Task 6: Extend Result Selection Beyond Table Clicks

**Files:**
- Modify: `apps/desktop-qt/third_party/qcustomplot/qcustomplot.h`
- Modify: `apps/desktop-qt/third_party/qcustomplot/qcustomplot.cpp`
- Modify: `apps/desktop-qt/widgets/traceability/TraceabilityWidget.cpp`

- [ ] **Step 1: Add graph point selection signal**

In `qcustomplot.h`, add `Q_OBJECT` to `class QCustomPlot` and declare:

```cpp
signals:
    void nearestPointClicked(double x, double y, int graphIndex);

protected:
    void mousePressEvent(QMouseEvent* event) override;
```

- [ ] **Step 2: Implement nearest point lookup**

In `qcustomplot.cpp`, implement `mousePressEvent` so it:

1. Builds the plot rect.
2. Iterates all graph points.
3. Maps each data point to pixel coordinates using existing `MapToPixel`.
4. Emits the nearest point if distance is less than 10 pixels.

Use:

```cpp
emit nearestPointClicked(bestX, bestY, bestGraphIndex);
```

- [ ] **Step 3: Connect result plot click to traceability selection**

In `TraceabilityWidget.cpp`, connect:

```cpp
connect(m_result_plot, &QCustomPlot::nearestPointClicked, this, [this](double x, double, int) {
    const int frameSeq = static_cast<int>(std::llround(x));
    ShowSelection(QStringLiteral("CH0"), frameSeq);
});
```

For the first implementation, plot clicks select CH0. A later extension can map graph index to channel.

- [ ] **Step 4: Build desktop**

Run:

```powershell
cmake --build build/cmake_de-Debug --target RoboSDPDesktop --parallel 4
```

Expected: desktop target builds successfully and clicking a result plot point updates the raw waveform panel.

- [ ] **Step 5: Commit**

```powershell
git add apps/desktop-qt/third_party/qcustomplot apps/desktop-qt/widgets/traceability/TraceabilityWidget.cpp
git commit -m "feat: select traceability result from plot"
```

---

### Task 7: Persist Real Run Data

**Files:**
- Modify: `modules/traceability/service/TraceabilityService.h`
- Modify: `modules/traceability/service/TraceabilityService.cpp`
- Modify: `modules/traceability/persistence/TraceabilityJsonStorage.h`
- Modify: `modules/traceability/persistence/TraceabilityJsonStorage.cpp`

- [ ] **Step 1: Add run document DTO**

In `TraceabilityDto.h`, add:

```cpp
struct TraceabilityRunDocumentDto
{
    RawSessionDto raw_session;
    CalcRunDto calc_run;
    std::vector<CalcResultDto> calc_results;
};
```

- [ ] **Step 2: Add document JSON functions**

In `TraceabilityJsonStorage.h`, add:

```cpp
QJsonObject ToJson(const RoboSDP::Traceability::Dto::TraceabilityRunDocumentDto& value);
RoboSDP::Traceability::Dto::TraceabilityRunDocumentDto TraceabilityRunDocumentFromJson(const QJsonObject& object);
```

In `TraceabilityJsonStorage.cpp`, implement with keys:

```text
raw_session
calc_run
calc_results
```

- [ ] **Step 3: Add service load/save methods**

In `TraceabilityService.h`, add:

```cpp
RoboSDP::Traceability::Dto::TraceabilityRunDocumentDto BuildDocument() const;
void LoadDocument(const RoboSDP::Traceability::Dto::TraceabilityRunDocumentDto& document);
```

In `TraceabilityService.cpp`, implement:

```cpp
RoboSDP::Traceability::Dto::TraceabilityRunDocumentDto TraceabilityService::BuildDocument() const
{
    RoboSDP::Traceability::Dto::TraceabilityRunDocumentDto document;
    document.raw_session = m_rawSession;
    document.calc_run = m_calcRun;
    document.calc_results = m_results;
    return document;
}

void TraceabilityService::LoadDocument(const RoboSDP::Traceability::Dto::TraceabilityRunDocumentDto& document)
{
    m_rawSession = document.raw_session;
    m_calcRun = document.calc_run;
    m_results = document.calc_results;
}
```

- [ ] **Step 4: Build**

Run:

```powershell
cmake --build build/cmake_de-Debug --target robosdp_traceability --parallel 4
```

Expected: traceability target builds successfully.

- [ ] **Step 5: Commit**

```powershell
git add modules/traceability
git commit -m "feat: persist traceability run documents"
```

---

### Task 8: Verification

**Files:**
- No new files.

- [ ] **Step 1: Build traceability library**

Run:

```powershell
cmake --build build/cmake_de-Debug --target robosdp_traceability --parallel 4
```

Expected: `[100%] Built target robosdp_traceability`.

- [ ] **Step 2: Build desktop**

Run:

```powershell
cmake --build build/cmake_de-Debug --target RoboSDPDesktop --parallel 4
```

Expected: `[100%] Built target RoboSDPDesktop`.

- [ ] **Step 3: Check whitespace**

Run:

```powershell
git diff --check
```

Expected: no whitespace errors. Existing CRLF warnings are acceptable only if they refer to pre-existing files touched by Git line-ending settings.

- [ ] **Step 4: Manual UI check**

Run the desktop executable from the build output. Open "数据追溯". Select a result row. Confirm:

```text
1. Detail panel shows run_id, raw_id, channel, frame_seq, raw sample range, and time range.
2. Result table row selection updates the raw waveform plot.
3. Raw waveform x-axis uses seconds.
4. Raw waveform only shows the selected result's raw_start_index to raw_end_index window.
5. RMS/DB/DropS/Warn fields shown in details match the selected table row.
```

- [ ] **Step 5: Final commit if any verification-only fixes were needed**

```powershell
git add modules/traceability apps/desktop-qt CMakeLists.txt
git commit -m "fix: complete traceability verification fixes"
```

Only run this commit if Step 1-4 required code fixes after the previous commits.

---

## Acceptance Criteria

- Every result row includes `run_id`, `frame_seq`, `channel`, `raw_start_index`, `raw_end_index`, `start_time_s`, and `end_time_s`.
- A result can be selected and mapped to one exact raw waveform window.
- The UI shows both result metrics and raw window context together.
- Algorithm metadata is stored in `calc_run`: algorithm name, version, window size, step size, parameter JSON, and parameter hash.
- The implementation does not require guessing raw data ranges from plot axes.
- Existing robot modules remain independent from traceability internals.

## Self-Review

- Spec coverage: The plan implements scheme B through raw session, calculation run, and calculation result records, plus UI selection from result to raw window.
- Placeholder scan: No task uses unresolved placeholder instructions. The only "later" scope is explicitly excluded from the first implementation or listed as optional.
- Type consistency: DTO names, namespace names, and service method names are consistent across tasks.
