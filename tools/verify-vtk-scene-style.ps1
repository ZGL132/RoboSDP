$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$sceneBuilderPath = Join-Path $repoRoot 'apps/desktop-qt/widgets/vtk/VtkSceneBuilder.cpp'
$robotVtkViewPath = Join-Path $repoRoot 'apps/desktop-qt/widgets/vtk/RobotVtkView.cpp'
$sceneBuilderSource = Get-Content -LiteralPath $sceneBuilderPath -Raw
$robotVtkViewSource = Get-Content -LiteralPath $robotVtkViewPath -Raw

$checks = @(
    @{
        Name = 'gradient background is enabled'
        Pattern = 'renderer->SetGradientBackground\(true\);'
        Source = $sceneBuilderSource
    },
    @{
        Name = 'background bottom is saturated blue purple'
        Pattern = 'renderer->SetBackground\(0\.18,\s*0\.20,\s*1\.0\);'
        Source = $sceneBuilderSource
    },
    @{
        Name = 'background top is near white'
        Pattern = 'renderer->SetBackground2\(0\.97,\s*0\.97,\s*1\.0\);'
        Source = $sceneBuilderSource
    },
    @{
        Name = 'grid line color matches reference gray'
        Pattern = 'SetColor\(0\.29,\s*0\.29,\s*0\.34\)'
        Source = $sceneBuilderSource
    },
    @{
        Name = 'minimal scene uses larger reference grid'
        Pattern = 'AddGroundGrid\(renderer,\s*5\.0,\s*0\.5\);'
        Source = $sceneBuilderSource
    },
    @{
        Name = 'render viewport border is removed'
        Pattern = 'QFrame#renderViewportFrame\{background:#000000;border:none;\}'
        Source = $robotVtkViewSource
    },
    @{
        Name = 'analysis layer title text is black'
        Pattern = 'QLabel#analysisLayerTitle\{color:#000000;'
        Source = $robotVtkViewSource
    },
    @{
        Name = 'analysis layer checkbox text is black'
        Pattern = 'QCheckBox\{color:#000000;'
        Source = $robotVtkViewSource
    }
)

$failed = @()
foreach ($check in $checks)
{
    if ($check.Source -notmatch $check.Pattern)
    {
        $failed += $check.Name
    }
}

if ($failed.Count -gt 0)
{
    Write-Error ("VTK scene style checks failed: " + ($failed -join '; '))
}

Write-Host "VTK scene style checks passed."
