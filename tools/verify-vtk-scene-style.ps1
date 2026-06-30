$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$sceneBuilderPath = Join-Path $repoRoot 'apps/desktop-qt/widgets/vtk/VtkSceneBuilder.cpp'
$source = Get-Content -LiteralPath $sceneBuilderPath -Raw

$checks = @(
    @{
        Name = 'gradient background is enabled'
        Pattern = 'renderer->SetGradientBackground\(true\);'
    },
    @{
        Name = 'background bottom is saturated blue purple'
        Pattern = 'renderer->SetBackground\(0\.18,\s*0\.20,\s*1\.0\);'
    },
    @{
        Name = 'background top is near white'
        Pattern = 'renderer->SetBackground2\(0\.97,\s*0\.97,\s*1\.0\);'
    },
    @{
        Name = 'grid line color matches reference gray'
        Pattern = 'SetColor\(0\.29,\s*0\.29,\s*0\.34\)'
    },
    @{
        Name = 'minimal scene uses larger reference grid'
        Pattern = 'AddGroundGrid\(renderer,\s*5\.0,\s*0\.5\);'
    }
)

$failed = @()
foreach ($check in $checks)
{
    if ($source -notmatch $check.Pattern)
    {
        $failed += $check.Name
    }
}

if ($failed.Count -gt 0)
{
    Write-Error ("VTK scene style checks failed: " + ($failed -join '; '))
}

Write-Host "VTK scene style checks passed."
