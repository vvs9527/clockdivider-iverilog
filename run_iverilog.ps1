Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$iverilog = Get-Command iverilog -ErrorAction Stop
$vvp = Get-Command vvp -ErrorAction Stop

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$outDir = Join-Path $repoRoot "out"
$simOut = Join-Path $outDir "tb_clockdivider.out"

New-Item -ItemType Directory -Force -Path $outDir | Out-Null

Write-Host "Compiling with $($iverilog.Source)..."
& $iverilog.Source -g2012 -o $simOut `
    (Join-Path $repoRoot "clockdivider.v") `
    (Join-Path $repoRoot "tb_clockdivider.v")

Write-Host "Running simulation with $($vvp.Source)..."
Push-Location $outDir
try {
    & $vvp.Source ".\\tb_clockdivider.out"
} finally {
    Pop-Location
}

Write-Host "Done. Waveform is in $outDir"
