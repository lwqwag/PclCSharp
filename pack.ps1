<#
.SYNOPSIS
    Package PclCSharp build outputs into runtime / lib / demo zip archives.

.DESCRIPTION
    Produces three zip archives that correspond to the three publish components:

      runtime-<Version>-win-x64.zip  – native C++ DLLs + PCL/VTK/Boost runtime DLLs
      lib-<Version>.zip              – .NET managed wrapper DLLs (PclCSharp, PointCloudSharpDll)
      demo-<Version>-win-x64.zip    – WpfHelixDemo self-contained publish + runtime DLLs

    The script is idempotent: it can be run multiple times and will overwrite
    any previously created archives.

.PARAMETER Version
    Version string to embed in the archive names (e.g. "1.0.0").
    Defaults to the GITHUB_REF_NAME environment variable (with a leading "v"
    stripped), or "0.0.0" if neither is set.

.PARAMETER OutDir
    Directory to write the zip archives into.  Defaults to "dist" under the
    repository root.

.PARAMETER RepoRoot
    Repository root directory.  Defaults to the directory that contains this
    script.

.EXAMPLE
    .\pack.ps1 -Version 1.2.3

.EXAMPLE
    .\pack.ps1 -Version 1.2.3 -OutDir C:\publish
#>
[CmdletBinding()]
param(
    [string] $Version  = "",
    [string] $OutDir   = "dist",
    [string] $RepoRoot = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
function Write-Step([string]$msg) {
    Write-Host ""
    Write-Host "==> $msg" -ForegroundColor Cyan
}

function New-ZipFrom([string]$srcDir, [string]$zipPath) {
    if (Test-Path $zipPath) { Remove-Item $zipPath -Force }
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    [System.IO.Compression.ZipFile]::CreateFromDirectory($srcDir, $zipPath)
    $size = (Get-Item $zipPath).Length / 1MB
    Write-Host ("  Created: {0}  ({1:N1} MB)" -f $zipPath, $size)
}

# ---------------------------------------------------------------------------
# Resolve paths / version
# ---------------------------------------------------------------------------
if (-not $RepoRoot) { $RepoRoot = $PSScriptRoot }
$RepoRoot = (Resolve-Path $RepoRoot).Path

if (-not $Version) {
    $ref = $env:GITHUB_REF_NAME
    if ($ref) { $Version = $ref -replace '^v', '' }
    else       { $Version = "0.0.0" }
}

$OutDir = Join-Path $RepoRoot $OutDir
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir | Out-Null }
$OutDir = (Resolve-Path $OutDir).Path

Write-Host "Version  : $Version"
Write-Host "RepoRoot : $RepoRoot"
Write-Host "OutDir   : $OutDir"

$binDir      = Join-Path $RepoRoot "bin"
$dependDir   = Join-Path $RepoRoot "depend\x64"
$demoPublish = Join-Path $RepoRoot "demo\PclCSharpDemo\WpfHelixDemo\publish"

# ===========================================================================
# 1. runtime-<version>-win-x64.zip
#    Contents: native C++ wrapper DLLs + PCL/VTK/Boost/FLANN runtime DLLs
# ===========================================================================
Write-Step "Packaging runtime"

$runtimeStage = Join-Path $env:TEMP "pclcsharp-runtime-$Version"
if (Test-Path $runtimeStage) { Remove-Item $runtimeStage -Recurse -Force }
New-Item -ItemType Directory -Path $runtimeStage | Out-Null

# Native C++ DLLs produced by CMake (PclSharpWrapper.dll, PointCloudDll.dll)
if (Test-Path $binDir) {
    Get-ChildItem $binDir -Filter "*.dll" |
        Where-Object { $_.Name -match "^(PclSharpWrapper|PointCloudDll)" } |
        Copy-Item -Destination $runtimeStage -Force
}

# PCL / VTK / Boost / FLANN etc. runtime DLLs collected into depend\x64
if (Test-Path $dependDir) {
    Get-ChildItem $dependDir -Filter "*.dll" |
        Copy-Item -Destination $runtimeStage -Force
}

Write-Host "  Files in runtime package: $((Get-ChildItem $runtimeStage).Count)"
$runtimeZip = Join-Path $OutDir "runtime-$Version-win-x64.zip"
New-ZipFrom $runtimeStage $runtimeZip
Remove-Item $runtimeStage -Recurse -Force

# ===========================================================================
# 2. lib-<version>.zip
#    Contents: managed .NET wrapper DLLs (PclCSharp.dll, PointCloudSharpDll.dll)
# ===========================================================================
Write-Step "Packaging lib"

$libStage = Join-Path $env:TEMP "pclcsharp-lib-$Version"
if (Test-Path $libStage) { Remove-Item $libStage -Recurse -Force }
New-Item -ItemType Directory -Path $libStage | Out-Null

if (Test-Path $binDir) {
    # Managed DLLs
    Get-ChildItem $binDir -Filter "*.dll" |
        Where-Object { $_.Name -match "^(PclCSharp|PointCloudSharpDll)" } |
        Copy-Item -Destination $libStage -Force

    # PDB files (optional – present in Release builds)
    Get-ChildItem $binDir -Filter "*.pdb" |
        Where-Object { $_.Name -match "^(PclCSharp|PointCloudSharpDll)" } |
        Copy-Item -Destination $libStage -Force
}

Write-Host "  Files in lib package: $((Get-ChildItem $libStage).Count)"
$libZip = Join-Path $OutDir "lib-$Version.zip"
New-ZipFrom $libStage $libZip
Remove-Item $libStage -Recurse -Force

# ===========================================================================
# 3. demo-<version>-win-x64.zip
#    Contents: WpfHelixDemo self-contained publish output + runtime DLLs
# ===========================================================================
Write-Step "Packaging demo"

$demoStage = Join-Path $env:TEMP "pclcsharp-demo-$Version"
if (Test-Path $demoStage) { Remove-Item $demoStage -Recurse -Force }
New-Item -ItemType Directory -Path $demoStage | Out-Null

# Self-contained dotnet publish output for WpfHelixDemo
if (Test-Path $demoPublish) {
    Copy-Item "$demoPublish\*" -Destination $demoStage -Recurse -Force
} else {
    Write-Warning "Demo publish output not found at: $demoPublish"
}

# Copy native C++ DLLs into the demo package so it is self-contained
if (Test-Path $binDir) {
    Get-ChildItem $binDir -Filter "*.dll" |
        Where-Object { $_.Name -match "^(PclSharpWrapper|PointCloudDll)" } |
        Copy-Item -Destination $demoStage -Force
}

# Copy PCL/VTK/Boost runtime DLLs so the demo runs without separate installation
if (Test-Path $dependDir) {
    Get-ChildItem $dependDir -Filter "*.dll" |
        Copy-Item -Destination $demoStage -Force
}

Write-Host "  Files in demo package: $((Get-ChildItem $demoStage).Count)"
$demoZip = Join-Path $OutDir "demo-$Version-win-x64.zip"
New-ZipFrom $demoStage $demoZip
Remove-Item $demoStage -Recurse -Force

# ===========================================================================
# Summary
# ===========================================================================
Write-Host ""
Write-Host "Release packages written to: $OutDir" -ForegroundColor Green
Get-ChildItem $OutDir -Filter "*.zip" |
    Select-Object Name, @{N="Size (MB)"; E={ "{0:N1}" -f ($_.Length / 1MB) }} |
    Format-Table -AutoSize
