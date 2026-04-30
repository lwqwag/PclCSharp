<#
.SYNOPSIS
    Local build script for PclCSharp (Windows x64).

.DESCRIPTION
    Mirrors the GitHub Actions CI workflow (build.yml).
    Steps performed:
      1. Locate and activate an MSVC x64 toolchain.
      2. Optionally download and silently install PCL 1.14.1.
      3. Locate PCLConfig.cmake and derive PCL_DIR.
      4. Configure and build the C++ DLLs with CMake + Ninja.
      5. Build the .NET solution with MSBuild.
      6. Collect PCL / VTK runtime DLLs into depend\x64.

.PARAMETER PCLRoot
    Path to an existing PCL installation.
    Defaults to the PCL_ROOT environment variable, then C:\PCL.

.PARAMETER Config
    CMake / MSBuild build configuration.  Default: Release.

.PARAMETER SkipInstallPCL
    Skip the PCL download-and-install step even when PCL is not found at
    the expected location.

.PARAMETER SkipCpp
    Skip the CMake configure + build step (C++ DLLs).

.PARAMETER SkipDotNet
    Skip the MSBuild step (.NET solution).

.PARAMETER SkipCollect
    Skip collecting runtime DLLs into depend\x64.

.EXAMPLE
    # Full build, auto-install PCL if missing:
    .\build.ps1

.EXAMPLE
    # Use a custom PCL root, Release build:
    .\build.ps1 -PCLRoot D:\SDK\PCL

.EXAMPLE
    # Only rebuild the C++ layer:
    .\build.ps1 -SkipDotNet -SkipCollect
#>
[CmdletBinding()]
param(
    [string] $PCLRoot        = "",
    [string] $Config         = "Release",
    [switch] $SkipInstallPCL,
    [switch] $SkipCpp,
    [switch] $SkipDotNet,
    [switch] $SkipCollect
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

function Find-File([string]$root, [string]$filter) {
    Get-ChildItem $root -Recurse -Filter $filter -ErrorAction SilentlyContinue |
        Select-Object -First 1
}

# ---------------------------------------------------------------------------
# Resolve repository root (script lives at repo root)
# ---------------------------------------------------------------------------
$RepoRoot = $PSScriptRoot
if (-not $RepoRoot) { $RepoRoot = (Get-Location).Path }

# ---------------------------------------------------------------------------
# Step 1 – Activate MSVC x64 toolchain
# ---------------------------------------------------------------------------
Write-Step "Setting up MSVC x64 environment"

if (-not (Get-Command "cl.exe" -ErrorAction SilentlyContinue)) {
    # Search for vcvarsall.bat in common VS install locations
    $vswherePath = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswherePath) {
        $vsPath = & $vswherePath -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null
    } else {
        $vsPath = $null
    }

    $vcvarsall = $null
    if ($vsPath) {
        $candidate = Join-Path $vsPath "VC\Auxiliary\Build\vcvarsall.bat"
        if (Test-Path $candidate) { $vcvarsall = $candidate }
    }

    if (-not $vcvarsall) {
        # Fallback: scan common VS paths
        $fallbacks = @(
            "${env:ProgramFiles}\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvarsall.bat",
            "${env:ProgramFiles}\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvarsall.bat",
            "${env:ProgramFiles}\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat",
            "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2019\Enterprise\VC\Auxiliary\Build\vcvarsall.bat",
            "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvarsall.bat",
            "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat"
        )
        foreach ($fb in $fallbacks) {
            if (Test-Path $fb) { $vcvarsall = $fb; break }
        }
    }

    if (-not $vcvarsall) {
        Write-Error "Could not locate vcvarsall.bat. Please open a 'Developer Command Prompt for VS' or install Visual Studio with C++ workload."
        exit 1
    }

    Write-Host "  Found vcvarsall.bat: $vcvarsall"
    Write-Host "  Activating x64 toolchain..."

    # Invoke vcvarsall and capture the resulting environment variables
    $envDump = cmd.exe /c "`"$vcvarsall`" x64 > NUL 2>&1 && set"
    foreach ($line in $envDump) {
        if ($line -match "^([^=]+)=(.*)$") {
            [System.Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], "Process")
        }
    }
    Write-Host "  MSVC x64 environment activated."
} else {
    Write-Host "  cl.exe already in PATH – skipping vcvarsall.bat."
}

# Verify Ninja is available
if (-not (Get-Command "ninja" -ErrorAction SilentlyContinue)) {
    Write-Warning "ninja not found in PATH. Attempting to install via winget..."
    winget install --id Ninja-build.Ninja -e --silent 2>$null
    if (-not (Get-Command "ninja" -ErrorAction SilentlyContinue)) {
        Write-Error "Ninja is required but could not be found or installed.`nInstall it from https://ninja-build.org/ or via: choco install ninja"
        exit 1
    }
}
Write-Host "  Ninja: $(ninja --version)"

# ---------------------------------------------------------------------------
# Step 2 – Resolve PCL root
# ---------------------------------------------------------------------------
Write-Step "Resolving PCL installation"

if (-not $PCLRoot) {
    if ($env:PCL_ROOT) {
        $PCLRoot = $env:PCL_ROOT
        Write-Host "  Using PCL_ROOT env var: $PCLRoot"
    } else {
        $PCLRoot = "C:\PCL"
    }
}

$pclInclude = Join-Path $PCLRoot "include"
if (-not (Test-Path $pclInclude)) {
    if ($SkipInstallPCL) {
        Write-Error "PCL not found at '$PCLRoot' and -SkipInstallPCL was set. Aborting."
        exit 1
    }

    Write-Step "Downloading PCL 1.14.1 AllInOne installer"
    $installerUrl  = "https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.14.1/PCL-1.14.1-AllInOne-msvc2022-win64.exe"
    $installerPath = Join-Path $env:TEMP "PCL-1.14.1-installer.exe"

    Write-Host "  Downloading to $installerPath ..."
    Invoke-WebRequest -Uri $installerUrl -OutFile $installerPath -TimeoutSec 600
    Write-Host "  Download complete. Size: $([math]::Round((Get-Item $installerPath).Length / 1MB, 1)) MB"

    Write-Step "Installing PCL 1.14.1 to $PCLRoot"
    $proc = Start-Process -Wait -PassThru -FilePath $installerPath -ArgumentList "/S /D=$PCLRoot"
    Write-Host "  Installer exit code: $($proc.ExitCode)"

    if (-not (Test-Path $pclInclude)) {
        Write-Error "PCL installation appears to have failed – '$pclInclude' not found."
        exit 1
    }
    Write-Host "  PCL installed successfully."
} else {
    Write-Host "  PCL found at: $PCLRoot"
}

# ---------------------------------------------------------------------------
# Step 3 – Locate PCLConfig.cmake → PCL_DIR
# ---------------------------------------------------------------------------
Write-Step "Locating PCLConfig.cmake"

$candidates = @(
    (Join-Path $PCLRoot "cmake"),
    (Join-Path $PCLRoot "lib\cmake\pcl")
)
$PCLDir = $null
foreach ($c in $candidates) {
    if (Test-Path (Join-Path $c "PCLConfig.cmake")) { $PCLDir = $c; break }
}
if (-not $PCLDir) {
    $found = Find-File $PCLRoot "PCLConfig.cmake"
    if ($found) { $PCLDir = $found.DirectoryName }
}
if (-not $PCLDir) {
    Write-Error "Could not locate PCLConfig.cmake under '$PCLRoot'."
    exit 1
}
Write-Host "  PCL_DIR = $PCLDir"
$env:PCL_DIR  = $PCLDir
$env:PCL_ROOT = $PCLRoot

# ---------------------------------------------------------------------------
# Step 4 – Configure & build C++ DLLs with CMake
# ---------------------------------------------------------------------------
if (-not $SkipCpp) {
    Write-Step "Configuring C++ projects with CMake"

    $buildDir = Join-Path $RepoRoot "build"
    cmake -S $RepoRoot -B $buildDir `
          -G Ninja `
          "-DCMAKE_BUILD_TYPE=$Config" `
          "-DPCL_DIR=$PCLDir" `
          "-DPCL_ROOT=$PCLRoot"
    if ($LASTEXITCODE -ne 0) { Write-Error "CMake configure failed."; exit 1 }

    Write-Step "Building C++ DLLs"
    cmake --build $buildDir --config $Config --parallel
    if ($LASTEXITCODE -ne 0) { Write-Error "CMake build failed."; exit 1 }

    Write-Host "  C++ DLLs built → $(Join-Path $RepoRoot 'bin')"
} else {
    Write-Host "  [SkipCpp] Skipping CMake configure + build."
}

# ---------------------------------------------------------------------------
# Step 5 – Build .NET solution with MSBuild
# ---------------------------------------------------------------------------
if (-not $SkipDotNet) {
    Write-Step "Building .NET solution with MSBuild"

    # Locate MSBuild
    $msbuild = $null
    $vswherePath2 = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswherePath2) {
        $msbuildPath = & $vswherePath2 -latest -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe 2>$null |
                        Select-Object -First 1
        if ($msbuildPath -and (Test-Path $msbuildPath)) { $msbuild = $msbuildPath }
    }
    if (-not $msbuild) {
        $msbuild = Get-Command "msbuild.exe" -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source
    }
    if (-not $msbuild) {
        Write-Error "MSBuild not found. Open a Visual Studio Developer PowerShell or install Build Tools."
        exit 1
    }
    Write-Host "  MSBuild: $msbuild"

    $sln = Join-Path $RepoRoot "src\PclSharpWrapper\PclSharpWrapper.sln"
    & $msbuild $sln /p:Configuration=$Config /p:Platform=x64 /m /v:minimal
    if ($LASTEXITCODE -ne 0) { Write-Error "MSBuild failed."; exit 1 }

    Write-Host "  .NET solution built successfully."
} else {
    Write-Host "  [SkipDotNet] Skipping MSBuild step."
}

# ---------------------------------------------------------------------------
# Step 6 – Collect runtime DLLs into depend\x64
# ---------------------------------------------------------------------------
if (-not $SkipCollect) {
    Write-Step "Collecting runtime DLLs into depend\x64"

    $destDir = Join-Path $RepoRoot "depend\x64"
    if (-not (Test-Path $destDir)) { New-Item -ItemType Directory -Path $destDir | Out-Null }

    # PCL release DLLs
    $pclBin = Join-Path $PCLRoot "bin"
    if (Test-Path $pclBin) {
        $copied = 0
        Get-ChildItem $pclBin -Filter "pcl_*_release.dll" | ForEach-Object {
            Copy-Item $_.FullName -Destination $destDir -Force
            $copied++
        }
        Write-Host "  Copied $copied PCL DLL(s) from $pclBin"
    } else {
        Write-Warning "PCL bin directory not found: $pclBin"
    }

    # VTK runtime DLLs
    $vtkModules = @(
        "vtkCommonCore", "vtkCommonDataModel", "vtkCommonExecutionModel",
        "vtkCommonMath", "vtkCommonMisc", "vtkCommonSystem", "vtkCommonTransforms",
        "vtkIOCore", "vtkIOGeometry", "vtksys", "vtkzlib", "vtklz4"
    )
    $vtkBinCandidates = @(
        (Join-Path $PCLRoot "3rdParty\VTK\bin"),
        (Join-Path $PCLRoot "bin")
    )
    $vtkBin = $vtkBinCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1

    if ($vtkBin) {
        $copied = 0
        foreach ($mod in $vtkModules) {
            $dll = Get-ChildItem $vtkBin -Filter "${mod}-*.dll" -ErrorAction SilentlyContinue |
                   Where-Object { $_.Name -notlike "*-gd-*" } |
                   Select-Object -First 1
            if ($dll) {
                Copy-Item $dll.FullName -Destination $destDir -Force
                $copied++
            }
        }
        Write-Host "  Copied $copied VTK DLL(s) from $vtkBin"
    } else {
        Write-Warning "VTK bin directory not found under PCL root – VTK DLLs not collected."
    }

    # Image-library DLLs (libpng16, libjpeg, libtiff, freetype, etc.) that are
    # required at runtime by PointCloudDll / VTK image I/O.
    $imgPatterns = @("libpng16.dll", "libjpeg*.dll", "libtiff*.dll",
                     "freetype.dll", "libexpat.dll", "double-conversion.dll")
    $imgSearchDirs = @(
        (Join-Path $PCLRoot "bin"),
        (Join-Path $PCLRoot "3rdParty\VTK\bin"),
        (Join-Path $PCLRoot "3rdParty\libpng\bin"),
        (Join-Path $PCLRoot "3rdParty\libjpeg\bin"),
        (Join-Path $PCLRoot "3rdParty\libtiff\bin")
    ) | Where-Object { Test-Path $_ }

    $imgCopied = 0
    foreach ($imgDir in $imgSearchDirs) {
        foreach ($pat in $imgPatterns) {
            Get-ChildItem $imgDir -Filter $pat -ErrorAction SilentlyContinue |
                Where-Object { $_.Name -notlike "*-gd-*" } |
                ForEach-Object {
                    Copy-Item $_.FullName -Destination $destDir -Force
                    $imgCopied++
                }
        }
    }
    if ($imgCopied -gt 0) {
        Write-Host "  Copied $imgCopied image-library DLL(s) (libpng16, libjpeg, libtiff, …)"
    }

    Write-Host ""
    Write-Host "  Runtime DLLs in depend\x64:"
    Get-ChildItem $destDir | Sort-Object Name | ForEach-Object { Write-Host "    $($_.Name)" }
} else {
    Write-Host "  [SkipCollect] Skipping DLL collection step."
}

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
Write-Host ""
Write-Host "Build complete!" -ForegroundColor Green
Write-Host "  C++ DLLs  : $(Join-Path $RepoRoot 'bin')"
Write-Host "  depend\x64: $(Join-Path $RepoRoot 'depend\x64')"
