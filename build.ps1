<#
.SYNOPSIS
    Local build script for PclCSharp (Windows x64, vcpkg based).

.DESCRIPTION
    Mirrors CI behavior using vcpkg as the dependency source.
    Steps performed:
      1. Locate and activate an MSVC x64 toolchain.
      2. Resolve (or clone/bootstrap) vcpkg.
      3. Install required dependencies with vcpkg (pcl).
      4. Configure and build the C++ DLLs with CMake + Ninja + vcpkg toolchain.
      5. Build the .NET solution with MSBuild.
      6. Collect runtime DLLs into depend\x64.

.PARAMETER VcpkgRoot
    Path to an existing vcpkg root folder.
    Defaults to VCPKG_ROOT env var, then common local paths.

.PARAMETER VcpkgTriplet
    vcpkg triplet. Default: x64-windows.

.PARAMETER Config
    CMake / MSBuild build configuration. Default: Release.

.PARAMETER SkipInstallDeps
    Skip the vcpkg install step.

.PARAMETER SkipCpp
    Skip the CMake configure + build step (C++ DLLs).

.PARAMETER SkipDotNet
    Skip the MSBuild step (.NET solution).

.PARAMETER SkipCollect
    Skip collecting runtime DLLs into depend\x64.

.EXAMPLE
    # Full build, auto-locate or clone vcpkg if needed:
    .\build.ps1

.EXAMPLE
    # Use a custom vcpkg root and triplet:
    .\build.ps1 -VcpkgRoot D:\pkg -VcpkgTriplet x64-windows

.EXAMPLE
    # Only rebuild the C++ layer:
    .\build.ps1 -SkipDotNet -SkipCollect
#>
[CmdletBinding()]
param(
    [string] $VcpkgRoot      = "",
    [string] $VcpkgTriplet   = "x64-windows",
    [string] $BuildDir       = "build",
    [string] $Config         = "Release",
    [switch] $SkipInstallDeps,
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

function Resolve-Vcpkg([string]$RepoRoot, [string]$VcpkgRootArg) {
    $roots = @()
    if ($VcpkgRootArg) { $roots += $VcpkgRootArg }
    if ($env:VCPKG_ROOT) { $roots += $env:VCPKG_ROOT }
    $roots += @(
        (Join-Path $RepoRoot ".vcpkg"),
        (Join-Path $RepoRoot "vcpkg"),
        "D:\pkg",
        "C:\vcpkg"
    )

    $roots = $roots | Where-Object { $_ } | Select-Object -Unique

    foreach ($r in $roots) {
        $exe = Join-Path $r "vcpkg.exe"
        if (Test-Path $exe) {
            return @{
                Root = $r
                Exe = $exe
                Toolchain = (Join-Path $r "scripts\buildsystems\vcpkg.cmake")
            }
        }
    }

    # Auto clone vcpkg into repo-local .vcpkg if not found.
    $cloneRoot = Join-Path $RepoRoot ".vcpkg"
    if (-not (Test-Path $cloneRoot)) {
        if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
            Write-Error "vcpkg 未找到，且 git 不可用，无法自动拉取 vcpkg。请安装 git 或手动指定 -VcpkgRoot。"
            exit 1
        }
        Write-Step "Cloning vcpkg into $cloneRoot"
        git clone https://github.com/microsoft/vcpkg $cloneRoot
        if ($LASTEXITCODE -ne 0) {
            Write-Error "git clone vcpkg 失败。"
            exit 1
        }
    }

    $bootstrap = Join-Path $cloneRoot "bootstrap-vcpkg.bat"
    $exe = Join-Path $cloneRoot "vcpkg.exe"
    if (-not (Test-Path $exe)) {
        if (-not (Test-Path $bootstrap)) {
            Write-Error "无法在 $cloneRoot 找到 bootstrap-vcpkg.bat。"
            exit 1
        }
        Write-Step "Bootstrapping vcpkg"
        cmd.exe /c "`"$bootstrap`""
        if ($LASTEXITCODE -ne 0 -or -not (Test-Path $exe)) {
            Write-Error "vcpkg bootstrap 失败。"
            exit 1
        }
    }

    return @{
        Root = $cloneRoot
        Exe = $exe
        Toolchain = (Join-Path $cloneRoot "scripts\buildsystems\vcpkg.cmake")
    }
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
# Step 2 – Resolve vcpkg
# ---------------------------------------------------------------------------
Write-Step "Resolving vcpkg"
$vcpkg = Resolve-Vcpkg -RepoRoot $RepoRoot -VcpkgRootArg $VcpkgRoot

if (-not (Test-Path $vcpkg.Toolchain)) {
    Write-Error "vcpkg toolchain 文件不存在: $($vcpkg.Toolchain)"
    exit 1
}

$VcpkgRoot = $vcpkg.Root
$env:VCPKG_ROOT = $VcpkgRoot
Write-Host "  VCPKG_ROOT = $VcpkgRoot"
Write-Host "  vcpkg.exe  = $($vcpkg.Exe)"
Write-Host "  triplet    = $VcpkgTriplet"

# ---------------------------------------------------------------------------
# Step 3 – Install dependencies with vcpkg
# ---------------------------------------------------------------------------
if (-not $SkipInstallDeps) {
    Write-Step "Installing dependencies via vcpkg"
    & $vcpkg.Exe install pcl --triplet $VcpkgTriplet
    if ($LASTEXITCODE -ne 0) {
        Write-Error "vcpkg install 失败。"
        exit 1
    }
} else {
    Write-Host "  [SkipInstallDeps] Skipping vcpkg dependency installation."
}

# ---------------------------------------------------------------------------
# Step 4 – Configure & build C++ DLLs with CMake
# ---------------------------------------------------------------------------
if (-not $SkipCpp) {
    Write-Step "Configuring C++ projects with CMake"

    if ([System.IO.Path]::IsPathRooted($BuildDir)) {
        $buildDir = $BuildDir
    } else {
        $buildDir = Join-Path $RepoRoot $BuildDir
    }

    # 如果已有缓存不是 Ninja 生成器，自动切换到独立目录，避免冲突。
    $cachePath = Join-Path $buildDir "CMakeCache.txt"
    if (Test-Path $cachePath) {
        $cacheLine = Get-Content $cachePath -ErrorAction SilentlyContinue |
            Where-Object { $_ -like "CMAKE_GENERATOR:*" } |
            Select-Object -First 1

        if ($cacheLine -and $cacheLine -match "=(.*)$") {
            $existingGenerator = $Matches[1].Trim()
            if ($existingGenerator -ne "Ninja") {
                $fallbackBuildDir = Join-Path $RepoRoot "build-ninja"
                Write-Warning "Detected existing generator '$existingGenerator' in '$buildDir'. Switching to '$fallbackBuildDir' for Ninja build."
                $buildDir = $fallbackBuildDir
            }
        }
    }

    Write-Host "  CMake binary dir: $buildDir"
    cmake -S $RepoRoot -B $buildDir `
          -G Ninja `
          "-DCMAKE_BUILD_TYPE=$Config" `
        "-DCMAKE_TOOLCHAIN_FILE=$($vcpkg.Toolchain)" `
        "-DVCPKG_TARGET_TRIPLET=$VcpkgTriplet" `
        "-DVCPKG_APPLOCAL_DEPS=ON"
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

    # Copy runtime DLLs directly from vcpkg installed/<triplet>/bin.
    $vcpkgBin = Join-Path $VcpkgRoot "installed\$VcpkgTriplet\bin"
    if (Test-Path $vcpkgBin) {
        $vcpkgCopied = 0
        Get-ChildItem $vcpkgBin -Filter "*.dll" | ForEach-Object {
            Copy-Item $_.FullName -Destination $destDir -Force
            $vcpkgCopied++
        }
        Write-Host "  Copied $vcpkgCopied DLL(s) from $vcpkgBin"
    } else {
        Write-Warning "vcpkg bin 目录不存在: $vcpkgBin"
    }

    Write-Host ""
    Write-Host "  Runtime DLLs in depend\x64:"
    Get-ChildItem $destDir | Sort-Object Name | ForEach-Object { Write-Host "    $($_.Name)" }

    # Also copy any applocal dependencies that CMake placed next to outputs.
    $binDir = Join-Path $RepoRoot "bin"
    if (Test-Path $binDir) {
        $binCopied = 0
        Get-ChildItem $binDir -Filter "*.dll" |
            Where-Object { $_.Name -notmatch "^(PclDll|PointCloudDll)\.dll$" } |
            ForEach-Object {
                Copy-Item $_.FullName -Destination $destDir -Force
                $binCopied++
            }
        if ($binCopied -gt 0) {
            Write-Host "  Copied $binCopied DLL(s) from bin\ (vcpkg applocal dependencies)"
        }
    }
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
