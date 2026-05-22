# PclCSharp 构建方法（vcpkg）

本文档说明如何在 Windows x64 环境下，使用 vcpkg 构建 PclCSharp。

## 1. 前置条件

1. 操作系统：Windows x64
2. 安装 Visual Studio 2022（或 Build Tools），并包含 C++ 工具链
3. 安装 CMake（建议 3.20+）
4. 安装 Ninja（脚本会尝试自动安装）
5. 安装 Git（用于自动拉取 vcpkg）

## 2. 脚本入口

根目录脚本：`build.ps1`

该脚本现在使用 vcpkg 管理依赖，不再下载 PCL AllInOne installer。

## 3. 快速开始

在仓库根目录执行：

```powershell
.\build.ps1
```

默认行为：

1. 激活 MSVC x64 环境
2. 自动解析或拉取 vcpkg（优先参数、环境变量和常见路径）
3. 执行 `vcpkg install pcl --triplet x64-windows`
4. 使用 vcpkg toolchain 执行 CMake configure/build
5. 执行 MSBuild 构建 .NET 包装层
6. 收集运行时 DLL 到 `depend\x64`

## 4. 常用参数

### 指定 vcpkg 根目录

```powershell
.\build.ps1 -VcpkgRoot D:\pkg
```

### 指定 triplet

```powershell
.\build.ps1 -VcpkgTriplet x64-windows
```

### 指定 CMake 二进制目录

```powershell
.\build.ps1 -BuildDir build-ninja
```

说明：如果指定目录里已有非 Ninja 的 CMake 缓存（例如 Visual Studio 生成器），脚本会自动切换到 `build-ninja`，避免出现生成器不匹配错误。

### 跳过依赖安装（已安装过时）

```powershell
.\build.ps1 -SkipInstallDeps
```

### 只构建 C++ 层

```powershell
.\build.ps1 -SkipDotNet -SkipCollect
```

### 跳过 C++ 或 .NET 构建

```powershell
.\build.ps1 -SkipCpp
.\build.ps1 -SkipDotNet
```

## 5. 输出位置

1. C++ / C# 主要输出目录：`bin`
2. 运行时依赖收集目录：`depend\x64`

## 6. 脚本内部使用的关键 CMake 参数

构建脚本会传入以下参数：

```text
-G Ninja
-DCMAKE_BUILD_TYPE=<Config>
-DCMAKE_TOOLCHAIN_FILE=<VCPKG_ROOT>/scripts/buildsystems/vcpkg.cmake
-DVCPKG_TARGET_TRIPLET=<VcpkgTriplet>
-DVCPKG_APPLOCAL_DEPS=ON
```

## 7. 常见问题

### 1) 找不到 MSVC 工具链

请确认安装了 Visual Studio C++ 工作负载，或在 Developer PowerShell 中运行脚本。

### 2) vcpkg 自动拉取失败

检查 Git 是否可用，并确认网络可访问 `https://github.com/microsoft/vcpkg`。

### 3) Ninja 不可用

脚本会尝试通过 winget 安装；若失败请手动安装 Ninja。

### 4) 运行时缺少 DLL

先确认构建成功，再检查 `depend\x64` 是否生成并包含依赖 DLL。

## 8. 打包发布

构建完成后可使用 `pack.ps1` 打包产物，输出 runtime/lib/demo 三个 zip。
