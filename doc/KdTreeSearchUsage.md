# KdTreeSearch 调用方法

本文档说明如何在当前项目中使用 C# wrapper 进行 KDTree 搜索，包括 KNN 搜索与半径搜索。

## 1. 前置条件

1. 已完成 native 层编译，且运行目录可找到 `PclDll.dll` 与 `PointCloudDll.dll`。
2. C# 项目已引用 wrapper 文件 `KdTreeSearch.cs` 与 `PointCloudXYZ.cs`。
3. 点云对象类型为 `PointCloudSharp.PointCloudXYZ`。

## 2. API 一览

命名空间：`PclCSharp`

- `KdTreeSearch.NearestKSearch(...)`
  - 功能：K 近邻搜索
  - 返回值：实际命中数量
  - 输出：`indices`（邻居索引）、`sqrDistances`（平方距离）

- `KdTreeSearch.RadiusSearch(...)`
  - 功能：半径搜索
  - 返回值：实际命中数量
  - 输出：`indices`（邻居索引）、`sqrDistances`（平方距离）

## 3. 最小调用示例

```csharp
using System;
using PclCSharp;
using PointCloudSharp;

public static class KdTreeUsageDemo
{
    public static void Run()
    {
        // 1) 载入点云
        var cloud = new PointCloudXYZ(@"source/plyFiles/your_cloud.ply");
        if (cloud.Size == 0)
        {
            Console.WriteLine("点云为空");
            return;
        }

        // 2) 选取查询点（这里使用第一个点）
        double qx = cloud.GetX(0);
        double qy = cloud.GetY(0);
        double qz = cloud.GetZ(0);

        // 3) KNN 查询
        int foundK = KdTreeSearch.NearestKSearch(
            cloud.PointCloudXYZPointer,
            qx, qy, qz,
            k: 10,
            out int[] knnIndices,
            out float[] knnDist2);

        Console.WriteLine($"KNN 命中: {foundK}");
        if (foundK > 0)
        {
            Console.WriteLine($"KNN 第1个邻居索引: {knnIndices[0]}, 距离^2: {knnDist2[0]}");
        }

        // 4) 半径查询
        int foundR = KdTreeSearch.RadiusSearch(
            cloud.PointCloudXYZPointer,
            qx, qy, qz,
            radius: 0.05,
            maxNeighbors: 128,
            out int[] radiusIndices,
            out float[] radiusDist2);

        Console.WriteLine($"Radius 命中: {foundR}");
        if (foundR > 0)
        {
            Console.WriteLine($"Radius 第1个邻居索引: {radiusIndices[0]}, 距离^2: {radiusDist2[0]}");
        }
    }
}
```

## 4. 参数建议

1. `k`：建议从 8 到 30 试起。
2. `radius`：与点云尺度相关，先按目标点间平均间距的 2 到 5 倍试参。
3. `maxNeighbors`：建议给足上限，例如 64、128、256，避免截断有效邻居。

## 5. 常见问题

1. 返回 0 个邻居
   - 检查查询点坐标是否合理。
   - 检查 `radius` 是否过小，或点云是否为空。

2. 运行时报 DLL 加载失败
   - 检查 `PclDll.dll`、`PointCloudDll.dll` 及其依赖是否位于可搜索路径。

3. 数组长度与命中数量
   - wrapper 内部会根据实际命中数裁剪输出数组长度。
   - 业务代码应以函数返回值或数组实际长度为准。

## 6. 相关源码位置

- C# wrapper: `src/PclSharpWrapper/PclCSharp/KdTreeSearch.cs`
- Native 导出: `src/PclSharpWrapper/PclSharpWrapper/kdtree.h`
- Native 实现: `src/PclSharpWrapper/PclSharpWrapper/kdtree.cpp`
- Demo 参考: `demo/PclCSharpDemo/WpfHelixDemo/KdTreeSearchCase.cs`
