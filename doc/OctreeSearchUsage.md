# OctreeSearch 调用方法

本文档说明如何在当前项目中使用 C# wrapper 进行 Octree 搜索，包括 KNN 搜索与半径搜索。

## 1. 前置条件

1. 已完成 native 层编译，且运行目录可找到 `PclDll.dll` 与 `PointCloudDll.dll`。
2. C# 项目已引用 wrapper 文件 `OctreeSearch.cs` 与 `PointCloudXYZ.cs`。
3. 点云对象类型为 `PointCloudSharp.PointCloudXYZ`。

## 2. API 一览

命名空间：`PclCSharp`

- `OctreeSearch.NearestKSearch(...)`
  - 功能：Octree K 近邻搜索
  - 返回值：实际命中数量
  - 输出：`indices`（邻居索引）、`sqrDistances`（平方距离）

- `OctreeSearch.RadiusSearch(...)`
  - 功能：Octree 半径搜索
  - 返回值：实际命中数量
  - 输出：`indices`（邻居索引）、`sqrDistances`（平方距离）

## 3. 最小调用示例

```csharp
using System;
using PclCSharp;
using PointCloudSharp;

public static class OctreeUsageDemo
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

        // 3) 设置八叉树分辨率（根据点云尺度调整）
        double resolution = 0.02;

        // 4) KNN 查询
        int foundK = OctreeSearch.NearestKSearch(
            cloud.PointCloudXYZPointer,
            resolution,
            qx, qy, qz,
            k: 10,
            out int[] knnIndices,
            out float[] knnDist2);

        Console.WriteLine($"Octree KNN 命中: {foundK}");
        if (foundK > 0)
        {
            Console.WriteLine($"Octree KNN 第1个邻居索引: {knnIndices[0]}, 距离^2: {knnDist2[0]}");
        }

        // 5) 半径查询
        int foundR = OctreeSearch.RadiusSearch(
            cloud.PointCloudXYZPointer,
            resolution,
            qx, qy, qz,
            radius: 0.05,
            maxNeighbors: 128,
            out int[] radiusIndices,
            out float[] radiusDist2);

        Console.WriteLine($"Octree Radius 命中: {foundR}");
        if (foundR > 0)
        {
            Console.WriteLine($"Octree Radius 第1个邻居索引: {radiusIndices[0]}, 距离^2: {radiusDist2[0]}");
        }
    }
}
```

## 4. 参数建议

1. `resolution`：优先设置为点云平均点间距到其 2 倍附近，然后微调。
2. `k`：建议从 8 到 30 试起。
3. `radius`：建议从目标点间平均间距的 2 到 5 倍试参。
4. `maxNeighbors`：建议给足上限，例如 64、128、256。

## 5. 常见问题

1. 返回 0 个邻居
   - 检查查询点坐标是否合理。
   - 检查 `resolution`、`radius` 是否过小，或点云是否为空。

2. 命中结果不稳定
   - 优先调整 `resolution`，过大或过小都可能影响查询效果。

3. 运行时报 DLL 加载失败
   - 检查 `PclDll.dll`、`PointCloudDll.dll` 及其依赖是否位于可搜索路径。

## 6. 相关源码位置

- C# wrapper: `src/PclSharpWrapper/PclCSharp/OctreeSearch.cs`
- Native 导出: `src/PclSharpWrapper/PclSharpWrapper/octree.h`
- Native 实现: `src/PclSharpWrapper/PclSharpWrapper/octree.cpp`
- Demo 参考: `demo/PclCSharpDemo/WpfHelixDemo/OctreeSearchCase.cs`
