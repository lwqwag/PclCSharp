using System;
using PclCSharp;
using PointCloudSharp;

namespace WpfHelixDemo
{
    internal static class KdTreeSearchCase
    {
        // 该方法用于演示 C# wrapper 的 KDTree 查询调用方式。
        public static void RunBasicCase(PointCloudXYZ cloud)
        {
            if (cloud == null || cloud.Size == 0)
                return;

            int queryIndex = 0;
            double qx = cloud.GetX(queryIndex);
            double qy = cloud.GetY(queryIndex);
            double qz = cloud.GetZ(queryIndex);

            int foundKnn = KdTreeSearch.NearestKSearch(
                cloud.PointCloudXYZPointer,
                qx, qy, qz,
                k: 10,
                out int[] knnIndices,
                out float[] knnDist2);

            int foundRadius = KdTreeSearch.RadiusSearch(
                cloud.PointCloudXYZPointer,
                qx, qy, qz,
                radius: 0.05,
                maxNeighbors: 128,
                out int[] radiusIndices,
                out float[] radiusDist2);

            Console.WriteLine($"KNN: found={foundKnn}, firstIdx={(foundKnn > 0 ? knnIndices[0] : -1)}");
            Console.WriteLine($"Radius: found={foundRadius}, firstIdx={(foundRadius > 0 ? radiusIndices[0] : -1)}");
            _ = knnDist2;
            _ = radiusDist2;
        }
    }
}
