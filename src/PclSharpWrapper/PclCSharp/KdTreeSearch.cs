using System;
using System.Runtime.InteropServices;

namespace PclCSharp
{
    public static class KdTreeSearch
    {
        [DllImport("PclDll.dll", CallingConvention = CallingConvention.StdCall, EntryPoint = "kdtreeNearestKSearch", CharSet = CharSet.Auto)]
        private static extern int nearestKSearchNative(
            IntPtr in_pc,
            double x,
            double y,
            double z,
            int k,
            [Out] int[] outIndices,
            [Out] float[] outSqrDistances);

        [DllImport("PclDll.dll", CallingConvention = CallingConvention.StdCall, EntryPoint = "kdtreeRadiusSearch", CharSet = CharSet.Auto)]
        private static extern int radiusSearchNative(
            IntPtr in_pc,
            double x,
            double y,
            double z,
            double radius,
            int maxNeighbors,
            [Out] int[] outIndices,
            [Out] float[] outSqrDistances);

        public static int NearestKSearch(
            IntPtr cloudPointer,
            double x,
            double y,
            double z,
            int k,
            out int[] indices,
            out float[] sqrDistances)
        {
            if (cloudPointer == IntPtr.Zero)
                throw new ArgumentException("cloudPointer 不能为空", nameof(cloudPointer));
            if (k <= 0)
                throw new ArgumentOutOfRangeException(nameof(k), "k 必须大于 0");

            indices = new int[k];
            sqrDistances = new float[k];

            int found = nearestKSearchNative(cloudPointer, x, y, z, k, indices, sqrDistances);
            if (found < 0) found = 0;
            if (found < k)
            {
                Array.Resize(ref indices, found);
                Array.Resize(ref sqrDistances, found);
            }

            return found;
        }

        public static int RadiusSearch(
            IntPtr cloudPointer,
            double x,
            double y,
            double z,
            double radius,
            int maxNeighbors,
            out int[] indices,
            out float[] sqrDistances)
        {
            if (cloudPointer == IntPtr.Zero)
                throw new ArgumentException("cloudPointer 不能为空", nameof(cloudPointer));
            if (radius <= 0)
                throw new ArgumentOutOfRangeException(nameof(radius), "radius 必须大于 0");
            if (maxNeighbors <= 0)
                throw new ArgumentOutOfRangeException(nameof(maxNeighbors), "maxNeighbors 必须大于 0");

            indices = new int[maxNeighbors];
            sqrDistances = new float[maxNeighbors];

            int found = radiusSearchNative(cloudPointer, x, y, z, radius, maxNeighbors, indices, sqrDistances);
            if (found < 0) found = 0;
            if (found < maxNeighbors)
            {
                Array.Resize(ref indices, found);
                Array.Resize(ref sqrDistances, found);
            }

            return found;
        }
    }
}
