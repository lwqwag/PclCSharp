using System;
using System.Runtime.InteropServices;

namespace PclCSharp
{
    public static class OctreeSearch
    {
        [DllImport("PclDll.dll", CallingConvention = CallingConvention.StdCall, EntryPoint = "octreeNearestKSearch", CharSet = CharSet.Auto)]
        private static extern int nearestKSearchNative(
            IntPtr in_pc,
            double resolution,
            double x,
            double y,
            double z,
            int k,
            [Out] int[] outIndices,
            [Out] float[] outSqrDistances);

        [DllImport("PclDll.dll", CallingConvention = CallingConvention.StdCall, EntryPoint = "octreeRadiusSearch", CharSet = CharSet.Auto)]
        private static extern int radiusSearchNative(
            IntPtr in_pc,
            double resolution,
            double x,
            double y,
            double z,
            double radius,
            int maxNeighbors,
            [Out] int[] outIndices,
            [Out] float[] outSqrDistances);

        public static int NearestKSearch(
            IntPtr cloudPointer,
            double resolution,
            double x,
            double y,
            double z,
            int k,
            out int[] indices,
            out float[] sqrDistances)
        {
            if (cloudPointer == IntPtr.Zero)
                throw new ArgumentException("cloudPointer 不能为空", nameof(cloudPointer));
            if (resolution <= 0)
                throw new ArgumentOutOfRangeException(nameof(resolution), "resolution 必须大于 0");
            if (k <= 0)
                throw new ArgumentOutOfRangeException(nameof(k), "k 必须大于 0");

            indices = new int[k];
            sqrDistances = new float[k];

            int found = nearestKSearchNative(cloudPointer, resolution, x, y, z, k, indices, sqrDistances);
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
            double resolution,
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
            if (resolution <= 0)
                throw new ArgumentOutOfRangeException(nameof(resolution), "resolution 必须大于 0");
            if (radius <= 0)
                throw new ArgumentOutOfRangeException(nameof(radius), "radius 必须大于 0");
            if (maxNeighbors <= 0)
                throw new ArgumentOutOfRangeException(nameof(maxNeighbors), "maxNeighbors 必须大于 0");

            indices = new int[maxNeighbors];
            sqrDistances = new float[maxNeighbors];

            int found = radiusSearchNative(cloudPointer, resolution, x, y, z, radius, maxNeighbors, indices, sqrDistances);
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
