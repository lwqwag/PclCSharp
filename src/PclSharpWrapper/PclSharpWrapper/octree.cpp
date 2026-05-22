#include "octree.h"

#include <algorithm>
#include <vector>

int CallingConvention octreeNearestKSearch(
    pcl::PointCloud<pcl::PointXYZ>* in_pc,
    double resolution,
    double x,
    double y,
    double z,
    int k,
    int* out_indices,
    float* out_sqdist)
{
    if (in_pc == nullptr || out_indices == nullptr || out_sqdist == nullptr || k <= 0 || resolution <= 0.0)
    {
        return 0;
    }

    if (in_pc->empty())
    {
        return 0;
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(static_cast<float>(resolution));
    octree.setInputCloud(in_pc->makeShared());
    octree.addPointsFromInputCloud();

    pcl::PointXYZ search_point(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    std::vector<int> indices(static_cast<size_t>(k));
    std::vector<float> sqdist(static_cast<size_t>(k));

    int found = octree.nearestKSearch(search_point, k, indices, sqdist);
    int copy_count = std::min(found, k);
    for (int i = 0; i < copy_count; ++i)
    {
        out_indices[i] = indices[static_cast<size_t>(i)];
        out_sqdist[i] = sqdist[static_cast<size_t>(i)];
    }

    return copy_count;
}

int CallingConvention octreeRadiusSearch(
    pcl::PointCloud<pcl::PointXYZ>* in_pc,
    double resolution,
    double x,
    double y,
    double z,
    double radius,
    int max_nn,
    int* out_indices,
    float* out_sqdist)
{
    if (in_pc == nullptr || out_indices == nullptr || out_sqdist == nullptr || radius <= 0.0 || max_nn <= 0 || resolution <= 0.0)
    {
        return 0;
    }

    if (in_pc->empty())
    {
        return 0;
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(static_cast<float>(resolution));
    octree.setInputCloud(in_pc->makeShared());
    octree.addPointsFromInputCloud();

    pcl::PointXYZ search_point(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    std::vector<int> indices;
    std::vector<float> sqdist;

    int found = octree.radiusSearch(search_point, static_cast<float>(radius), indices, sqdist, static_cast<unsigned int>(max_nn));
    int copy_count = std::min(found, max_nn);
    for (int i = 0; i < copy_count; ++i)
    {
        out_indices[i] = indices[static_cast<size_t>(i)];
        out_sqdist[i] = sqdist[static_cast<size_t>(i)];
    }

    return copy_count;
}
