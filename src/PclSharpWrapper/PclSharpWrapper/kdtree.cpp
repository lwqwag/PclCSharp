#include "kdtree.h"

#include <algorithm>
#include <vector>

int CallingConvention kdtreeNearestKSearch(
    pcl::PointCloud<pcl::PointXYZ>* in_pc,
    double x,
    double y,
    double z,
    int k,
    int* out_indices,
    float* out_sqdist)
{
    if (in_pc == nullptr || out_indices == nullptr || out_sqdist == nullptr || k <= 0)
    {
        return 0;
    }

    if (in_pc->empty())
    {
        return 0;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(in_pc->makeShared());

    pcl::PointXYZ search_point(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    std::vector<int> indices(static_cast<size_t>(k));
    std::vector<float> sqdist(static_cast<size_t>(k));

    int found = tree.nearestKSearch(search_point, k, indices, sqdist);
    int copy_count = std::min(found, k);
    for (int i = 0; i < copy_count; ++i)
    {
        out_indices[i] = indices[static_cast<size_t>(i)];
        out_sqdist[i] = sqdist[static_cast<size_t>(i)];
    }

    return copy_count;
}

int CallingConvention kdtreeRadiusSearch(
    pcl::PointCloud<pcl::PointXYZ>* in_pc,
    double x,
    double y,
    double z,
    double radius,
    int max_nn,
    int* out_indices,
    float* out_sqdist)
{
    if (in_pc == nullptr || out_indices == nullptr || out_sqdist == nullptr || radius <= 0.0 || max_nn <= 0)
    {
        return 0;
    }

    if (in_pc->empty())
    {
        return 0;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(in_pc->makeShared());

    pcl::PointXYZ search_point(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    std::vector<int> indices;
    std::vector<float> sqdist;

    int found = tree.radiusSearch(search_point, radius, indices, sqdist, static_cast<unsigned int>(max_nn));
    int copy_count = std::min(found, max_nn);
    for (int i = 0; i < copy_count; ++i)
    {
        out_indices[i] = indices[static_cast<size_t>(i)];
        out_sqdist[i] = sqdist[static_cast<size_t>(i)];
    }

    return copy_count;
}
