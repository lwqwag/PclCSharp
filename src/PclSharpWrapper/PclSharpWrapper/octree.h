#pragma once

#include <pcl/common/common_headers.h>
#include <pcl/octree/octree_search.h>

// 导出约定与现有 wrapper 保持一致
#define EXTERNC extern "C"
#define HEAD EXTERNC __declspec(dllexport)
#define CallingConvention __stdcall

/*
功能：Octree 最近邻搜索（KNN）
param[in] in_pc        输入点云
param[in] resolution   八叉树分辨率（建议与点云尺度匹配）
param[in] x,y,z        查询点坐标
param[in] k            邻居数量
param[out] out_indices 输出索引数组（长度至少为 k）
param[out] out_sqdist  输出平方距离数组（长度至少为 k）
return 实际命中数量
*/
HEAD int CallingConvention octreeNearestKSearch(
    pcl::PointCloud<pcl::PointXYZ>* in_pc,
    double resolution,
    double x,
    double y,
    double z,
    int k,
    int* out_indices,
    float* out_sqdist);

/*
功能：Octree 半径搜索
param[in] in_pc        输入点云
param[in] resolution   八叉树分辨率
param[in] x,y,z        查询点坐标
param[in] radius       搜索半径
param[in] max_nn       最多返回邻居数量（用于限制输出）
param[out] out_indices 输出索引数组（长度至少为 max_nn）
param[out] out_sqdist  输出平方距离数组（长度至少为 max_nn）
return 实际命中数量
*/
HEAD int CallingConvention octreeRadiusSearch(
    pcl::PointCloud<pcl::PointXYZ>* in_pc,
    double resolution,
    double x,
    double y,
    double z,
    double radius,
    int max_nn,
    int* out_indices,
    float* out_sqdist);
