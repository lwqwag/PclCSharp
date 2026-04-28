#pragma once

#include "pcl_util.h"
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>// Euclidean clustering
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <set>

/*
 * Copyright (c) 2022, ShuDengdeng
 * All rights reserved.
 * Author: ShuDengdeng
 * Email:2237380450@qq.com
 * sample consensus algorithms
 */

using namespace std;
// Export symbols with C linkage.
#define EXTERNC extern "C"
// Export from DLL in this project.
#define HEAD EXTERNC __declspec(dllexport)
// Use stdcall for C ABI compatibility.
#define CallingConvention __stdcall





/*
Purpose: fit a plane with RANSAC and return its tilt angle.
param[in] in_pc input point cloud pointer
param[in] distance_thresh RANSAC distance threshold
param[in] max_itera maximum RANSAC iterations
param[out] normal plane coefficients a,b,c,d for ax+by+cz+d=0
*/
HEAD float CallingConvention fitPlane(pcl::PointCloud<pcl::PointXYZ> * in_pc,
	                                  float distance_thresh, int max_itera, float * normal);