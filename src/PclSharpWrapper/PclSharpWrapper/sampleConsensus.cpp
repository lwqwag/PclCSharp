#include "sampleConsensus.h"

/*
 * Copyright (c) 2022, ShuDengdeng
 * All rights reserved.
 * Author: ShuDengdeng
 * Email:2237380450@qq.com
 * sample consensus algorithms
 */

// Fit plane.
HEAD float CallingConvention fitPlane(pcl::PointCloud<pcl::PointXYZ> * in_pc,
	float distance_thresh, int max_itera, float * normal)
{
	float angel = 0.0;
	pcl::ModelCoefficients::Ptr modelCoeff(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setInputCloud(in_pc->makeShared());
	seg.setDistanceThreshold(distance_thresh);
	seg.setMaxIterations(max_itera);
	seg.setOptimizeCoefficients(true);
	seg.segment(*pointIndices, *modelCoeff);

	double line1[3] = { 0,0,1 };// reference normal vector
	double line2[3] = { modelCoeff->values[0],modelCoeff->values[1],modelCoeff->values[2] };// fitted plane normal
	angel = getAngle(line1, line2);// angle between two vectors
	normal[0] = modelCoeff->values[0];
	normal[1] = modelCoeff->values[1];
	normal[2] = modelCoeff->values[2];
	normal[3] = modelCoeff->values[3];
	return angel;

}
