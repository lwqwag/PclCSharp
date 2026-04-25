#pragma once
#include <iostream>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>

/*
 * Copyright (c) 2022, ��ǵ�
 * All rights reserved.
 * Auther:��ǵ�(ShuDengdeng)
 * Email:2237380450@qq.com
 * ���ƴ�����һЩ���ߺ��������ṩ����ӿ�
 */

using namespace std;

//�����������������������ĵ�ص���������
int getMaxPointCluster(vector<pcl::PointIndices> cluster);
//����ƽ������л����Ϻ��ƽ����xy�ο������б��
double getAngle(double line1[3], double line2[3]);
//���ڻ�ȡ�������ı���ת������
Eigen::Matrix4f getRotationMatrix(Eigen::Vector3f vector_before, Eigen::Vector3f vector_after);
//��������������Ϊ���ƶ���
void connect_pc(vector<pcl::PointCloud<pcl::PointXYZ>>& input_pc1, pcl::PointCloud<pcl::PointXYZ> & out_cloud);

//�������أ�3�����귨���޳��쳣ֵ�������ǵ��ƣ�����ǵ�������
void stdDev_filter(vector<pcl::PointCloud<pcl::PointXYZ>>& input_cloud, vector<pcl::PointCloud<pcl::PointXYZ>>& out_cloud);