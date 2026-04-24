#pragma once
#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
// VTK headers: flat-style includes work for both VTK 8.x and VTK 9.x
// (VTK 9 ships a compatibility header directory that preserves the flat layout)
#include <vtkSmartPointer.h>
#include <vtkSTLReader.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>


/*
 * Copyright (c) 2022, ��ǵ�
 * All rights reserved.
 * Auther:��ǵ�(ShuDengdeng)
 * Email:2237380450@qq.com
 * 2022626,���������������
 */

using namespace std;
//���嵼����ʽ����C���Եķ�ʽ��������ΪC���Է�ʽ���������ֲ���
#define EXTERNC extern "C"
//����dll������ʽ���˴��ǵ���������ǵ�����Ϊdllinport
#define HEAD EXTERNC __declspec(dllexport)
//�������Լ�����˴�ѡ���׼����Լ����Ҳ������c����Լ��
#define CallingConvention __stdcall


/*
���ܣ���pointcloud�ṹ����������洢,������ԭʼ�ķ�ʽ�����潫���ƶ����װ�ˣ��ⷽ��������
*/
void Pc2Array(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, double * out_x, double * out_y, double * out_z);

/*
���ܣ�����ply��pcd��ʽ�����ļ������������е�X��Y��Z����洢������double������,��������
param[in] path �����ļ�·��
param[out] out_x ���е��X����
param[out] out_y ���е��Y����
param[out] out_z ���е��Z����
�ɹ������ļ�����1��ʧ���򷵻�0
*/
HEAD int CallingConvention loadFile(char* path, double * out_x, double * out_y, double * out_z);

/*
���ܣ�����ply��ʽ�����ļ����������ƴ���PointCloud������
param[in] path �����ļ�·��
param[out] pc ���ƶ���
�ɹ������ļ�����1��ʧ���򷵻�0
*/
HEAD int CallingConvention loadPlyFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc);

/*
���ܣ�����pcd��ʽ�����ļ����������ƴ���PointCloud������
param[in] path �����ļ�·��
param[out] pc ���ƶ���
�ɹ������ļ�����1��ʧ���򷵻�0
*/
HEAD int CallingConvention loadPcdFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc);

/*
���ܣ�����obj�ļ�
param[in] path �ļ�·��
param[out] pc ���ƶ���
�ɹ������ļ�����1��ʧ���򷵻�0
*/
HEAD int CallingConvention loadObjFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc);

/*
���ܣ�����txt�ļ�
param[in] path �ļ�·��
param[out] pc ���ƶ���
�ɹ������ļ�����1��ʧ���򷵻�0
*/
HEAD int CallingConvention loadTxtFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc);


/*
���ܣ�����pcd��ʽ�����ļ�
param[in] path �����ļ�·��
param[out] pc ���ƶ���
�ɹ������ļ�����1��ʧ���򷵻�0
*/
HEAD void CallingConvention savePcdFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc, int binaryMode);

/*
���ܣ�����ply��ʽ�����ļ�
param[in] path �����ļ�·��
param[out] pc ���ƶ���
�ɹ������ļ�����1��ʧ���򷵻�0
*/
HEAD void CallingConvention savePlyFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc, int binaryMode);

/*
���ܣ���stl��ʽ�ļ�תΪ���ƶ���
param[in] path stl��ʽ�ļ�·��
param[out] pc ���ƶ���
*/
HEAD void CallingConvention stl2PointCloud(char* path, pcl::PointCloud<pcl::PointXYZ> * pc);

/*
//TODO ��δʵ�ָù��� 2022620
���ܣ����ƴ�Ϊobj��ʽ
param[in] path �洢·��
param[out] pc ������ƶ���
*/
HEAD void CallingConvention saveObjFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc);