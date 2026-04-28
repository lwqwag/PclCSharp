#include "util_module.h"

/*
 * Copyright (c) 2022, ��ǵ�
 * All rights reserved.
 * Auther:��ǵ�(ShuDengdeng)
 * Email:2237380450@qq.com
 * utilģ�飬�����ܶ���ƴ������ߺ���
 */

//У��ƽ��
HEAD void CallingConvention correctPlane(pcl::PointCloud<pcl::PointXYZ> * in_pc, float * normal,
	pcl::PointCloud<pcl::PointXYZ> * out_pc)
{
	Eigen::Vector3f vector;
	vector << normal[0], normal[1], normal[2];
	Eigen::Vector3f vector_after(0, 0, 1);
	//�����б�任�ľ���
	Eigen::Matrix4f rotion_matrix = getRotationMatrix(vector, vector_after);
	//����б�ĵ���ƽ�����任��У���ط�����Ϊ��0,0,1����ƽ��
	pcl::transformPointCloud(*in_pc, *out_pc, rotion_matrix);

}

//sigam�����޳���ֵ
HEAD void CallingConvention sigamFilter(pcl::PointCloud<pcl::PointXYZ> * in_pc, int sigam_thresh,
	pcl::PointCloud<pcl::PointXYZ> * out_pc)
{
	double mean = 0;
	double stdDev = 0;
	vector<float> vec_z;
	for (int i = 0; i < in_pc->points.size(); i++)
	{
		vec_z.push_back(in_pc->points[i].z);
	}
	pcl::getMeanStdDev(vec_z, mean, stdDev);
	for (int j = 0; j < in_pc->points.size(); j++)
	{
		if (fabs(in_pc->points[j].z - mean) < sigam_thresh * stdDev)
		{
			out_pc->push_back(in_pc->points[j]);
		}
	}

}

//���ܣ������ư����з���洢��ÿһ�д���һ������Բ�ܣ�������û�ָ����n�в���Բ�ܵ���
HEAD void CallingConvention getRunoutPoints(pcl::PointCloud<pcl::PointXYZ> * in_pc, int num,
	pcl::PointCloud<pcl::PointXYZ> * out_pc)
{
	/*��ԭʼ���ư����еķ�ʽ�洢�����������е�ÿһ��Ԫ�ض���һ�����ƶ���һ�����ƶ���洢����һ�������ߵĵ���
	  */
	vector<pcl::PointCloud<pcl::PointXYZ>> out_res;//�����ư��д洢
	vector<pcl::PointCloud<pcl::PointXYZ>> out_std;//�޳���Ч�е���֮�������֯����
	vector<pcl::PointCloud<pcl::PointXYZ>> out_pc1;//�����ư��д洢
	vector<pcl::PointCloud<pcl::PointXYZ>> out_pc2;//�޳��쳣ֵ��İ��д洢����

	int y_interval = 100;//ÿһ�������ߵ�y�����100,�������������Ĳ��������Ե���
	int y_pc = 0;
	pcl::PointXYZ minPt(0, 0, 0), maxPt(0, 0, 0);
	pcl::getMinMax3D(*in_pc, minPt, maxPt);//�ҵ�y�ļ�ֵ
	int y_min = round(minPt.y);
	int y_max = round(maxPt.y);
	//����100����Ϊÿһ�������ߵ�y�����100��
	//y�������Сֵ֮���ٳ��Լ���õ����Ǽ����������������и����⣬����ָ��ʱ���е�����
	//���²�����ÿ��һ��һ����һ�������ߣ�Ҳ�п��ܸ��˺ü��ٲ���һ�������ߡ���������Ľ���ͻ�ƫ��
	//��Ŀǰ���Ե�ʱ��û�����������������ʱ�ǰ�ȫ�ġ�2022.5.24
	int line_nums = (y_max - y_min) / y_interval;
	line_nums = line_nums + 1;
	int pc_num = in_pc->points.size();//��������
	//ǰ��Ĺ�������Ϊ��ȷ���ж����������ߣ��ó�ʼ��vector
	//resize֮��ע���ڴ�˳������,��һ��û�н�����
	out_res.resize(line_nums);
	//��ʼ�����ư����д洢
	int k = 0;
	int flag = 0;
	for (int i = y_min; i <= y_max; i = i + 100)//�ܹ��ж�����������
	{
		for (int j = 0; j < pc_num; j++)//�����е㼯����yֵһ���ĵ㣬yֵһ���ĵ㱻��Ϊ��һ�������ߵĵ���
		{
			y_pc = round(in_pc->points[j].y);
			if (y_pc == i)
			{

				out_res[k].push_back(in_pc->points[j]);
				flag = flag + 1;//ͳ��һ�������ж��ٸ���
			}
		}
		//�����K����û�е㣬����һ�������䣬��ֹ������
		//������Ԥ����һ������Ŀƫ������ЩԪ��Ϊ�գ��������ᵽ�����һ�������ʽ.���˸о����Ǻܺã��������ܲ��ܸĽ�
		if (flag == 0)
		{
			pcl::PointXYZ p(0, 0, 0);
			out_res[k].push_back(p);
		}
		k++;
		flag = 0;
	}
	//�޳�һЩ���������ߡ����ƾ����д洢֮�󣬿϶�����һЩ�����ߵ������٣���Щ�������ٵĵ���Ӧ�ñ��޳�
	for (int i = 0; i < out_res.size(); i++)
	{
		//С��70�����������Ϊ����Ч�����ߣ�ʵ���Ϸ�ӳ��������ı�Եë�̣�ֱ������
		if (70 <= out_res[i].points.size()) //70��240������Ϊ����Ч�����ߣ����Ǹü����߰������ɿ׶�
		{
			out_std.push_back(out_res[i]);
		}

	}

	//Ŀ�ģ��������ռ�����Ƕ����������д洢���ƻ����ܽ�����⡣���滹��Ҫ���д洢���Ʊ�Ϊ�д洢���ƣ�
	//��Ϊÿһ�е��ƲŴ���һ������Բ�ܡ�

	/*�������ĵ��Ʋ�������ѵģ���Щ���Ƶ�����࣬���м���ܿ���һ�飬������ĵ�����һ�鲢û�п�
	Ӧ�ý�����xֵ����һ�������У����ڴˣ�ʹ�ü���set��set��������Ҫ���ԣ�һ����
	û���ظ�Ԫ�أ���һ���ǻ��Զ�������������*/

	//��õ������ж�����
	set<int, greater<int>> x_set;//��������
	for (int i = 0; i < out_std.size(); i++)
	{
		for (int j = 0; j < out_std[i].points.size(); j++)
		{
			//������������Ϊ���ȹ�ϵ�����ܻ���С�������Ҿ���ǰ���У��ƽ��֮�󣬵��������Ҳ����С��
			//����������Ҫʹ��round�����ݽ���Բ��
			x_set.insert(round(out_std[i].points[j].x));
		}
	}
	//Ϊ�˷����������x_set����תΪvector
	vector<int> pc_x;
	for (set<int>::iterator it = x_set.begin(); it != x_set.end(); it++)
	{
		pc_x.push_back(*it);
	}


	out_pc1.resize(pc_x.size());

	//��x��ֵΪ������xֵһ����Ϊͬһ��
	//������㷨���Ӷ�Ϊn3����Ҫ�Ľ�
	for (int i = 0; i < pc_x.size(); i++)
	{
		for (int j = 0; j < out_std.size(); j++)
		{
			for (int k = 0; k < out_std[j].points.size(); k++)
			{
				//xֵһ������Ϊ��ͬһ�е�
				if (round(pc_x[i]) == round(out_std[j].points[k].x))
				{
					out_pc1[i].push_back(out_std[j].points[k]);
					//ÿһ��ֻ��һ��һ���ģ�һ���ҵ����������ѭ������ʡʱ��
					break;
				}
			}
		}
	}
	//��ÿһ���е������ٵĵ��޳�
	for (int i = 0; i < out_pc1.size(); i++)
	{
		if (out_pc1[i].points.size() > 120)
		{
			out_pc2.push_back(out_pc1[i]);
		}
	}


	//���ѡ��n������Բ�ܣ����������
	vector<pcl::PointCloud<pcl::PointXYZ>> out_resVec;//����ָ���Ķ���Բ��
	int column_size = out_pc2.size();
	//����������Ϊ��������һ���ܱ�10���������Կ������������
	int surplus = column_size % 10;
	//���������������ʣ�µ�������10�ı���������ָ����Բ����
	//�Ϳ��Եõ�ÿһ��Բ�ܵļ����������ֻ������10
	//���ǲ���³��
	int gap = (column_size - surplus) / num;
	for (int i = surplus; i < out_pc2.size(); i = i + gap)
	{
		out_resVec.push_back(out_pc2[i]);
	}
	connect_pc(out_resVec, *out_pc);

}

//�����Լ����ԣ��������ṩ�ӿ�
HEAD float CallingConvention getRunoutPointsWithResult(pcl::PointCloud<pcl::PointXYZ> * in_pc, int num,
	pcl::PointCloud<pcl::PointXYZ> * out_pc)
{
	/*��ԭʼ���ư����еķ�ʽ�洢�����������е�ÿһ��Ԫ�ض���һ�����ƶ���һ�����ƶ���洢����һ�������ߵĵ���
	  */
	vector<pcl::PointCloud<pcl::PointXYZ>> out_res;//�����ư��д洢
	vector<pcl::PointCloud<pcl::PointXYZ>> out_std1;//�޳���Ч�е���֮�������֯����
	vector<pcl::PointCloud<pcl::PointXYZ>> out_std;//sigam�����޳��쳣ֵ�����������
	vector<pcl::PointCloud<pcl::PointXYZ>> out_pc1;//�����ư��д洢
	vector<pcl::PointCloud<pcl::PointXYZ>> out_pc2;//�޳��쳣ֵ��İ��д洢����

	int y_interval = 100;//ÿһ�������ߵ�y�����100,�������������Ĳ��������Ե���
	int y_pc = 0;
	pcl::PointXYZ minPt(0, 0, 0), maxPt(0, 0, 0);
	pcl::getMinMax3D(*in_pc, minPt, maxPt);//�ҵ�y�ļ�ֵ
	int y_min = round(minPt.y);
	int y_max = round(maxPt.y);
	//����100����Ϊÿһ�������ߵ�y�����100��
	//y�������Сֵ֮���ٳ��Լ���õ����Ǽ����������������и����⣬����ָ��ʱ���е�����
	//���²�����ÿ��һ��һ����һ�������ߣ�Ҳ�п��ܸ��˺ü��ٲ���һ�������ߡ���������Ľ���ͻ�ƫ��
	//��Ŀǰ���Ե�ʱ��û�����������������ʱ�ǰ�ȫ�ġ�2022.5.24
	int line_nums = (y_max - y_min) / y_interval;
	line_nums = line_nums + 1;
	int pc_num = in_pc->points.size();//��������
	//ǰ��Ĺ�������Ϊ��ȷ���ж����������ߣ��ó�ʼ��vector
	//resize֮��ע���ڴ�˳������,��һ��û�н�����
	out_res.resize(line_nums);
	//��ʼ�����ư����д洢
	int k = 0;
	int flag = 0;
	for (int i = y_min; i <= y_max; i = i + 100)//�ܹ��ж�����������
	{
		for (int j = 0; j < pc_num; j++)//�����е㼯����yֵһ���ĵ㣬yֵһ���ĵ㱻��Ϊ��һ�������ߵĵ���
		{
			y_pc = round(in_pc->points[j].y);
			if (y_pc == i)
			{

				out_res[k].push_back(in_pc->points[j]);
				flag = flag + 1;//ͳ��һ�������ж��ٸ���
			}
		}
		//�����K����û�е㣬����һ�������䣬��ֹ������
		//������Ԥ����һ������Ŀƫ������ЩԪ��Ϊ�գ��������ᵽ�����һ�������ʽ.���˸о����Ǻܺã��������ܲ��ܸĽ�
		if (flag == 0)
		{
			pcl::PointXYZ p(0, 0, 0);
			out_res[k].push_back(p);
		}
		k++;
		flag = 0;
	}
	//�޳�һЩ���������ߡ����ƾ����д洢֮�󣬿϶�����һЩ�����ߵ������٣���Щ�������ٵĵ���Ӧ�ñ��޳�
	for (int i = 0; i < out_res.size(); i++)
	{
		//С��70�����������Ϊ����Ч�����ߣ�ʵ���Ϸ�ӳ��������ı�Եë�̣�ֱ������
		if (70 <= out_res[i].points.size()) //70��240������Ϊ����Ч�����ߣ����Ǹü����߰������ɿ׶�
		{
			out_std1.push_back(out_res[i]);
		}

	}
	//�޳��쳣ֵ������ʵ��
	stdDev_filter(out_std1, out_std);

	//Ŀ�ģ��������ռ�����Ƕ����������д洢���ƻ����ܽ�����⡣���滹��Ҫ���д洢���Ʊ�Ϊ�д洢���ƣ�
	//��Ϊÿһ�е��ƲŴ���һ������Բ�ܡ�

	/*�������ĵ��Ʋ�������ѵģ���Щ���Ƶ�����࣬���м���ܿ���һ�飬������ĵ�����һ�鲢û�п�
	Ӧ�ý�����xֵ����һ�������У����ڴˣ�ʹ�ü���set��set��������Ҫ���ԣ�һ����
	û���ظ�Ԫ�أ���һ���ǻ��Զ�������������*/

	//��õ������ж�����
	set<int, greater<int>> x_set;//��������
	for (int i = 0; i < out_std.size(); i++)
	{
		for (int j = 0; j < out_std[i].points.size(); j++)
		{
			//������������Ϊ���ȹ�ϵ�����ܻ���С�������Ҿ���ǰ���У��ƽ��֮�󣬵��������Ҳ����С��
			//����������Ҫʹ��round�����ݽ���Բ��
			x_set.insert(round(out_std[i].points[j].x));
		}
	}
	//Ϊ�˷����������x_set����תΪvector
	vector<int> pc_x;
	for (set<int>::iterator it = x_set.begin(); it != x_set.end(); it++)
	{
		pc_x.push_back(*it);
	}


	out_pc1.resize(pc_x.size());

	//��x��ֵΪ������xֵһ����Ϊͬһ��
	//������㷨���Ӷ�Ϊn3����Ҫ�Ľ�
	for (int i = 0; i < pc_x.size(); i++)
	{
		for (int j = 0; j < out_std.size(); j++)
		{
			for (int k = 0; k < out_std[j].points.size(); k++)
			{
				//xֵһ������Ϊ��ͬһ�е�
				if (round(pc_x[i]) == round(out_std[j].points[k].x))
				{
					out_pc1[i].push_back(out_std[j].points[k]);
					//ÿһ��ֻ��һ��һ���ģ�һ���ҵ����������ѭ������ʡʱ��
					break;
				}
			}
		}
	}
	//��ÿһ���е������ٵĵ��޳�
	for (int i = 0; i < out_pc1.size(); i++)
	{
		if (out_pc1[i].points.size() > 120)
		{
			out_pc2.push_back(out_pc1[i]);
		}
	}


	//���ѡ��n������Բ�ܣ����������
	vector<pcl::PointCloud<pcl::PointXYZ>> out_resVec;//����ָ���Ķ���Բ��
	float runout = 0;
	float res = 0;
	int column_size = out_pc2.size();
	//����������Ϊ��������һ���ܱ�10���������Կ������������
	int surplus = column_size % 10;
	//���������������ʣ�µ�������10�ı���������ָ����Բ����
	//�Ϳ��Եõ�ÿһ��Բ�ܵļ����������ֻ������10
	//���ǲ���³��
	int gap = (column_size - surplus) / num;
	vector<float> runouts;
	pcl::PointXYZ min_pt(0, 0, 0);
	pcl::PointXYZ max_pt(0, 0, 0);
	for (int i = surplus; i < out_pc2.size(); i = i + gap)
	{
		out_resVec.push_back(out_pc2[i]);
		pcl::getMinMax3D(out_pc2[i], min_pt, max_pt);
		runout = max_pt.z - min_pt.z;
		runouts.push_back(runout);
	}
	connect_pc(out_resVec, *out_pc);
	//res = *max_element(runouts.begin(), runouts.end());
	for (int g = 0; g < runouts.size(); g++)
	{
		if (res <= runouts[g])
		{
			res = runouts[g];
		}
	}
	return res;
}


HEAD double CallingConvention calculateRunout(pcl::PointCloud<pcl::PointXYZ> * in_pc, int * indices)
{
	int min_indice = 0;
	int max_indice = 0;
	pcl::PointXYZ min_point(0, 0, 0);
	pcl::PointXYZ max_point(0, 0, 0);
	pcl::getMinMax3D(*in_pc, min_point, max_point);
	//�ҵ�Zֵ��С������������

	int len1 = in_pc->points.size();

	for (int i = 0; i < len1; i++)
	{
		double z = in_pc->points[i].z;
		if (z == min_point.z)
		{
			min_indice = i;
		}
		else if (z == max_point.z)
		{
			max_indice = i;
		}
	}
	indices[0] = min_indice;
	indices[1] = max_indice;
	double detect_result = max_point.z - min_point.z;//�������
	return detect_result;
}

HEAD void CallingConvention copyPcBaseOnIndice(pcl::PointCloud<pcl::PointXYZ> * in_pc,
	pcl::PointIndices * in_indice,
	pcl::PointCloud<pcl::PointXYZ> * out_pc)
{
	pcl::copyPointCloud(*in_pc, *in_indice, *out_pc);
}