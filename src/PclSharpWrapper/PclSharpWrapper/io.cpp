#include "io.h"

void Pc2Array(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, double * out_x, double * out_y, double * out_z)
{
	for (size_t i = 0; i < cloud_in->points.size(); i++) {

		out_x[i] = cloud_in->points[i].x;
		out_y[i] = cloud_in->points[i].y;
		out_z[i] = cloud_in->points[i].z;

	}
}
// Load point-cloud file and export XYZ to separate arrays.
HEAD int CallingConvention loadFile(char * path, double * out_x, double * out_y, double * out_z)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPLYFile(path, *cloud) == -1)
	{
		return 0;
	}
	else
	{
		Pc2Array(cloud, out_x, out_y, out_z);
		return 1;
	}
}
// Load PLY file.
HEAD int CallingConvention loadPlyFile(char * path, pcl::PointCloud<pcl::PointXYZ>* pc)
{
	if (pcl::io::loadPLYFile(path, *pc) == -1)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
// Load PCD file.
HEAD int CallingConvention loadPcdFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc)
{
	if (pcl::io::loadPCDFile(path, *pc) == -1)
	{
		return 0;
	}
	else
	{
		return 1;
	}

}
// Load OBJ file.
HEAD int CallingConvention loadObjFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc)
{
	// Direct use of pcl::io::loadOBJFile is not suitable here.
	// Use loadPolygonFile first, then convert mesh.cloud with fromPCLPointCloud2.
	pcl::PolygonMesh mesh;

	if (pcl::io::loadPolygonFile(path, mesh) < 0)
	{
		cout << "load failed" << endl;
		return 0;
	}
	else
	{
		pcl::fromPCLPointCloud2(mesh.cloud, *pc);
		return 1;
	}

}
// Load TXT file.
HEAD int CallingConvention loadTxtFile(char * path, pcl::PointCloud<pcl::PointXYZ>* pc)
{
	
	ifstream Points_in(path);
	pcl::PointXYZ tmpoint;
	if (Points_in.is_open())
	{
		while (!Points_in.eof())   // Read until end-of-file.
		{
			// Operator>> treats spaces and newlines as separators.
			Points_in >> tmpoint.x >> tmpoint.y >> tmpoint.z;
			pc->points.push_back(tmpoint);
		}
		pc->width = (int)pc->points.size();
		pc->height = 1;
		return 1;
	}
	else
	{
		return 0;
	}
	
}

// Save PCD file.
HEAD void CallingConvention savePcdFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc, int binaryMode)
{
	if (binaryMode >= 1)
	{
		pcl::io::savePCDFile(path, *pc, true);
	}
	else
	{
		pcl::io::savePCDFile(path, *pc, false);
	}

}
// Save PLY file.
HEAD void CallingConvention savePlyFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc, int binaryMode)
{
	if (binaryMode >= 1)
	{
		pcl::io::savePLYFile(path, *pc, true);
	}
	else
	{
		pcl::io::savePLYFile(path, *pc, false);
	}

}
// Save OBJ file.

HEAD void CallingConvention saveObjFile(char* path, pcl::PointCloud<pcl::PointXYZ> * pc)
{
	// TODO: add implementation to save point cloud as OBJ.

}

// Convert STL mesh to point cloud.
HEAD void CallingConvention stl2PointCloud(char * path, pcl::PointCloud<pcl::PointXYZ>* pc)
{
	vtkSmartPointer<vtkSTLReader> stlreader = vtkSmartPointer<vtkSTLReader>::New();
	stlreader->SetFileName(path);
	stlreader->Update();
	vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
	poly = stlreader->GetOutput();
	pcl::io::vtkPolyDataToPointCloud(poly, *pc);
}