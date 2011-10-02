#ifndef DEF_CCTOOLS
#define DEF_CCTOOLS

#include <iostream>
#include <string>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <iostream>
#include <unistd.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "CCMesh.h"



class CCTools
{

	public:
		static void VisualizeNormalCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cl);
		static pcl::PointCloud<pcl::PointNormal>::Ptr getNormalsFromXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_);
		static pcl::PolygonMesh* getMeshFromXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);	

		static pcl::PointCloud<pcl::PointNormal>::Ptr IsolateBody(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
		static pcl::PolygonMesh* getMeshFromNormalCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

		static double CompareSpinImages(CCMesh* mesh1,int index1, CCMesh* mesh2, int index2);

	private:
		
		

};

#endif

