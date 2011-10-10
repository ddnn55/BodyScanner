#ifndef DEF_CCPOINTSET
#define DEF_CCPOINTSET

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


class CCPointSet
{

	public:
		CCPointSet();

		CCPointSet(const std::string& str);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();
		void visualize();

		//getters
		pcl::PolygonMesh* getMesh();
			

	private:
		void setMesh(pcl::PolygonMesh *mesh_);
		void initCloudFromPCD(const std::string& str);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		pcl::PolygonMesh *mesh;
		

};

#endif


