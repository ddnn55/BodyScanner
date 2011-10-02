#ifndef DEF_CCMESH
#define DEF_CCMESH

#include <iostream>
#include <string>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>
#include <unistd.h>
#include <pcl/point_cloud.h>


class CCMesh : pcl::PolygonMesh
{

	public:
		CCMesh(pcl::PolygonMesh *mesh_);
		int countVertices();
		int countLinks();
		int countPolygons();
		std::vector<pcl::Vertices> *getPolygons();
		

	private:
		pcl::PolygonMesh *mesh;		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

};

#endif

