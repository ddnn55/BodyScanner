#include "CCMesh.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "CCTools.h"

using namespace std;

CCMesh::CCMesh(pcl::PolygonMesh *mesh_){

	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	mesh = mesh_;
	pcl::fromROSMsg (mesh->cloud, *cloud);


}

int CCMesh::countVertices(){

	return cloud->size();
}

int CCMesh::countPolygons(){

	return mesh->polygons.size();

}

std::vector<pcl::Vertices>* CCMesh::getPolygons()
{

	return &(mesh->polygons);

}

