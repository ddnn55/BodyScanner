#include "CCPointSet.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "CCTools.h"

using namespace std;

CCPointSet::CCPointSet()
{
	;
}

CCPointSet::CCPointSet(const std::string& str)
{
	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	this->initCloudFromPCD(str);

	
	pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
	normals = CCTools::IsolateBody(CCTools::getNormalsFromXYZRGB(cloud));
	CCTools::VisualizeNormalCloud(normals);

	mesh = CCTools::getMeshFromNormalCloud(normals);

}

void CCPointSet::setMesh(pcl::PolygonMesh *mesh_){

	mesh = mesh_;

}

void CCPointSet::initCloudFromPCD(const string& str){

  sensor_msgs::PointCloud2 cloud_blob;
  pcl::io::loadPCDFile(str, cloud_blob);
  pcl::fromROSMsg (cloud_blob, *cloud);

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr CCPointSet::getCloud()
{

return cloud;

}

void CCPointSet::visualize() {

 }

//getters

pcl::PolygonMesh* CCPointSet::getMesh(){

	return this->mesh;

}



