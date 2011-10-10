/*
 * Builder.cpp
 *
 *  Created on: Oct 9, 2011
 *      Author: stolrsky
 */

#include <Body/Builder.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace Body {

Builder::Builder(pcl::visualization::PCLVisualizer* viewer) {
	// TODO Auto-generated constructor stub
	viewer_ = viewer;
	//viewer_->addPolygonMesh(pcl::PolygonMesh(), "body"); // add empty mesh s update...() can simply remove and add

}

Builder::~Builder() {
	// TODO Auto-generated destructor stub
}

void Builder::pushSample(Body::BodyPointCloud::ConstPtr cloud, Body::Skeleton::Pose::Ptr skeleton_pose)
{

}

void Builder::updateOutputVisualizer()
{
	// output model updated
	if (viewer_ != NULL && !viewer_->wasStopped()) {
		viewer_->removeShape("body");
		viewer_->addPolygonMesh (pcl::PolygonMesh(), "body");
	}
}

}
