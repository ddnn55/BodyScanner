/*
 * Surface.h
 *
 *  Created on: Oct 10, 2011
 *      Author: mluffel
 */

#ifndef SURFACE_H_
#define SURFACE_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

namespace Body
{
	pcl::PolygonMesh::Ptr buildSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
}

#endif /* SURFACE_H_ */
