/*
 * BodyPointCloud.h
 *
 *  Created on: Oct 8, 2011
 *      Author: webaba
 */

#ifndef BODYPOINTCLOUD_H_
#define BODYPOINTCLOUD_H_

#include <vector>
#include <map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Body/Skeleton/Skeleton.h"

namespace Body {

class BodyPointCloud : public pcl::PointCloud<pcl::PointXYZRGB> {

public:
	typedef boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > Ptr;
	typedef boost::shared_ptr< const pcl::PointCloud<pcl::PointXYZRGB> > ConstPtr;

private:

};

}

#endif /* BODYPOINTCLOUD_H_ */
