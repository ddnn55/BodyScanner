/*
 * Builder.h
 *
 *  Created on: Oct 9, 2011
 *      Author: stolrsky
 */

#include <boost/thread.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <Body/BodyPointCloud.h>

#ifndef BUILDER_H_
#define BUILDER_H_


namespace Body {

class Builder {
public:
	Builder(pcl::visualization::PCLVisualizer* viewer = NULL);
	virtual ~Builder();

	void pushSample(Body::BodyPointCloud::ConstPtr cloud, Body::Skeleton::Pose::Ptr skeleton_pose);

private:
	void updateOutputVisualizer();
	pcl::visualization::PCLVisualizer* viewer_;
};

}

#endif /* BUILDER_H_ */
