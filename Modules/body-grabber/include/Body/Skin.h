/*
 * Skin.h
 *
 *  Created on: Oct 8, 2011
 *      Author: mluffel
 */

#ifndef SKIN_H_
#define SKIN_H_

#include <vector>
#include <map>

#include <XnCppWrapper.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Body/Skeleton/Skeleton.h"

namespace Body
{

struct SkinPoint {
	int index;
	float weight;
	pcl::PointXYZ pos; // in local coordinates
};

class LimbSkin
{
	std::vector<SkinPoint> points;

	/**
	 * Segmentation should use this to build each limb.
	 */
	void addPoint(int index, float weight);
};

class Skin
{
public:
	/**
	 * Segmentation should call this for each limb, and add points to the return value.
	 * 
	 */
	LimbSkin& addLimb(std::string joint_key);
	
	/**
	 * Pass in the canonical pose, only once to compute local coordinates for each limb.
	 */
	void bind(pcl::PointCloud<pcl::PointXYZ> all_points, Body::Skeleton::Pose bind_pose);
	
	/**
	 * Pass in the current pose.
	 */
	void pose(Body::Skeleton::Pose pose);
	
	/**
	 * Get the points, skinned with the current pose.
	 */
	const std::vector<pcl::PointXYZ>& getPosedPoints() const;

private:
	/**
	 * All the limbs of the bone, includes local coordinates of each included point.
	 */
	std::vector<LimbSkin> limbs;
		
	/**
	 * All the points in the mesh, transformed by the current pose.
	 * - This is empty until pose() is called.
	 */
	std::vector<pcl::PointXYZ> posed_points;
	
	typedef std::map<std::string, int> NameToIndex;
	NameToIndex limb_map;
};

}

#endif /* SKIN_H_ */
