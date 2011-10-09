/*
 * Skin.cpp
 *
 *  Created on: Oct 8, 2011
 *      Author: mluffel
 */

#include "Body/Skin.h"

namespace Body
{


SkinBinding::SkinBinding() {}

void Skin::addBone(std::string joint_key) {
	limb_map.insert(std::make_pair(joint_key, num_bones));
	num_bones++;
}

void Skin::setNumPoints(int num_points) {
	bindings.resize(num_points);
}

void Skin::addPointToBone(int point_index, int bone_index, float weight) {}

void Skin::bind(pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_points, const Body::Skeleton::Pose::Ptr bind_pose) {}

void Skin::renderPosed(const Body::Skeleton::Pose::Ptr pose) {}

const pcl::PointCloud<pcl::PointXYZRGB>::Ptr Skin::pose(const Body::Skeleton::Pose::Ptr pose) const {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nil;
	return nil;
}

Skin::Skin() : num_bones(0) {}
/*
		std::vector<SkinBinding> bindings;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_points;

		pcl::PointCloud<pcl::PointXYZ> bound_points[MAX_BINDINGS];

		typedef std::map<std::string, int> NameToIndex;
		NameToIndex limb_map;
*/
}