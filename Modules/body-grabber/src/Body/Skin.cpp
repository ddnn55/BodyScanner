/*
 * Skin.cpp
 *
 *  Created on: Oct 8, 2011
 *      Author: mluffel
 */

#include "Body/Skin.h"

namespace Body
{

void LimbSkin::addPoint(int index, float weight) {
	SkinPoint sp;
	sp.index = index;
	sp.weight = weight;
	sp.pos.x = 0;
	sp.pos.y = 0;
	sp.pos.z = 0;
	points.push_back(sp);
}

LimbSkin& Skin::addLimb(std::string joint_key) {
	int index = limbs.size();
	limb_map.insert(std::make_pair(joint_key, index));
	
	LimbSkin ls;
	limbs.push_back(ls);
	return limbs.back();
}
	
void Skin::bind(pcl::PointCloud<pcl::PointXYZ> all_points, Body::Skeleton::Pose bind_pose) {
	
}

void Skin::pose(Body::Skeleton::Pose pose) {
	
}

const std::vector<pcl::PointXYZ>& Skin::getPosedPoints() const {
	return posed_points;
}


}