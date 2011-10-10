/*
 * Skin.cpp
 *
 *  Created on: Oct 8, 2011
 *      Author: mluffel
 */

#include "Body/Skin.h"

namespace Body
{


SkinBinding::SkinBinding() {
	for(int i = 0; i < MAX_BINDINGS; i++) {
		index[i] = -1;
		weight[i] = 0;
	}
}

void Skin::addBone(std::string joint_key) {
	limb_map.insert(std::make_pair(joint_key, num_bones));
	num_bones++;
}

void Skin::setNumPoints(int num_points) {
	bindings.resize(num_points);
}

void Skin::addPointToBone(int point_index, int bone_index, float weight) {
	int i = 0;
	for(; i < MAX_BINDINGS; i++) {
		if(bindings[point_index].index[i] == -1) { // an unused slot
			bindings[point_index].index[i] = bone_index;
			bindings[point_index].weight[i] = weight;
			break;
		}
	}
	assert(i < MAX_BINDINGS); // otherwise, too many bindings for this point
}

// TODO: use eigen instead
float dot(float *A, float *B) {
	return A[0]*B[0] + A[1]*B[1] + A[2]*B[2];
}

void transpose(float *src, float *dst) {
	dst[0] = src[0]; dst[1] = src[3]; dst[2] = src[6];
	dst[3] = src[1]; dst[4] = src[4]; dst[5] = src[7];
	dst[6] = src[2]; dst[7] = src[5]; dst[8] = src[8];
}

void localize(float *pos, float *orientation, pcl::PointXYZRGB *input, pcl::PointXYZ *output) {
	output->x = input->x - pos[0];
	output->y = input->y - pos[1];
	output->z = input->z - pos[2];
	
	// matrix multiply
	output->x = dot(&orientation[0], (float*)output);
	output->y = dot(&orientation[3], (float*)output);
	output->z = dot(&orientation[6], (float*)output);
}

void globalize(Body::Skeleton::Joint::Pose *bone, float weight, pcl::PointXYZ *local, pcl::PointXYZRGB *output) {
	float *position = (float*)&bone->position.position;
	float *orientation = bone->orientation.orientation.elements;
	output->x += weight*(position[0] + dot(&orientation[0], (float*)local));
	output->y += weight*(position[1] + dot(&orientation[3], (float*)local));
	output->z += weight*(position[2] + dot(&orientation[6], (float*)local));
}

void Skin::bind(pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_points, const Body::Skeleton::Pose::Ptr bind_pose) {
	int num_points = all_points->size();
	// allocate space for localized points
	for(int i = 0; i < MAX_BINDINGS; i++) {
		bound_points[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
		bound_points[i]->resize(num_points);
	}
	// put bone data in arrays
	float positions[num_bones][3];
	float orientations[num_bones][9];
	for(NameToIndex::const_iterator it = limb_map.begin(); it != limb_map.end(); ++it) {
		int index = it->second;
		Body::Skeleton::Joint::Pose config = (*bind_pose)[it->first];
		XnVector3D v = config.position.position;
		positions[index][0] = v.X; positions[index][1] = v.Y; positions[index][2] = v.Z;
		// inverse
		transpose((float*)config.orientation.orientation.elements, orientations[index]);
	}
	// put all points in local coordinate frame
	for(int i = 0; i < num_points; i++) {
		for(int j = 0; j < MAX_BINDINGS; j++) {
			int index = bindings[i].index[j];
			localize(positions[index], orientations[index], &all_points->at(i), &bound_points[j]->at(i));
		}
	}
}

void Skin::renderPosed(const Body::Skeleton::Pose::Ptr pose) {
	// do some vertex shader setup here
}

ColorCloud::Ptr Skin::pose(const Body::Skeleton::Pose::Ptr pose, ColorCloud::Ptr output) const {
	int num_points = input_points->size();
	
	// allocate data (unless already allocated)
	if(output == NULL) {
		output = ColorCloud::Ptr(new ColorCloud());
		output->resize(num_points);
		// set the colors just once
		for(int i = 0; i < num_points; i++) {
			output->at(i).r = input_points->at(i).r;
			output->at(i).g = input_points->at(i).g;
			output->at(i).b = input_points->at(i).b;
		}
	}
	
	// put bones in array
	Body::Skeleton::Joint::Pose bones[num_bones];
	for(NameToIndex::const_iterator it = limb_map.begin(); it != limb_map.end(); ++it) {
		bones[it->second] = (*pose)[it->first];
	}
	
	// put all points in global coordinate frame
	for(int i = 0; i < num_points; i++) {
		// clear values
		pcl::PointXYZRGB& p = output->at(i);
		p.x = p.y = p.z = 0;
		for(int j = 0; j < MAX_BINDINGS; j++) { // all contributions from each binding
			int index = bindings[i].index[j];
			float weight = bindings[i].weight[j];
			globalize(&bones[index], weight, &bound_points[j]->at(i), &p);
		}
	}
	
	return output;
}

Skin::Skin() : num_bones(0) {}
}
/**
 * We allocate an output cloud in pose, and localize points in bind.
 * Both are wrapped in boost::shared_ptr, so (I think) we don't need to deallocate them directly.
 */

