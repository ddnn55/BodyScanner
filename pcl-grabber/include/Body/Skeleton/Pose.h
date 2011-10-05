/*
 * Pose.h
 *
 *  Created on: Oct 5, 2011
 *      Author: david stolarsky
 */


#ifndef POSE_H_
#define POSE_H_

#include <XnCppWrapper.h>

namespace Body {

namespace Skeleton {

class Pose {
public:
	Pose();
	virtual ~Pose();

	XnSkeletonJointPosition head;
	XnSkeletonJointPosition neck;
	XnSkeletonJointPosition torso;

	XnSkeletonJointPosition left_shoulder;
	XnSkeletonJointPosition left_elbow;
	XnSkeletonJointPosition left_hand;

	XnSkeletonJointPosition right_shoulder;
	XnSkeletonJointPosition right_elbow;
	XnSkeletonJointPosition right_hand;

	XnSkeletonJointPosition left_hip;
	XnSkeletonJointPosition left_knee;
	XnSkeletonJointPosition left_foot;

	XnSkeletonJointPosition right_hip;
	XnSkeletonJointPosition right_knee;
	XnSkeletonJointPosition right_foot;
};

}

}

#endif /* POSE_H_ */
