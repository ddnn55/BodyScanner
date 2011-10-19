/*
 * Pose.cpp
 *
 *  Created on: Oct 5, 2011
 *      Author: stolrsky
 */

#include <Body/Body.h>
#include <Body/Skeleton/Skeleton.h>

#include <string>
#include <sstream>
#include <cmath>
#include <stack>
#include <algorithm>
#include <set>

#include <ofMain.h>

namespace Body {




Skeleton::Pose::Pose() {
	// TODO Auto-generated constructor stub

}

// TODO merge SkeletonYaml and Skeleton::Pose
Skeleton::Pose::Pose(SkeletonYaml skeletonYaml)
{
	for(int j = 0; j < skeletonYaml.numJoints; j++)
	{
		Skeleton::JointPose jointPose; // TODO use our own joint pose class
		jointPose.position.x = skeletonYaml.translations[j][0];
		jointPose.position.y = skeletonYaml.translations[j][1];
		jointPose.position.z = skeletonYaml.translations[j][2];
		jointPose.orientation[0] = skeletonYaml.rotations[j][0];
		jointPose.orientation[1] = skeletonYaml.rotations[j][1];
		jointPose.orientation[2] = skeletonYaml.rotations[j][2];
		jointPose.orientation[3] = skeletonYaml.rotations[j][3];
		jointPose.orientation[4] = skeletonYaml.rotations[j][4];
		jointPose.orientation[5] = skeletonYaml.rotations[j][5];
		jointPose.orientation[6] = skeletonYaml.rotations[j][6];
		jointPose.orientation[7] = skeletonYaml.rotations[j][7];
		jointPose.orientation[8] = skeletonYaml.rotations[j][8];
		joint_poses[JointNameToJointKey(skeletonYaml.jointNames()[j])] = jointPose;
	}
}

Skeleton::Pose::~Pose() {
	// TODO Auto-generated destructor stub
}

void Skeleton::Pose::draw()
{
	for(int j = FirstJoint; j <= LastJoint; j++)
	{
		Joint jointKey = (Joint)j;
		ofSphere(joint_poses[jointKey].position.x / 1000.0,
				 joint_poses[jointKey].position.y / 1000.0,
				 joint_poses[jointKey].position.z / 1000.0,
				 0.03);
	}
}

void Skeleton::Pose::setTransformationForJointKey(Joint joint_key, JointPose transformation) {
	joint_poses[joint_key] = transformation;
}

Skeleton::JointPose& Skeleton::Pose::operator[](Joint joint_key) {
	return joint_poses[joint_key];
}

Skeleton::Pose::JointPoses Skeleton::Pose::getJointPoses() const {

	return joint_poses;
}

const std::string Skeleton::Pose::toYaml() const
 {
	 std::stringstream yaml;


	 //for(JointPoses::const_iterator j = joint_poses.begin(); j != joint_poses.end(); j++)
	 for(int jointIndex = FirstJoint; jointIndex <= LastJoint; jointIndex++)
	 {
		 Joint j = (Joint) jointIndex;

		 yaml << GetJointStringKeyByKey(j) << ":" << std::endl;
		 yaml << "  position:" << std::endl;
		 yaml << "    - confidence -- " << getJointPoses()[j].position_confidence << std::endl;
		 yaml << "    translation:" << std::endl;
		 yaml << "      - x -- " << getJointPoses()[j].position.x << std::endl;
		 yaml << "      - y -- " << getJointPoses()[j].position.y << std::endl;
		 yaml << "      - z -- " << getJointPoses()[j].position.z << std::endl;
		 yaml << "  orientation:" << std::endl;
		 yaml << "    - confidence -- " << getJointPoses()[j].orientation_confidence << std::endl;
		 yaml << "    matrix3x3:" << std::endl;
		 yaml << "      - " << getJointPoses()[j].orientation[0] << std::endl;
		 yaml << "      - " << getJointPoses()[j].orientation[1] << std::endl;
		 yaml << "      - " << getJointPoses()[j].orientation[2] << std::endl;
		 yaml << "      - " << getJointPoses()[j].orientation[3] << std::endl;
		 yaml << "      - " << getJointPoses()[j].orientation[4] << std::endl;
		 yaml << "      - " << getJointPoses()[j].orientation[5] << std::endl;
		 yaml << "      - " << getJointPoses()[j].orientation[6] << std::endl;
		 yaml << "      - " << getJointPoses()[j].orientation[7] << std::endl;
		 yaml << "      - " << getJointPoses()[j].orientation[8] << std::endl;
	 }



	 return yaml.str();
 }



float Skeleton::Pose::getBoneLength(const Bone bone) {
	JointPair endpoints = GetBoneJoints(bone);

	return getJointDistance(endpoints.parent, endpoints.child);
}

float Skeleton::Pose::getJointDistance(const Joint joint1, const Joint joint2) {
	JointPose& a = joint_poses[joint1];
	JointPose& b = joint_poses[joint2];

	float x = a.position.x - b.position.x,
		  y = a.position.y - b.position.y,
		  z = a.position.z - b.position.z;
	return sqrt(x * x + y * y + z * z);
}

double dot(Vec3 vector1, Vec3 vector2) {
	double sum = 0;

	sum = (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z
			* vector2.z);

	return sum;

}

double crossNorm(Vec3 vector1, Vec3 vector2) {
	return sqrt(pow(vector1.y * vector2.z - vector2.y * vector1.z, 2.0) + pow(
			vector1.x * vector2.z - vector2.x * vector1.z, 2.0) + pow(vector1.x
			* vector2.y - vector2.x * vector1.y, 2.0));
}

int argmin(std::vector<double> vec) {

	int res = 0;

	for (int i = 0; i < vec.size(); i++) {

		if (vec[i] < vec[res]) {
			res = i;
		}

	}

	return res;

}

int argmin2(std::vector<double> vec, int argmin) {

	int res = 0;

	for (int i = 0; i < vec.size(); i++) {

		if (vec[i] < vec[res] && i != argmin) {
			res = i;
		}

	}

	return res;

}

float smoothFunction(float x, float l, float p, float t) {
	float b = -1.0 / (p / 100 * l) * log(2.0 * t);

	float s = 1.0 / 2.0 * exp(-x * b);

	if (s < t) {
		s = 0;
	}

	return s;

}

Skeleton::Pose::PointRigging Skeleton::Pose::getRiggingForPoint(float x, float y, float z)
{
	Vec3 c;
	c.x = x;
	c.y = y;
	c.z = z;



	/**************** start single point weight calculation procedure *****************/
	std::map<Bone, double> distances;


	// Compute distances from point to bones
	std::set<Bone> seen_bones;
	for (int boneIndex = FirstBone; boneIndex <= LastBone; boneIndex++) {

		Bone bone = (Bone) boneIndex;
		Joint parent = Skeleton::GetBoneJoints(bone).parent;
		Joint child = Skeleton::GetBoneJoints(bone).child;

		Vec3 ac;
		ac.x = c.x - joint_poses[parent].position.x;
		ac.y = c.y - joint_poses[parent].position.y;
		ac.z = c.z - joint_poses[parent].position.z;

		Vec3 bc;
		bc.x = c.x - joint_poses[child].position.x;
		bc.y = c.y - joint_poses[child].position.y;
		bc.z = c.z - joint_poses[child].position.z;

		Vec3 boneVec = joint_poses[child].position - joint_poses[parent].position;
		double length = sqrt(dot(boneVec, boneVec));
		double proj = dot(boneVec, ac) / length;

		double distance;
		if (proj > length) {

			distance = sqrt(dot(bc, bc));

		} else if (proj < 0) {
			distance = sqrt(dot(ac, ac));
		} else {
			distance = crossNorm(boneVec, ac) / length;
		}

		distances[bone] = distance;

		//std::cout << length << std::endl;

	}

	// Find 2 nearest bones
	Bone bone1 = argmin(distances);
	Bone bone2 = argmin2(distances, arg1);
	seen_bones.insert(arg1);
	seen_bones.insert(arg2);

	double d1 = distances[arg1];
	double d2 = distances[arg2];

	// Computes the weight
	float l1 = sqrt(dot(bones[arg1], bones[arg1])); // Length nearest bone
	float th = 0.1; // Zero threshold
	float smoothProp = 20; // Pourcentage of the limb affected by smooth skining.

	weight = smoothFunction(sqrt(d2 * d2 - d1 * d1), l1, smoothProp, th);



	/************************* end single point weighting routine ****************************/
	/* arg1, 1-weight
	 * arg2, weight
	 */
}










// TODO: check that these are right, how are the mesh/bone aligned?
/*void offsetUp(JointPose& xform, float length) {
	xform.position.y -= length;
}
void offsetLeft(JointPose& xform, float length) {
	xform.position.x -= length;
}*/

/*
 * Find median bone lengths
 */
/*Skeleton::Pose::Ptr Skeleton::Pose::GetCanonical(Skeleton::Pose::List& poses) {
	// FIXME store bone lengths, use to build skeleton, iterate over bones

	using Body::Skeleton;
	// gather lengths
	int num_bones = poses.front()->getJointPoses().size();
	std::vector<float> all_bone_lengths[num_bones];
	for (Pose::List::const_iterator it = poses.begin(); it != poses.end(); ++it) {
		int j = 0;
		// uh, oh, this assumes that all joints are in the same order in each map
		// FIXME
		const JointPoses& joints = (*it)->getJointPoses();
		for (JointPoses::const_iterator jit = joints.begin(); jit
				!= joints.end(); ++jit) {
			if (jit->first == T) { // messy hack, store pelvis width in "torso"
				all_bone_lengths[j].push_back((**it).getBoneLength(LHI, RHI));
			} else {
				all_bone_lengths[j].push_back((**it).getBoneLength(jit->first));
			}
			j++;
		}
	}
	// get medians
	int median_index = all_bone_lengths[0].size() / 2;
	std::map<Joint, float> bone_lengths;
	int j = 0;
	const JointPoses& joints = poses.front()->getJointPoses(); // just for the key ordering
	for (JointPoses::const_iterator jit = joints.begin(); jit != joints.end(); ++jit) {
		std::sort(all_bone_lengths[j].begin(), all_bone_lengths[j].end());
		bone_lengths[jit->first] = all_bone_lengths[j][median_index];
		j++;
	}
	// build pose
	Pose::Ptr canon(new Pose());
	XnSkeletonJointTransformation xform;
	// identity
	float *o = xform.orientation.orientation.elements;
	o[0] = 1;
	o[1] = 0;
	o[2] = 0;
	o[3] = 0;
	o[4] = 1;
	o[5] = 0;
	o[6] = 0;
	o[7] = 0;
	o[8] = 1;
	xform.position.position.X = 0;
	xform.position.position.Y = 0;
	xform.position.position.Z = 0;

	// go down limbs, set positions, update in place
	std::stack<XnVector3D> stk; // old positions, for backtracking
	stk.push(xform.position.position); // remember for hips
	canon->setTransformationForJointKey(T, xform);

	// head/neck
	offsetUp(xform, bone_lengths[N]);
	canon->setTransformationForJointKey(N, xform);
	stk.push(xform.position.position); // remember for shoulders
	offsetUp(xform, bone_lengths[H]);
	canon->setTransformationForJointKey(H, xform);

	// arms
	xform.position.position = stk.top();
	offsetLeft(xform, bone_lengths[LS]);
	canon->setTransformationForJointKey(LS, xform);
	offsetLeft(xform, bone_lengths[LE]);
	canon->setTransformationForJointKey(LE, xform);
	offsetLeft(xform, bone_lengths[LH]);
	canon->setTransformationForJointKey(LH, xform);

	xform.position.position = stk.top();
	offsetLeft(xform, -bone_lengths[RS]);
	canon->setTransformationForJointKey(RS, xform);
	offsetLeft(xform, -bone_lengths[RE]);
	canon->setTransformationForJointKey(RE, xform);
	offsetLeft(xform, -bone_lengths[RH]);
	canon->setTransformationForJointKey(RH, xform);

	stk.pop(); // done with upper body
	// figure out the pelvic triangle dimensions
	float pelvisBase = bone_lengths[T]; // hack, see above
	float pelvisTop = bone_lengths[LHI] + bone_lengths[RHI];
	float hipDip = sqrt(pelvisTop * pelvisTop - pelvisBase * pelvisBase) / 2;

	// legs
	xform.position.position = stk.top();
	offsetUp(xform, -hipDip);
	offsetLeft(xform, pelvisBase / 2);
	canon->setTransformationForJointKey(LHI, xform);
	offsetUp(xform, -bone_lengths[LK]);
	canon->setTransformationForJointKey(LK, xform);
	offsetUp(xform, -bone_lengths[LF]);
	canon->setTransformationForJointKey(LF, xform);

	xform.position.position = stk.top();
	offsetUp(xform, -hipDip);
	offsetLeft(xform, -pelvisBase / 2);
	canon->setTransformationForJointKey(RHI, xform);
	offsetUp(xform, -bone_lengths[RK]);
	canon->setTransformationForJointKey(RK, xform);
	offsetUp(xform, -bone_lengths[RF]);
	canon->setTransformationForJointKey(RF, xform);

	return canon;
}*/


}
