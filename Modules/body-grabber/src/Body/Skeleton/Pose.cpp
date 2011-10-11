/*
 * Pose.cpp
 *
 *  Created on: Oct 5, 2011
 *      Author: stolrsky
 */

#include <Body/Skeleton/Skeleton.h>

#include <string>
#include <sstream>
#include <cmath>
#include <stack>
#include <algorithm>

namespace Body {

Skeleton::Pose::Pose() {
	// TODO Auto-generated constructor stub

}

Skeleton::Pose::~Pose() {
	// TODO Auto-generated destructor stub
}

void Skeleton::Pose::setTransformationForJointKey(Joint joint_key,
		XnSkeletonJointTransformation transformation) {
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

 // TODO abstract this..? protocol buffer?

 //for(JointPoses::const_iterator j = joint_poses.begin(); j != joint_poses.end(); j++)
 for(int jointIndex = FirstJoint; jointIndex <= LastJoint; jointIndex++)
 {
	 Joint j = (Joint) jointIndex;

	 yaml << GetJointStringKeyByKey(j) << ":" << std::endl;
	 yaml << "  position:" << std::endl;
	 yaml << "    - confidence -- " << getJointPoses()[j].position.fConfidence << std::endl;
	 yaml << "    translation:" << std::endl;
	 yaml << "      - x -- " << getJointPoses()[j].position.position.X << std::endl;
	 yaml << "      - y -- " << getJointPoses()[j].position.position.Y << std::endl;
	 yaml << "      - z -- " << getJointPoses()[j].position.position.Z << std::endl;
	 yaml << "  orientation:" << std::endl;
	 yaml << "    - confidence -- " << getJointPoses()[j].orientation.fConfidence << std::endl;
	 yaml << "    matrix3x3:" << std::endl;
	 yaml << "      - " << getJointPoses()[j].orientation.orientation.elements[0] << std::endl;
	 yaml << "      - " << getJointPoses()[j].orientation.orientation.elements[1] << std::endl;
	 yaml << "      - " << getJointPoses()[j].orientation.orientation.elements[2] << std::endl;
	 yaml << "      - " << getJointPoses()[j].orientation.orientation.elements[3] << std::endl;
	 yaml << "      - " << getJointPoses()[j].orientation.orientation.elements[4] << std::endl;
	 yaml << "      - " << getJointPoses()[j].orientation.orientation.elements[5] << std::endl;
	 yaml << "      - " << getJointPoses()[j].orientation.orientation.elements[6] << std::endl;
	 yaml << "      - " << getJointPoses()[j].orientation.orientation.elements[7] << std::endl;
	 yaml << "      - " << getJointPoses()[j].orientation.orientation.elements[8] << std::endl;
 }



 /*yaml << "neck:" << std::endl; /////////////////////////////////////////////////////////////////////////////////////
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << neck.position.X << std::endl;
 yaml << "    - y -- " << neck.position.Y << std::endl;
 yaml << "    - z -- " << neck.position.Z << std::endl;
 yaml << "torso:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << torso.position.X << std::endl;
 yaml << "    - y -- " << torso.position.Y << std::endl;
 yaml << "    - z -- " << torso.position.Z << std::endl;
 yaml << std::endl;

 yaml << "left_shoulder:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << left_shoulder.position.X << std::endl;
 yaml << "    - y -- " << left_shoulder.position.Y << std::endl;
 yaml << "    - z -- " << left_shoulder.position.Z << std::endl;
 yaml << "left_elbow:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << left_elbow.position.X << std::endl;
 yaml << "    - y -- " << left_elbow.position.Y << std::endl;
 yaml << "    - z -- " << left_elbow.position.Z << std::endl;
 yaml << "left_hand:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << left_hand.position.X << std::endl;
 yaml << "    - y -- " << left_hand.position.Y << std::endl;
 yaml << "    - z -- " << left_hand.position.Z << std::endl;
 yaml << std::endl;

 yaml << "right_shoulder:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << right_shoulder.position.X << std::endl;
 yaml << "    - y -- " << right_shoulder.position.Y << std::endl;
 yaml << "    - z -- " << right_shoulder.position.Z << std::endl;
 yaml << "right_elbow:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << right_elbow.position.X << std::endl;
 yaml << "    - y -- " << right_elbow.position.Y << std::endl;
 yaml << "    - z -- " << right_elbow.position.Z << std::endl;
 yaml << "right_hand:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << right_hand.position.X << std::endl;
 yaml << "    - y -- " << right_hand.position.Y << std::endl;
 yaml << "    - z -- " << right_hand.position.Z << std::endl;
 yaml << std::endl;

 yaml << "left_hip:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << left_hip.position.X << std::endl;
 yaml << "    - y -- " << left_hip.position.Y << std::endl;
 yaml << "    - z -- " << left_hip.position.Z << std::endl;
 yaml << "left_knee:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << left_knee.position.X << std::endl;
 yaml << "    - y -- " << left_knee.position.Y << std::endl;
 yaml << "    - z -- " << left_knee.position.Z << std::endl;
 yaml << "left_foot:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << left_foot.position.X << std::endl;
 yaml << "    - y -- " << left_foot.position.Y << std::endl;
 yaml << "    - z -- " << left_foot.position.Z << std::endl;
 yaml << std::endl;

 yaml << "right_hip:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << right_hip.position.X << std::endl;
 yaml << "    - y -- " << right_hip.position.Y << std::endl;
 yaml << "    - z -- " << right_hip.position.Z << std::endl;
 yaml << "right_knee:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << right_knee.position.X << std::endl;
 yaml << "    - y -- " << right_knee.position.Y << std::endl;
 yaml << "    - z -- " << right_knee.position.Z << std::endl;
 yaml << "right_foot:" << std::endl;
 yaml << "  translation:" << std::endl;
 yaml << "    - x -- " << right_foot.position.X << std::endl;
 yaml << "    - y -- " << right_foot.position.Y << std::endl;
 yaml << "    - z -- " << right_foot.position.Z << std::endl;
 yaml << std::endl;*/

 return yaml.str();

 }

JointPair Skeleton::GetBoneJoints(const Bone bone) {

	JointPair p;

	switch (bone) {
	case LH2E:
		p.parent = LE;
		p.child = LH;
		break;

	case LE2S:
		p.parent = LS;
		p.child = LE;
		break;

	case N2H:
		p.parent = N;
		p.child = H;
		break;

	case RS2E:
		p.parent = RS;
		p.child = RE;
		break;

	case RE2H:
		p.parent = RE;
		p.child = RH;
		break;

	case LH2K:
		p.parent = LHI;
		p.child = LK;
		break;

	case RH2K:
		p.parent = RHI;
		p.child = RK;
		break;

	case LK2F:
		p.parent = LK;
		p.child = LF;
		break;

	case RK2F:
		p.parent = RK;
		p.child = RF;
		break;

	case TORSO:
		p.parent = T;
		p.child = N;
		break;

	case TORSO1:
		p.parent = T;
		p.child = RHI;
		break;

	case TORSO2:
		p.parent = T;
		p.child = LHI;
		break;

	case TORSO3:
		p.parent = T;
		p.child = LS;
		break;

	case TORSO4:
		p.parent = T;
		p.child = RS;
		break;

	default:
		assert(false);
		break;

	}

	return p;

}

float Skeleton::Pose::getBoneLength(const Bone bone) {
	JointPair endpoints = GetBoneJoints(bone);

	return getJointDistance(endpoints.parent, endpoints.child);
}

float Skeleton::Pose::getJointDistance(const Joint joint1, const Joint joint2) {
	XnSkeletonJointTransformation& a = joint_poses[joint1];
	XnSkeletonJointTransformation& b = joint_poses[joint2];

	float x = a.position.position.X - b.position.position.X, y =
			a.position.position.Y - b.position.position.Y, z =
			a.position.position.Z - b.position.position.Z;
	return sqrt(x * x + y * y + z * z);
}

// TODO: check that these are right, how are the mesh/bone aligned?
void offsetUp(XnSkeletonJointTransformation& xform, float length) {
	xform.position.position.Y -= length;
}
void offsetLeft(XnSkeletonJointTransformation& xform, float length) {
	xform.position.position.X -= length;
}

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
