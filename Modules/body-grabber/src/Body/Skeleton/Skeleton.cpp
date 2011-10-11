/*
 * Skeleton.h
 *
 *  Created on: Oct 5, 2011
 *      Author: stolrsky
 */

#include <Body/Skeleton/Skeleton.h>

namespace Body
{





	/*static JointList Skeleton::GetJointList()
	{
		JointList joint_list;

		for(int j = 0; j < GetJointKeys().size(); j++)
		{
			joint_list.push_back(std::pair< Joint, XnSkeletonJoint >(GetJointKeys()[j], GetJointXnKeys()[j]));
		}

		return joint_list;
	}*/

Body::JointPair Skeleton::GetBoneJoints(const Bone bone) {

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

	Joint Skeleton::JointNameToJointKey(std::string joint_name)
	{
		std::map<std::string, Joint> joint_list;

		joint_list["head"] = H;
		joint_list["neck"] = N;
		joint_list["torso"] = T;

		joint_list["left_shoulder"] = LS;
		joint_list["left_elbow"] = LE;
		joint_list["left_hand"] = LH;

		joint_list["right_shoulder"] = RS;
		joint_list["right_elbow"] = RE;
		joint_list["right_hand"] = RH;

		joint_list["left_hip"] = LHI;
		joint_list["left_knee"] = LK;
		joint_list["left_foot"] = LF;

		joint_list["right_hip"] = RHI;
		joint_list["right_knee"] = RK;
		joint_list["right_foot"] = RF;

		return joint_list[joint_name];
	}

	std::string Skeleton::GetJointStringKeyByKey(Joint k)
	{
		std::map<Joint, std::string> joint_list;

		joint_list[H] = "head";
		joint_list[N] = "neck";
		joint_list[T] = "torso";

		joint_list[LS] = "left_shoulder";
		joint_list[LE] = "left_elbow";
		joint_list[LH] = "left_hand";

		joint_list[RS] = "right_shoulder";
		joint_list[RE] = "right_elbow";
		joint_list[RH] = "right_hand";

		joint_list[LHI] = "left_hip";
		joint_list[LK] = "left_knee";
		joint_list[LF] = "left_foot";

		joint_list[RHI] = "right_hip";
		joint_list[RK] = "right_knee";
		joint_list[RF] = "right_foot";

		return joint_list[k];
	}

	std::vector<Joint> Skeleton::GetJointKeys()
	{
		std::vector<Joint> joint_list;

		joint_list.push_back(H);
		joint_list.push_back(N);
		joint_list.push_back(T);

		joint_list.push_back(LS);
		joint_list.push_back(LE);
		joint_list.push_back(LH);

		joint_list.push_back(RS);
		joint_list.push_back(RE);
		joint_list.push_back(RH);

		joint_list.push_back(LHI);
		joint_list.push_back(LK);
		joint_list.push_back(LF);

		joint_list.push_back(RHI);
		joint_list.push_back(RK);
		joint_list.push_back(RF);

		return joint_list;
	}

	/*std::vector<Skeleton::JointPose> Skeleton::GetJointXnKeys()
	{
		std::vector<JointPose> joint_list;

		joint_list.push_back(XN_SKEL_HEAD);
		joint_list.push_back(XN_SKEL_NECK);
		joint_list.push_back(XN_SKEL_TORSO);

		joint_list.push_back(XN_SKEL_LEFT_SHOULDER);
		joint_list.push_back(XN_SKEL_LEFT_ELBOW);
		joint_list.push_back(XN_SKEL_LEFT_HAND);

		joint_list.push_back(XN_SKEL_RIGHT_SHOULDER);
		joint_list.push_back(XN_SKEL_RIGHT_ELBOW);
		joint_list.push_back(XN_SKEL_RIGHT_HAND);

		joint_list.push_back(XN_SKEL_LEFT_HIP);
		joint_list.push_back(XN_SKEL_LEFT_KNEE);
		joint_list.push_back(XN_SKEL_LEFT_FOOT);

		joint_list.push_back(XN_SKEL_RIGHT_HIP);
		joint_list.push_back(XN_SKEL_RIGHT_KNEE);
		joint_list.push_back(XN_SKEL_RIGHT_FOOT);

		return joint_list;
	}*/

	//JointPair Skeleton::GetBoneJoints(const Bone bone);



}

