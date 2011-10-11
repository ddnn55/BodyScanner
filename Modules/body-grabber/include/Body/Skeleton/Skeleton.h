/*
 * Skeleton.h
 *
 *  Created on: Oct 5, 2011
 *      Author: stolrsky
 */

#ifndef SKELETON_H_
#define SKELETON_H_

#include <vector>
#include <map>
#include <string>

#include <boost/shared_ptr.hpp>

#include <XnCppWrapper.h>

//#include <Body/Skeleton/Joint.h>


namespace Body
{



	enum Joint {
		LH, LE, LS, LHI, LK, LF, N, H, T, RH, RE, RS, RHI, RK, RF
	};
	#define FirstJoint LH
	#define LastJoint RF
	#define SKELETON_NUMBER_OF_JOINTS ((int)Body::LastJoint + 1)

	enum Bone {
		LH2E,
		LE2S,
		N2H,
		RS2E,
		RE2H,
		LH2K,
		RH2K,
		LK2F,
		TORSO,
		TORSO1,
		TORSO2,
		TORSO3,
		TORSO4,
		RK2F
	};
	#define FirstBone LH2E
	#define LastBone RK2F

	struct JointPair {
		Joint parent;
		Joint child;
	};

	class Skeleton
	{
public:
		typedef XnSkeletonJointTransformation JointPose;

	public:
		typedef std::vector< std::pair< Joint, XnSkeletonJoint > > JointList;

		static JointList GetJointList()
		{
			JointList joint_list;

			for(int j = 0; j < GetJointKeys().size(); j++)
			{
				joint_list.push_back(std::pair< Joint, XnSkeletonJoint >(GetJointKeys()[j], GetJointXnKeys()[j]));
			}

			return joint_list;
		}

		static std::vector<Joint> GetJointKeys()
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

		static std::vector<XnSkeletonJoint> GetJointXnKeys()
		{
			std::vector<XnSkeletonJoint> joint_list;

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
		}

		static JointPair GetBoneJoints(const Bone bone);

		//class Joint {
		//public:
		//	typedef XnSkeletonJointTransformation Pose;
		//};

		class Pose {
		public:
			typedef boost::shared_ptr<Pose> Ptr;
			typedef std::map<Joint, Body::Skeleton::JointPose> JointPoses;

			Pose();
			virtual ~Pose();

			//const std::string toYaml() const;

			void setTransformationForJointKey(Joint joint, XnSkeletonJointTransformation transformation);
			Body::Skeleton::JointPose operator[](Joint joint_key);

			JointPoses getJointPoses();
			
			/**
			 * Get the length of the bone in this pose.
			 * IDs are as defined by BodySegmentation.h
			 */
			float getBoneLength(const Bone bone);
			float getJointDistance(const Joint joint1, const Joint joint2);
			
			typedef std::vector<Body::Skeleton::Pose::Ptr> List;
			/*static Body::Skeleton::Pose::Ptr GetCanonical(List& poses);*/

		private:
			JointPoses joint_poses;
		};



	};

}

#endif /* SKELETON_H_ */
