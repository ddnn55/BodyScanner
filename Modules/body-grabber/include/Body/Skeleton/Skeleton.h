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


namespace Body
{

	class Skeleton
	{
	public:
		typedef std::vector< std::pair< std::string, XnSkeletonJoint > > JointList;

		static JointList GetJointList()
		{
			JointList joint_list;

			for(int j = 0; j < GetJointStringKeys().size(); j++)
			{
				joint_list.push_back(std::pair< std::string, XnSkeletonJoint >(GetJointStringKeys()[j], GetJointXnKeys()[j]));
			}

			return joint_list;
		}

		static std::vector<std::string> GetJointStringKeys()
		{
			std::vector<std::string> joint_list;

			joint_list.push_back("head");
			joint_list.push_back("neck");
			joint_list.push_back("torso");

			joint_list.push_back("left_shoulder");
			joint_list.push_back("left_elbow");
			joint_list.push_back("left_hand");

			joint_list.push_back("right_shoulder");
			joint_list.push_back("right_elbow");
			joint_list.push_back("right_hand");

			joint_list.push_back("left_hip");
			joint_list.push_back("left_knee");
			joint_list.push_back("left_foot");

			joint_list.push_back("right_hip");
			joint_list.push_back("right_knee");
			joint_list.push_back("right_foot");

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

		class Joint {
		public:
			typedef XnSkeletonJointTransformation Pose;
		};

		class Pose {
		public:
			typedef boost::shared_ptr<Pose> Ptr;
			typedef std::map<std::string, Body::Skeleton::Joint::Pose> JointPoses;

			Pose();
			virtual ~Pose();

			const std::string toYaml() const;

			void setTransformationForJointKey(std::string, XnSkeletonJointTransformation transformation);
			Body::Skeleton::Joint::Pose operator[](std::string joint_key);

			JointPoses getJointPoses();
			
			/**
			 * Get the length of the bone in this pose.
			 * IDs are as defined by BodySegmentation.h
			 */
			float getBoneLength(const std::string key);
			float getBoneLength(const std::string key1, const std::string key2);
			
			typedef std::vector<Body::Skeleton::Pose::Ptr> List;
			static Body::Skeleton::Pose::Ptr GetCanonical(List& poses);

		private:
			JointPoses joint_poses;
		};



	};

}

#endif /* SKELETON_H_ */
