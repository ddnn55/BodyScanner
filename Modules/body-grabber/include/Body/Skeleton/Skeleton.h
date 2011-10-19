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


//#include <XnCppWrapper.h>

//#include <Body/Skeleton/Joint.h>
#include <Body/Body.h>
#include <Body/Skeleton/SkeletonYaml.h>


namespace Body
{



	enum Joint {
		LH, LE, LS, LHI, LK, LF, N, H, T, RH, RE, RS, RHI, RK, RF
	};
	#define FirstJoint LH
	#define LastJoint RF
	#define SKELETON_NUMBER_OF_JOINTS ((int)Body::LastJoint + 1)

	enum Bone { // TODO should be BoneName. Bone should be an actual bone with a length and a name and a related BonePose.
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

	typedef struct JointPair {
		Joint parent;
		Joint child;
	} JointPair;

	typedef Vec3 Translation;
	typedef float Matrix3x3[9];

	class Skeleton
	{
	public:
		class JointPose
		{
		public:
			Translation position;
			float position_confidence;
			Matrix3x3 orientation;
			float orientation_confidence;
		};
		//typedef XnSkeletonJointTransformation JointPose; // TODO make our own JointPose class
		//typedef std::vector< std::pair< Joint, XnSkeletonJoint > > JointList;



		// TODO clean these up
		//static JointList GetJointList();
		static Joint JointNameToJointKey(std::string joint_name);
		static std::string GetJointStringKeyByKey(Joint k);
		static std::vector<Joint> GetJointKeys();
		//static std::vector<XnSkeletonJoint> GetJointXnKeys();
		static JointPair GetBoneJoints(const Bone bone);

		class Pose { // TODO move to own file
		public:
			typedef boost::shared_ptr<Pose> Ptr;
			typedef std::map<Joint, Body::Skeleton::JointPose> JointPoses;
			typedef struct PointRigging
			{
				Bone bone1;
				Bone bone2;
				float weight1;
				float weight2;
			} PointRigging;

			Pose();
			// TODO merge SkeletonYaml and Skeleton::Pose
			Pose(SkeletonYaml skeletonYaml);
			virtual ~Pose();

			void draw();

			const std::string toYaml() const;

			void setTransformationForJointKey(Joint joint, JointPose transformation);
			Body::Skeleton::JointPose& operator[](Joint joint_key);

			JointPoses getJointPoses() const;
			
			/**
			 * Get the length of the bone in this pose.
			 * IDs are as defined by BodySegmentation.h
			 */
			float getBoneLength(const Bone bone);
			float getJointDistance(const Joint joint1, const Joint joint2);
			
			PointRigging getRiggingForPoint(float x, float y, float z);

			typedef std::vector<Body::Skeleton::Pose::Ptr> List;
			/*static Body::Skeleton::Pose::Ptr GetCanonical(List& poses);*/

		private:
			JointPoses joint_poses;
		};



	};

}

#endif /* SKELETON_H_ */
