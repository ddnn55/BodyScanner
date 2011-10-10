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

void Skeleton::Pose::setTransformationForJointKey(std::string joint_key, XnSkeletonJointTransformation transformation)
{
	joint_poses[joint_key] = transformation;
}

Skeleton::Joint::Pose Skeleton::Pose::operator[](std::string joint_key)
{
	return joint_poses[joint_key];
}

Skeleton::Pose::JointPoses Skeleton::Pose::getJointPoses()
{

	return joint_poses;
}

const std::string Skeleton::Pose::toYaml() const
{
	std::stringstream yaml;

	// TODO abstract this..? protocol buffer?

	for(JointPoses::const_iterator j = joint_poses.begin(); j != joint_poses.end(); j++)
	{
		yaml << (*j).first << ":" << std::endl;
		yaml << "  position:" << std::endl;
		yaml << "    - confidence -- " << (*j).second.position.fConfidence << std::endl;
		yaml << "    translation:" << std::endl;
		yaml << "      - x -- " << (*j).second.position.position.X << std::endl;
		yaml << "      - y -- " << (*j).second.position.position.Y << std::endl;
		yaml << "      - z -- " << (*j).second.position.position.Z << std::endl;
		yaml << "  orientation:" << std::endl;
		yaml << "    - confidence -- " << (*j).second.orientation.fConfidence << std::endl;
		yaml << "    matrix3x3:" << std::endl;
		yaml << "      - " << (*j).second.orientation.orientation.elements[0] << std::endl;
		yaml << "      - " << (*j).second.orientation.orientation.elements[1] << std::endl;
		yaml << "      - " << (*j).second.orientation.orientation.elements[2] << std::endl;
		yaml << "      - " << (*j).second.orientation.orientation.elements[3] << std::endl;
		yaml << "      - " << (*j).second.orientation.orientation.elements[4] << std::endl;
		yaml << "      - " << (*j).second.orientation.orientation.elements[5] << std::endl;
		yaml << "      - " << (*j).second.orientation.orientation.elements[6] << std::endl;
		yaml << "      - " << (*j).second.orientation.orientation.elements[7] << std::endl;
		yaml << "      - " << (*j).second.orientation.orientation.elements[8] << std::endl;
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
	
float Skeleton::Pose::getBoneLength(const std::string key) {
	std::string parent;
	
	// FIXME: this duplicates the code in BodySegmentation::map
	// switch away from string keys eventually
	     if(key.compare("neck")
	     || key.compare("left_hip")
	     || key.compare("right_hip")) parent = "torso";
	else if(key.compare("left_shoulder")
	     || key.compare("right_shoulder")
	     || key.compare("head")) parent = "neck";
	else if(key.compare("left_elbow")) parent = "left_shoulder";
	else if(key.compare("right_elbow")) parent = "right_shoulder";
	else if(key.compare("left_hand")) parent = "left_elbow";
	else if(key.compare("right_hand")) parent = "right_elbow";
	else if(key.compare("left_knee")) parent = "left_hip";
	else if(key.compare("right_knee")) parent = "right_hip";
	else if(key.compare("left_foot")) parent = "left_knee";
	else if(key.compare("right_foot")) parent = "right_knee";
	else assert(false);

	return getBoneLength(key, parent);
}

float Skeleton::Pose::getBoneLength(const std::string key1, const std::string key2) {
	XnSkeletonJointTransformation& a = joint_poses[key1];
	XnSkeletonJointTransformation& b = joint_poses[key2];
	
	float
	x = a.position.position.X-b.position.position.X,
	y = a.position.position.Y-b.position.position.Y,
	z = a.position.position.Z-b.position.position.Z;
	return sqrt(x*x + y*y + z*z);
}


// TODO: check that these are right, how are the mesh/bone aligned?
void offsetUp(XnSkeletonJointTransformation& xform, float length) {
	xform.position.position.Y -= length;
}
void offsetLeft(XnSkeletonJointTransformation& xform, float length) {
	xform.position.position.X -= length;
}

Skeleton::Pose::Ptr Skeleton::Pose::GetCanonical(Skeleton::Pose::List& poses)
{
	using Body::Skeleton;
	// gather lengths
  int num_bones = poses.front()->getJointPoses().size();
  std::vector<float> all_bone_lengths[num_bones];
  for(Pose::List::const_iterator it = poses.begin(); it != poses.end(); ++it)
	{
		int j = 0;
		// uh, oh, this assumes that all joints are in the same order in each map
		// FIXME
		const JointPoses& joints = (*it)->getJointPoses();
		for(JointPoses::const_iterator jit = joints.begin(); jit != joints.end(); ++jit)
		{
			if(jit->first.compare("torso") == 0) { // messy hack, store pelvis width in "torso"
				all_bone_lengths[j].push_back((**it).getBoneLength("left_hip", "right_hip"));
			} else {
      	all_bone_lengths[j].push_back((**it).getBoneLength(jit->first));
			}
			j++;
    }
  }
	// get medians
  int median_index = all_bone_lengths[0].size()/2;
  std::map<std::string,float> bone_lengths;
	int j = 0;
	const JointPoses& joints = poses.front()->getJointPoses(); // just for the key ordering
	for(JointPoses::const_iterator jit = joints.begin(); jit != joints.end(); ++jit)
	{
    std::sort(all_bone_lengths[j].begin(), all_bone_lengths[j].end());
    bone_lengths[jit->first] = all_bone_lengths[j][median_index];
		j++;
  }
	// build pose
	Pose::Ptr canon(new Pose());
	XnSkeletonJointTransformation xform;
	// identity
	float *o = xform.orientation.orientation.elements;
	o[0] = 1; o[1] = 0; o[2] = 0;
	o[3] = 0; o[4] = 1; o[5] = 0;
	o[6] = 0; o[7] = 0; o[8] = 1;
	xform.position.position.X = 0;
	xform.position.position.Y = 0;
	xform.position.position.Z = 0;
	
	// go down limbs, set positions, update in place
	std::stack<XnVector3D> stk; // old positions, for backtracking 
	stk.push(xform.position.position); // remember for hips
  canon->setTransformationForJointKey("torso", xform);
	
	// head/neck
	offsetUp(xform, bone_lengths["neck"]); canon->setTransformationForJointKey("neck", xform);
	stk.push(xform.position.position); // remember for shoulders
	offsetUp(xform, bone_lengths["head"]); canon->setTransformationForJointKey("head", xform);
	
	// arms
	xform.position.position = stk.top();
	offsetLeft(xform, bone_lengths["left_shoulder"]); canon->setTransformationForJointKey("left_shoulder", xform);
	offsetLeft(xform, bone_lengths["left_elbow"]); canon->setTransformationForJointKey("left_elbow", xform);
	offsetLeft(xform, bone_lengths["left_hand"]); canon->setTransformationForJointKey("left_hand", xform);

	xform.position.position = stk.top();
	offsetLeft(xform, -bone_lengths["right_shoulder"]); canon->setTransformationForJointKey("right_shoulder", xform);
	offsetLeft(xform, -bone_lengths["right_elbow"]); canon->setTransformationForJointKey("right_elbow", xform);
	offsetLeft(xform, -bone_lengths["right_hand"]); canon->setTransformationForJointKey("right_hand", xform);
	
	stk.pop(); // done with upper body
	// figure out the pelvic triangle dimensions
	float pelvisBase = bone_lengths["torso"]; // hack, see above
	float pelvisTop = bone_lengths["left_hip"]+bone_lengths["right_hip"];
	float hipDip = sqrt(pelvisTop*pelvisTop - pelvisBase*pelvisBase)/2;

	// legs
	xform.position.position = stk.top();
	offsetUp(xform, -hipDip); offsetLeft(xform, pelvisBase/2);
	canon->setTransformationForJointKey("left_hip", xform);
	offsetUp(xform, -bone_lengths["left_knee"]); canon->setTransformationForJointKey("left_knee", xform);
	offsetUp(xform, -bone_lengths["left_foot"]); canon->setTransformationForJointKey("left_foot", xform);

	xform.position.position = stk.top();
	offsetUp(xform, -hipDip); offsetLeft(xform, -pelvisBase/2);
	canon->setTransformationForJointKey("right_hip", xform);
	offsetUp(xform, -bone_lengths["right_knee"]); canon->setTransformationForJointKey("right_knee", xform);
	offsetUp(xform, -bone_lengths["right_foot"]); canon->setTransformationForJointKey("right_foot", xform);
	
	return canon;
}

}
