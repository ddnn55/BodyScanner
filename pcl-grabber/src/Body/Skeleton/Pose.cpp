/*
 * Pose.cpp
 *
 *  Created on: Oct 5, 2011
 *      Author: stolrsky
 */

#include <Body/Skeleton/Pose.h>

#include <string>
#include <sstream>

namespace Body {

namespace Skeleton {

Pose::Pose() {
	// TODO Auto-generated constructor stub

}

Pose::~Pose() {
	// TODO Auto-generated destructor stub
}

const std::string Pose::toYaml() const
{
	std::stringstream yaml;

	// TODO abstract this..? protocol buffer?

	yaml << "head:" << std::endl;
	yaml << "  translation:" << std::endl;
	yaml << "    - x -- " << head.position.X << std::endl;
	yaml << "    - y -- " << head.position.Y << std::endl;
	yaml << "    - z -- " << head.position.Z << std::endl;
	yaml << "neck:" << std::endl;
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
	yaml << std::endl;

	return yaml.str();

}

}

}
