/*
 * BodySegmentation.cpp
 *
 *  Created on: Oct 8, 2011
 *      Author: webaba
 */

#include <Body/BodySegmentation.h>
#include <iostream>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include <string>
#include <sstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
//#include <Body/Skeleton/Joint.h>
#include "Body/Skeleton/SkeletonYaml.h"
#include "Body/Skin.h"
#include "time.h"
#include <set> // debug

#define NB_JOINTS 15
#define NB_BONES 14

double dot(pcl::PointXYZ vector1, pcl::PointXYZ vector2) {
	double sum = 0;

	sum = (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z
			* vector2.z);

	return sum;

}

double crossNorm(pcl::PointXYZ vector1, pcl::PointXYZ vector2) {
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

namespace Body {

BodySegmentation::BodySegmentation(std::string const skeletonfilename,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr bodycloud_, Skin *pskin_) {

	pskin = pskin_;

	bodycloud = bodycloud_;

	// Reserve space for the vectors
	joints = std::vector<pcl::PointXYZ>(NB_JOINTS);
	bones = std::vector<pcl::PointXYZ>(NB_BONES);
	limbs_clouds
			= std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(NB_BONES);

	SkeletonYaml skel(skeletonfilename.c_str());
	// create points to store the skeleton data
	for (int i = 0; i < SkeletonYaml::numJoints; i++) {
		joints[i]
				= pcl::PointXYZ(skel.translations[i][0] / 1000,
						-skel.translations[i][1] / 1000,
						skel.translations[i][2] / 1000);

	}

	initBones();

}

BodySegmentation::BodySegmentation(Skeleton::Pose *pose_, pcl::PointCloud<
		pcl::PointXYZRGB>::ConstPtr bodycloud_, Skin *pskin_) {

	pskin = pskin_;
	sk_pose = pose_;
	bodycloud = bodycloud_;

	joint_poses = sk_pose->getJointPoses();

	// Reserve space for the vectors
	joints = std::vector<pcl::PointXYZ>(NB_JOINTS);
	bones = std::vector<pcl::PointXYZ>(NB_BONES);
	limbs_clouds
			= std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(NB_BONES);

	// Initilizations

	//initIndexMap();
	initJoints();
	initBones();

}

void BodySegmentation::initJoints() {

	int i = 0;
	for (Skeleton::Pose::JointPoses::const_iterator j = joint_poses.begin(); j
			!= joint_poses.end(); ++j) {
		i = (*j).first;
		joints[i].x = (*j).second.position.position.X/1000;
		joints[i].y = -(*j).second.position.position.Y/1000;
		joints[i].z = (*j).second.position.position.Z/1000;
	}

}

void BodySegmentation::initBones() {

	for (int i = LH2E; i <= RK2F; i++) {

		Bone m = (Bone) i;

		limbs_clouds[m] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
				new pcl::PointCloud<pcl::PointXYZRGB>);
		limbs_clouds[m]->resize(bodycloud->size());

		int i1 = Skeleton::GetBoneJoints(m).parent;
		int i2 = Skeleton::GetBoneJoints(m).child;

		bones[m].x = joints[i2].x - joints[i1].x;
		bones[m].y = joints[i2].y - joints[i1].y;
		bones[m].z = joints[i2].z - joints[i1].z;

		// Code relative to skin

		//pskin->addParentJoint(Skeleton::GetBoneJoints(m).parent);

	}

	// Skin Bindings initilization
	pskin->setNumPoints(bodycloud->points.size());

}

/*void BodySegmentation::initIndexMap() {

	indexFromKey.insert(std::make_pair("head", H));
	indexFromKey.insert(std::make_pair("neck", N));
	indexFromKey.insert(std::make_pair("torso", T));

	indexFromKey.insert(std::make_pair("left_shoulder", LS));
	indexFromKey.insert(std::make_pair("left_elbow", LE));
	indexFromKey.insert(std::make_pair("left_hand", LH));

	indexFromKey.insert(std::make_pair("right_shoulder", RS));
	indexFromKey.insert(std::make_pair("right_elbow", RE));
	indexFromKey.insert(std::make_pair("right_hand", RH));

	indexFromKey.insert(std::make_pair("left_hip", LHI));
	indexFromKey.insert(std::make_pair("left_knee", LK));
	indexFromKey.insert(std::make_pair("left_foot", LF));

	indexFromKey.insert(std::make_pair("right_hip", RHI));
	indexFromKey.insert(std::make_pair("right_knee", RK));
	indexFromKey.insert(std::make_pair("right_foot", RF));

}*/



/*std::string BodySegmentation::getKeyFromBoneIndex(int bone) {

	std::string p;

	switch (bone) {
	case LH2E:
		p = "LH2E";
		break;

	case LE2S:
		p = "LE2S";
		break;

	case N2H:
		p = "N2H";
		break;

	case RS2E:
		p = "RS2E";
		break;

	case RE2H:
		p = "RE2H";
		break;

	case LH2K:
		p = "LH2K";
		break;

	case RH2K:
		p = "RH2K";
		break;

	case LK2F:
		p = "LK2F";
		break;

	case RK2F:
		p = "RK2F";
		break;

	case TORSO:
		p = "TORSO";
		break;

	case TORSO1:
		p = "TORSO1";
		break;

	case TORSO2:
		p = "TORSO2";
		break;

	case TORSO3:
		p = "TORSO3";
		break;

	case TORSO4:
		p = "TORSO4";
		break;

	default:
		p = "unknown";
		break;

	}

	return p;

}*/

/*int BodySegmentation::getIndexFromKey(std::string const key) {
	std::map<std::string,int>::iterator entry = indexFromKey.find(key);
	assert(entry != indexFromKey.end());
	return entry->second;
	/*
	//if(entry == indexFromKey.end()) {
		std::cout << "keys in indexFromKey:\n";
		for(std::map<std::string,int>::iterator it = indexFromKey.begin(); it != indexFromKey.end(); ++it) {
			std::cout << it->first << " ";
		}
		std::cout << "\n";
		assert(false);
	//}
	return entry->second;

}*/

void BodySegmentation::run() {

	pcl::PointXYZRGB c;
	pcl::PointXYZ ac;
	pcl::PointXYZ bc;

	// Temporary variables
	int j = 0;
	int arg1;
	int arg2;

	double length;
	double proj;
	double d1, d2;
	double weight;
	//input->points.resize(cloud->points.size());

	std::set<int> seen_bones;
	for (size_t i = 0; i < bodycloud->points.size(); i++) {
		std::vector<double> distances;
		double temp = 0;

		c = bodycloud->points[i];

		// Compute distances from point to bones

		for (int boneIndex = FirstBone; boneIndex <= LastBone; boneIndex++) {

			if(i == 0)
				std::cout << "(BodySegmentation) boneIndex - " << boneIndex << std::endl;

			Bone j = (Bone) boneIndex;
			Joint parent = Skeleton::GetBoneJoints(j).parent;
			Joint child = Skeleton::GetBoneJoints(j).child;
			ac.x = c.x - joints[parent].x;
			ac.y = c.y - joints[parent].y;
			ac.z = c.z - joints[parent].z;

			bc.x = c.x - joints[child].x;
			bc.y = c.y - joints[child].y;
			bc.z = c.z - joints[child].z;

			length = sqrt(dot(bones[j], bones[j]));
			proj = dot(bones[j], ac) / length;

			if (proj > length) {

				temp = sqrt(dot(bc, bc));

			} else if (proj < 0) {
				temp = sqrt(dot(ac, ac));
			} else {
				temp = crossNorm(bones[j], ac) / length;
			}

			distances.push_back(temp);

			//std::cout << length << std::endl;

		}

		// Find 2 nearest bones
		arg1 = argmin(distances);
		arg2 = argmin2(distances, arg1);
		seen_bones.insert(arg1);
		seen_bones.insert(arg2);

		d1 = distances[arg1];
		d2 = distances[arg2];

		// Computes the weight
		float l1 = sqrt(dot(bones[arg1], bones[arg1])); // Length nearest bone
		float th = 0.1; // Zero threshold
		float smoothProp = 20; // Pourcentage of the limb affected by smooth skining.

		weight = smoothFunction(sqrt(d2 * d2 - d1 * d1), l1, smoothProp, th);

		//std::cout << c << d1 << "(" << arg1 << ")" << "/" << d2  <<  "(" << arg2 << ")" << ": " << temps << std::endl;


		// Skin Code for nearest bone

		pskin->addPointToBone(i, arg1, 1-weight);

		// Add skin weight to second nearest bone if positive
		if (weight > 0) {
			
		pskin->addPointToBone(i, arg2, weight);


		}

		if(arg1 > 0) {
			// Vizualisation
			limbs_clouds[arg1]->points[i].x = c.x;
			limbs_clouds[arg1]->points[i].y = c.y;
			limbs_clouds[arg1]->points[i].z = c.z;

			limbs_clouds[arg1]->points[i].g = 2 * 255 * weight;
			limbs_clouds[arg1]->points[i].b = 0 * weight;
			limbs_clouds[arg1]->points[i].r = 255;
		}
	}

	std::cout << "seen bones (in segmentation)\n";
	for(std::set<int>::iterator it = seen_bones.begin(); it != seen_bones.end(); ++it) {
		std::cout << *it << " ";
	}
	std::cout << "\n";


}

/**
 * Visualization.
 */
/*void BodySegmentation::visualize(int index) {

	// Remove points from the body point cloud
	for (size_t i = 0; i < bodycloud->points.size(); i++) {

		if (limbs_clouds[index]->points[i].x == bodycloud->points[i].x) {
			bodycloud->points[i].x = 0;
			bodycloud->points[i].y = 0;
			bodycloud->points[i].z = 0;
		}

	}

	// Prepare visualization

	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(
			new pcl::visualization::PCLVisualizer("3D Viewer"));

	view->addCoordinateSystem(1.0);
	view->initCameraParameters();

	// Visualization of the joints position
	for (int m = LH; m <= RF; m++) {

		std::stringstream s;
		s << m;

		view->addSphere(joints[m], 0.025, s.str(), 0);
	}

	for (int m = LH2E; m <= RK2F; m++) {
		std::stringstream s;
		s << m << m << m;

		if (m == index) {
			view->addLine(joints[(int) (GetBoneJoints(m).y)], joints[(int) (GetBoneJoints(m).x)],
					255, 0, 0, s.str(), 0);
		} else {
			view->addLine(joints[(int) (GetBoneJoints(m).y)], joints[(int) (GetBoneJoints(m).x)],
					s.str(), 0);
		}
	}

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(limbs_clouds[index], 255, 0, 0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(bodycloud, 0, 120, 0);

	view->addPointCloud(limbs_clouds[index], "littlecloud");
	//view->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE*10, 1, "littlecloud");
	view->addPointCloud(bodycloud, "cloud");
	//view->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE/4, 1, "cloud");


	//blocks until the cloud is actually rendered

	while (!view->wasStopped()) {
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}// End Visualize */


} // End namespace body

#ifdef BODY_SEGMENTATION_STANDALONE
int main (int argc , char** argv)
{
	if(argc != 4)
	{
		std::cout<< "Usage: segmentation2 <cloud_file_name> <skeleton_file_name> <integer from 0 to 8>" << std::endl;
		exit(0);
	}
	int index = atoi(argv[3]);

	std::string skname = argv[2];
	std::string cloudFilename = argv[1];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> );

	if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (cloudFilename , *cloud) == -1)
	{
		std::stringstream x;
		x <<"Couldn't read file " << cloudFilename << ".pcd\n";
		PCL_ERROR (x.str().c_str());
		return (-1);
	}

	Body::Skin s;

	std::cout << "Size of cloud : " << cloud->points.size() << std::endl;

	std::clock_t start, finish;
	start = std::clock();

	Body::BodySegmentation bodyseg(skname,cloud,&s);

	bodyseg.run();

	finish = clock();

	std::cout << (finish - start)*1.0/CLOCKS_PER_SEC*1000.0 << " milliseconds ";

	bodyseg.visualize(index);

	return(0);
}
#endif
