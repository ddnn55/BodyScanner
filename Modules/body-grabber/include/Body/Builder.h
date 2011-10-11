/*
 * Builder.h
 *
 *  Created on: Oct 9, 2011
 *      Author: stolrsky
 */



#ifndef BUILDER_H_
#define BUILDER_H_

#include <boost/thread.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <Body/BodyPointCloud.h>


namespace Body {

class Builder {
public:
	/*class Sample {
	public:
		BodyPointCloud::ConstPtr cloud;
		Skeleton::Pose::Ptr skeleton_pose;
	};*/

	Builder(pcl::visualization::PCLVisualizer* viewer = NULL, boost::mutex *viewer_lock = NULL, std::string build_id = "build");
	virtual ~Builder();

	void pushSample(Body::BodyPointCloud::ConstPtr cloud, Body::Skeleton::Pose::Ptr /*FIXME use ConstPtr for more thread safety*/ skeleton_pose);

	void saveObj(std::string outfile);
	void saveSkeleton(std::string outfile);

private:
	void run();
	//void updateOutputVisualizer();

	pcl::visualization::PCLVisualizer* viewer_;
	boost::mutex* viewer_lock_;

	boost::thread* builder_thread_;
	bool end_;

	//bool ready_for_new_sample_;

	//bool new_sample_flag_;
	//boost::condition_variable new_sample_;
	//boost::mutex new_sample_mutex_;
	boost::mutex pending_sample_access_;
	bool have_new_sample_;

	//boost::interprocess::message_queue sample_queue_;

	BodyPointCloud::ConstPtr pending_sample_cloud_;
	Skeleton::Pose::Ptr pending_sample_skeleton_pose_;

	boost::mutex canonical_skeleton_pose_mutex_;
	Skeleton::Pose::Ptr canonical_skeleton_pose_;


	boost::mutex body_point_cloud_accumulation_mutex_;
	BodyPointCloud::Ptr body_point_cloud_accumulation;

	boost::mutex mesh_mutex_;
	pcl::PolygonMesh::Ptr mesh_;

	std::string build_id_;


};

}

#endif /* BUILDER_H_ */
