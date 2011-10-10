/*
 * Builder.h
 *
 *  Created on: Oct 9, 2011
 *      Author: stolrsky
 */

#include <boost/thread.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <Body/BodyPointCloud.h>

#ifndef BUILDER_H_
#define BUILDER_H_


namespace Body {

class Builder {
public:
	/*class Sample {
	public:
		BodyPointCloud::ConstPtr cloud;
		Skeleton::Pose::Ptr skeleton_pose;
	};*/

	Builder(pcl::visualization::PCLVisualizer* viewer = NULL, boost::mutex *viewer_lock = NULL);
	virtual ~Builder();

	void pushSample(Body::BodyPointCloud::ConstPtr cloud, Body::Skeleton::Pose::Ptr /*FIXME use ConstPtr for more thread safety*/ skeleton_pose);

private:
	void run();
	void updateOutputVisualizer();

	pcl::visualization::PCLVisualizer* viewer_;
	boost::mutex* viewer_lock_;

	boost::thread builder_thread_;
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


	Skeleton::Pose::Ptr canonical_skeleton_pose_;


};

}

#endif /* BUILDER_H_ */
