/*
 * Builder.cpp
 *
 *  Created on: Oct 9, 2011
 *      Author: stolrsky
 */

#include <boost/thread/locks.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <Body/Builder.h>

namespace Body {

Builder::Builder(pcl::visualization::PCLVisualizer* viewer, boost::mutex *viewer_lock)
	: builder_thread_(&Builder::run, this)
	, end_(false)
	, have_new_sample_(false)
	//, new_sample_flag_(false)
	//, ready_for_new_sample_(true)
{

	viewer_ = viewer;
	viewer_lock_ = viewer_lock;
	//viewer_->addPolygonMesh(pcl::PolygonMesh(), "body"); // add empty mesh s update...() can simply remove and add

}

Builder::~Builder() {
	// TODO Auto-generated destructor stub
}

void Builder::pushSample(Body::BodyPointCloud::ConstPtr cloud, Body::Skeleton::Pose::Ptr skeleton_pose)
{


	if(!pending_sample_access_.try_lock())
		return;

	//if(ready_for_new_sample_)
	//{
		pending_sample_cloud_ = cloud;
		pending_sample_skeleton_pose_ = skeleton_pose;
		have_new_sample_ = true;
		//ready_for_new_sample_ = false;

	/*if(!ready_for_new_sample_)
		return;
	else
	{
		pending_sample_cloud_ = cloud;
		pending_sample_skeleton_pose_ = skeleton_pose;
		ready_for_new_sample_ = false;
	}*/

	//Sample body_sample;
	//body_sample.cloud = cloud;
	//body_sample.skeleton_pose = skeleton_pose;

	//sample_queue_.try_send(body_sample)


		//new_sample_flag_ = true;
		//new_sample_.notify_all();
	//}
	pending_sample_access_.unlock();

}

void Builder::run()
{
	boost::posix_time::milliseconds sleep_time(30);
	boost::posix_time::seconds fake_work_time(1);


	while(!end_)
	{
		boost::this_thread::sleep(sleep_time);

		pending_sample_access_.lock();
		if(have_new_sample_)
		{
			BodyPointCloud::ConstPtr cloud = pending_sample_cloud_;
			Skeleton::Pose::Ptr skeleton_pose = pending_sample_skeleton_pose_;
			have_new_sample_ = false;
			pending_sample_access_.unlock();


			static int cloud_index = 0;
			std::stringstream cloud_id;
			cloud_id << "body_cloud_" << cloud_index++;


			boost::this_thread::sleep(fake_work_time); // pretend to work for a while

			viewer_lock_->lock();
				viewer_->addPointCloud(cloud, cloud_id.str());
			viewer_lock_->unlock();
			std::cout << "added cloud to viewer" << std::endl;
		}
		else // no new sample
		{
			pending_sample_access_.unlock();
		}


	}
}

void Builder::updateOutputVisualizer()
{
	// output model updated
	if (viewer_ != NULL && !viewer_->wasStopped()) {
		viewer_->removeShape("body");
		viewer_->addPolygonMesh (pcl::PolygonMesh(), "body");
	}
}

}
