#pragma once

#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_device.h>
#include <pcl/point_cloud.h>

#include <pcl_addons/common/synchronizer3.h>

#include <boost/tuple/tuple.hpp>



namespace BodyScanner
{


	class OpenNIHumanGrabber : public pcl::OpenNIGrabber
	{
		public:
			//typedef void (sig_cb_openni_user) (const boost::shared_ptr<openni_wrapper::Image>&);
			typedef void (sig_cb_openni_user_skeleton_and_point_cloud_rgb)
				(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&, /* TODO put skeleton here! */ float skeleton) ;


		public:
			OpenNIHumanGrabber();
			~OpenNIHumanGrabber() throw();

			/** \brief ... */
			virtual void
			start () throw (pcl::PCLIOException);

			void startUserStream () throw (openni_wrapper::OpenNIException);

			bool hasUserStream () const throw ();

			static void __stdcall NewUserDataAvailable (xn::ProductionNode& node, void* cookie) throw ();
			//void __stdcall NewUserCallback(xn::UserGenerator& generator, XnUserID user, void* cookie) throw ();
			//void __stdcall LostUserCallback(xn::UserGenerator& generator, XnUserID user, void* cookie) throw ();

			void imageForUserCallback(boost::shared_ptr<openni_wrapper::Image> image, void* cookie);

			void depthForUserCallback(boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie);

		protected:
			//typedef boost::function<void(boost::shared_ptr<openni_wrapper::Image>, void* cookie) > UserDataCallbackFunction;

			void UserDataThreadFunction () throw (openni_wrapper::OpenNIException);
			//openni_wrapper::OpenNIDevice::CallbackHandle registerUserCallback (const UserDataCallbackFunction& callback, void* custom_data) throw ();

			//void userCallback(boost::shared_ptr<openni_wrapper::Image> image, void* cookie);

			void imageDepthImageUserCallback(const boost::shared_ptr<openni_wrapper::Image> &image,
				                             const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image,
				                             float skeleton);
		private:
			//typedef boost::function<void(boost::shared_ptr<openni_wrapper::Image>) > ActualUserCallbackFunction;
			//std::map< openni_wrapper::OpenNIDevice::CallbackHandle, ActualUserCallbackFunction > user_callback_;

			xn::UserGenerator user_generator_;
			XnCallbackHandle user_callback_handle_;
			//openni_wrapper::OpenNIDevice::CallbackHandle user_callback_handle_counter_;

			pcl_addons::Synchronizer3<boost::shared_ptr<openni_wrapper::Image>,
			                          boost::shared_ptr<openni_wrapper::DepthImage>,
			                          float /* skeleton goes here */ > rgb_depth_user_sync_;


			//boost::signals2::signal<sig_cb_openni_user >* user_signal_;

			mutable boost::mutex user_mutex_;
			boost::condition_variable user_condition_;
			boost::thread user_thread_;

			bool quit_;

			boost::signals2::signal<sig_cb_openni_user_skeleton_and_point_cloud_rgb>* user_skeleton_and_point_cloud_rgb_signal_;

			openni_wrapper::OpenNIDevice::CallbackHandle depth_for_user_callback_handle;
			openni_wrapper::OpenNIDevice::CallbackHandle image_for_user_callback_handle;
	};


};

