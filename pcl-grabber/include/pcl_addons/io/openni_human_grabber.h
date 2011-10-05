#pragma once

#include <boost/tuple/tuple.hpp>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_device.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_addons/common/synchronizer3.h>

#include <Body/Skeleton/Pose.h>


namespace BodyScanner
{


	class OpenNIHumanGrabber : public pcl::OpenNIGrabber
	{
		public:
			class BodyPose // TODO move to Body namespace, rename ImagePresence? ImageRegion?
			{
			  public:
				BodyPose(XnLabel user_label, boost::shared_ptr<xn::SceneMetaData>& smd);
				bool bodyIsAtPixel(int p);
				Body::Skeleton::Pose::Ptr skeleton_pose;
			  private:
				boost::shared_ptr<xn::SceneMetaData> scene_meta_data_;
				XnLabel user_label_;
			};

		public:
			//typedef void (sig_cb_openni_user) (const boost::shared_ptr<openni_wrapper::Image>&);
			typedef void (sig_cb_openni_user_skeleton_and_point_cloud_rgb)
				(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&, const boost::shared_ptr<OpenNIHumanGrabber::BodyPose> &) ;


		public:
			OpenNIHumanGrabber();
			~OpenNIHumanGrabber() throw();

			/** \brief ... */
			virtual void
			start () throw (pcl::PCLIOException);

			void startUserStream () throw (openni_wrapper::OpenNIException);

			bool hasUserStream () const throw ();

			static void __stdcall UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie);
			static void __stdcall UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus bStatus, void* pCookie);
			static void __stdcall UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie);
			static void __stdcall NewUserDataAvailable (xn::ProductionNode& node, void* cookie) throw ();
			static void __stdcall NewUserCallback(xn::UserGenerator& generator, XnUserID user, void* cookie) throw ();
			static void __stdcall LostUserCallback(xn::UserGenerator& generator, XnUserID user, void* cookie) throw ();

			void imageForUserCallback(boost::shared_ptr<openni_wrapper::Image> image, void* cookie);

			void depthForUserCallback(boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie);

		      /** \brief ... */
		      virtual inline void
		      checkImageStreamRequired ();

		      /** \brief ... */
		      virtual inline void
		      checkDepthStreamRequired ();

		protected:
			//typedef boost::function<void(boost::shared_ptr<openni_wrapper::Image>, void* cookie) > UserDataCallbackFunction;

			void UserDataThreadFunction () throw (openni_wrapper::OpenNIException);
			//openni_wrapper::OpenNIDevice::CallbackHandle registerUserCallback (const UserDataCallbackFunction& callback, void* custom_data) throw ();

			//void userCallback(boost::shared_ptr<openni_wrapper::Image> image, void* cookie);

			void imageDepthImageUserCallback(const boost::shared_ptr<openni_wrapper::Image> &image,
				                             const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image,
				                             const boost::shared_ptr<OpenNIHumanGrabber::BodyPose> & body_pose);

			// TODO: make this convert function in order to throw away background earlier
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToXYZRGBPointCloud(const boost::shared_ptr<openni_wrapper::Image> &image,
			//  const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const;
		private:
			//typedef boost::function<void(boost::shared_ptr<openni_wrapper::Image>) > ActualUserCallbackFunction;
			//std::map< openni_wrapper::OpenNIDevice::CallbackHandle, ActualUserCallbackFunction > user_callback_;

			xn::UserGenerator user_generator_;
			XnCallbackHandle user_callback_handle_;
			XnCallbackHandle calibration_start_callback_handle_;
			XnCallbackHandle calibration_complete_callback_handle_;
			XnCallbackHandle user_pose_detected_callback_handle_;
			bool need_pose_;
			XnChar pose_[20];
			//openni_wrapper::OpenNIDevice::CallbackHandle user_callback_handle_counter_;

			pcl_addons::Synchronizer3<boost::shared_ptr<openni_wrapper::Image>,
			                          boost::shared_ptr<openni_wrapper::DepthImage>,
			                          const boost::shared_ptr<OpenNIHumanGrabber::BodyPose> /* skeleton goes here */ > rgb_depth_user_sync_;


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

