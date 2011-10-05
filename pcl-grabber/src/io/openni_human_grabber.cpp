#include <pcl/console/print.h>
#include <pcl/io/openni_camera/openni_exception.h>

#include <BodyScanner/io/openni_human_grabber.h>
#include <BodyScanner/io/openni_camera/openni_driver_nite.h>
#include <XnCppWrapper.h>

using namespace BodyScanner;
using namespace boost;
using namespace openni_wrapper;

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		PCL_ERROR("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
	}



void __stdcall OpenNIHumanGrabber::NewUserDataAvailable (xn::ProductionNode& node, void* cookie) throw ()
{
	OpenNIHumanGrabber* grabber = reinterpret_cast<OpenNIHumanGrabber*>(cookie);
	grabber->user_condition_.notify_all ();
}

void __stdcall /*OpenNIHumanGrabber::*/NewUserCallback (xn::UserGenerator& generator, XnUserID user, void* cookie) throw ()
{
  OpenNIHumanGrabber* grabber = reinterpret_cast<OpenNIHumanGrabber*>(cookie);
  PCL_INFO("new user\n");
  //grabber->user_condition_.notify_all();
}

void __stdcall /*OpenNIHumanGrabber::*/LostUserCallback (xn::UserGenerator& generator, XnUserID user, void* cookie) throw ()
{
  OpenNIHumanGrabber* grabber = reinterpret_cast<OpenNIHumanGrabber*>(cookie);
  PCL_INFO("lost user\n");
  //grabber->user_condition_.notify_all();
}


OpenNIHumanGrabber::OpenNIHumanGrabber()
{
	rgb_depth_user_sync_.addCallback(boost::bind(&OpenNIHumanGrabber::imageDepthImageUserCallback, this, _1, _2, _3));


	BodyScanner::OpenNIDriverNITE& driver = (BodyScanner::OpenNIDriverNITE&) openni_wrapper::OpenNIDriver::getInstance();
	xn::Context* context = driver.getOpenNIContext();

	XnStatus nRetVal = context->FindExistingNode(XN_NODE_TYPE_USER, user_generator_);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = user_generator_.Create(*context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!user_generator_.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		PCL_INFO("Supplied user generator doesn't support skeleton\n");
		//return 1;
	}


	user_generator_.RegisterUserCallbacks((xn::UserGenerator::UserHandler)NewUserCallback, (xn::UserGenerator::UserHandler)LostUserCallback, this, user_callback_handle_);
	user_generator_.RegisterToNewDataAvailable ((xn::StateChangedHandler)NewUserDataAvailable, this, user_callback_handle_);

	quit_ = false;

	//user_callback_handle = registerUserCallback(&OpenNIHumanGrabber::userCallback, this);

	//XnCallbackHandle hUserCallbacks;
	//user_generator_.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);



	/*
	XnCallbackHandle hCalibrationCallbacks;
	user_generator_.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (user_generator_.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!user_generator_.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		user_generator_.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		user_generator_.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	user_generator_.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	*/


	image_for_user_callback_handle = getDevice()->registerImageCallback(&OpenNIHumanGrabber::imageForUserCallback, *this);
	depth_for_user_callback_handle = getDevice()->registerDepthCallback(&OpenNIHumanGrabber::depthForUserCallback, *this);

}

void OpenNIHumanGrabber::start () throw (pcl::PCLIOException)
{
	pcl::OpenNIGrabber::start();

	startUserStream();
}


void OpenNIHumanGrabber::startUserStream () throw (openni_wrapper::OpenNIException)
{
  if (hasUserStream ())
  {
    lock_guard<mutex> user_lock (user_mutex_);
    if (!user_generator_.IsGenerating ())
    {
      XnStatus status = user_generator_.StartGenerating ();

      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("starting user stream failed. Reason: %s\nDo you need to install NITE?\n", xnGetStatusString (status));
      else
      {
			user_thread_ = boost::thread (&OpenNIHumanGrabber::UserDataThreadFunction, this);
      }
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Current OpenNI installation does not provide a user stream. Do you need to install NITE?\n");
}

bool OpenNIHumanGrabber::hasUserStream () const throw ()
{
  lock_guard<mutex> lock (user_mutex_);
  return user_generator_.IsValid ();
}

void OpenNIHumanGrabber::UserDataThreadFunction () throw (openni_wrapper::OpenNIException)
{
  while (true)
  {

	// lock before checking running flag
	unique_lock<mutex> user_lock (user_mutex_);
	if (quit_)
	  return;
	user_condition_.wait (user_lock);
	if (quit_)
	  return;

	user_generator_.WaitAndUpdateData ();
	boost::shared_ptr<xn::SceneMetaData> user_pixels_scene_data (new xn::SceneMetaData);

	XnUserID users[15];
	XnUInt16 users_count = 15;
	user_generator_.GetUsers(users, users_count);

	if(users_count > 0) // TODO handle multiple users
	{
	  user_generator_.GetUserPixels (users[0], *user_pixels_scene_data);
	  printf("got user pixels and stuff\n");
	}
	user_lock.unlock ();

	//boost::shared_ptr<UserMask> user_mask ( new DepthImage (user_pixels_scene_data, baseline_, getDepthFocalLength (), shadow_value_, no_sample_value_) );

	//for (map< OpenNIDevice::CallbackHandle, ActualUserCallbackFunction >::iterator callbackIt = user_callback_.begin ();
	//     callbackIt != user_callback_.end (); ++callbackIt)
	//{
	//  callbackIt->second.operator()(user_mask);
	//}
  }
}

void OpenNIHumanGrabber::imageForUserCallback(boost::shared_ptr<openni_wrapper::Image> image, void* cookie)
{
  if (num_slots<sig_cb_openni_user_skeleton_and_point_cloud_rgb > () > 0 //||
      /*num_slots<sig_cb_openni_image_depth_image > () > 0*/)
    rgb_depth_user_sync_.add0(image, image->getTimeStamp());

//  if (image_signal_->num_slots() > 0)
//    image_signal_->operator()(image);

  return;
}

void OpenNIHumanGrabber::depthForUserCallback(boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie)
{
  if (num_slots<sig_cb_openni_user_skeleton_and_point_cloud_rgb > () > 0 //||
      /*num_slots<sig_cb_openni_image_depth_image > () > 0*/)
	  rgb_depth_user_sync_.add1(depth_image, depth_image->getTimeStamp());

  /*if (num_slots<sig_cb_openni_point_cloud_i > () > 0 ||
      num_slots<sig_cb_openni_ir_depth_image > () > 0)
    ir_sync_.add1(depth_image, depth_image->getTimeStamp());

  if (depth_image_signal_->num_slots() > 0)
    depth_image_signal_->operator()(depth_image);

  if (point_cloud_signal_->num_slots() > 0)
    point_cloud_signal_->operator()(convertToXYZPointCloud(depth_image));*/

  return;
}

void OpenNIHumanGrabber::imageDepthImageUserCallback(const boost::shared_ptr<openni_wrapper::Image> &image,
		                                             const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image,
		                                             float skeleton)
{
  // check if we have color point cloud slots
  if (user_skeleton_and_point_cloud_rgb_signal_->num_slots() > 0)
	  user_skeleton_and_point_cloud_rgb_signal_->operator()(convertToXYZRGBPointCloud(image, depth_image), 0.0f);

  printf("hello???????????");

  /*if (ir_depth_image_signal_->num_slots() > 0)
  {
    float constant = 1.0f / device_->getDepthFocalLength(depth_width_);
    ir_depth_image_signal_->operator()(ir_image, depth_image, constant);
  }*/
}

/*OpenNIDevice::CallbackHandle OpenNIHumanGrabber::registerUserCallback (const UserDataCallbackFunction& callback, void* custom_data) throw ()
{
  if (!hasUserStream ())
    THROW_OPENNI_EXCEPTION ("Current OpenNI installation does not provide user detection ... do you need to install NITE?");

  user_callback_[user_callback_handle_counter_] = boost::bind (callback, _1, custom_data);
  return user_callback_handle_counter_++;
}

void OpenNIHumanGrabber::userCallback(boost::shared_ptr<openni_wrapper::Image> image, void* cookie)
{
  if (num_slots<sig_cb_openni_point_cloud_rgb > () > 0 ||
      num_slots<sig_cb_openni_image_depth_image > () > 0)
    user_sync_.add0(image, image->getTimeStamp());

  if (user_signal_->num_slots() > 0)
    user_signal_->operator()(image);

  return;
}*/

OpenNIHumanGrabber::~OpenNIHumanGrabber() throw()
{
	  // stop streams
	  if (user_generator_.IsValid () && user_generator_.IsGenerating ())
	    user_generator_.StopGenerating ();

	  // lock before changing running flag
	  user_mutex_.lock ();

	  user_generator_.UnregisterFromNewDataAvailable (user_callback_handle_);
	  quit_ = true;
	  user_condition_.notify_all ();

	  user_mutex_.unlock ();

	  if (hasUserStream ())
	    user_thread_.join ();
}
