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
	PCL_INFO("hello from human grabber through pcl_info\n");
	BodyScanner::OpenNIDriverNITE& driver = (BodyScanner::OpenNIDriverNITE&) openni_wrapper::OpenNIDriver::getInstance();
	PCL_INFO("after driver getInstance()\n");
	xn::Context* context = driver.getOpenNIContext();

	PCL_INFO("before find or create user node\n");


	XnStatus nRetVal = context->FindExistingNode(XN_NODE_TYPE_USER, user_generator_);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = user_generator_.Create(*context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!user_generator_.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		PCL_INFO("Supplied user generator doesn't support skeleton\n");
		//return 1;
	}

	PCL_INFO("before register callbacks\n");

	user_generator_.RegisterUserCallbacks((xn::UserGenerator::UserHandler)NewUserCallback, (xn::UserGenerator::UserHandler)LostUserCallback, this, user_callback_handle_);
	user_generator_.RegisterToNewDataAvailable ((xn::StateChangedHandler)NewUserDataAvailable, this, user_callback_handle_);

	quit_ = false;

	//XnCallbackHandle hUserCallbacks;
	//user_generator_.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);


	//nRetVal = user_generator_.StartGenerating();
	//CHECK_RC(nRetVal, "user_generator_.StartGenerating()");

	PCL_INFO("finished OpenNIHumanGrabber()\n");

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
        THROW_OPENNI_EXCEPTION ("starting user stream failed. Reason: %s\n", xnGetStatusString (status));
      else
      {
    	  PCL_INFO("starting user stream success?\n");
			user_thread_ = boost::thread (&OpenNIHumanGrabber::UserDataThreadFunction, this);
			PCL_INFO("after boost::thread(userdatathreadfunction)");
      }
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide a user stream\n");
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
	  printf("start user data thread loop\n");

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

	printf("user data thread\n");
	//boost::shared_ptr<UserMask> user_mask ( new DepthImage (user_pixels_scene_data, baseline_, getDepthFocalLength (), shadow_value_, no_sample_value_) );

	//for (map< OpenNIDevice::CallbackHandle, ActualUserCallbackFunction >::iterator callbackIt = user_callback_.begin ();
	//     callbackIt != user_callback_.end (); ++callbackIt)
	//{
	//  callbackIt->second.operator()(user_mask);
	//}
  }
}

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
