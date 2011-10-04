#include <pcl/console/print.h>

#include <BodyScanner/io/openni_human_grabber.h>
#include <BodyScanner/io/openni_camera/openni_driver_nite.h>
#include <XnCppWrapper.h>

using namespace BodyScanner;

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		PCL_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
	}



void __stdcall /*OpenNIHumanGrabber::*/NewUserCallback (xn::UserGenerator& generator, XnUserID user, void* cookie) throw ()
{
  OpenNIHumanGrabber* grabber = reinterpret_cast<OpenNIHumanGrabber*>(cookie);
  PCL_INFO("new user");
  //grabber->user_condition_.notify_all();
}

void __stdcall /*OpenNIHumanGrabber::*/LostUserCallback (xn::UserGenerator& generator, XnUserID user, void* cookie) throw ()
{
  OpenNIHumanGrabber* grabber = reinterpret_cast<OpenNIHumanGrabber*>(cookie);
  PCL_INFO("lost user");
  //grabber->user_condition_.notify_all();
}


OpenNIHumanGrabber::OpenNIHumanGrabber()
{
	printf("hello from human grabber\n");
	BodyScanner::OpenNIDriverNITE& driver = (BodyScanner::OpenNIDriverNITE&) openni_wrapper::OpenNIDriver::getInstance();
	xn::Context* context = driver.getOpenNIContext();




	XnStatus nRetVal = context->FindExistingNode(XN_NODE_TYPE_USER, user_generator_);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = user_generator_.Create(*context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!user_generator_.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		PCL_INFO("Supplied user generator doesn't support skeleton");
		//return 1;
	}

	user_generator_.RegisterUserCallbacks((xn::UserGenerator::UserHandler)NewUserCallback, (xn::UserGenerator::UserHandler)LostUserCallback, this, user_callback_handle_);
	//user_generator_.RegisterToNewDataAvailable ((xn::StateChangedHandler)NewUserDataAvailable, this, user_callback_handle_);

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


}

OpenNIHumanGrabber::~OpenNIHumanGrabber() throw()
{
	user_mutex_.lock ();
	user_generator_.UnregisterFromNewDataAvailable (user_callback_handle_);
	user_mutex_.unlock ();
}
