#include <pcl/console/print.h>
#include <pcl/io/openni_camera/openni_exception.h>

#include <pcl_addons/io/openni_human_grabber.h>
#include <pcl_addons/io/openni_camera/openni_driver_nite.h>

#include <XnCppWrapper.h>

#include <boost/date_time/posix_time/posix_time.hpp>

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

void __stdcall OpenNIHumanGrabber::NewUserCallback (xn::UserGenerator& generator, XnUserID user, void* cookie) throw ()
{
  OpenNIHumanGrabber* grabber = reinterpret_cast<OpenNIHumanGrabber*>(cookie);
  PCL_INFO("new user\n");
  //grabber->user_condition_.notify_all();
	if (grabber->need_pose_)
		grabber->user_generator_.GetPoseDetectionCap().StartPoseDetection(grabber->pose_, user);
	else
		grabber->user_generator_.GetSkeletonCap().RequestCalibration(user, TRUE);
}

void __stdcall OpenNIHumanGrabber::LostUserCallback (xn::UserGenerator& generator, XnUserID user, void* cookie) throw ()
{
  OpenNIHumanGrabber* grabber = reinterpret_cast<OpenNIHumanGrabber*>(cookie);
  PCL_INFO("lost user\n");
  //grabber->user_condition_.notify_all();
}

void __stdcall OpenNIHumanGrabber::UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	PCL_INFO("Calibration started for user %d\n", nId);
}

void __stdcall OpenNIHumanGrabber::UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus bStatus, void* pCookie) {
	OpenNIHumanGrabber* grabber = (OpenNIHumanGrabber*) pCookie;

	if (bStatus == XN_CALIBRATION_STATUS_OK) {
		PCL_INFO("Calibration complete, start tracking user %d\n", nId);
		grabber->user_generator_.GetSkeletonCap().StartTracking(nId);
	}
	else {
		PCL_INFO("Calibration failed for user %d\n", nId);
		if (grabber->need_pose_)
			grabber->user_generator_.GetPoseDetectionCap().StartPoseDetection(grabber->pose_, nId);
		else
			grabber->user_generator_.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void __stdcall OpenNIHumanGrabber::UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
	OpenNIHumanGrabber* grabber = (OpenNIHumanGrabber*) pCookie;

	PCL_INFO("Pose %s detected for user %d", strPose, nId);
	grabber->user_generator_.GetPoseDetectionCap().StopPoseDetection(nId);
	grabber->user_generator_.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

OpenNIHumanGrabber::OpenNIHumanGrabber()
{

	user_skeleton_and_point_cloud_rgb_signal_ = createSignal <sig_cb_openni_user_skeleton_and_point_cloud_rgb > ();

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



	//XnCallbackHandle hCalibrationCallbacks;
	//user_generator_.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, /*TODO calibration_callback_handle_, shutdown, etc*/ hCalibrationCallbacks);
	user_generator_.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, this, calibration_start_callback_handle_);
	user_generator_.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, this, calibration_complete_callback_handle_);

	if (user_generator_.GetSkeletonCap().NeedPoseForCalibration()) {
		need_pose_ = TRUE;
		if (!user_generator_.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			PCL_INFO("Pose required, but not supported");
			//return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		//user_generator_.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
		user_generator_.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, this, user_pose_detected_callback_handle_);

		user_generator_.GetSkeletonCap().GetCalibrationPose(pose_);
		printf("Getting calibration pose %s\n", pose_);
	}

	user_generator_.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);



	image_for_user_callback_handle = getDevice()->registerImageCallback(&OpenNIHumanGrabber::imageForUserCallback, *this);
	depth_for_user_callback_handle = getDevice()->registerDepthCallback(&OpenNIHumanGrabber::depthForUserCallback, *this);



}

void OpenNIHumanGrabber::checkImageStreamRequired()
{
  // do we have anyone listening to images or color point clouds?
  if (num_slots<sig_cb_openni_image > () > 0 ||
      num_slots<sig_cb_openni_image_depth_image > () > 0 ||
      num_slots<sig_cb_openni_point_cloud_rgb > () > 0 ||
      num_slots<sig_cb_openni_user_skeleton_and_point_cloud_rgb > () > 0)
    image_required_ = true;
  else
    image_required_ = false;
}

void OpenNIHumanGrabber::checkDepthStreamRequired()
{
  // do we have anyone listening to depth images or (color) point clouds?
  if (num_slots<sig_cb_openni_depth_image > () > 0 ||
      num_slots<sig_cb_openni_image_depth_image > () > 0 ||
      num_slots<sig_cb_openni_ir_depth_image > () > 0 ||
      num_slots<sig_cb_openni_point_cloud_rgb > () > 0 ||
      num_slots<sig_cb_openni_point_cloud > () > 0 ||
      num_slots<sig_cb_openni_point_cloud_i > () > 0 ||
      num_slots<sig_cb_openni_user_skeleton_and_point_cloud_rgb > () > 0 )
    depth_required_ = true;
  else
    depth_required_ = false;
}

void OpenNIHumanGrabber::start () throw (pcl::PCLIOException)
{
	pcl::OpenNIGrabber::start();

	startUserStream();

	// debug FIXME remove
	//BodyScanner::OpenNIDriverNITE& driver = (BodyScanner::OpenNIDriverNITE&) openni_wrapper::OpenNIDriver::getInstance();
	//xn::Context* context = driver.getOpenNIContext();
	//context->StartGeneratingAll();
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
      {
        THROW_OPENNI_EXCEPTION ("starting user stream failed. Reason: %s\n", xnGetStatusString (status));
      }
      else
      {
    	  printf("started user stream\n");
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
	boost::shared_ptr<xn::SceneMetaData> user_pixels_scene_meta_data (new xn::SceneMetaData);

	XnUserID users[15];
	XnUInt16 users_count = 15;
	user_generator_.GetUsers(users, users_count);

	if(users_count > 0) // TODO handle multiple users
	{
		XnUserID user = users[0];


	  user_generator_.GetUserPixels (user, *user_pixels_scene_meta_data);
	  boost::shared_ptr<OpenNIHumanGrabber::BodyPose> body_pose(new BodyPose(user, user_pixels_scene_meta_data));
	  unsigned long time = (unsigned long) user_generator_.GetTimestamp();



      if (user_generator_.GetSkeletonCap().IsTracking(user))
      {
    	  Body::Skeleton::Pose::Ptr skeleton_pose(new Body::Skeleton::Pose());

    	  for(int j = 0; j < Body::Skeleton::GetJointList().size(); j++)
    	  {
    		  XnSkeletonJoint xn_joint_key = Body::Skeleton::GetJointList()[j].second;
    		  std::string joint_key = Body::Skeleton::GetJointList()[j].first;

        	  XnSkeletonJointTransformation joint_transformation;
    		  user_generator_.GetSkeletonCap().GetSkeletonJoint(user, xn_joint_key, joint_transformation);


        	  //(*skeleton_pose)[joint_key] = joint_transformation; // FIXME doesn't seem to be working?
        	  skeleton_pose->setTransformationForJointKey(joint_key, joint_transformation);


        	  //assert(0);
    	  }



    	  /*user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_HEAD, 		  skeleton_pose->head);
    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_NECK, 		  skeleton_pose->neck);
    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, 		  skeleton_pose->torso);

    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_SHOULDER,  skeleton_pose->left_shoulder);
    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_ELBOW, 	  skeleton_pose->left_elbow);
    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HAND, 	  skeleton_pose->left_hand);

    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_SHOULDER, skeleton_pose->right_shoulder);
    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_ELBOW,    skeleton_pose->right_elbow);
    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HAND,     skeleton_pose->right_hand);

    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HIP,       skeleton_pose->left_hip);
    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_KNEE,      skeleton_pose->left_knee);
    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_FOOT,      skeleton_pose->left_foot);

    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HIP,      skeleton_pose->right_hip);
    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_KNEE,     skeleton_pose->right_knee);
    	  user_generator_.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_FOOT,     skeleton_pose->right_foot);*/



    	  body_pose->skeleton_pose = skeleton_pose;

    	  //assert(0);
      }

      rgb_depth_user_sync_.add2(body_pose, time);
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
	//printf("imageforusercallback\n");
  if (num_slots<sig_cb_openni_user_skeleton_and_point_cloud_rgb > () > 0 //||
      /*num_slots<sig_cb_openni_image_depth_image > () > 0*/)
    rgb_depth_user_sync_.add0(image, image->getTimeStamp());

//  if (image_signal_->num_slots() > 0)
//    image_signal_->operator()(image);

  return;
}

void OpenNIHumanGrabber::depthForUserCallback(boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie)
{
	//printf("depth for user callback\n");
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
		                                             const boost::shared_ptr<OpenNIHumanGrabber::BodyPose> &body_pose)
{
	//printf("-------------------------------------------------hello???????????");


	// mask depth image with user presence!
	boost::shared_ptr<openni_wrapper::DepthImage> masked_depth_image(new openni_wrapper::DepthImage(*depth_image));
	XnDepthPixel* masked_depth_map = masked_depth_image->getDepthMetaData().WritableData();


	//for(int x = 0; x < depth_image->getWidth(); x++)
	//{
	//  for(int y = 0; y < depth_image->getHeight(); y++)
	//  {
	for(int p = 0; p < depth_image->getWidth() * depth_image->getHeight(); p++)
		 if(!body_pose->bodyIsAtPixel(p))
			 masked_depth_map[p] = masked_depth_image->getNoSampleValue();
	//  }
	//}


  // check if we have color point cloud slots
  if (user_skeleton_and_point_cloud_rgb_signal_->num_slots() > 0)
  {
	  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud = convertToXYZRGBPointCloud(image, depth_image);
	  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr body_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(scene_cloud));

	  //for(int p = 0; p < scene_cloud->size(); p++)
	  //{
	//	  if(body_pose->bodyIsAtPixel(p))
	//		  body_cloud->push_back((*scene_cloud)[p]);
	 // }

	  user_skeleton_and_point_cloud_rgb_signal_->operator()(
			  (Body::BodyPointCloud::ConstPtr) convertToXYZRGBPointCloud(image, masked_depth_image),
			  body_pose);
  }



  /*if (ir_depth_image_signal_->num_slots() > 0)
  {
    float constant = 1.0f / device_->getDepthFocalLength(depth_width_);
    ir_depth_image_signal_->operator()(ir_image, depth_image, constant);
  }*/
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

OpenNIHumanGrabber::BodyPose::BodyPose(XnLabel user_label, boost::shared_ptr<xn::SceneMetaData>& smd)
{
	scene_meta_data_ = smd;
	user_label_ = user_label;
}

bool OpenNIHumanGrabber::BodyPose::bodyIsAtPixel(int p)
{
	return scene_meta_data_->Data()[p] == user_label_;
}
