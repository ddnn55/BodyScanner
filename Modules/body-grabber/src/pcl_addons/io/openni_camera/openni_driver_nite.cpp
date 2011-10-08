#include <pcl_addons/io/openni_camera/openni_driver_nite.h>




xn::Context* BodyScanner::OpenNIDriverNITE::getOpenNIContext()
{
	return & this->context_;
}







#if 0







diff ~/Downloads/pcl-1.2/io/src/openni_camera/openni_device.cpp ~/Desktop/BodyScanner/ros_modules/body_camera/src/openni_device.cpp 
37,43c37,40
< #include <pcl/pcl_config.h>
< #ifdef HAVE_OPENNI
< 
< #include <pcl/io/openni_camera/openni_driver.h>
< #include <pcl/io/openni_camera/openni_device.h>
< #include <pcl/io/openni_camera/openni_depth_image.h>
< #include <pcl/io/openni_camera/openni_ir_image.h>
---
> #include <body_camera/openni_driver.h>
> #include <body_camera/openni_device.h>
> #include <body_camera/openni_depth_image.h>
> #include <body_camera/openni_ir_image.h>
61,91c58
< // workaround for MAC from Alex Ichim
< #ifdef __APPLE__
<   cerr << "Creating OpenNIDevice" << endl;
<   XnStatus rc;
< 
<     xn::EnumerationErrors errors;
<     rc = context_.InitFromXmlFile("/etc/primesense/SamplesConfig.xml", &errors);
<     if (rc == XN_STATUS_NO_NODE_PRESENT)
<     {
<             XnChar strError[1024];
<             errors.ToString(strError, 1024);
<             printf("%s\n", strError);
<     }
<     else if (rc != XN_STATUS_OK)
<     {
<             printf("Open failed: %s\n", xnGetStatusString(rc));
<     }
< 
<   XnStatus status = context_.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator_);
<   if (status != XN_STATUS_OK)
<     cerr << "node depth problems" << endl;
<   status = context_.FindExistingNode(XN_NODE_TYPE_IMAGE, image_generator_);
<   if (status != XN_STATUS_OK)
<     cerr << "node image problems" << endl;
<   status = context_.FindExistingNode(XN_NODE_TYPE_IR, ir_generator_);
<     if (status != XN_STATUS_OK)
<       cerr << "node ir problems" << endl;
< 
< 
< #else
< // create the production nodes
---
>   // create the production nodes
95a63,66
>   //status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(user_node));
>   //if (status != XN_STATUS_OK)
>   //  THROW_OPENNI_EXCEPTION ("creating user generator failed. Reason: %s", xnGetStatusString (status));
> 
108a80,83
>   //status = user_node.GetInstance (user_generator_);
>   //if (status != XN_STATUS_OK)
>   //  THROW_OPENNI_EXCEPTION ("creating user generator instance failed. Reason: %s", xnGetStatusString (status));
> 
118d92
< #endif
130,159c104,105
< // workaround for MAC from Alex Ichim
< #ifdef __APPLE__
<   cerr << "Creating OpenNIDevice" << endl;
<   XnStatus rc;
< 
<     xn::EnumerationErrors errors;
<     rc = context_.InitFromXmlFile("/etc/primesense/SamplesConfig.xml", &errors);
<     if (rc == XN_STATUS_NO_NODE_PRESENT)
<     {
<             XnChar strError[1024];
<             errors.ToString(strError, 1024);
<             printf("%s\n", strError);
<     }
<     else if (rc != XN_STATUS_OK)
<     {
<             printf("Open failed: %s\n", xnGetStatusString(rc));
<     }
< 
<   XnStatus status = context_.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator_);
<   if (status != XN_STATUS_OK)
<     cerr << "node depth problems" << endl;
<   status = context_.FindExistingNode(XN_NODE_TYPE_IR, ir_generator_);
<     if (status != XN_STATUS_OK)
<       cerr << "node ir problems" << endl;
< 
< #else
<   XnStatus status;
< 
<   // create the production nodes
<   status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(depth_node));
---
>     // create the production nodes
>   XnStatus status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(depth_node));
162a109,112
>   //status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(user_node));
>   //if (status != XN_STATUS_OK)
>   //  THROW_OPENNI_EXCEPTION ("creating user generator failed. Reason: %s", xnGetStatusString (status));
> 
171a122,125
>   //status = user_node.GetInstance (user_generator_);
>   //if (status != XN_STATUS_OK)
>   //  THROW_OPENNI_EXCEPTION ("creating user generator instance failed. Reason: %s", xnGetStatusString (status));
> 
175c129
< 
---
>   
177d130
<   #endif
181c134
<   Init ();
---
>   Init ();  
191c144
< OpenNIDevice::~OpenNIDevice () throw ()
---
> OpenNIDevice::~OpenNIDevice () throw () // FIXME add user stuff here
199a153,155
>   if (user_generator_.IsValid () && user_generator_.IsGenerating ())
>     user_generator_.StopGenerating ();
> 
205a162
>   user_mutex_.lock ();
209a167
>   user_condition_.notify_all ();
211a170
> 
212a172
>   user_mutex_.unlock ();
221a182,184
>   if (hasUserStream ())
>     user_thread_.join ();
> 
228a192,224
>   XnStatus status;
> 
> 
>   status = context_.FindExistingNode(XN_NODE_TYPE_USER, user_generator_);
>   if (status != XN_STATUS_OK)
>   {
>     status = user_generator_.Create(context_);
>     if (status != XN_STATUS_OK)
>       THROW_OPENNI_EXCEPTION ("Couldn't create user generator. Reason: %s", xnGetStatusString (status));
>   }
> 
>   if(hasUserStream ())
>   {
> 
> 	user_generator_.RegisterToNewDataAvailable ((xn::StateChangedHandler)NewUserDataAvailable, this, user_callback_handle_);
> 
> 	//if (!user_generator_.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
> 	//	ROS_INFO("Supplied user generator doesn't support skeleton");
> 	//
> 	//}
> 
> 	//XnCallbackHandle hUserCallbacks;
> 	//user_generator_.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, user_callback_handle_);
> 
> 	status = user_generator_.StartGenerating();
> 	if (status != XN_STATUS_OK)
> 	        THROW_OPENNI_EXCEPTION ("UserGenerator StartGenerating() failed. Reason: %s", xnGetStatusString (status));
> 
>     lock_guard<mutex> user_lock (user_mutex_);
> 	user_thread_ = boost::thread (&OpenNIDevice::UserDataThreadFunction, this);
> 
>   }
> 
234c230
<     XnStatus status = depth_generator_.GetRealProperty ("ZPPS", pixel_size);
---
>     status = depth_generator_.GetRealProperty ("ZPPS", pixel_size);
276c272
< 
---
>   
359d354
< #ifndef __APPLE__
362d356
< #endif
413a408,413
> bool OpenNIDevice::hasUserStream () const throw ()
> {
>   lock_guard<mutex> lock (user_mutex_);
>   return user_generator_.IsValid ();
> }
> 
577c577
< 
---
>     
602c602
< 
---
>     
612a613,649
> void OpenNIDevice::UserDataThreadFunction () throw (OpenNIException)
> {
>   while (true)
>   {
>     // lock before checking running flag
>     unique_lock<mutex> user_lock (user_mutex_);
>     if (quit_)
>       return;
>     user_condition_.wait (user_lock);
>     if (quit_)
>       return;
> 
>     user_generator_.WaitAndUpdateData ();
>     boost::shared_ptr<xn::SceneMetaData> user_pixels_scene_data (new xn::SceneMetaData);
> 
>     XnUserID users[15];
>     XnUInt16 users_count = 15;
>     user_generator_.GetUsers(users, users_count);
> 
>     if(users_count > 0) // TODO handle multiple users
>     {
>       user_generator_.GetUserPixels (users[0], *user_pixels_scene_data);
>       printf("got user pixels and stuff\n");
>     }
>     user_lock.unlock ();
> 
>     printf("user data thread\n");
>     //boost::shared_ptr<UserMask> user_mask ( new DepthImage (user_pixels_scene_data, baseline_, getDepthFocalLength (), shadow_value_, no_sample_value_) );
> 
>     //for (map< OpenNIDevice::CallbackHandle, ActualUserCallbackFunction >::iterator callbackIt = user_callback_.begin ();
>     //     callbackIt != user_callback_.end (); ++callbackIt)
>     //{
>     //  callbackIt->second.operator()(user_mask);
>     //}
>   }
> }
> 
645a683,688
> void __stdcall OpenNIDevice::NewUserDataAvailable (xn::ProductionNode& node, void* cookie) throw ()
> {
>   OpenNIDevice* device = reinterpret_cast<OpenNIDevice*>(cookie);
>   device->user_condition_.notify_all ();
> }
> 
724c767
< 
---
>   
728c771
< 
---
>   
957d999
< #ifndef __APPLE__
960d1001
< #endif
1005d1045
< #endif //OPENNI


#endif
