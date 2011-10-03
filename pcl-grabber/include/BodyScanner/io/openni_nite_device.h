diff /usr/local/include/pcl-1.2/pcl/io/openni_camera/openni_device.h ~/Desktop/BodyScanner/ros_modules/body_camera/include/body_camera/openni_device.h
38,40d37
< #include <pcl/pcl_config.h>
< #ifdef HAVE_OPENNI
<
53,54d49
< #include <pcl/pcl_macros.h>
<
70d64
<  * @ingroup io
72c66
< class PCL_EXPORTS OpenNIDevice : public boost::noncopyable
---
> class OpenNIDevice : public boost::noncopyable
135a130
>   bool hasUserStream () const throw ();
175a171
>   static void __stdcall NewUserDataAvailable (xn::ProductionNode& node, void* cookie) throw ();
182a179
>   void UserDataThreadFunction () throw (OpenNIException);
208a206,207
>   /** \brief User generator object. **/
>   xn::UserGenerator user_generator_;
214a214
>   XnCallbackHandle user_callback_handle_;
236a237
>   mutable boost::mutex user_mutex_;
239a241
>   boost::condition_variable user_condition_;
242a245
>   boost::thread user_thread_;
292d294
< #endif // HAVE_OPENNI
