#pragma once

#include <pcl/io/openni_grabber.h>

namespace BodyScanner
{


	class OpenNIHumanGrabber : public pcl::OpenNIGrabber
	{
		public:
			OpenNIHumanGrabber();
			~OpenNIHumanGrabber() throw();

			//void __stdcall NewUserCallback(xn::UserGenerator& generator, XnUserID user, void* cookie) throw ();
			//void __stdcall LostUserCallback(xn::UserGenerator& generator, XnUserID user, void* cookie) throw ();

		protected:
			xn::UserGenerator user_generator_;
			XnCallbackHandle user_callback_handle_;
			mutable boost::mutex user_mutex_;
	};


};

