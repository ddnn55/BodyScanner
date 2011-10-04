#pragma once

#include <pcl/io/openni_grabber.h>

namespace BodyScanner
{


	class OpenNIHumanGrabber : public pcl::OpenNIGrabber
	{
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

		protected:
			void UserDataThreadFunction () throw (openni_wrapper::OpenNIException);

		private:
			xn::UserGenerator user_generator_;
			XnCallbackHandle user_callback_handle_;
			mutable boost::mutex user_mutex_;
			boost::condition_variable user_condition_;
			boost::thread user_thread_;

			bool quit_;
	};


};

