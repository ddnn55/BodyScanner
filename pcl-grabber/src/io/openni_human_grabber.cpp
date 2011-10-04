#include <BodyScanner/io/openni_human_grabber.h>
//#include <pcl/io/openni_camera/openni_driver.h>
#include <BodyScanner/io/openni_camera/openni_driver_nite.h>
#include <XnCppWrapper.h>

using namespace BodyScanner;

OpenNIHumanGrabber::OpenNIHumanGrabber()
{
	printf("hello from human grabber\n");
	BodyScanner::OpenNIDriverNITE& driver = (BodyScanner::OpenNIDriverNITE&) openni_wrapper::OpenNIDriver::getInstance();
	xn::Context *context = driver.getOpenNIContext();
}
