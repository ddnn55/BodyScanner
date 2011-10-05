#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <pcl_addons/io/openni_human_grabber.h>

#include <string>
using namespace std;

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer (int argc, char** argv) : viewer ("Body Scanner")
     {
    	 record = argc > 1;
    	 if(record)
    		 basename = argv[1];
     }

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
     {
    	 //assert(0);
       if (!viewer.wasStopped())
         viewer.showCloud (cloud);
     }

     void body_cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    		              const boost::shared_ptr<BodyScanner::OpenNIHumanGrabber::BodyPose> & body_pose)
     {
    	 //assert(0);
       if (!viewer.wasStopped())
         viewer.showCloud (cloud);

       if(record)
       {
		   static int frame = 0;
		   char filename[strlen(basename)+11];
		   sprintf(filename, "%s_%05d.pcd", basename, frame++);
		   writer.writeBinary(string(filename), *cloud);
		   printf("saved %s\n", filename);
       }
     }

     void run ()
     {
       BodyScanner::OpenNIHumanGrabber* interface = new BodyScanner::OpenNIHumanGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&, const boost::shared_ptr<BodyScanner::OpenNIHumanGrabber::BodyPose>&)> g =
         boost::bind (&SimpleOpenNIViewer::body_cloud_cb_, this, _1, _2);

       //interface->registerCallback (f);
       interface->registerCallback (g);

       interface->start ();



       while (!viewer.wasStopped())
       {
         sleep (1);
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
     pcl::PCDWriter writer;
     bool record;
     char* basename;
 };

 int main (int argc, char** argv)
 {
   SimpleOpenNIViewer v(argc, argv);
   v.run ();
   return 0;
 }
