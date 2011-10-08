#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <pcl_addons/io/openni_human_grabber.h>

#include <fstream>
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

     /*void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
     {
    	 //assert(0);
       if (!viewer.wasStopped())
         viewer.showCloud (cloud);
     }*/

     void body_cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    		              const boost::shared_ptr<BodyScanner::OpenNIHumanGrabber::BodyPose> & body_pose)
     {
    	 //assert(0);
       if (!viewer.wasStopped())
       {
         viewer.showCloud (cloud);

       }

       if(record && body_pose->skeleton_pose != NULL)
       {
		   static int frame = 0;

		   char pcd_filename[strlen(basename)+11];
		   sprintf(pcd_filename, "%s_%05d.pcd", basename, frame);
		   writer.writeBinary(string(pcd_filename), *cloud);

		   char skeleton_filename[strlen(basename)+21];
		   sprintf(skeleton_filename, "%s_skeleton_%05d.yaml", basename, frame);
		   skeleton_writer.open(skeleton_filename);
		   Body::Skeleton::Pose pose = * body_pose->skeleton_pose;
		   skeleton_writer << pose.toYaml() << endl;
		   skeleton_writer.close();

		   printf("saved %s and %s\n", pcd_filename, skeleton_filename);
		   frame++;

		   //assert(0);
       }
     }

     void run ()
     {
       BodyScanner::OpenNIHumanGrabber* interface = new BodyScanner::OpenNIHumanGrabber();

       //boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
       //  boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

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
     ofstream skeleton_writer;
     bool record;
     char* basename;
 };

 int main (int argc, char** argv)
 {
   SimpleOpenNIViewer v(argc, argv);
   v.run ();
   return 0;
 }
