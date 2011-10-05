#include <pcl/visualization/cloud_viewer.h>

#include <pcl_addons/io/openni_human_grabber.h>

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("Body Scanner") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
     {
    	 //assert(0);
       if (!viewer.wasStopped())
         viewer.showCloud (cloud);
     }

     void body_cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, float skeleton)
     {
    	 //assert(0);
       if (!viewer.wasStopped())
         viewer.showCloud (cloud);
     }

     void run ()
     {
       BodyScanner::OpenNIHumanGrabber* interface = new BodyScanner::OpenNIHumanGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&, float)> g =
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
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
