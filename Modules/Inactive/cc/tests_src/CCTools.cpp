#include "CCTools.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

// Forward declarations
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud);

void handleKeyEvent(const pcl::visualization::KeyboardEvent& ev, void *data);
float mag(pcl::PointNormal& p);

// Methods declarations

void CCTools::VisualizeNormalCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = normalsVis(cloud);
  viewer->registerKeyboardCallback(&handleKeyEvent, NULL);

  pcl::PointXYZ o;
  o.x = 1.0;
  o.y = 0;
  o.z = 0;
  viewer->addSphere (o, 0.25, "sphere", 0);
  
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}

pcl::PointCloud<pcl::PointNormal>::Ptr CCTools::IsolateBody(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{

pcl::PointCloud<pcl::PointNormal>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointNormal>());

  int l = 0;
  for(int j = 0; j < 480; j += 1) {
    for(int i = j%2; i < 640; i += 2) {
      int k = i + j*640;
      pcl::PointNormal& p = (*cloud)[k];
      if((i&1 && j&1) // keep 1/4 of the points
         && mag(p) < 3.5) { // keep only points within a close distance to the origin
        filtered_cloud->push_back(p);
      }
    }
  }

return filtered_cloud;

}

pcl::PointCloud<pcl::PointNormal>::Ptr CCTools::getNormalsFromXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud)
{
 cout << "test 1";

  // drop the colors
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::copyPointCloud(*color_cloud, *cloud);

  // Create a KD-Tree
  pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  // Output has the same type as the input one, it will be only smoothed
  pcl::PointCloud<pcl::PointXYZ> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;

  // Optionally, a pointer to a cloud can be provided, to be set by MLS
  pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
  mls.setOutputNormals (mls_normals);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.reconstruct (mls_points);

  // Concatenate fields for saving
  pcl::PointCloud<pcl::PointNormal> mls_cloud;
  pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr result (new pcl::PointCloud<pcl::PointNormal>());
  pcl::copyPointCloud(mls_cloud,*result);



 return  result;

}



pcl::PolygonMesh* CCTools::getMeshFromXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

   
   return CCTools::getMeshFromNormalCloud(CCTools::getNormalsFromXYZRGB(cloud));

}

// Somefunctions for the surface reconstruction

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointNormal>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  //viewer->addPointCloudNormals<pcl::PointNormal, pcl::PointNormal> (cloud, cloud, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

float mag(pcl::PointNormal& p) {
  return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

void handleKeyEvent(const pcl::visualization::KeyboardEvent& ev, void *data) {
  //if(ev
}


pcl::PolygonMesh* CCTools::getMeshFromNormalCloud(pcl::PointCloud<pcl::PointNormal>::Ptr filtered_cloud) {
 
  // Create search tree*
  pcl::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointNormal>);
  tree2->setInputCloud(filtered_cloud);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh *triangles = new pcl::PolygonMesh();

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAgle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud(filtered_cloud);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(*triangles);

  //pcl::io::savePLYFile(outfile, triangles);
  //pcl::io::saveVTKFile(outfile, triangles);
 
 return triangles;
}

// Compute spin images comparison

static double CompareSpinImages(CCMesh* mesh1,int index1, CCMesh* mesh2, int index2)
{

	


	return 0;


}
