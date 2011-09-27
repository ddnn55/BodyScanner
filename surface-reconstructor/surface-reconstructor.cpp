#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <iostream>
#include <unistd.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/io/vtk_io.h>

// forward declarations
void mls(std::string fname, std::string outname);
void viz(std::string fname);
void recons(std::string infile, std::string outfile);

void print_usage() {
  std::cout << "usage: surface-reconstructor [-s something.pcd | -v | -r ]\n"
            << "  -s   smooth points using mls, dump to scan-mls.pcd\n"
            << "  -v   visualize smoothed points, load from scan-mls.pcd\n"
            << "  -r   smooth and reconstruct surface, load from scan-mls.pcd dump to scan.vtk\n";
}

// from the resample demo
int main (int argc, char** argv) {
  std::string infile = "../../recordings/mark1_pcd/1316652689.744386960.pcd";    
  std::string smoothfile = "scan-mls.pcd";
  std::string outmesh = "scan.vtk"; // there's a ply export in pcl 1.2, but ... we don't have that 
  if(argc > 1) {
    if(strcmp(argv[1], "-s") == 0) {
      if(argc > 2) {
        infile = argv[2];
      }
      if(access(infile.c_str(), R_OK) >= 0) {
        std::cout << "running Moving Least Squares smoothing, this takes about 30s\n";
        mls(infile, smoothfile);
      } else {
        print_usage();
        std::cout << " ! unable to find " << infile << std::endl;
      }
    } else if(strcmp(argv[1], "-v") == 0) {
      if(access(smoothfile.c_str(), R_OK) >= 0) {
        viz(smoothfile);
      } else {
        print_usage();
        std::cout << " ! unable to find " << smoothfile << std::endl;
      }
    } else if(strcmp(argv[1], "-r") == 0) {

      if(access(smoothfile.c_str(), R_OK) >= 0) {
        std::cout << "running greedy projection, this takes about 4s\n";
        recons(smoothfile, outmesh);
      } else {
        print_usage();
        std::cout << " ! unable to find " << smoothfile << std::endl;
      }
    } else {
      print_usage();
    }
  } else {
    print_usage();
  }
}

void mls(std::string fname, std::string outname) {
  // Load input file into a PointCloud<T> with an appropriate type
  sensor_msgs::PointCloud2 cloud_blob;
  // run this from BodyScanner/build/surface_reconstructor/
  pcl::io::loadPCDFile(fname, cloud_blob);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());  
  pcl::fromROSMsg (cloud_blob, *color_cloud);
  
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

  // Save output
  pcl::io::savePCDFile(outname, mls_cloud);
}

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

void handleKeyEvent(const pcl::visualization::KeyboardEvent& ev, void *data) {
  //if(ev
}

float mag(pcl::PointNormal& p) {
  return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

void viz(std::string fname) {
  sensor_msgs::PointCloud2 cloud_blob;
  // run this from BodyScanner/build/surface_reconstructor/
  pcl::io::loadPCDFile(fname, cloud_blob);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal> ());  
  pcl::fromROSMsg (cloud_blob, *cloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointNormal>());
  int l = 0;
  for(int j = 0; j < 480; j += 1) {
    for(int i = j%2; i < 640; i += 2) {
      int k = i + j*640;
      pcl::PointNormal& p = (*cloud)[k];
      if((i&1 && j&1) // keep 1/4 of the points
         && mag(p) < 3) { // keep only points within a close distance to the origin
        filtered_cloud->push_back(p);
      }
    }
  }

  // potentially fancier downsampling
/*
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);
*/

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = normalsVis(filtered_cloud);
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

void recons(std::string infile, std::string outfile) {
  sensor_msgs::PointCloud2 cloud_blob;
  // run this from BodyScanner/build/surface_reconstructor/
  pcl::io::loadPCDFile(infile, cloud_blob);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal> ());  
  pcl::fromROSMsg (cloud_blob, *cloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointNormal>());
  int l = 0;
  for(int j = 0; j < 480; j += 1) {
    for(int i = j%2; i < 640; i += 2) {
      int k = i + j*640;
      pcl::PointNormal& p = (*cloud)[k];
      if((i&1 && j&1) // keep 1/4 of the points
         && mag(p) < 3) { // keep only points within a close distance to the origin
        filtered_cloud->push_back(p);
      }
    }
  }

  // Create search tree*
  pcl::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointNormal>);
  tree2->setInputCloud(filtered_cloud);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

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
  gp3.reconstruct(triangles);

  //pcl::io::savePLYFile(outfile, triangles);
  pcl::io::saveVTKFile(outfile, triangles);
}

