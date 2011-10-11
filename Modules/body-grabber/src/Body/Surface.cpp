
#include "Body/Surface.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>

namespace Body {

void filterPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
									pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud) {
										
		pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZRGB> > sor;
		sor.setInputCloud(input_cloud);
		sor.setLeafSize(0.01f, 0.01f, 0.01f);
		sor.filter(*filtered_cloud);
}

void smoothPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_points,
									pcl::PointCloud<pcl::PointNormal>::Ptr output_points) {

  //pcl::PointCloud<pcl::PointXYZ>::Ptr no_color_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::copyPointCloud(input_points);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::Normal> mls; // can we input xyzrgb?

  pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
  tree->setInputCloud(input_points);

  // Optionally, a pointer to a cloud can be provided, to be set by MLS
  pcl::PointCloud<pcl::Normal>::Ptr mls_normals(new pcl::PointCloud<pcl::Normal>());
  mls.setOutputNormals(mls_normals);

  // Set parameters
  mls.setInputCloud(input_points);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);

	pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>());
  // Reconstruct
  mls.reconstruct(mls_points);

  pcl::concatenateFields(mls_points, *mls_normals, output_points)
}


pcl::PolygonMesh::Ptr buildSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointNormal>::Ptr smooth_cloud(new pcl::PointCloud<pcl::PointNormal>());
	printf("filtering points\n");
	filterPoints(cloud, filtered_cloud); // this will reduce the # of points
	printf("smoothing points\n");
	// this will not reduce the # of points, indices will remain the same
	// at the end we will copy colors from filtered_cloud onto result
	smoothPoints(filtered_cloud, smooth_cloud); // TODO: be more careful with memory? can we reuse other clouds?
	
	printf("constructing kdtree\n");
  // Create search tree*
  pcl::KdTree<pcl::PointNormal>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointNormal>);
  tree->setInputCloud(smooth_cloud);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(0.1);

  // Set typical values for the parameters
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(50); // reducing this didn't fix flips
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees // requires pcl 1.2 (typo fix)
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true); // changing this didn't fix flips

  // Get result
  gp3.setInputCloud(smooth_cloud);
  gp3.setSearchMethod(tree);
	printf("constructing surface\n");
  gp3.reconstruct(triangles);
   
	// copy colors back onto surface
	pcl::copyPointCloud(*filtered_cloud, triangles.points);
	return triangles;
}

}

