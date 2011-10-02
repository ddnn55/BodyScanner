#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <string>
using std::string;

int iterative_closest_point( string input , string target)
{
   size_t j = 0;
   int count_valid = 0;
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp1 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp2 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
   
      
   // read data points from both files
   
   if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (input , *temp1) == -1)
  {
    std::stringstream x;
     x <<"Couldn't read file " << input << ".pcd\n";
     PCL_ERROR (x.str().c_str());
     return (-1);
  }
//  *cloud_in = *temp1;
  
  for( size_t i = 0 ; i < temp1->points.size() ; i++)
  {
     if(!(temp1->points[i].x!= temp1->points[i].x) && !(temp1->points[i].y!= temp1->points[i].y) && !(temp1->points[i].z!= temp1->points[i].z))
         count_valid++;
         
  }
  
  cloud_in->points.resize(count_valid);
  count_valid = 0;
  
  for( size_t i = 0 ; i < temp1->points.size() ; i++)
  {
     if(!(temp1->points[i].x!= temp1->points[i].x) && !(temp1->points[i].y!= temp1->points[i].y) && !(temp1->points[i].z!= temp1->points[i].z))
         cloud_in->points[j++] = temp1->points[i];
         
  }
  
//  std::cout<< "Size of temp " << temp1->points.size() << std::endl;
//  std::cout<< "Size of cloud in " << cloud_in->points.size() << std::endl;
  
   
/*  for( size_t i = 0 ; i < cloud_in->points.size() ; i++)
  {
     if(!(cloud_in->points[i].x!= cloud_in->points[i].x) && !(cloud_in->points[i].y!= cloud_in->points[i].y) && !(cloud_in->points[i].z!= cloud_in->points[i].z))
     std::cout << "Point " << i << " ";
         
  }*/
   
   if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (target , *temp2) == -1)
  {
     PCL_ERROR ("Couldn't read file test_pcd.pcd\n");
     return (-1);
  }
  
  
  
//  *cloud_out = *temp2;

  for( size_t i = 0 , j = 0 ; i < temp2->points.size() ; i++)
  {
     if(!(temp2->points[i].x!= temp2->points[i].x) && !(temp2->points[i].y!= temp2->points[i].y) && !(temp2->points[i].z!= temp2->points[i].z))
         count_valid++;
  }
  
  cloud_out->points.resize(count_valid);
  
  for( size_t i = 0 , j = 0 ; i < temp2->points.size() ; i++)
  {
     if(!(temp2->points[i].x!= temp2->points[i].x) && !(temp2->points[i].y!= temp2->points[i].y) && !(temp2->points[i].z!= temp2->points[i].z))
         cloud_out->points[j++] = temp2->points[i];
  }
  
//  cloud_out->points.resize(5);
  
 // std::cout << " target point size = " << cloud_out->points.size() << std::endl;
  
  pcl::IterativeClosestPoint<pcl::PointXYZRGB , pcl::PointXYZRGB> icp;// create an object of icp
//  icp.setMaxCorrespondenceDistance(100);
//  icp.setMaximumIterations(2);
  icp.setInputCloud(cloud_in); // set input cloud
  icp.setInputTarget(cloud_out); // set target cloud
  pcl::PointCloud<pcl::PointXYZRGB> Final;
  std::cout<<" Alignment started" << std::endl;
  icp.align(Final); // start alignment and store result in Final
  std::cout<<" Alignment completed" << std::endl;
  pcl::io::savePCDFileASCII ("../final_pcd.pcd", Final);
  std::cerr << "Saved " << Final.points.size () << " data points to final_pcd.pcd." << std::endl;

  std::cout<< " has converged " << icp.hasConverged() << " fitness score " << icp.getFitnessScore() << std::endl;
  std::cout<< icp.getFinalTransformation() << std::endl;
//  std::cout<< " final has points = " << Final.points.size()<< std::endl;
  
  return(0);
  
  
}

int main (int argc , char** argv)
{
  if(argc < 3) {
    printf("usage: icp_align left.pcd right.pcd\n");
    exit(1);
  }
  iterative_closest_point(argv[1], argv[2]);
  return (0);

}
