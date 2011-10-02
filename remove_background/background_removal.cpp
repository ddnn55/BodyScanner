#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include "pcl/io/pcd_io.h"
#include <string.h>

int
 main (int argc, char** argv)
{
  if (argc != 8)
  {
    std::cerr << "Please give the following input -r/-c <file_path> < file_name> <lower_bound z> <upper_bound z> <lower_bound x> <upper_bound x> " << std::endl;
    exit(0);
  }
  
  float lower_z = atof(argv[4]);
  float upper_z = atof(argv[5]);
  float lower_x = atof(argv[6]);
  float upper_x = atof(argv[7]);
  std::cout<< lower_z  << upper_z  << lower_x << upper_x << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  int count_valid = 0;
  size_t j = 0;
  
  // get the cloud data in cloud ptr
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[2] , *temp) == -1)
   {
      PCL_ERROR(" Cannot read file: ");
      std:: cout<< argv[2] << std::endl;
      return(-1);
   }
   
   // remove nan data
   for( size_t i = 0 ; i < temp->points.size() ; i++)
  {
     if(!(temp->points[i].x!= temp->points[i].x) && !(temp->points[i].y!= temp->points[i].y) && !(temp->points[i].z!= temp->points[i].z))
         count_valid++;
         
  }
  
  cloud->points.resize(count_valid);
  count_valid = 0;
  
  for( size_t i = 0 ; i < temp->points.size() ; i++)
  {
     if(!(temp->points[i].x!= temp->points[i].x) && !(temp->points[i].y!= temp->points[i].y) && !(temp->points[i].z!= temp->points[i].z))
         cloud->points[j++] = temp->points[i];
         
  }

 

  if (strcmp(argv[1], "-r") == 0){
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    outrem.filter (*cloud_filtered);
  }
  else if (strcmp(argv[1], "-c") == 0){
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new
      pcl::ConditionAnd<pcl::PointXYZRGB> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, lower_z)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, upper_z)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, lower_x)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, upper_x)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*cloud_filtered);
    // write point data to a file
    std::cout << "writing to file" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> output;
    output.points.resize(cloud_filtered->points.size());
    std::cout << "entering for loop" << std::endl;
    for(size_t i = 0 ; i < cloud_filtered->points.size() ; i++)
    {
       output.points[i].x = cloud_filtered->points[i].x;
       output.points[i].y = cloud_filtered->points[i].y;
       output.points[i].z = cloud_filtered->points[i].z;
    }
    std::cout<< " written to file" << std::endl;
    char* out = (char*) malloc(sizeof(char)*strlen(argv[3]));
    char* in = (char*) malloc(sizeof(char*));
    strcpy(out,argv[3]);
    strcpy(in,"../filtered_files/");
    pcl::io::savePCDFileASCII (strcat(in,out) , output);
    std::cout<< " saved to file" << std::endl;
    free(out);
  }
  else{
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
/*  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;*/
  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
                        << cloud_filtered->points[i].y << " "
                        << cloud_filtered->points[i].z << std::endl;
  return (0);
}

