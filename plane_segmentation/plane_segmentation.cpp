#include <iostream>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include <string.h>
#include <math.h>
#include <vector>
using std::string;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> );
pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ> );

class Plane
{
  public:
    pcl::PointXYZ vector1 , vector2; // vector1 and vector2 define the plane
    pcl::PointXYZ normal;
    
    /**constructor*/
    Plane();
    
    /**destructor*/
    ~Plane();
};

Plane::Plane()
{
  this->vector1.x = 0.0;
  this->vector1.y = 0.0;
  this->vector1.z = 0.0;
  this->vector2.x = 0.0;
  this->vector2.y = 0.0;
  this->vector2.z = 0.0;
  this->normal.x = 0.0;
  this->normal.y = 0.0;
  this->normal.z = 0.0;
}


Plane::~Plane()
{
}

double Euclidean_Distance(pcl::PointXYZ a , pcl::PointXYZ b)
{
  double distance = sqrt(pow((a.x - b.x),2.0) + pow((a.y - b.y),2.0) + pow((a.z - b.z),2.0));
  return distance;
}

pcl::PointXYZ normalize( pcl::PointXYZ vector, double mag)
{ 

  pcl::PointXYZ vector1;
  vector1.x = (vector.x)/mag;
  vector1.y = (vector.y)/mag;
  vector1.z = (vector.z)/mag;
  
  return vector;
}

double dot(pcl::PointXYZ vector1 , pcl::PointXYZ vector2)
{
   double sum = 0;
   
   sum = (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z * vector2.z);
      
   return sum; 
      
}

pcl::PointXYZ crossproduct(pcl::PointXYZ vector1 , pcl::PointXYZ vector2)
{
  pcl::PointXYZ cross;
  
  cross.x = (vector1.y * vector2.z) - (vector1.z * vector2.y);
  cross.y = (vector1.z * vector2.x) - (vector1.x * vector2.z);
  cross.z = (vector1.x * vector2.y) - (vector1.y * vector2.x);
  
  return cross;
}

void createPlane(pcl::PointXYZ joint1 , pcl::PointXYZ planejoint , pcl::PointXYZ joint2 , Plane *plane)
{
  // create the two vectors
  pcl::PointXYZ vector_planejoint_joint1;
  pcl::PointXYZ vector_planejoint_joint2;
  pcl::PointXYZ midpoint;
  
  
  vector_planejoint_joint1.x = (joint1.x - planejoint.x);
  vector_planejoint_joint1.y = (joint1.y - planejoint.y);
  vector_planejoint_joint1.z = (joint1.z - planejoint.z);
  
  vector_planejoint_joint2.x = (joint2.x - planejoint.x);
  vector_planejoint_joint2.y = (joint2.y - planejoint.y);
  vector_planejoint_joint2.z = (joint2.z - planejoint.z);
  
  // get the cross product 
  pcl::PointXYZ crosspro = crossproduct(vector_planejoint_joint2 , vector_planejoint_joint1);
  
  // get the vector that divides the angle in half
  // first normalize the two vectors
  double vector_planejoint_joint1_length = Euclidean_Distance(planejoint,joint1);
  double vector_planejoint_joint2_length = Euclidean_Distance(planejoint,joint2);
  pcl::PointXYZ vector_planejoint_joint1_normalized = normalize(vector_planejoint_joint1,vector_planejoint_joint1_length);
  pcl::PointXYZ vector_planejoint_joint2_normalized = normalize(vector_planejoint_joint2,vector_planejoint_joint2_length);
  
  std::cout << "vector_planejoint_joint1_length"
  
  midpoint.x = (vector_planejoint_joint1_normalized.x + vector_planejoint_joint2_normalized.x)/2.0;
  midpoint.y = (vector_planejoint_joint1_normalized.y + vector_planejoint_joint2_normalized.y)/2.0;
  midpoint.z = (vector_planejoint_joint1_normalized.z + vector_planejoint_joint2_normalized.z)/2.0;
  
  // get the vector joining planejoint to midpoint
  pcl::PointXYZ vector_planejoint_midpoint;
  vector_planejoint_midpoint.x = midpoint.x - planejoint.x;
  vector_planejoint_midpoint.y = midpoint.y - planejoint.y;
  vector_planejoint_midpoint.z = midpoint.z - planejoint.z;
  
  // assign vectors to the plane
  plane->vector1 = crosspro;
  plane->vector2 = vector_planejoint_midpoint;
  
  // normal to the plane...always defined as the crossproduct of vector1 to vector2
  pcl::PointXYZ normal = crossproduct(plane->vector1 , plane->vector2);
    
  // assign plane normal
  plane->normal = normal; // this will always be pointing to joint1
  
}

pcl::PointXYZ inverse(pcl::PointXYZ vector)
{
  pcl::PointXYZ inv ( -vector.x , -vector.y , -vector.z);
  return inv;
}

void assignpoints( pcl::PointXYZ normal , pcl::PointXYZ origin , pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  int j = 0;
//  input->points.resize(cloud->points.size());
//  normal = inverse(normal);
  for( size_t i = 0 ; i < cloud->points.size() ; i++)
  {
     pcl::PointXYZ c = cloud->points[i];
     pcl::PointXYZ vector_origin_c;
     vector_origin_c.x = (c.x - origin.x);
     vector_origin_c.y = (c.y - origin.y);
     vector_origin_c.z = (c.z - origin.z);
     
//     normal = inverse(normal);
     
     double dotproduct = dot(vector_origin_c , normal);
     
     if( dotproduct >= 0)
       input->points.push_back(c);
     else 
       {
         std::cout<< " dot product = " << dotproduct << std::endl;
         continue;
       }
     
  }
  
//  input->points.resize(j);
}

int plane_segmentation(string filename , string outputfilename)
{

   int count_valid = 0;
   int j = 0;

   // load the file 
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename , *temp) == -1)
  {
     std::stringstream x;
     x <<"Couldn't read file " << filename << ".pcd\n";
     PCL_ERROR (x.str().c_str());
     return (-1);
  }
  
  
/*  for( size_t i = 0 ; i < temp->points.size() ; i++)
  {
     if(!(temp->points[i].x!= temp->points[i].x) && !(temp->points[i].y!= temp->points[i].y) && !(temp->points[i].z!= temp->points[i].z))
         count_valid++;
         
  }
  
  cloud->points.resize(count_valid);*/
   // remove NAN's 
  for( size_t i = 0 ; i < temp->points.size() ; i++)
  {
     if(!(temp->points[i].x!= temp->points[i].x) && !(temp->points[i].y!= temp->points[i].y) && !(temp->points[i].z!= temp->points[i].z))
        { 
           cloud->points.push_back(temp->points[i]);
                   
        }
         
  }

   // create point clouds for each segment
   pcl::PointCloud<pcl::PointXYZ>::Ptr lefthand_to_leftelbow_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr leftelbow_to_leftshoulder_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr leftshoulder_to_neck_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr neck_to_head_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr head_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr righthand_to_rightelbow_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr rightelbow_to_rightshoulder_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr rightshoulder_to_neck_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr torso_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr lefthip_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr righthip_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr lefthip_to_leftknee_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr leftknee_to_leftfoot_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr righthip_to_rightknee_cloud (new pcl::PointCloud<pcl::PointXYZ> );
   pcl::PointCloud<pcl::PointXYZ>::Ptr rightknee_to_rightfoot_cloud (new pcl::PointCloud<pcl::PointXYZ> );

  // create points to store the skeleton data
   pcl::PointXYZ lefthand(465.138/1000.0 , 721.439/1000.0 , 2733/1000.0);
   pcl::PointXYZ leftelbow(525.743/1000.0 , 459.264/1000.0 , 2763.42/1000.0);
   pcl::PointXYZ leftshoulder(269.907/1000.0 , 344.263/1000.0 , 2720.81/1000.0);
   pcl::PointXYZ lefthip(209.827/1000.0 , -69.1339/1000.0 , 2732.49/1000.0);
   pcl::PointXYZ leftknee(228.105/1000.0 , -452.24/1000.0 , 2625.13/1000.0);
   pcl::PointXYZ leftfoot(308.767/1000.0 , -831.693/1000.0 , 2715.15/1000.0);
   
   pcl::PointXYZ neck(118.697/1000.0 , 345.682/1000.0 , 2715.33/1000.0);
   pcl::PointXYZ head(108.743/1000.0 , 570.901/1000.0 , 2696.07/1000.0);
   pcl::PointXYZ torso(116.507/1000.0 , 138.722/1000.0 , 2722.18/1000.0);
   
   pcl::PointXYZ righthand(-298.985/1000.0 , 654.029/1000.0 , 2690/1000.0);
   pcl::PointXYZ rightelbow(-361.125/1000.0 , 394.904/1000.0 , 2738.24/1000.0);
   pcl::PointXYZ rightshoulder(-32.5126/1000.0 , 347.1/1000.0 , 2709.84/1000.0);
   pcl::PointXYZ righthip(18.8066/1000.0 , -67.3416/1000.0 , 2725.56/1000.0);
   pcl::PointXYZ rightknee(19.7786/1000.0 , -444.286/1000.0 , 2646.43/1000.0);
   pcl::PointXYZ rightfoot(-33.27/1000.0 , -819.004/1000.0 , 2770.39/1000.0);
   
   // length of each joint
   double lefthand_to_leftelbow_length = Euclidean_Distance(lefthand , leftelbow);
   double righthand_to_rightelbow_length = Euclidean_Distance(righthand , rightelbow);
   double leftelbow_to_leftshoulder_length = Euclidean_Distance(leftelbow , leftshoulder);
   double rightelbow_to_rightshoulder_length = Euclidean_Distance(rightelbow , rightshoulder);
   double leftshoulder_to_neck_length = Euclidean_Distance(leftshoulder , neck);
   double rightshoulder_to_neck_length = Euclidean_Distance(rightshoulder , neck);
   double neck_to_head_length = Euclidean_Distance(neck , head);
   double hip_to_hip_length = Euclidean_Distance(lefthip , righthip);
   double leftshoulder_to_torso_length = Euclidean_Distance(leftshoulder , torso);
   double rightshoulder_to_torso_length = Euclidean_Distance(rightshoulder , torso);
   double lefthip_to_torso_length = Euclidean_Distance(lefthip , torso);
   double righthip_to_torso_length = Euclidean_Distance(righthip , torso);
   double lefthip_to_leftknee_length = Euclidean_Distance(lefthip , leftknee);
   double righthip_to_rightknee_length = Euclidean_Distance(righthip , rightknee);
   double leftknee_to_leftfoot_length = Euclidean_Distance(leftknee , leftfoot);
   double rightknee_to_rightfoot_length = Euclidean_Distance(rightknee , rightfoot);
   
   // create plane object for each plane
   Plane *lefthand_plane = new Plane();
   Plane *leftelbow_plane = new Plane();
   Plane *leftshoulder_plane = new Plane();
   Plane *neck_plane = new Plane();
   Plane *head_plane = new Plane();
   Plane *rightshoulder_plane = new Plane();
   Plane *rightelbow_plane = new Plane();
   Plane *righthand_plane = new Plane();
   Plane *torso_horizontal_plane = new Plane();
   Plane *torso_vertical_plane = new Plane();
   Plane *lefthip_plane = new Plane();
   Plane *leftknee_plane = new Plane();
   Plane *leftfoot_plane = new Plane();
   Plane *righthip_plane = new Plane();
   Plane *rightknee_plane = new Plane();
   Plane *rightfoot_plane = new Plane();
   
   // create the planes
   createPlane(lefthand , leftelbow , leftshoulder , leftelbow_plane);
//   createPlane(rightfoot , rightknee , righthip , rightknee_plane);
   // assign points 
   assignpoints(leftelbow_plane->normal , leftelbow , lefthand_to_leftelbow_cloud);
//   assignpoints(rightelbow_plane->normal , rightelbow , rightknee_to_rightfoot_cloud);
   
   pcl::PointCloud<pcl::PointXYZ>::Ptr currentcloud (new pcl::PointCloud<pcl::PointXYZ>);
   currentcloud = lefthand_to_leftelbow_cloud;
   pcl::PointCloud<pcl::PointXYZ> output;
   output.points.resize(currentcloud->points.size());
    std::cout << "entering for loop" << std::endl;
    for(size_t i = 0 ; i < currentcloud->points.size() ; i++)
    {
       output.points[i].x = currentcloud->points[i].x;
       output.points[i].y = currentcloud->points[i].y;
       output.points[i].z = currentcloud->points[i].z;
    }
    std::cout<< " written to file" << std::endl;
    string in = "../../plane_segmentation/segmented/" ;
    in += outputfilename;
    pcl::io::savePCDFileASCII (in.c_str(), output);
    std::cout<< " saved to file" << std::endl;
   
   std::cout << " currentcloud size is = " << currentcloud->points.size() << std::endl;
   
//   std::cout << " leftelbow plane vector1 is " << leftelbow_plane->vector1.x << leftelbow_plane->vector1.y << leftelbow_plane->vector1.z << std::endl;
   
   
  
   
   

  return(0);
}

int main (int argc , char** argv)
{
   if(argc != 3)
   {
     std::cout<< "Usage: segmentation <file_name> " << std::endl;
     exit(0);
   }
   
   plane_segmentation(argv[1] , argv[2]);
  
  return(0);
   
}
