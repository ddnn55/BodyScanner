#include <iostream>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include <string.h>
#include <math.h>
#include <vector>
using std::string;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> );
pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ> );
   

double Euclidean_Distance(pcl::PointXYZ a , pcl::PointXYZ b)
{
  double distance = sqrt(pow((a.x - b.x),2.0) + pow((a.y-b.y),2.0) + pow((a.z-b.z),2.0));
  return distance;
}

pcl::PointXYZ normalize( pcl::PointXYZ a , pcl::PointXYZ b , double mag)
{ 

  pcl::PointXYZ vector;
  vector.x = (b.x - a.x)/mag;
  vector.y = (b.y - a.y)/mag;
  vector.z = (b.z - a.z)/mag;
  
  return vector;
}

double dot(pcl::PointXYZ vector1 , pcl::PointXYZ vector2)
{
   double sum = 0;
   
   sum = (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z * vector2.z);
      
   return sum; 
      
}

void assignPoints( pcl::PointXYZ a , pcl::PointXYZ b , double length , double radius , pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{

   pcl::PointXYZ normalized = normalize(a,b,length);
   int j = 0;
   input->points.resize(cloud->points.size());
   std::cout<< "cloud size = " << cloud->points.size() << std::endl;
   std::cout<< "length = " << length << std::endl;
   std::cout<< "radius = " << radius << std::endl;
  
   for(size_t i = 0 ; i < cloud->points.size() ; i++)
   {
    
          pcl::PointXYZ c = cloud->points[i];
          pcl::PointXYZ vector_ac;
          
          vector_ac.x = c.x - a.x;
          vector_ac.y = c.y - a.y;
          vector_ac.z = c.z - a.z;
          
          double dotproduct = dot(normalized , vector_ac);
//          std::cout<< " dot product = " << dotproduct << std::endl;
          if(dotproduct < 0) // dotproduct should not be less than 0
            continue;
          else                     
          if(dotproduct <= length) // if dotproduct is greater than 0, it must be less than the length of the vector between two joints
          {
 //          std::cout<< " dot product = " << dotproduct << std::endl;
            double distance = sqrt(pow(Euclidean_Distance(a,c),2.0) - pow(dotproduct,2.0));
            
 //           std::cout << " distance = " << distance << std::endl;
            if (distance > radius) // lies outside the affiliated radius
               continue;
               
            else 
              if(distance <= radius)// add the point to the cloud
               {
                 input->points[j++] = c;
                 
               }
          }
   }
   
  input->points.resize(j);
   
     
} 



int segmentation (string filename , string outputfilename)
{
      
   int count_valid = 0;
   int j = 0;
   
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
   pcl::PointXYZ lefthand(2.61650214587 , 0.42341399938 , 0.741739473523);
   pcl::PointXYZ leftelbow(2.6694900322 , 0.47918342009 , 0.490597951981);
   pcl::PointXYZ leftshoulder(2.62625012634 , 0.262870272978 , 0.363720638888);
   pcl::PointXYZ lefthip(2.63965052228 , 0.192339774104 , -0.0376897763944);
   pcl::PointXYZ leftknee(2.61901994098 , 0.247822165975 , -0.438225348963);
   pcl::PointXYZ leftfoot(2.61006204065 , 0.279194157608 , -0.776382878318);
   
   pcl::PointXYZ neck(2.62798041482 , 0.120929086706 , 0.372099904995);
   pcl::PointXYZ head(2.62009243892 , 0.133765620842 , 0.574407307017);
   pcl::PointXYZ torse(2.6342453413 , 0.109085436012 , 0.170089673852);
   
   pcl::PointXYZ righthand(2.57094874609 , -0.230743653914 , 0.704013261464);
   pcl::PointXYZ rightelbow(2.67929422096 , -0.275082718219 , 0.464603531212);
   pcl::PointXYZ rightshoulder(2.62971094745 , -0.0210120804922 , 0.380479201619);
   pcl::PointXYZ righthip(2.64326965169 , 0.00574492112083 , -0.0279414280981);
   pcl::PointXYZ rightknee(2.61471274281 , 0.00723112289278 , -0.43181264393);
   pcl::PointXYZ rightfoot(2.62011127176 , -0.0365560763252 , -0.783027446477);

   // radius of the cylinder between joints   
   double hand_to_elbow_radius = 1.3;
   double elbow_to_shoulder_radius = 0.05;
   double shoulder_to_neck_radius = 0.08;
   double neck_to_head_radius = 0.06;
   double head_radius = 0.1;
   double torso_radius = 0.3;
   double hip_to_hip_radius = 0.3 ;
   double hip_to_knee_radius = 0.1;
   double knee_to_foot_radius = 0.07;
   
   // length of each cylinder
   double hand_to_elbow_length = Euclidean_Distance(lefthand , leftelbow);
   double elbow_to_shoulder_length = Euclidean_Distance(leftelbow , leftshoulder);
   double shoulder_to_neck_length = Euclidean_Distance(leftshoulder , neck);
   double neck_to_head_length = Euclidean_Distance(neck , head);
   double head_length = 0.14;
   double hip_to_hip_length = Euclidean_Distance(lefthip , righthip);
   double torso_length = hip_to_hip_length - (hip_to_hip_length * 0.1);
   double hip_to_knee_length = Euclidean_Distance(lefthip , leftknee);
   double knee_to_foot_length = Euclidean_Distance(leftknee , leftfoot);
   
  
      
  // load the file 
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename , *temp) == -1)
  {
     std::stringstream x;
     x <<"Couldn't read file " << filename << ".pcd\n";
     PCL_ERROR (x.str().c_str());
     return (-1);
  }
  
  
  for( size_t i = 0 ; i < temp->points.size() ; i++)
  {
     if(!(temp->points[i].x!= temp->points[i].x) && !(temp->points[i].y!= temp->points[i].y) && !(temp->points[i].z!= temp->points[i].z))
         count_valid++;
         
  }
  
  cloud->points.resize(count_valid);
   // remove NAN's 
  for( size_t i = 0 ; i < temp->points.size() ; i++)
  {
     if(!(temp->points[i].x!= temp->points[i].x) && !(temp->points[i].y!= temp->points[i].y) && !(temp->points[i].z!= temp->points[i].z))
        { 
           cloud->points[j++] = temp->points[i];
                   
        }
         
  }
  
  assignPoints(rightelbow , righthand , hand_to_elbow_length , hand_to_elbow_radius , righthand_to_rightelbow_cloud);
  
//  for(size_t i = 0 ; i < righthand_to_rightelbow_cloud->points.size() ; i++)
  // std::cout << i << "th point is " << righthand_to_rightelbow_cloud->points[i] << std::endl;
   
   std::cout << " size of righthand_to_rightelbow_cloud is " << righthand_to_rightelbow_cloud->points.size();
   
   pcl::PointCloud<pcl::PointXYZ> output;
   output.points.resize(righthand_to_rightelbow_cloud->points.size());
    std::cout << "entering for loop" << std::endl;
    for(size_t i = 0 ; i < righthand_to_rightelbow_cloud->points.size() ; i++)
    {
       output.points[i].x = righthand_to_rightelbow_cloud->points[i].x;
       output.points[i].y = righthand_to_rightelbow_cloud->points[i].y;
       output.points[i].z = righthand_to_rightelbow_cloud->points[i].z;
    }
    std::cout<< " written to file" << std::endl;
    string in = "../../segmentation/segmented/" ;
    in += outputfilename;
    pcl::io::savePCDFileASCII (in.c_str(), output);
    std::cout<< " saved to file" << std::endl;
   
   
/*   std::cout << "hand_to_elbow_length: " << hand_to_elbow_length << std::endl;
   std::cout << "elbow_to_shoulder_length: " << elbow_to_shoulder_length << std::endl;
   std::cout << "shoulder_to_neck_length: " << shoulder_to_neck_length << std::endl;
   std::cout << "neck_to_head_length: " << neck_to_head_length << std::endl;
   std::cout << "hip_to_hip_length: " << hip_to_hip_length << std::endl;
   std::cout << "torso_length: " << torso_length << std::endl;
   std::cout << "hip_to_knee_length: " << hip_to_knee_length << std::endl;
   std::cout << "knee_to_foot_length: " << knee_to_foot_length << std::endl;*/
   
   return(0);
}

int main (int argc , char** argv)
{
  if(argc != 3)
   {
     std::cout<< "Usage: segmentation <file_name> " << std::endl;
     exit(0);
   }
   
   segmentation(argv[1] , argv[2]);
  
  return(0);
}
