#include <iostream>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include <string.h>
#include <math.h>
#include <vector>
#include <sstream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "SkeletonYaml.h"

using std::string;

#define NB_JOINTS 15
#define NB_BONES 9

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> );
pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ> );

// Define the structure for skinning weights.
struct skinWeights {
    int index;   // x coordinate
    double weight;   // y coordinate
};
// Labels for bones

enum joints {LH,LE,LS,LHI,LK,LF,N,H,T,RH,RE,RS,RHI,RK,RF};
enum bones {LH2E,LE2S,N2H,RS2E,RE2H,LH2K,RH2K,LK2F,RK2F};


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

double crossNorm(pcl::PointXYZ vector1 , pcl::PointXYZ vector2)
{
  return sqrt( pow(vector1.y*vector2.z - vector2.y*vector1.z, 2.0)
             + pow(vector1.x*vector2.z - vector2.x*vector1.z, 2.0)
             + pow(vector1.x*vector2.y - vector2.x*vector1.y, 2.0));
}

pcl::PointXYZ map(int bone)
{


	pcl::PointXYZ p;
	
	switch(bone)
	{
		case LH2E:
		p.x = LH;
		p.y = LE;
		break;

		case LE2S:
		p.x = LE;
		p.y = LS;
		break;

		case N2H:
		p.x = N;
		p.y = H;
		break;

		case RS2E:
		p.x = RS;
		p.y = RE;
		break;

		case RE2H:
		p.x = RE;
		p.y = RH;
		break;

		case LH2K:
		p.x = LHI;
		p.y = LK;
		break;

		case RH2K:
		p.x = RHI;
		p.y = RK;
		break;

		case LK2F:
		p.x = LK;
		p.y = LF;
		break;

		case RK2F:
		p.x = RK;
		p.y = RF;
		break;

		default:
		p.x = 0;
		p.y = 1;
		break;


	}

	return p;
	


}

int argmin(std::vector<double> vec)
{

	int res = 0;
	
	for (int i = 0 ; i< vec.size() ; i++)
	{

		if (vec[i] < vec[res])
		{	
			res = i;
		}

	}



	return res;

}

int argmin2(std::vector<double> vec, int argmin)
{

	int res = 0;
	int init;

	if (argmin = 0){
	init = 1
	}
	else {
	init = 0;
	}
	
	for (int i = init ; i< vec.size() ; i++)
	{

		if (vec[i] < vec[res] && i != argmin)
		{	
			res = i;
		}

	}



	return res;

}




void assignPoints(std::vector<pcl::PointXYZ> bones, std::vector<pcl::PointXYZ> joints, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> *limbs_clouds,int vizindex)
{
    std::vector<double> distances(NB_BONES);  
	pcl::PointXYZ c;
	pcl::PointXYZ ac;
	pcl::PointXYZ bc;


    int j = 0;
    int temparg;
    double temp;
    double length;
    double proj;
    for(size_t i = 0 ; i < cloud->points.size() ; i++)
    {    
        c = cloud->points[i];
          
        // Compute distances from point to bones

        for (int j = LH2E; j<= RK2F; j++)
        {
            ac.x = c.x - joints[(int)(map(j).x)].x;
            ac.y = c.y - joints[(int)(map(j).x)].y;
            ac.z = c.z - joints[(int)(map(j).x)].z;

            bc.x = c.x - joints[(int)(map(j).y)].x;
            bc.y = c.y - joints[(int)(map(j).y)].y;
            bc.z = c.z - joints[(int)(map(j).y)].z;


            length = sqrt(dot(bones[j],bones[j]));
            proj = dot(bones[j],ac)/length;

            if (proj > length) {
            temp = sqrt(dot(bc,bc));
            }
            else if (proj <0)
            {
            temp = sqrt(dot(ac,ac));
            }
            else {
                temp = crossNorm(bones[j],ac)/length;
            }
	        distances[j] = temp;                    
        }

	// Add point to argmin bone

	//  (*limbs_clouds)[argmin(distances)]->push_back(c);
	 
	temparg = argmin(distances);

    if (temparg == vizindex){
        cloud->points[i].x = 0;
        cloud->points[i].y = 0;
        cloud->points[i].z = 0;
    }
	
	(*limbs_clouds)[temparg]->points[i] = c; 
   }
} 


int segmentation(string cloudFilename, string skeletonFilename, int index)
{	
    int timestamp = 9999;
   int count_valid = 0;
   int j = 0;

 if(pcl::io::loadPCDFile<pcl::PointXYZ> (cloudFilename , *cloud) == -1)
  {
     std::stringstream x;
     x <<"Couldn't read file " << cloudFilename << ".pcd\n";
     PCL_ERROR (x.str().c_str());
     return (-1);
  }
  SkeletonYaml skel(skeletonFilename.c_str());
   

// create vectors for bones and joints
   std::vector<pcl::PointXYZ> joints(NB_JOINTS);
   std::vector<pcl::PointXYZ> bones(NB_BONES);
   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> limbs_clouds(NB_BONES);


      for (int k = LH2E; k<= RK2F; k++)
	{

	limbs_clouds[k] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	limbs_clouds[k]->resize(cloud->size());

	}

    // create points to store the skeleton data
    for(int i = 0; i < SkeletonYaml::numJoints; i++) {
        joints[i] = pcl::PointXYZ(
            skel.translations[i][0],
            skel.translations[i][1],
            skel.translations[i][2]
        );
    }
   

     
// Compute bones vectors   

	double a = 0;
	double b = 0;

	for (int m = LH; m<= RF; m++){
		joints[m].y = -joints[m].y/1000;
		joints[m].z = joints[m].z/1000;
		joints[m].x = joints[m].x/1000;
	}

	for (int m = LH2E; m<= RK2F; m++){
		bones[m].x = joints[(int)(map(m).y)].x - joints[(int)(map(m).x)].x;
		bones[m].y = joints[(int)(map(m).y)].y - joints[(int)(map(m).x)].y;
		bones[m].z = joints[(int)(map(m).y)].z - joints[(int)(map(m).x)].z;
	}

// Compute 1st method bone ownership
      
    
  assignPoints(bones,joints,&limbs_clouds,index);

// Check bones clouds size and output

	
     /* for (int k = LH2E; k<= RK2F; k++)
	{

	
	std::cout << "Limb # " << k << " "  << std::endl;
	std::cout<< " written to file" << std::endl;
   	string in = "../segmented" ;
    	std::stringstream ss;
	ss << in << timestamp << "/" << k << ".pcd";
	
    	pcl::io::savePCDFileASCII (ss.str().c_str(), *limbs_clouds[k]);
    	std::cout<< " saved to file" << std::endl;

	}
*/


  

//pcl::visualization::CloudViewer viewer("Cloud Viewer");


// Visualization stuff
boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer ("3D Viewer"));

view->addCoordinateSystem (1.0);
view->initCameraParameters ();

// Visualizasion of the joints position
	for (int m = LH; m<= RF; m++){

		std::stringstream s;
		s << m;
		
	  	view->addSphere(joints[m], 0.025, s.str(), 0);
	}

	for (int m = LH2E; m<= RK2F; m++){
		std::stringstream s;
		s << m << m << m;

		if (m == index){
		view->addLine(joints[(int)(map(m).y)],joints[(int)(map(m).x)],255,0,0,s.str(),0);
		}
		else {
		view->addLine(joints[(int)(map(m).y)],joints[(int)(map(m).x)],s.str(),0);
		}
	}

	
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(limbs_clouds[index], 255, 0, 0);
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud, 0, 120, 0);
     
	view->addPointCloud(limbs_clouds[index],color1,"littlecloud");
	view->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE*10, 1, "littlecloud");
	view->addPointCloud(cloud,color2,"cloud");
	view->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE/4, 1, "cloud");
	


    //blocks until the cloud is actually rendered
    
while (!view->wasStopped ())
    {

view->spinOnce (100);
boost::this_thread::sleep (boost::posix_time::microseconds (100000));
       }

    
   

   return(0);
}

int main (int argc , char** argv)
{
  if(argc != 4)
   {
     std::cout<< "Usage: segmentation2 <cloud_file_name> <skeleton_file_name> <integer from 0 to 8>" << std::endl;
     exit(0);
   }
   
   segmentation(argv[1], argv[2], atoi(argv[3]));
  
  return(0);
}
