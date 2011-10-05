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

using std::string;

#define NB_JOINTS 15
#define NB_BONES 9

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> );
pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ> );


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
   return (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z * vector2.z);
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


void assignPoints(std::vector<pcl::PointXYZ> bones, std::vector<pcl::PointXYZ> joints, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> *limbs_clouds)
{



        std::vector<double> distances(NB_BONES);  
	pcl::PointXYZ c;
	pcl::PointXYZ ac;


   int j = 0;
   //input->points.resize(cloud->points.size());
   
  
   for(size_t i = 0 ; i < cloud->points.size() ; i++)
   {
    
          c = cloud->points[i];
          
       	  // Compute distances from point to bones

	  for (int j = LH2E; j<= RK2F; j++)
	  {

		 
          ac.x = c.x - joints[(int)(map(j).y)].x;
          ac.y = c.y - joints[(int)(map(j).y)].y;
          ac.z = c.z - joints[(int)(map(j).y)].z;




	  distances[j] = crossNorm(bones[j],ac)/sqrt(dot(bones[j],bones[j]));

	//std::cout << "Bone length " << j << "  = " << sqrt(dot(bones[j],bones[j])) << std::endl;

	  //std::cout << "Distance from point " << i << " to bone " << j << " = " << distances[j] << std::endl;
                    
          }

	// Add point to argmin bone

	  (*limbs_clouds)[argmin(distances)]->push_back(c); 

	
          
   }


     
} 



int segmentation (string filename , int index)
{
   int count_valid = 0;
   int j = 0;

  // load the file 
 
// if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename , *temp) == -1)


 if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename , *cloud) == -1)
  {
     std::stringstream x;
     x <<"Couldn't read file " << filename << ".pcd\n";
     PCL_ERROR (x.str().c_str());
     return (-1);
  }

/*

  // Fix cloud
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

*/
   

// create vectors for bones and joints
   std::vector<pcl::PointXYZ> joints(NB_JOINTS);
   std::vector<pcl::PointXYZ> bones(NB_BONES);
   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> limbs_clouds(NB_BONES);


      for (int k = LH2E; k<= RK2F; k++)
	{

	limbs_clouds[k] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	}



// create points to store the skeleton data
/*
   joints[LH] = pcl::PointXYZ(2.61650214587 , 0.42341399938 , 0.741739473523);
   joints[LE] = pcl::PointXYZ(2.6694900322 , 0.47918342009 , 0.490597951981);
   joints[LS] = pcl::PointXYZ(2.62625012634 , 0.262870272978 , 0.363720638888);
   joints[LHI] = pcl::PointXYZ(2.63965052228 , 0.192339774104 , -0.0376897763944);
   joints[LK] = pcl::PointXYZ(2.61901994098 , 0.247822165975 , -0.438225348963);
   joints[LF] = pcl::PointXYZ(2.61006204065 , 0.279194157608 , -0.776382878318);
   
   joints[N] = pcl::PointXYZ(2.62798041482 , 0.120929086706 , 0.372099904995);
   joints[H] = pcl::PointXYZ(2.62009243892 , 0.133765620842 , 0.574407307017);
   joints[T] = pcl::PointXYZ(2.6342453413 , 0.109085436012 , 0.170089673852);
   
   joints[RH] = pcl::PointXYZ(2.57094874609 , -0.230743653914 , 0.704013261464);
   joints[RE] = pcl::PointXYZ(2.67929422096 , -0.275082718219 , 0.464603531212);
   joints[RS] = pcl::PointXYZ(2.62971094745 , -0.0210120804922 , 0.380479201619);
   joints[RHI] = pcl::PointXYZ(2.64326965169 , 0.00574492112083 , -0.0279414280981);
   joints[RK] = pcl::PointXYZ(2.61471274281 , 0.00723112289278 , -0.43181264393);
   joints[RF] = pcl::PointXYZ(2.62011127176 , -0.0365560763252 , -0.783027446477);
*/

joints[LH] = pcl::PointXYZ(2.94981203421, -0.310165403751, 0.22766924897);
joints[RH] = pcl::PointXYZ(2.25765830251, 0.525413078002, 0.22013626892);
joints[LK] = pcl::PointXYZ(2.54905170716, -0.0750193843694, -0.421143976229);
joints[LS] = pcl::PointXYZ(2.56312031822, -0.098670814321, 0.354840713311);
joints[RE] = pcl::PointXYZ(2.17971916283, 0.267622661098, 0.247073627964);
joints[H] = pcl::PointXYZ(2.54155516756, 0.0321924411224, 0.561667788373);
joints[LHI] = pcl::PointXYZ(2.44146131848, -0.0681756099542, -0.0309315391697);
joints[LF] = pcl::PointXYZ(2.47534371769, -0.108826806909, -0.782286008953);
joints[T] = pcl::PointXYZ(2.43844688376, 0.0076821794574, 0.169575622553);
joints[LE] = pcl::PointXYZ(2.65717451047, -0.343801885163, 0.240742024938);
joints[RF] = pcl::PointXYZ(2.29353501992, 0.15205008112, -0.749121278636);
joints[RS] = pcl::PointXYZ(2.40287003609, 0.134917197207, 0.37869085028);
joints[N] = pcl::PointXYZ(2.48299517715, 0.018123191443, 0.366765781795);
joints[RK] = pcl::PointXYZ(2.44338954755, 0.22577034287, -0.333426852274);
joints[RHI] = pcl::PointXYZ(2.3326878762, 0.0827771917936, -0.0142110859517);
     
// Compute bones vectors   

	double a = 0;
	double b = 0;

	for (int m = LH; m<= RF; m++){
/*
		a = joints[m].y;
		joints[m].y = joints[m].z;
		joints[m].z = joints[m].x;
		joints[m].x = a;
*/
	}
	

	for (int m = LH2E; m<= RK2F; m++){

		bones[m].x = joints[(int)(map(m).y)].x - joints[(int)(map(m).x)].x;
		bones[m].y = joints[(int)(map(m).y)].y - joints[(int)(map(m).x)].y;
		bones[m].z = joints[(int)(map(m).y)].z - joints[(int)(map(m).x)].z;
	}

// Compute 1st method bone ownership
      
    
  assignPoints(bones,joints,&limbs_clouds);

// Check bones clouds size and output

	
      for (int k = LH2E; k<= RK2F; k++)
	{

	
	std::cout << "Limb # " << k << " " << limbs_clouds[k]->points.size() << std::endl;

/*
	std::cout<< " written to file" << std::endl;
   	string in = "../segmented/" ;
    	std::stringstream ss;
	ss << in << k << ".pcd";
	
    	pcl::io::savePCDFileASCII (ss.str().c_str(), *limbs_clouds[k]);
    	std::cout<< " saved to file" << std::endl;
*/
	}



  

//pcl::visualization::CloudViewer viewer("Cloud Viewer");

boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer ("3D Viewer"));



view->addCoordinateSystem (1.0);
view->initCameraParameters ();

	for (int m = LH2E; m<= RK2F; m++){

		std::stringstream s;
		s << m;
		
	  	view->addSphere(joints[m], 0.025, s.str(), 0);
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

    //std::cout<< " written to file" << std::endl;
    //string in = "../../segmentation/segmented/" ;
    //in += outputfilename;
    //pcl::io::savePCDFileASCII (in.c_str(), output);
    //std::cout<< " saved to file" << std::endl;
   
   
   

   return(0);
}

int main (int argc , char** argv)
{
  if(argc != 3)
   {
     std::cout<< "Usage: segmentation <file_name> " << std::endl;
     exit(0);
   }
   
   segmentation(argv[1] , atoi(argv[2]));
  
  return(0);
}
