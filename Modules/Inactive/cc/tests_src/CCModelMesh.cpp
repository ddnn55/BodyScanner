#include "CCModelMesh.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "CCTools.h"


using namespace std;


CCModelMesh::CCModelMesh(pcl::PolygonMesh *mesh_) : CCMesh(mesh_)
{
}
