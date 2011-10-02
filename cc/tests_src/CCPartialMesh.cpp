#include "CCPartialMesh.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "CCTools.h"

using namespace std;


CCPartialMesh::CCPartialMesh(pcl::PolygonMesh *mesh_) : CCMesh(mesh_)
{
}
