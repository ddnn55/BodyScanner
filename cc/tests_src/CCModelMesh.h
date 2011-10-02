#ifndef DEF_CCMODELMESH
#define DEF_CCMODELMESH

#include <iostream>
#include <string>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include "CCMesh.h"

#include <unistd.h>


class CCModelMesh : public CCMesh
{

	public:
		CCModelMesh(pcl::PolygonMesh *mesh);

	private:
				

};

#endif

