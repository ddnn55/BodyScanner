#ifndef DEF_CCPARTIALMESH
#define DEF_CCPARTIALMESH

#include <iostream>
#include <string>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include "CCMesh.h"
#include <unistd.h>


class CCPartialMesh : public CCMesh
{

	public:
		CCPartialMesh(pcl::PolygonMesh *mesh);

	private:
		
				

};

#endif

