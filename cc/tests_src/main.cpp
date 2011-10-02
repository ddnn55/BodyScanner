#include <iostream>
#include "CCPointSet.h"
#include "Logger.h"
#include "CCFactorGraph.h"
using namespace std;

int main()
{

	Logger logger;

	CCPointSet testSet("/home/webaba/Downloads/mark1_pcd/1316652680.064411800.pcd");
	
	logger.log("test",0);

	CCPartialMesh xmesh(testSet.getMesh());

	CCModelMesh ymesh(testSet.getMesh());


	CCFactorGraph fg(&xmesh,&ymesh);


return 0;

}

