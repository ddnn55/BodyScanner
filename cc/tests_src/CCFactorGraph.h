#ifndef DEF_CCFACTORGRAPH
#define DEF_CCFACTORGRAPH

#include <iostream>
#include <map>
#include <dai/alldai.h>  // Include main libDAI header file
#include <dai/jtree.h>
#include <dai/bp.h>
#include <dai/decmap.h>
#include <dai/factorgraph.h>
#include "CCModelMesh.h"
#include "CCPartialMesh.h"

class CCFactorGraph
{

	public:
		CCFactorGraph(CCPartialMesh *zmesh_, CCModelMesh *xmesh_);
		void addPairWiseFactors();
				

	private:
		CCPartialMesh *zmesh;
		CCModelMesh *xmesh;
		dai::FactorGraph fg;
		std::vector<dai::Factor> factors;

};

#endif

