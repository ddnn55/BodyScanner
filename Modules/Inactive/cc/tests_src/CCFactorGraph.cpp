#include "CCFactorGraph.h"
#include <pcl/io/io.h>
#include <pcl/Vertices.h>
#include <numeric>
#include <dai/varset.h>

using namespace std;
using namespace dai;


CCFactorGraph::CCFactorGraph(CCPartialMesh *zmesh_, CCModelMesh *xmesh_)
{
		xmesh = xmesh_;
		zmesh = zmesh_;

		cout << xmesh->countPolygons() << endl;

		addPairWiseFactors();
	
}


void CCFactorGraph::addPairWiseFactors(){

	// Needed initializations
	int NPx = xmesh->countPolygons();
	int NPz = zmesh->countPolygons();

	int Nx = xmesh->countVertices();
	int Nz = zmesh->countVertices();

	cout << "Nx : " << Nx << " / Nz : " << Nz << endl;

	vector<int> check2x(Nx*Nx);
	vector<int> check2z(Nz*Nz);

	vector<int> checkx(Nx);
	vector<int> checkz(Nz);

	vector<pcl::Vertices> *xpoly = xmesh->getPolygons();  
	vector<pcl::Vertices> *zpoly = zmesh->getPolygons();

	pcl::Vertices v;

	int index[3];

	int sum;

	vector<int *> rotations(1);

	// Correspondances variables
	vector<Var> covars(rotations.size()*Nz);
	VarSet coVarsSet;
	Var tempVar;
	int Nco = rotations.size()*Nx;

	// Factors
	Factor tempFac;
	vector<Factor> sFactors;
	vector<Factor> pFactors;	

	// Pairwise potential computation
	
	for (int k = 0; k<NPz; k++){ // Browse zmesh polygons

		// We get the polygon numbered k
		v = (*zpoly)[k];

		if (v.vertices.size() >= 3){

			for (int m = 0; m<3 ; m++)
			{

				// Get the vertex's index
				
				index[m] = v.vertices[m];
				
				// Add the single factors if not already created

				if (checkz[index[m]] == 0){

					tempVar = Var(index[m],Nco);
					coVarsSet |= tempVar; 

					tempFac = Factor(coVarsSet[index[m]]) // create single potential Factors

					for (int u = 0; u < Nx ; u++ ){

						double spinsig = CCTools.CompareSpinImages(xmesh,u,ymesh,index[m]);
						tempFac.set(u, )

					}
					

					sFactors.push_back(tempFac);

					checkz[index[m]] = 1;

					cout << "Correspondance variable c" << index[m] << " created" << endl;


				}


			}// End of vertices loop
			
			check2x[index[0]+Nx*index[1]] = 1;

		}
		


	
	}// End of polygons loop

	sum = std::accumulate( checkx.begin(), checkx.end(), 0);
	cout << endl << "Nb edges" << sum;


}
