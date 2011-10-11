#ifndef JOINT_YAML
#define JOINT_YAML

#include <cstdio>
#include <ctype.h>
#include <cstring>
#include <cassert>

#include <boost/shared_ptr.hpp>




class SkeletonYaml {
public:
	typedef boost::shared_ptr<SkeletonYaml> Ptr;

	// FIXME get this list out of here (include from Skeleton.h)
	static std::vector<std::string> jointNames() {
		std::vector<std::string> jointNamesVar;
		jointNamesVar.push_back("left_hand");
		jointNamesVar.push_back("left_elbow");
		jointNamesVar.push_back("left_shoulder");
		jointNamesVar.push_back("left_hip");
		jointNamesVar.push_back("left_knee");
		jointNamesVar.push_back("left_foot");
		jointNamesVar.push_back("neck");
		jointNamesVar.push_back("head");
		jointNamesVar.push_back("torso");
		jointNamesVar.push_back("right_hand");
		jointNamesVar.push_back("right_elbow");
		jointNamesVar.push_back("right_shoulder");
		jointNamesVar.push_back("right_hip");
		jointNamesVar.push_back("right_knee");
		jointNamesVar.push_back("right_foot");
		return jointNamesVar;
	}

	static float namedFloat(char *buffer) {
	    while(buffer[0] == ' ') buffer++; // eat space
	    assert(buffer[0] == '-'); buffer++; // eat dash
	    while(buffer[0] == ' ') buffer++; // eat space
	    while(isalpha(buffer[0])) buffer++; // eat identifier
	    while(buffer[0] == ' ') buffer++; // eat space
	    assert(buffer[0] == '-'); buffer++; // eat dash
	    assert(buffer[0] == '-'); buffer++; // eat dash
	    while(buffer[0] == ' ') buffer++; // eat space
	    float v = atof(buffer);
	    return v;
	};
	static float unnamedFloat(char *buffer) {
	    while(buffer[0] == ' ') buffer++; // eat space
	    assert(buffer[0] == '-'); buffer++; // eat dash
	    while(buffer[0] == ' ') buffer++; // eat space
	    float v = atof(buffer);
	    return v;
	};

    //enum joints {LH,LE,LS,LHI,LK,LF,N,H,T,RH,RE,RS,RHI,RK,RF};
    static const int numJoints = 15;
    
    // data is placed here
    float translations[numJoints][3];
    float rotations[numJoints][9];
    
    SkeletonYaml() { };

    SkeletonYaml(const char * fname) {
        FILE *f = fopen(fname, "r");
        assert(f != NULL);
        
        int currentJoint = -1;
        char buffer[255];
        char *front;
        while(fgets(buffer, 255, f) != NULL) {
            if(buffer[0] != ' ') { // some new joint
                for(int i = 0; i < numJoints; i++) {
                    if(strstr(buffer, jointNames()[i].c_str()) == buffer) {
                        currentJoint = i;
                        // clear existing
                        for(int k = 0; k < 3; k++) translations[currentJoint][k] = 0;
                        for(int k = 0; k < 9; k++) rotations[currentJoint][k] = 0;
                        break;
                    }
                }
            } else {
                front = &buffer[1];
                while(front[0] == ' ') front++; // eat the spaces
                if(strstr(front, "translation") == front) {
                    for(int i = 0; i < 3; i++) {
                        assert(fgets(buffer, 255, f) != NULL);
                        translations[currentJoint][i] = namedFloat(buffer);
                    }
                } else if(strstr(front, "matrix3x3") == front) {
                    for(int i = 0; i < 9; i++) {
                        assert(fgets(buffer, 255, f) != NULL);
                        rotations[currentJoint][i] = unnamedFloat(buffer);
                    }                
                }
            }
        }
    }
};

/*
int main() {
    SkeletonYaml skel("../recordings/manohar/manohar_skeleton_00000.yaml");
    
    for(int j = 0; j < SkeletonYaml::numJoints; j++) {
        printf("%s: t(%f,%f,%f) r(%f,%f,%f, %f,%f,%f, %f,%f,%f)\n\n",
            jointNames[j],
            skel.translations[j][0],skel.translations[j][1],skel.translations[j][2],
            skel.rotations[j][0],skel.rotations[j][1],skel.rotations[j][2],
            skel.rotations[j][3],skel.rotations[j][4],skel.rotations[j][5],
            skel.rotations[j][6],skel.rotations[j][7],skel.rotations[j][8]
        );
    }
    return 0;
}
*/
#endif

