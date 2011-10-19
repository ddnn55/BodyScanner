#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include <ofxAssimpModelLoader.h>


#include <Body/Skeleton/Skeleton.h>
//#include <Body/Skin.h>

// listen on port 12345
#define PORT 7110
#define NUM_MSG_STRINGS 20

#include <map>

class BodyPuppet : public ofBaseApp {
	public:

		BodyPuppet(int argc, char** argv);
		ofMesh loadObj(string filename);

		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		void drawOSCSkeleton();

		ofEasyCam cam; // add mouse controls for camera movement
		ofShader skinRiggingShader;

		std::string meshFilename;
		std::string skeletonFilename;

		//ofxAssimpModelLoader body;

		ofMesh bodyMesh; // TODO use ofVboMesh?
		ofVec3f bodyMeshCentroid;
		Body::Skeleton::Pose rawMeshSkeletonPose;
		//Body::Skin skin;


		ofxOscReceiver	receiver;
		std::map<std::string, ofVec3f> quickJoints;

		int				current_msg_string;
		string		msg_strings[NUM_MSG_STRINGS];
		float			timers[NUM_MSG_STRINGS];

		int				mouseX, mouseY;
		string			mouseButtonState;


};
