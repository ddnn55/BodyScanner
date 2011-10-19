#include "BodyPuppetApp.h"



BodyPuppet::BodyPuppet(int argc, char** argv)
{
	if(argc < 3)
	{
		std::cout << "Usage: BodyPuppet <body_mesh_file> <skeleton_filename.yaml>" << std::endl;
		std::exit(0);
	}
	meshFilename = argv[1];
	skeletonFilename = argv[2];
}

ofMesh BodyPuppet::loadObj(string filename)
{

	ofMesh mesh;

	FILE *f = fopen(filename.c_str(), "r");
	char buffer[255];

	unsigned int l = 0;
	while(fgets(buffer, 255, f))
	{
		//cout << buffer << endl;
		if(buffer[0] == 'v')
		{
			float x, y, z, r, g, b;
			sscanf(buffer, "v %f %f %f %f %f %f\n", &x, &y, &z, &r, &g, &b);
			//printf("vertex %f - %f  - %f - %f - %f - %f\n", x, y, z, r, g, b);
			mesh.addVertex(ofVec3f(x, y, z));
			mesh.addColor(ofFloatColor(r, g, b));
		}
		else if(buffer[0] == 'f')
		{
			unsigned int i1, i2, i3;
			sscanf(buffer, "f %u %u %u\n", &i1, &i2, &i3);
			//printf("face %u - %u - %u\n", i1, i2, i3);
			mesh.addTriangle(i1-1, i2-1, i3-1);
		}

//		if(l % 1000 == 0)
//			std::cout << "read line " << l << std::endl;

		l++;
	}

	fclose(f);

	return mesh;
}

//--------------------------------------------------------------
void BodyPuppet::setup(){
	ofDisableDataPath();

    receiver.setup( PORT );

	ofSetVerticalSync(true);

	// this uses depth information for occlusion
	// rather than always drawing things on top of each other
	glEnable(GL_DEPTH_TEST);

	// this sets the camera's distance from the object
	cam.setDistance(100);

	skinRiggingShader.load("../src/Shaders/SkinRiggingShader.vert", "../src/Shaders/SkinRiggingShader.frag"); // TODO handle this better with a build copy


	//body.loadModel(meshFilename, false);
	//body.enableColors();
	//cout << "body.getMesh(0).hasColors(): " << body.getMesh(0).hasColors() << endl;
	bodyMesh = loadObj(meshFilename);
	bodyMeshCentroid = bodyMesh.getCentroid();

	cout << "centroid: " << bodyMeshCentroid << endl;

	rawMeshSkeletonPose = Body::Skeleton::Pose(SkeletonYaml(skeletonFilename.c_str()));

	std::cout << "loaded skeleton" << std::endl;

	vector<ofVec3f> vertices = bodyMesh.getVertices();
	for(int v = 0; v < vertices.size(); v++)
	{
		Body::Skeleton::Pose::PointRigging rigging = rawMeshSkeletonPose.getRiggingForPoint(vertices[v].x, vertices[v].y, vertices[v].z);
		// TODO put weights in gl vertex user data
	}
	// TODO tell GL about user data
	// TODO write poser into vertex shader

	std::cout << "created BodySegmentation" << endl;


	//ofScale(1, 1, -1);
}

//--------------------------------------------------------------
void BodyPuppet::update(){
	// check for waiting messages
	while( receiver.hasWaitingMessages() )
	{
		// get the next message
		ofxOscMessage m;
		receiver.getNextMessage( &m );

        std::cout << m.getAddress();
        if(m.getAddress() == "/joint")
        {
            std::string jointName = m.getArgAsString(0);
            //if(quickJoints.find(jointName) == quickJoints.end())
            quickJoints[jointName] = ofVec3f(m.getArgAsFloat(2), m.getArgAsFloat(3), m.getArgAsFloat(4));
        }


        for ( int i=0; i<m.getNumArgs(); i++ )
        {
            // get the argument type

            // display the argument - make sure we get the right type
            if( m.getArgType( i ) == OFXOSC_TYPE_INT32 )
                std::cout << " " << ofToString( m.getArgAsInt32( i ) );
            else if( m.getArgType( i ) == OFXOSC_TYPE_FLOAT )
                std::cout << " " << ofToString( m.getArgAsFloat( i ) );
            else if( m.getArgType( i ) == OFXOSC_TYPE_STRING )
             {

             }
            /*else
                msg_string += "unknown";*/
        }

        std::cout << std::endl;


        // add to the list of strings to display
        //msg_strings[current_msg_string] = msg_string;
        //timers[current_msg_string] = ofGetElapsedTimef() + 5.0f;
        //current_msg_string = ( current_msg_string + 1 ) % NUM_MSG_STRINGS;
        // clear the next line
        //msg_strings[current_msg_string] = "";


	}

}

void BodyPuppet::drawOSCSkeleton()
{
    ofPushMatrix();
		ofScale(1.0/1000.0, -1.0/1000.0, 1.0/1000.0);

		for(std::map<std::string, ofVec3f>::iterator joint = quickJoints.begin();
			joint != quickJoints.end();
			joint++)
		{
				//ofSetColor(255,0,0);
				//ofFill();
				ofSphere(-joint->second.x, -joint->second.y, -joint->second.z, 10);
				//ofCircle(0, 0, 20);
				//ofNoFill();
				//ofSetColor(0);
				//ofBox(30);
		}

	ofPopMatrix();
}

//--------------------------------------------------------------
void BodyPuppet::draw(){

	cam.begin();
		ofRotateX(ofRadToDeg(.5));
		ofRotateY(ofRadToDeg(-.5));

		ofBackground(0);
        ofSetColor(255, 255, 255, 255);

        ofDrawAxis(1.0);

		ofScale(10, 10, 10);


		// draw OSC skeleton
		ofPushStyle();
			ofSetColor(50, 250, 50);
			drawOSCSkeleton();
		ofPopStyle();

        ofPushMatrix();
			ofTranslate(-bodyMeshCentroid.x, -bodyMeshCentroid.y, -bodyMeshCentroid.z);

			// draw body skeleton
			ofPushMatrix();
				rawMeshSkeletonPose.draw();
			ofPopMatrix();

			// draw body mesh
			ofPushMatrix();
				ofScale(1, -1, 1); // FIXME skeleton and obj files should have same scale and stuff...
								   // this might in fact be a rotation, not just flip??
				skinRiggingShader.begin();
					//body.draw(OF_MESH_FILL);
					bodyMesh.draw();
				skinRiggingShader.end();
			ofPopMatrix();
		ofPopMatrix();



	cam.end();

	ofSetColor(255);
	string msg = string("Using mouse inputs to navigate ('m' to toggle): ") + (cam.getMouseInputEnabled() ? "YES" : "NO");
	msg += "\nfps: " + ofToString(ofGetFrameRate(), 2);
	ofDrawBitmapString(msg, 10, 20);


}

//--------------------------------------------------------------
void BodyPuppet::keyPressed(int key){
	switch(key) {
		case 'M':
		case 'm':
			if(cam.getMouseInputEnabled()) cam.disableMouseInput();
			else cam.enableMouseInput();
			break;

		case 'F':
		case 'f':
			ofToggleFullscreen();
			break;
	}
}

//--------------------------------------------------------------
void BodyPuppet::keyReleased(int key){

}

//--------------------------------------------------------------
void BodyPuppet::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void BodyPuppet::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void BodyPuppet::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void BodyPuppet::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void BodyPuppet::windowResized(int w, int h){

}

//--------------------------------------------------------------
void BodyPuppet::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void BodyPuppet::dragEvent(ofDragInfo dragInfo){

}
