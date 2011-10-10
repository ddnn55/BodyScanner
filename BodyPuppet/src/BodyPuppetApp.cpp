#include "BodyPuppetApp.h"

//--------------------------------------------------------------
void BodyPuppet::setup(){
    receiver.setup( PORT );

	ofSetVerticalSync(true);

	// this uses depth information for occlusion
	// rather than always drawing things on top of each other
	glEnable(GL_DEPTH_TEST);

	// this sets the camera's distance from the object
	cam.setDistance(100);


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

//--------------------------------------------------------------
void BodyPuppet::draw(){

	cam.begin();
		ofRotateX(ofRadToDeg(.5));
		ofRotateY(ofRadToDeg(-.5));

		ofBackground(0);

        ofCircle(100,100, 10);

        for(std::map<std::string, ofVec3f>::iterator joint = quickJoints.begin();
            joint != quickJoints.end();
            joint++)
        {
            //std::cout << "drawing a circle..." << std::endl;
            ofPushMatrix();
                ofTranslate( - joint->second.x, - joint->second.y, 0.0 /*joint->second.z*/);
                //ofSetColor(255,0,0);
                //ofFill();
                //ofBox(30);
                ofCircle(0, 0, 20);
                //ofNoFill();
                //ofSetColor(0);
                //ofBox(30);
            ofPopMatrix();
        }


		/*ofPushMatrix();
			ofTranslate(0,0,20);
			ofSetColor(0,0,255);
			ofFill();
			ofBox(5);
			ofNoFill();
			ofSetColor(0);
			ofBox(5);
		ofPopMatrix();*/
	cam.end();

	ofSetColor(255);
	string msg = string("Using mouse inputs to navigate ('m' to toggle): ") + (cam.getMouseInputEnabled() ? "YES" : "NO");
	msg += "\nfps: " + ofToString(ofGetFrameRate(), 2);
	ofDrawBitmapString(msg, 10, 20);




	// osc receive dump
    string buf;
	buf = "listening for osc messages on port" + ofToString( PORT );
	ofDrawBitmapString( buf, 10, 20 );

	// draw mouse state
	buf = "mouse: " + ofToString( mouseX, 4) +  " " + ofToString( mouseY, 4 );
	ofDrawBitmapString( buf, 430, 20 );
	ofDrawBitmapString( mouseButtonState, 580, 20 );

	for ( int i=0; i<NUM_MSG_STRINGS; i++ )
	{
		ofDrawBitmapString( msg_strings[i], 10, 40+15*i );
	}


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
