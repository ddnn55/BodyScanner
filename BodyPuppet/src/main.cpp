#include <ofAppGlutWindow.h>
#include "BodyPuppetApp.h"

int main(int argc, char** argv) {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 640, 480, OF_WINDOW);
	ofRunApp(new BodyPuppet(argc, argv));
}
