#include <ofAppGlutWindow.h>
#include "BodyPuppetApp.h"

int main() {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 640, 480, OF_WINDOW);
	ofRunApp(new BodyPuppet());
}
