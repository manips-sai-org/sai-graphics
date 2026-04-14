// This example application loads a URDF world file and displays a simple robot
// that does not move

#include <iostream>
#include <string>

#include "SaiGraphics.h"

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/00-joint_slider/world.urdf";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics =
		new SaiGraphics::SaiGraphics(world_file, "sai world", true);

	cout << endl
		 << "This example parses a world file containing a simple robot, it "
			"creates a display to render the world. It is possible to move the "
			"camera in several ways."
		 << endl;
	cout << "Left mouse button: rotate camera" << endl;
	cout << "Left mouse button + ctrl: translate camera" << endl;
	cout << "Left mouse button + alt/shift: zoom camera" << endl;
	cout << "Middle mouse button: translate camera" << endl;
	cout << "Mouse wheel scroll: zoom camera" << endl;
	cout << "up, down, left, right keys: translate camera" << endl;
	cout << "A, Z keys: zoom camera" << endl;
	cout << "S key: print camera position in world" << endl;
	cout << "ESC key or close the window: exit the program" << endl;
	cout << endl << endl;

	graphics->showJointPositionSliders(true, "RBot");

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update graphics the rendering and the window display.
		// this automatically waits for the correct amount of time
		graphics->renderGraphicsWorld();
	}

	return 0;
}
