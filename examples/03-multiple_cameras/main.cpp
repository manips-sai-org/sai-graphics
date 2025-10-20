#include <iostream>
#include <string>

#include "SaiGraphics.h"

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/03-multiple_cameras/world.urdf";
const string robot_name = "RBot";
const string camera_name = "camera_fixed";
const string object_name = "Box";

int main()
{
	SaiModel::URDF_FOLDERS["EXAMPLE_03_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/03-multiple_cameras";
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics =
		new SaiGraphics::SaiGraphics(world_file, "sai world", true);

	unsigned long long counter = 0;

	cout << endl
		 << "A single robot but with several cameras.\nPress the N key to "
			"switch to the next camera and the B key to switch to the previous "
			"one. After some time, a screenshot from each camera will be saved."
		 << endl;

	// while window is open:
	while (graphics->isWindowOpen())
	{
		// update graphics robot and object poses in graphics and render
		graphics->renderGraphicsWorld();

		if (counter == 1000)
		{
			// save camera images
			graphics->getCameraImage("camera1")->saveToFile("camera1_image.png");
			graphics->getCameraImage("camera2")->saveToFile("camera2_image.png");
			graphics->getCameraImage("camera3")->saveToFile("camera3_image.png");
			graphics->getCameraImage("camera4")->saveToFile("camera4_image.png");
		}

		counter++;
	}

	return 0;
}
